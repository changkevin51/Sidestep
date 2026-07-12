#include "car_collision.hpp"
#include "joint_actions.hpp"
#include "v2v_network.hpp"

#ifdef SIDESTEP_ENABLE_GEMINI
#include "gemini_client.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using car_collision::Car;
using car_collision::CollisionResult;
using car_collision::Vec2;
using Clock = std::chrono::steady_clock;

const double kMaximumTtcSeconds = 10.0;
const double kSweepStepSeconds = 0.05;
const double kSmartDecisionThresholdSeconds = 3.0;
const double kMaximumDecelerationMps2 = 7.5;
const double kEmergencyStopDecelerationMps2 = 9.0;
const double kEmergencyStopDurationSeconds = 4.0;
const double kDefaultSteeringRateDegPerSecond = 28.0;
const double kDefaultSwerveDurationSeconds = 2.2;
const double kMinimumControlAccelerationMps2 = -12.0;
const double kMaximumControlAccelerationMps2 = 5.0;
const double kMaximumSteeringRateDegPerSecond = 90.0;
const double kMaximumControlDurationSeconds = 10.0;
const std::chrono::milliseconds kBroadcastPeriod(50);       // 20 Hz
const std::chrono::milliseconds kConsensusTimeout(150);
#ifdef SIDESTEP_ENABLE_GEMINI
const std::chrono::milliseconds kGeminiRequestTimeout(20000);
const std::chrono::milliseconds kGeminiConnectTimeout(1000);
#endif
const std::chrono::milliseconds kGeminiConsensusTimeout(20500);
const std::chrono::milliseconds kTelemetryFreshness(750);
const std::chrono::milliseconds kMaximumPairArrivalSkew(125);
const std::chrono::milliseconds kPostMatchProposalRelay(250);
const std::chrono::milliseconds kClearHoldTime(1000);

volatile std::sig_atomic_t gStopRequested = 0;
std::mutex gLogMutex;

void signalHandler(int) {
    gStopRequested = 1;
}

void logLine(const std::string& message) {
    std::lock_guard<std::mutex> lock(gLogMutex);
    std::cout << message << std::endl;
}

enum class SafetyState {
    MONITORING,
    PROPOSING,
    EXECUTE,
    EMERGENCY_STOP
};

struct Telemetry {
    std::string id;
    double latitude_deg;
    double longitude_deg;
    double heading_deg;
    double speed_mps;
    double width_m;
    double length_m;
    std::string source_state;
    std::string scenario_id;
    Clock::time_point received_at;

    Telemetry()
        : latitude_deg(0.0),
          longitude_deg(0.0),
          heading_deg(0.0),
          speed_mps(0.0),
          width_m(0.0),
          length_m(0.0),
          received_at(Clock::now()) {}
};

struct PendingProposal {
    std::string car1_id;
    std::string car2_id;
    JointActions actions;
    double ttc_seconds;

    PendingProposal() : ttc_seconds(0.0) {}
};

struct Assessment {
    bool ready;
    bool collision;
    PendingProposal proposal;

    Assessment() : ready(false), collision(false) {}
};

struct ExecutionEvent {
    bool occurred;
    VehicleControl control;
    double ttc_seconds;
    std::string reason;

    ExecutionEvent() : occurred(false), ttc_seconds(0.0) {}
};

struct Runtime {
    explicit Runtime(const std::string& own_id,
                     const std::string& peer_id,
                     bool enable_audio)
        : local_id(own_id),
          configured_peer_id(peer_id),
          audio_enabled(enable_audio),
          state(SafetyState::MONITORING),
          has_pending(false),
          proposal_transmitted(false),
          execution_ttc(0.0),
          clear_timer_active(false),
          network_send_failure_reported(false) {}

    std::string local_id;
    std::string configured_peer_id;
    bool audio_enabled;
    std::mutex mutex;
    std::map<std::string, Telemetry> cars;
    SafetyState state;
    bool has_pending;
    bool proposal_transmitted;
    PendingProposal pending;
    Clock::time_point proposal_deadline;
    Clock::time_point proposal_relay_until;
    VehicleControl execution_control;
    double execution_ttc;
    Clock::time_point execution_started;
    bool clear_timer_active;
    Clock::time_point clear_since;
    bool network_send_failure_reported;
    std::string scenario_id;
};

std::string trim(const std::string& input) {
    const std::string whitespace(" \t\r\n");
    const std::string::size_type first = input.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    const std::string::size_type last = input.find_last_not_of(whitespace);
    return input.substr(first, last - first + 1U);
}

bool splitCsvExact(const std::string& input,
                   std::size_t expected_fields,
                   std::vector<std::string>& fields) {
    fields.clear();
    if (input.size() > v2v::kMaxDatagramSize) {
        return false;
    }

    std::string::size_type start = 0U;
    while (start <= input.size()) {
        const std::string::size_type comma = input.find(',', start);
        const std::string field = trim(input.substr(
            start,
            comma == std::string::npos ? std::string::npos : comma - start));
        if (field.empty() || field.size() > 128U) {
            fields.clear();
            return false;
        }
        fields.push_back(field);
        if (fields.size() > expected_fields) {
            fields.clear();
            return false;
        }
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1U;
    }
    return fields.size() == expected_fields;
}

bool parseFiniteDouble(const std::string& text, double& value) {
    errno = 0;
    char* end = 0;
    const char* begin = text.c_str();
    const double parsed = std::strtod(begin, &end);
    if (begin == end || end == 0 ||
        end != begin + static_cast<std::ptrdiff_t>(text.size()) ||
        errno == ERANGE ||
        std::isfinite(parsed) == 0) {
        return false;
    }
    value = parsed;
    return true;
}

bool validIdentifier(const std::string& identifier) {
    if (identifier.empty() || identifier.size() > 32U) {
        return false;
    }
    for (std::string::const_iterator it = identifier.begin();
         it != identifier.end(); ++it) {
        const char c = *it;
        const bool valid = (c >= 'A' && c <= 'Z') ||
                           (c >= 'a' && c <= 'z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!valid) {
            return false;
        }
    }
    return true;
}

bool validStateText(const std::string& state) {
    return validIdentifier(state);
}

bool validVehicleControl(const VehicleControl& control) {
    return std::isfinite(control.acceleration_mps2) &&
           std::isfinite(control.steering_rate_deg_per_second) &&
           std::isfinite(control.duration_seconds) &&
           control.acceleration_mps2 >= kMinimumControlAccelerationMps2 &&
           control.acceleration_mps2 <= kMaximumControlAccelerationMps2 &&
           std::fabs(control.steering_rate_deg_per_second) <=
               kMaximumSteeringRateDegPerSecond &&
           control.duration_seconds >= 0.0 &&
           control.duration_seconds <= kMaximumControlDurationSeconds;
}

std::string scenarioIdFromState(const std::string& state) {
    const char* prefixes[] = {
        "SIMULATED_", "SENSOR_", "INJECTED_", "MONITORING_",
        "PROPOSING_", "AI_ANALYZING_", "EXECUTE_", "EMERGENCY_STOP_"
    };
    for (std::size_t index = 0U;
         index < sizeof(prefixes) / sizeof(prefixes[0]); ++index) {
        const std::string prefix(prefixes[index]);
        if (state.compare(0U, prefix.size(), prefix) == 0) {
            const std::string scenario_id = state.substr(prefix.size());
            return validIdentifier(scenario_id) ? scenario_id : std::string();
        }
    }
    return std::string();
}

bool parseTelemetry(const std::string& payload, Telemetry& telemetry) {
    std::vector<std::string> fields;
    // The topic has already been removed by V2VNetwork. The remaining fields
    // are car_id, latitude, longitude, heading, speed, width, length, state.
    if (!splitCsvExact(payload, 8U, fields)) {
        return false;
    }

    Telemetry parsed;
    parsed.id = fields[0];
    parsed.source_state = fields[7];
    if (!validIdentifier(parsed.id) || !validStateText(parsed.source_state) ||
        !parseFiniteDouble(fields[1], parsed.latitude_deg) ||
        !parseFiniteDouble(fields[2], parsed.longitude_deg) ||
        !parseFiniteDouble(fields[3], parsed.heading_deg) ||
        !parseFiniteDouble(fields[4], parsed.speed_mps) ||
        !parseFiniteDouble(fields[5], parsed.width_m) ||
        !parseFiniteDouble(fields[6], parsed.length_m)) {
        return false;
    }
    parsed.scenario_id = scenarioIdFromState(parsed.source_state);

    if (parsed.latitude_deg < -90.0 || parsed.latitude_deg > 90.0 ||
        parsed.longitude_deg < -180.0 || parsed.longitude_deg > 180.0 ||
        parsed.heading_deg < 0.0 || parsed.heading_deg >= 360.0 ||
        parsed.speed_mps < 0.0 || parsed.speed_mps > 120.0 ||
        parsed.width_m <= 0.0 || parsed.width_m > 10.0 ||
        parsed.length_m <= 0.0 || parsed.length_m > 30.0) {
        return false;
    }

    parsed.received_at = Clock::now();
    telemetry = parsed;
    return true;
}

bool parseProposal(const std::string& payload,
                   std::string& sender_id,
                   JointActions& actions) {
    std::vector<std::string> fields;
    if (!splitCsvExact(payload, 7U, fields) ||
        !validIdentifier(fields[0])) {
        return false;
    }

    JointActions parsed;
    if (!parseFiniteDouble(fields[1], parsed.car1.acceleration_mps2) ||
        !parseFiniteDouble(fields[2],
                           parsed.car1.steering_rate_deg_per_second) ||
        !parseFiniteDouble(fields[3], parsed.car1.duration_seconds) ||
        !parseFiniteDouble(fields[4], parsed.car2.acceleration_mps2) ||
        !parseFiniteDouble(fields[5],
                           parsed.car2.steering_rate_deg_per_second) ||
        !parseFiniteDouble(fields[6], parsed.car2.duration_seconds) ||
        !validVehicleControl(parsed.car1) ||
        !validVehicleControl(parsed.car2)) {
        return false;
    }

    sender_id = fields[0];
    actions = parsed;
    return true;
}

std::string formatDouble(double value, int precision) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::string safetyStateText(const Runtime& runtime) {
    switch (runtime.state) {
        case SafetyState::MONITORING:
            return "MONITORING";
        case SafetyState::PROPOSING:
            if (runtime.has_pending &&
                (runtime.pending.actions.decision_path == "SMART_AI_STUB" ||
                 runtime.pending.actions.decision_path == "GEMMA_3" ||
                 runtime.pending.actions.decision_path ==
                     "GEMINI_3_5_FLASH")) {
                return "AI_ANALYZING";
            }
            return "PROPOSING";
        case SafetyState::EXECUTE:
            return "EXECUTE";
        case SafetyState::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
    }
    return "MONITORING";
}

std::string scenarioSafetyState(const Runtime& runtime) {
    const std::string state = safetyStateText(runtime);
    return runtime.scenario_id.empty()
        ? state
        : state + "_" + runtime.scenario_id;
}

std::string formatTelemetryPayload(const Telemetry& telemetry,
                                   const std::string& state) {
    std::ostringstream output;
    output << telemetry.id << ','
           << std::fixed << std::setprecision(8)
           << telemetry.latitude_deg << ',' << telemetry.longitude_deg << ','
           << std::setprecision(3)
           << telemetry.heading_deg << ',' << telemetry.speed_mps << ','
           << telemetry.width_m << ',' << telemetry.length_m << ',' << state;
    return output.str();
}

std::string formatProposalPayload(const std::string& sender_id,
                                  const PendingProposal& proposal) {
    return sender_id + "," +
           formatDouble(proposal.actions.car1.acceleration_mps2, 3) + "," +
           formatDouble(
               proposal.actions.car1.steering_rate_deg_per_second, 3) + "," +
           formatDouble(proposal.actions.car1.duration_seconds, 3) + "," +
           formatDouble(proposal.actions.car2.acceleration_mps2, 3) + "," +
           formatDouble(
               proposal.actions.car2.steering_rate_deg_per_second, 3) + "," +
           formatDouble(proposal.actions.car2.duration_seconds, 3);
}

std::string formatExecutionPayload(const Runtime& runtime) {
    return runtime.local_id + "," +
           formatDouble(runtime.execution_control.acceleration_mps2, 3) + "," +
           formatDouble(
               runtime.execution_control.steering_rate_deg_per_second, 3) +
           "," + formatDouble(runtime.execution_control.duration_seconds, 3) +
           "," + safetyStateText(runtime) + "," +
           formatDouble(runtime.execution_ttc, 3);
}

bool controlsMatch(const VehicleControl& first, const VehicleControl& second) {
    const double tolerance = 0.000501;
    return std::fabs(first.acceleration_mps2 - second.acceleration_mps2) <=
               tolerance &&
           std::fabs(first.steering_rate_deg_per_second -
                     second.steering_rate_deg_per_second) <= tolerance &&
           std::fabs(first.duration_seconds - second.duration_seconds) <=
               tolerance;
}

bool actionsMatch(const JointActions& first, const JointActions& second) {
    return controlsMatch(first.car1, second.car1) &&
           controlsMatch(first.car2, second.car2);
}

bool proposalsMatch(const PendingProposal& first,
                    const PendingProposal& second) {
    return first.car1_id == second.car1_id &&
           first.car2_id == second.car2_id &&
           actionsMatch(first.actions, second.actions);
}

std::chrono::milliseconds consensusTimeoutFor(
    const JointActions& actions) {
    return actions.decision_path == "GEMINI_3_5_FLASH"
        ? kGeminiConsensusTimeout
        : kConsensusTimeout;
}

std::string consensusTimeoutReason(const PendingProposal& proposal,
                                   bool local_send_failed) {
    const std::chrono::milliseconds timeout =
        consensusTimeoutFor(proposal.actions);
    return std::to_string(timeout.count()) +
           " ms consensus timeout" +
           (local_send_failed ? " (local proposal send failed)" : "");
}

double cross(const Vec2& first, const Vec2& second) {
    return first.x * second.y - first.y * second.x;
}

Vec2 estimatedIntersectionPoint(const Car& first,
                                const Car& second,
                                double ttc_seconds) {
    const Vec2 first_velocity = first.velocity();
    const Vec2 second_velocity = second.velocity();
    const double denominator = cross(first_velocity, second_velocity);

    if (std::fabs(denominator) > 1e-9) {
        const Vec2 delta = second.center - first.center;
        const double first_time = cross(delta, second_velocity) / denominator;
        const double second_time = cross(delta, first_velocity) / denominator;
        if (first_time >= 0.0 && second_time >= 0.0) {
            return first.center + first_velocity * first_time;
        }
    }

    // Parallel/head-on paths do not have a unique ray intersection. The center
    // of the two projected collision boxes is a stable common fallback.
    return (first.positionAt(ttc_seconds) + second.positionAt(ttc_seconds)) /
           2.0;
}

JointActions evaluateDefaultDecision(const Car& first,
                                     const Car& second,
                                     double ttc_seconds) {
    const Vec2 intersection =
        estimatedIntersectionPoint(first, second, ttc_seconds);
    const double first_centerline_distance =
        (intersection - first.center).length();
    const double second_centerline_distance =
        (intersection - second.center).length();

    // The SAT TTC marks first body contact, which can occur before either car's
    // center reaches the mathematical path intersection. Reserve the full
    // consensus window, then use the smaller of centerline distance and travel
    // remaining before first overlap. This keeps the mandated v^2/(2a) check
    // conservative for perpendicular, head-on, and nearly parallel paths.
    const double decision_latency_seconds =
        static_cast<double>(kConsensusTimeout.count()) / 1000.0;
    const double usable_time =
        std::max(0.0, ttc_seconds - decision_latency_seconds);
    const double first_distance =
        std::min(first_centerline_distance, first.speed_mps * usable_time);
    const double second_distance =
        std::min(second_centerline_distance, second.speed_mps * usable_time);
    const double first_stopping_distance =
        first.speed_mps * first.speed_mps / (2.0 * kMaximumDecelerationMps2);
    const double second_stopping_distance =
        second.speed_mps * second.speed_mps / (2.0 * kMaximumDecelerationMps2);

    JointActions actions;
    actions.decision_path = "DEFAULT_HEURISTIC";
    const bool first_can_stop = first.speed_mps <= 0.01 ||
                                first_distance > first_stopping_distance;
    const bool second_can_stop = second.speed_mps <= 0.01 ||
                                 second_distance > second_stopping_distance;
    if (first_can_stop && second_can_stop) {
        actions.car1 = VehicleControl(
            -kMaximumDecelerationMps2,
            0.0,
            first.speed_mps / kMaximumDecelerationMps2);
        actions.car2 = VehicleControl(
            -kMaximumDecelerationMps2,
            0.0,
            second.speed_mps / kMaximumDecelerationMps2);
    } else {
        // The canonical lower-ID car always moves right and the higher-ID car
        // left. Both Pis therefore derive the same opposite-direction plan.
        actions.car1 = VehicleControl(
            0.0,
            kDefaultSteeringRateDegPerSecond,
            kDefaultSwerveDurationSeconds);
        actions.car2 = VehicleControl(
            0.0,
            -kDefaultSteeringRateDegPerSecond,
            kDefaultSwerveDurationSeconds);
    }
    return actions;
}

// -------------------------------------------------------------------------
// SMART PATHWAY -- used only when TTC >= 3.0 seconds.
// -------------------------------------------------------------------------
JointActions smartBrakingFallback(const Car& first, const Car& second) {
    JointActions fallback;
    fallback.car1 = VehicleControl(
        -kMaximumDecelerationMps2,
        0.0,
        first.speed_mps / kMaximumDecelerationMps2);
    fallback.car2 = VehicleControl(
        -kMaximumDecelerationMps2,
        0.0,
        second.speed_mps / kMaximumDecelerationMps2);
    fallback.decision_path = "SMART_BRAKING_FALLBACK";
    return fallback;
}

JointActions evaluateSmartDecision(const Car& first,
                                   const Car& second,
                                   double ttc_seconds) {
#ifdef SIDESTEP_ENABLE_GEMINI
    // SideStep's canonical first/second ordering maps directly to Gemini's
    // Car A/Car B ordering, so both peers build the same model input.
    collision_ai::VehicleState state;
    state.timeToCollision = ttc_seconds;
    state.carAX = first.center.x;
    state.carAY = first.center.y;
    state.carASpeed = first.speed_mps;
    state.carAHeading = first.heading_deg;
    state.carAAcceleration = 0.0;  // Not present in current telemetry packets.
    state.carAWidth = first.width_m;
    state.carALength = first.length_m;

    state.carBX = second.center.x;
    state.carBY = second.center.y;
    state.carBSpeed = second.speed_mps;
    state.carBHeading = second.heading_deg;
    state.carBAcceleration = 0.0;  // Not present in current telemetry packets.
    state.carBWidth = second.width_m;
    state.carBLength = second.length_m;

    const Vec2 intersection =
        estimatedIntersectionPoint(first, second, ttc_seconds);
    state.carADistanceToCollision = (intersection - first.center).length();
    state.carBDistanceToCollision = (intersection - second.center).length();
    // This deployment uses the same fixed road/obstacle context as the Gemini
    // client's demonstration main.cpp. Kinematics remain live so the model sees
    // the current SideStep positions, speeds, headings, dimensions, and TTC.
    state.scene.carALeftClear = false;
    state.scene.carARightClear = false;
    state.scene.carAAheadOccupied = true;
    state.scene.carABehindOccupied = false;
    state.scene.carBLeftClear = false;
    state.scene.carBRightClear = false;
    state.scene.carBAheadOccupied = false;
    state.scene.carBBehindOccupied = false;
    state.scene.carALeftHazard =
        "Concrete highway median barrier separating frontage road from "
        "Interstate 40.";
    state.scene.carARightHazard =
        "Steep soil embankment and bridge concrete support structure.";
    state.scene.carAAheadHazard =
        "Flatbed semi-truck trailer crossing perpendicular to the travel lane.";
    state.scene.carBLeftHazard =
        "Bridge underpass concrete pillars and abutment wall.";
    state.scene.carBRightHazard =
        "Bridge underpass concrete pillars and opposing road curb.";
    state.scene.pedestrianPresent = false;
    state.scene.wetRoad = false;
    state.scene.intersection = true;
    state.scene.constructionZone = false;
    state.scene.additionalDescription =
        "High-speed frontage road underpass conflict. High concrete barriers "
        "and bridge structures remove any lateral escape paths for both "
        "vehicles.";

    try {
        collision_ai::GeminiClient::Config config;
        if (const char* api_key = std::getenv("GEMINI_API_KEY")) {
            config.apiKey = api_key;
        }
        if (const char* endpoint = std::getenv("GEMINI_API_ENDPOINT")) {
            config.endpoint = endpoint;
        }
        if (const char* model = std::getenv("GEMINI_MODEL")) {
            config.model = model;
        }

        config.timeout = kGeminiRequestTimeout;
        config.connectTimeout = kGeminiConnectTimeout;

        const collision_ai::GeminiClient client(config);
        const collision_ai::CollisionAvoidanceDecision decision =
            client.getDecision(state);

        JointActions result;
        result.car1 = VehicleControl(
            decision.carA.accelerationMps2,
            decision.carA.steeringRateDegreesPerSecond,
            decision.carA.steeringDurationSeconds);
        result.car2 = VehicleControl(
            decision.carB.accelerationMps2,
            decision.carB.steeringRateDegreesPerSecond,
            decision.carB.steeringDurationSeconds);
        result.decision_path = "GEMINI_3_5_FLASH";

        if (!validVehicleControl(result.car1) ||
            !validVehicleControl(result.car2)) {
            logLine("Gemini controls failed SideStep validation; using "
                    "deterministic braking fallback.");
            return smartBrakingFallback(first, second);
        }
        return result;
    } catch (const std::exception& error) {
        logLine(std::string("Gemini integration failed: ") + error.what() +
                "; using deterministic braking fallback.");
        return smartBrakingFallback(first, second);
    }
#else
    (void)ttc_seconds;
    return smartBrakingFallback(first, second);
#endif
}

Assessment assessTelemetry(const Telemetry& first_telemetry,
                           const Telemetry& second_telemetry,
                           bool choose_actions = true) {
    Assessment assessment;
    assessment.ready = true;
    assessment.proposal.car1_id = first_telemetry.id;
    assessment.proposal.car2_id = second_telemetry.id;

    try {
        const double reference_latitude = first_telemetry.latitude_deg;
        const double reference_longitude = first_telemetry.longitude_deg;
        const Car first(
            first_telemetry.id,
            car_collision::latLonToLocalMeters(
                first_telemetry.latitude_deg,
                first_telemetry.longitude_deg,
                reference_latitude,
                reference_longitude),
            first_telemetry.length_m,
            first_telemetry.width_m,
            first_telemetry.heading_deg,
            first_telemetry.speed_mps);
        const Car second(
            second_telemetry.id,
            car_collision::latLonToLocalMeters(
                second_telemetry.latitude_deg,
                second_telemetry.longitude_deg,
                reference_latitude,
                reference_longitude),
            second_telemetry.length_m,
            second_telemetry.width_m,
            second_telemetry.heading_deg,
            second_telemetry.speed_mps);

        const CollisionResult collision = car_collision::sweepForCollision(
            first, second, kMaximumTtcSeconds, kSweepStepSeconds);
        assessment.collision = collision.will_collide;
        if (!collision.will_collide) {
            return assessment;
        }

        assessment.proposal.ttc_seconds = collision.ttc_seconds;

        // EXECUTE/EMERGENCY_STOP still need collision/clear monitoring, but
        // must not launch another model request for the same event.
        if (!choose_actions) {
            return assessment;
        }

        // -----------------------------------------------------------------
        // DEFAULT (if-statement) PATHWAY -- immediate deterministic response.
        // -----------------------------------------------------------------
        if (collision.ttc_seconds < kSmartDecisionThresholdSeconds) {
            assessment.proposal.actions =
                evaluateDefaultDecision(first, second, collision.ttc_seconds);
        } else {
            assessment.proposal.actions =
                evaluateSmartDecision(first, second, collision.ttc_seconds);
        }
    } catch (const std::exception& error) {
        assessment.ready = false;
        assessment.collision = false;
        logLine(std::string("Collision assessment rejected invalid data: ") +
                error.what());
    }
    return assessment;
}

bool externalLocalTelemetry(const Telemetry& telemetry) {
    return telemetry.source_state == "SIMULATED" ||
           telemetry.source_state == "SENSOR" ||
           telemetry.source_state == "INJECTED" ||
           telemetry.source_state.compare(0U, 10U, "SIMULATED_") == 0 ||
           telemetry.source_state.compare(0U, 7U, "SENSOR_") == 0 ||
           telemetry.source_state.compare(0U, 9U, "INJECTED_") == 0;
}

void resetForScenario(Runtime& runtime, const std::string& scenario_id) {
    runtime.cars.clear();
    runtime.state = SafetyState::MONITORING;
    runtime.has_pending = false;
    runtime.proposal_transmitted = false;
    runtime.pending = PendingProposal();
    runtime.execution_control = VehicleControl();
    runtime.execution_ttc = 0.0;
    runtime.clear_timer_active = false;
    runtime.network_send_failure_reported = false;
    runtime.scenario_id = scenario_id;
}

void processScenarioReset(Runtime& runtime, const std::string& payload) {
    const std::string scenario_id = trim(payload);
    if (!validIdentifier(scenario_id)) {
        logLine("Discarded malformed RESET packet.");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        resetForScenario(runtime, scenario_id);
    }
    logLine("Simulator reset " + scenario_id +
            "; cleared telemetry, proposal, and execution state.");
}

bool copyFreshPair(Runtime& runtime,
                   Telemetry& first,
                   Telemetry& second) {
    const Clock::time_point now = Clock::now();
    std::lock_guard<std::mutex> lock(runtime.mutex);

    std::map<std::string, Telemetry>::const_iterator local =
        runtime.cars.find(runtime.local_id);
    if (local == runtime.cars.end() ||
        now - local->second.received_at > kTelemetryFreshness) {
        return false;
    }

    std::map<std::string, Telemetry>::const_iterator peer = runtime.cars.end();
    if (!runtime.configured_peer_id.empty()) {
        peer = runtime.cars.find(runtime.configured_peer_id);
    } else {
        for (std::map<std::string, Telemetry>::const_iterator it =
                 runtime.cars.begin(); it != runtime.cars.end(); ++it) {
            if (it->first != runtime.local_id) {
                peer = it;
                break;
            }
        }
    }
    if (peer == runtime.cars.end() ||
        now - peer->second.received_at > kTelemetryFreshness) {
        return false;
    }

    const Clock::duration arrival_skew =
        local->second.received_at >= peer->second.received_at
            ? local->second.received_at - peer->second.received_at
            : peer->second.received_at - local->second.received_at;
    if (arrival_skew > kMaximumPairArrivalSkew) {
        return false;
    }

    if (local->first < peer->first) {
        first = local->second;
        second = peer->second;
    } else {
        first = peer->second;
        second = local->second;
    }
    return true;
}

void applyAssessment(Runtime& runtime, const Assessment& assessment) {
    if (!assessment.ready) {
        return;
    }

    const Clock::time_point now = Clock::now();
    bool announce = false;
    PendingProposal announced_proposal;
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);

        if (runtime.state == SafetyState::EXECUTE ||
            runtime.state == SafetyState::EMERGENCY_STOP) {
            if (assessment.collision) {
                runtime.clear_timer_active = false;
            } else if (!runtime.clear_timer_active) {
                runtime.clear_timer_active = true;
                runtime.clear_since = now;
            } else if (now - runtime.clear_since >= kClearHoldTime) {
                runtime.state = SafetyState::MONITORING;
                runtime.has_pending = false;
                runtime.proposal_transmitted = false;
                runtime.execution_control = VehicleControl();
                runtime.execution_ttc = 0.0;
                runtime.clear_timer_active = false;
                logLine("Collision corridor clear; returning to MONITORING.");
            }
            return;
        }

        // Once this collision event has a proposal, freeze it and its original
        // deadline. A transient mixed telemetry sample must not cancel or
        // indefinitely restart a safety decision already under consensus.
        if (runtime.state == SafetyState::PROPOSING && runtime.has_pending) {
            return;
        }

        if (!assessment.collision) {
            runtime.state = SafetyState::MONITORING;
            runtime.has_pending = false;
            runtime.proposal_transmitted = false;
            return;
        }

        if (!runtime.has_pending ||
            !proposalsMatch(runtime.pending, assessment.proposal)) {
            runtime.pending = assessment.proposal;
            runtime.has_pending = true;
            runtime.proposal_transmitted = false;
            runtime.proposal_deadline =
                now + consensusTimeoutFor(runtime.pending.actions);
            runtime.state = SafetyState::PROPOSING;
            announce = true;
            announced_proposal = assessment.proposal;
        }
    }

    if (announce) {
        std::ostringstream message;
        message << "Collision predicted: TTC=" << std::fixed
                << std::setprecision(3) << announced_proposal.ttc_seconds
                << "s, path=" << announced_proposal.actions.decision_path
                << ", proposal=[car1(accel="
                << announced_proposal.actions.car1.acceleration_mps2
                << "m/s^2, steer="
                << announced_proposal.actions.car1
                       .steering_rate_deg_per_second
                << "deg/s, duration="
                << announced_proposal.actions.car1.duration_seconds
                << "s), car2(accel="
                << announced_proposal.actions.car2.acceleration_mps2
                << "m/s^2, steer="
                << announced_proposal.actions.car2
                       .steering_rate_deg_per_second
                << "deg/s, duration="
                << announced_proposal.actions.car2.duration_seconds << "s)]";
        logLine(message.str());
    }
}

void processTelemetry(Runtime& runtime, const std::string& payload) {
    Telemetry telemetry;
    if (!parseTelemetry(payload, telemetry)) {
        logLine("Discarded malformed TELEMETRY packet.");
        return;
    }

    if (telemetry.id != runtime.local_id &&
        !runtime.configured_peer_id.empty() &&
        telemetry.id != runtime.configured_peer_id) {
        return;
    }

    // Laptop packets model each vehicle's local sensor input. A Pi accepts the
    // injected sample only for its own ID; peer data must arrive as that peer's
    // rebroadcast (with MONITORING/PROPOSING/etc. state). This preserves the
    // actual Pi-to-Pi V2V path instead of letting the injector act as a server.
    if (telemetry.id != runtime.local_id &&
        externalLocalTelemetry(telemetry)) {
        return;
    }

    bool scenario_changed = false;
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        // Reject our own rebroadcast so it cannot keep stale injected sensor
        // data alive forever after the laptop simulator stops.
        if (telemetry.id == runtime.local_id &&
            !externalLocalTelemetry(telemetry)) {
            return;
        }
        if (telemetry.id == runtime.local_id &&
            !telemetry.scenario_id.empty() &&
            telemetry.scenario_id != runtime.scenario_id) {
            resetForScenario(runtime, telemetry.scenario_id);
            scenario_changed = true;
        } else if (telemetry.id != runtime.local_id &&
                   !runtime.scenario_id.empty() &&
                   telemetry.scenario_id != runtime.scenario_id) {
            // Do not combine a new local scenario with a delayed peer sample
            // or rebroadcast from the prior run.
            return;
        }
        runtime.cars[telemetry.id] = telemetry;
    }

    if (scenario_changed) {
        logLine("New simulator scenario " + telemetry.scenario_id +
                "; cleared prior proposal and execution state.");
    }

    Telemetry first;
    Telemetry second;
    bool choose_actions = true;
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        if (runtime.state == SafetyState::PROPOSING && runtime.has_pending) {
            return;
        }
        choose_actions = runtime.state != SafetyState::EXECUTE &&
                         runtime.state != SafetyState::EMERGENCY_STOP;
    }
    if (!copyFreshPair(runtime, first, second)) {
        return;
    }
    applyAssessment(runtime,
                    assessTelemetry(first, second, choose_actions));
}

ExecutionEvent transitionToExecution(Runtime& runtime,
                                     const VehicleControl& control,
                                     double ttc_seconds,
                                     SafetyState target,
                                     const std::string& reason) {
    ExecutionEvent event;
    runtime.state = target;
    runtime.execution_control = control;
    runtime.execution_ttc = ttc_seconds;
    runtime.execution_started = Clock::now();
    runtime.clear_timer_active = false;
    if (target == SafetyState::EXECUTE) {
        // Continue relaying the matching proposal briefly after local commit.
        // Otherwise a Pi that heard its peer first could stop proposing before
        // the peer ever received its proposal, causing asymmetric timeout.
        runtime.proposal_relay_until =
            runtime.execution_started + kPostMatchProposalRelay;
    } else {
        runtime.has_pending = false;
        runtime.proposal_transmitted = false;
    }
    event.occurred = true;
    event.control = control;
    event.ttc_seconds = ttc_seconds;
    event.reason = reason;
    return event;
}

ExecutionEvent processPeerProposal(Runtime& runtime,
                                   v2v::V2VNetwork& network,
                                   const std::string& payload) {
    std::string sender_id;
    JointActions peer_actions;
    if (!parseProposal(payload, sender_id, peer_actions)) {
        logLine("Discarded malformed PROPOSAL packet.");
        return ExecutionEvent();
    }

    std::string own_proposal_payload;
    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        if (sender_id == runtime.local_id ||
            runtime.state != SafetyState::PROPOSING ||
            !runtime.has_pending) {
            return ExecutionEvent();
        }

        if (Clock::now() >= runtime.proposal_deadline) {
            return transitionToExecution(runtime,
                                         VehicleControl(
                                             -kEmergencyStopDecelerationMps2,
                                             0.0,
                                             kEmergencyStopDurationSeconds),
                                         runtime.pending.ttc_seconds,
                                         SafetyState::EMERGENCY_STOP,
                                         consensusTimeoutReason(
                                             runtime.pending, false));
        }

        const std::string expected_peer =
            runtime.pending.car1_id == runtime.local_id
                ? runtime.pending.car2_id
                : runtime.pending.car1_id;
        if (sender_id != expected_peer ||
            !actionsMatch(runtime.pending.actions, peer_actions)) {
            return ExecutionEvent();
        }
        own_proposal_payload =
            formatProposalPayload(runtime.local_id, runtime.pending);
    }

    // Send our matching proposal immediately before committing. This avoids a
    // 20 Hz phase gap where one Pi could hear the peer and stop proposing before
    // it had transmitted its own plan. A failed send can never authorize action.
    if (!network.broadcast("PROPOSAL", own_proposal_payload)) {
        logLine("Could not transmit the local matching proposal; waiting for "
                "the consensus timeout.");
        return ExecutionEvent();
    }

    std::lock_guard<std::mutex> lock(runtime.mutex);
    if (runtime.state != SafetyState::PROPOSING ||
        !runtime.has_pending || Clock::now() >= runtime.proposal_deadline ||
        formatProposalPayload(runtime.local_id, runtime.pending) !=
            own_proposal_payload) {
        if (runtime.state == SafetyState::PROPOSING && runtime.has_pending &&
            Clock::now() >= runtime.proposal_deadline) {
                return transitionToExecution(runtime,
                                         VehicleControl(
                                             -kEmergencyStopDecelerationMps2,
                                             0.0,
                                             kEmergencyStopDurationSeconds),
                                         runtime.pending.ttc_seconds,
                                         SafetyState::EMERGENCY_STOP,
                                         consensusTimeoutReason(
                                             runtime.pending, false));
        }
        return ExecutionEvent();
    }

    runtime.proposal_transmitted = true;
    const VehicleControl own_control =
        runtime.pending.car1_id == runtime.local_id
            ? runtime.pending.actions.car1
            : runtime.pending.actions.car2;
    return transitionToExecution(runtime,
                                 own_control,
                                 runtime.pending.ttc_seconds,
                                 SafetyState::EXECUTE,
                                 "matching peer proposal");
}

ExecutionEvent checkConsensusDeadline(Runtime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.mutex);
    if (runtime.state != SafetyState::PROPOSING ||
        !runtime.has_pending || Clock::now() < runtime.proposal_deadline) {
        return ExecutionEvent();
    }
    const std::string timeout_reason = consensusTimeoutReason(
        runtime.pending, !runtime.proposal_transmitted);
    return transitionToExecution(runtime,
                                 VehicleControl(
                                     -kEmergencyStopDecelerationMps2,
                                     0.0,
                                     kEmergencyStopDurationSeconds),
                                 runtime.pending.ttc_seconds,
                                 SafetyState::EMERGENCY_STOP,
                                 timeout_reason);
}

void playWarning(const ExecutionEvent& event, bool enabled) {
    if (!event.occurred) {
        return;
    }

    std::ostringstream message;
    message << "CONTROL acceleration=" << event.control.acceleration_mps2
            << "m/s^2, steering_rate="
            << event.control.steering_rate_deg_per_second
            << "deg/s, duration=" << event.control.duration_seconds
            << "s (" << event.reason
            << ", TTC=" << std::fixed << std::setprecision(3)
            << event.ttc_seconds << "s)";
    logLine(message.str());

    if (!enabled) {
        return;
    }

    // Fixed commands avoid shell injection from network-provided action text.
    // Stock QNX 8.0 has no bundled audio framework, so deployments may provide
    // /home/qnx/bin/v2v_alert as a GPIO buzzer/LED helper. If an aplay port is
    // installed, the requested WAV warning remains the fallback.
    int alert_status = 0;
    if (std::fabs(event.control.steering_rate_deg_per_second) > 0.001) {
        alert_status = std::system(
            "if [ -x /home/qnx/bin/v2v_alert ]; then "
            "/home/qnx/bin/v2v_alert swerve >/dev/null 2>&1 & "
            "elif command -v aplay >/dev/null 2>&1; then "
            "aplay /home/qnx/assets/swerve_warning.wav >/dev/null 2>&1 & "
            "else exit 127; fi");
    } else if (event.control.acceleration_mps2 <=
               -kEmergencyStopDecelerationMps2) {
        alert_status = std::system(
            "if [ -x /home/qnx/bin/v2v_alert ]; then "
            "/home/qnx/bin/v2v_alert emergency_stop >/dev/null 2>&1 & "
            "elif command -v aplay >/dev/null 2>&1; then "
            "aplay /home/qnx/assets/emergency_warning.wav >/dev/null 2>&1 & "
            "else exit 127; fi");
    } else {
        alert_status = std::system(
            "if [ -x /home/qnx/bin/v2v_alert ]; then "
            "/home/qnx/bin/v2v_alert brake >/dev/null 2>&1 & "
            "elif command -v aplay >/dev/null 2>&1; then "
            "aplay /home/qnx/assets/brake_warning.wav >/dev/null 2>&1 & "
            "else exit 127; fi");
    }
    if (alert_status != 0) {
        logLine("Warning backend could not be launched; install v2v_alert or "
                "an aplay-compatible player, or use --no-audio.");
    }
}

void broadcastLoop(v2v::V2VNetwork& network, Runtime& runtime) {
    Clock::time_point next_send = Clock::now();
    while (gStopRequested == 0) {
        std::string telemetry_payload;
        std::string proposal_payload;
        std::string execution_payload;
        {
            const Clock::time_point now = Clock::now();
            std::lock_guard<std::mutex> lock(runtime.mutex);
            std::map<std::string, Telemetry>::const_iterator own =
                runtime.cars.find(runtime.local_id);
            if (own != runtime.cars.end() &&
                now - own->second.received_at <= kTelemetryFreshness) {
                telemetry_payload =
                    formatTelemetryPayload(own->second,
                                           scenarioSafetyState(runtime));
            }
            if (runtime.state == SafetyState::PROPOSING &&
                runtime.has_pending) {
                proposal_payload =
                    formatProposalPayload(runtime.local_id, runtime.pending);
            } else if (runtime.state == SafetyState::EXECUTE &&
                       runtime.has_pending &&
                       now < runtime.proposal_relay_until) {
                proposal_payload =
                    formatProposalPayload(runtime.local_id, runtime.pending);
            } else if (runtime.state == SafetyState::EXECUTE &&
                       runtime.has_pending &&
                       now >= runtime.proposal_relay_until) {
                runtime.has_pending = false;
            }
            if (runtime.state == SafetyState::EXECUTE ||
                runtime.state == SafetyState::EMERGENCY_STOP) {
                execution_payload = formatExecutionPayload(runtime);
            }
        }

        bool attempted_send = false;
        bool all_sends_succeeded = true;
        bool proposal_send_succeeded = false;
        if (!telemetry_payload.empty()) {
            attempted_send = true;
            all_sends_succeeded =
                network.broadcast("TELEMETRY", telemetry_payload) &&
                all_sends_succeeded;
        }
        if (!proposal_payload.empty()) {
            attempted_send = true;
            proposal_send_succeeded =
                network.broadcast("PROPOSAL", proposal_payload);
            all_sends_succeeded =
                proposal_send_succeeded && all_sends_succeeded;
        }
        if (!execution_payload.empty()) {
            attempted_send = true;
            all_sends_succeeded =
                network.broadcast("EXECUTION", execution_payload) &&
                all_sends_succeeded;
        }

        bool report_send_failure = false;
        {
            std::lock_guard<std::mutex> lock(runtime.mutex);
            if (proposal_send_succeeded && runtime.has_pending &&
                formatProposalPayload(runtime.local_id, runtime.pending) ==
                    proposal_payload) {
                runtime.proposal_transmitted = true;
            }
            if (attempted_send && !all_sends_succeeded &&
                !runtime.network_send_failure_reported) {
                runtime.network_send_failure_reported = true;
                report_send_failure = true;
            } else if (attempted_send && all_sends_succeeded) {
                runtime.network_send_failure_reported = false;
            }
        }
        if (report_send_failure) {
            logLine("UDP broadcast failed; verify the Wi-Fi interface and "
                    "broadcast route.");
        }

        next_send += kBroadcastPeriod;
        if (next_send < Clock::now()) {
            next_send = Clock::now();
        }
        std::this_thread::sleep_until(next_send);
    }
}

void listenAndProcessLoop(v2v::V2VNetwork& network, Runtime& runtime) {
    while (gStopRequested == 0) {
        std::string topic;
        std::string payload;
        ExecutionEvent event;
        if (network.listen(topic, payload)) {
            if (topic == "TELEMETRY") {
                processTelemetry(runtime, payload);
            } else if (topic == "PROPOSAL") {
                event = processPeerProposal(runtime, network, payload);
            } else if (topic == "RESET") {
                processScenarioReset(runtime, payload);
            }
        }
        if (!event.occurred) {
            event = checkConsensusDeadline(runtime);
        }
        playWarning(event, runtime.audio_enabled);
    }
}

void printUsage(const char* program) {
    std::cout
        << "Usage: " << program
        << " --id CAR1 [--peer CAR2] [--no-audio]\n\n"
        << "Each Pi must use a unique --id matching the laptop simulator.\n"
        << "Examples:\n"
        << "  " << program << " --id CAR1 --peer CAR2\n"
        << "  " << program << " --id CAR2 --peer CAR1\n";
}

bool parseArguments(int argc,
                    char* argv[],
                    std::string& local_id,
                    std::string& peer_id,
                    bool& audio_enabled) {
    audio_enabled = true;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if ((argument == "--id" || argument == "--peer") &&
            index + 1 < argc) {
            const std::string value(argv[++index]);
            if (argument == "--id") {
                local_id = value;
            } else {
                peer_id = value;
            }
        } else if (argument == "--no-audio") {
            audio_enabled = false;
        } else if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown or incomplete argument: " << argument << '\n';
            printUsage(argv[0]);
            return false;
        }
    }

    if (!validIdentifier(local_id) ||
        (!peer_id.empty() && !validIdentifier(peer_id)) ||
        (!peer_id.empty() && peer_id == local_id)) {
        std::cerr << "--id is required; IDs must be distinct and contain only "
                     "letters, digits, '_' or '-'.\n";
        printUsage(argv[0]);
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    std::string local_id;
    std::string peer_id;
    bool audio_enabled = true;
    if (!parseArguments(argc, argv, local_id, peer_id, audio_enabled)) {
        return 2;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::unique_ptr<v2v::V2VNetwork> network = v2v::makeUdpNetwork();
    if (!network->initialize()) {
        std::cerr << "Unable to initialize UDP broadcast/listen on port "
                  << v2v::kUdpPort << ". Check the interface and port.\n";
        return 1;
    }

    Runtime runtime(local_id, peer_id, audio_enabled);
    logLine("V2V brain " + local_id + " listening on UDP 12345.");
    logLine("Waiting for SIMULATED telemetry from the laptop injector...");

    std::thread broadcaster(broadcastLoop, std::ref(*network),
                            std::ref(runtime));
    std::thread listener(listenAndProcessLoop, std::ref(*network),
                         std::ref(runtime));

    while (gStopRequested == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    network->shutdown();
    broadcaster.join();
    listener.join();
    logLine("V2V brain stopped cleanly.");
    return 0;
}
