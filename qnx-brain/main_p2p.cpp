#include "car_collision.hpp"
#include "v2v_network.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
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
const std::chrono::milliseconds kBroadcastPeriod(50);       // 20 Hz
const std::chrono::milliseconds kConsensusTimeout(150);
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

struct JointActions {
    std::string car1_action;
    std::string car2_action;
    std::string decision_path;
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
    std::string action;
    double ttc_seconds;
    std::string reason;

    ExecutionEvent() : occurred(false), ttc_seconds(0.0) {}
};

struct Instruction {
    std::uint64_t seq;
    std::string target_id;
    std::string action;
    std::string reason;
    std::uint32_t ttl_ms;
    Clock::time_point received_at;

    Instruction()
        : seq(0), ttl_ms(0), received_at(Clock::now()) {}
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
    std::string execution_action;
    double execution_ttc;
    Clock::time_point execution_started;
    bool clear_timer_active;
    Clock::time_point clear_since;
    bool network_send_failure_reported;
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
    if (!splitCsvExact(payload, 3U, fields) ||
        !validIdentifier(fields[0])) {
        return false;
    }
    const std::string allowed_actions[] = {
        "BRAKE", "SWERVE_RIGHT", "SWERVE_LEFT"
    };
    bool first_allowed = false;
    bool second_allowed = false;
    for (std::size_t i = 0U; i < 3U; ++i) {
        first_allowed = first_allowed || fields[1] == allowed_actions[i];
        second_allowed = second_allowed || fields[2] == allowed_actions[i];
    }
    if (!first_allowed || !second_allowed) {
        return false;
    }
    sender_id = fields[0];
    actions.car1_action = fields[1];
    actions.car2_action = fields[2];
    actions.decision_path.clear();
    return true;
}

bool parseInstruction(const std::string& payload, Instruction& instruction) {
    std::vector<std::string> fields;
    if (!splitCsvExact(payload, 5U, fields) || !validIdentifier(fields[1]) ||
        !validIdentifier(fields[2])) {
        return false;
    }

    errno = 0;
    char* end = 0;
    const char* begin = fields[0].c_str();
    const unsigned long long parsed_seq = std::strtoull(begin, &end, 10);
    if (begin == end || end == 0 ||
        end != begin + static_cast<std::ptrdiff_t>(fields[0].size()) ||
        errno == ERANGE) {
        return false;
    }

    errno = 0;
    begin = fields[3].c_str();
    const unsigned long parsed_ttl = std::strtoul(begin, &end, 10);
    if (begin == end || end == 0 ||
        end != begin + static_cast<std::ptrdiff_t>(fields[3].size()) ||
        errno == ERANGE || parsed_ttl == 0UL || parsed_ttl > 60000UL) {
        return false;
    }

    instruction.seq = static_cast<std::uint64_t>(parsed_seq);
    instruction.target_id = fields[1];
    instruction.action = fields[2];
    instruction.ttl_ms = static_cast<std::uint32_t>(parsed_ttl);
    instruction.reason = fields[4];
    instruction.received_at = Clock::now();
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
                runtime.pending.actions.decision_path == "SMART_AI_STUB") {
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
    return sender_id + "," + proposal.actions.car1_action + "," +
           proposal.actions.car2_action;
}

std::string formatExecutionPayload(const Runtime& runtime) {
    return runtime.local_id + "," + runtime.execution_action + "," +
           safetyStateText(runtime) + "," +
           formatDouble(runtime.execution_ttc, 3);
}

std::string formatInstructionPayload(const Instruction& instruction) {
    std::ostringstream output;
    output << instruction.seq << ','
           << instruction.target_id << ','
           << instruction.action << ','
           << instruction.ttl_ms << ','
           << instruction.reason;
    return output.str();
}

bool actionsMatch(const JointActions& first, const JointActions& second) {
    return first.car1_action == second.car1_action &&
           first.car2_action == second.car2_action;
}

void applyInstruction(Runtime& runtime, const Instruction& instruction) {
    if (instruction.target_id != runtime.local_id) {
        return;
    }

    std::lock_guard<std::mutex> lock(runtime.mutex);
    if (instruction.seq <= runtime.last_instruction_seq) {
        return;
    }

    runtime.last_instruction_seq = instruction.seq;
    runtime.latest_instruction = instruction;
    runtime.has_instruction = true;

    std::ostringstream message;
    message << "Instruction received for " << instruction.target_id
            << ": seq=" << instruction.seq
            << ", action=" << instruction.action
            << ", ttl_ms=" << instruction.ttl_ms;
    if (!instruction.reason.empty()) {
        message << ", reason=" << instruction.reason;
    }
    logLine(message.str());
}

bool proposalsMatch(const PendingProposal& first,
                    const PendingProposal& second) {
    return first.car1_id == second.car1_id &&
           first.car2_id == second.car2_id &&
           actionsMatch(first.actions, second.actions);
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
        actions.car1_action = "BRAKE";
        actions.car2_action = "BRAKE";
    } else {
        // The canonical lower-ID car always moves right and the higher-ID car
        // left. Both Pis therefore derive the same opposite-direction plan.
        actions.car1_action = "SWERVE_RIGHT";
        actions.car2_action = "SWERVE_LEFT";
    }
    return actions;
}

// -------------------------------------------------------------------------
// SMART (AI stub) PATHWAY -- used only when TTC >= 3.0 seconds.
// -------------------------------------------------------------------------
JointActions evaluateSmartDecision(Car first, Car second) {
    // Future onboard-model integration point:
    //   1. Serialize both validated Car objects into model features.
    //   2. Invoke the local QNX-hosted model with a strict bounded deadline.
    //   3. Validate that it returned one action per canonical vehicle.
    //   4. Fall back to deterministic braking on timeout/invalid output.
    // No cloud service belongs in this safety-critical execution path.
    (void)first;
    (void)second;

    JointActions mock_result;
    mock_result.car1_action = "BRAKE";
    mock_result.car2_action = "BRAKE";
    mock_result.decision_path = "SMART_AI_STUB";
    return mock_result;
}

Assessment assessTelemetry(const Telemetry& first_telemetry,
                           const Telemetry& second_telemetry) {
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

        // -----------------------------------------------------------------
        // DEFAULT (if-statement) PATHWAY -- immediate deterministic response.
        // -----------------------------------------------------------------
        if (collision.ttc_seconds < kSmartDecisionThresholdSeconds) {
            assessment.proposal.actions =
                evaluateDefaultDecision(first, second, collision.ttc_seconds);
        } else {
            assessment.proposal.actions =
                evaluateSmartDecision(first, second);
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
           telemetry.source_state == "INJECTED";
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
                runtime.execution_action.clear();
                runtime.execution_ttc = 0.0;
                runtime.clear_timer_active = false;
                logLine("Collision corridor clear; returning to MONITORING.");
            }
            return;
        }

        // Once this collision event has a proposal, freeze it and its original
        // 150 ms deadline. A transient mixed telemetry sample must not cancel or
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
            runtime.proposal_deadline = now + kConsensusTimeout;
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
                << ", proposal=[" << announced_proposal.actions.car1_action
                << ", " << announced_proposal.actions.car2_action << ']';
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

    {
        std::lock_guard<std::mutex> lock(runtime.mutex);
        // Reject our own rebroadcast so it cannot keep stale injected sensor
        // data alive forever after the laptop simulator stops.
        if (telemetry.id == runtime.local_id &&
            !externalLocalTelemetry(telemetry)) {
            return;
        }
        runtime.cars[telemetry.id] = telemetry;
    }

    Telemetry first;
    Telemetry second;
    if (!copyFreshPair(runtime, first, second)) {
        return;
    }
    applyAssessment(runtime, assessTelemetry(first, second));
}

ExecutionEvent transitionToExecution(Runtime& runtime,
                                     const std::string& action,
                                     double ttc_seconds,
                                     SafetyState target,
                                     const std::string& reason) {
    ExecutionEvent event;
    runtime.state = target;
    runtime.execution_action = action;
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
    event.action = action;
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
                                         "EMERGENCY_STOP",
                                         runtime.pending.ttc_seconds,
                                         SafetyState::EMERGENCY_STOP,
                                         "150 ms consensus timeout");
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
                                         "EMERGENCY_STOP",
                                         runtime.pending.ttc_seconds,
                                         SafetyState::EMERGENCY_STOP,
                                         "150 ms consensus timeout");
        }
        return ExecutionEvent();
    }

    runtime.proposal_transmitted = true;
    const std::string own_action =
        runtime.pending.car1_id == runtime.local_id
            ? runtime.pending.actions.car1_action
            : runtime.pending.actions.car2_action;
    return transitionToExecution(runtime,
                                 own_action,
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
    const std::string timeout_reason =
        runtime.proposal_transmitted
            ? "150 ms consensus timeout"
            : "150 ms consensus timeout (local proposal send failed)";
    return transitionToExecution(runtime,
                                 "EMERGENCY_STOP",
                                 runtime.pending.ttc_seconds,
                                 SafetyState::EMERGENCY_STOP,
                                 timeout_reason);
}

void playWarning(const ExecutionEvent& event, bool enabled) {
    if (!event.occurred) {
        return;
    }

    std::ostringstream message;
    message << "ACTION " << event.action << " (" << event.reason
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
    if (event.action == "SWERVE_LEFT" || event.action == "SWERVE_RIGHT") {
        alert_status = std::system(
            "if [ -x /home/qnx/bin/v2v_alert ]; then "
            "/home/qnx/bin/v2v_alert swerve >/dev/null 2>&1 & "
            "elif command -v aplay >/dev/null 2>&1; then "
            "aplay /home/qnx/assets/swerve_warning.wav >/dev/null 2>&1 & "
            "else exit 127; fi");
    } else if (event.action == "EMERGENCY_STOP") {
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
                                           safetyStateText(runtime));
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
            } else if (topic == "INSTRUCTION") {
                Instruction instruction;
                if (parseInstruction(payload, instruction)) {
                    applyInstruction(runtime, instruction);
                } else {
                    logLine("Discarded malformed INSTRUCTION packet.");
                }
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
