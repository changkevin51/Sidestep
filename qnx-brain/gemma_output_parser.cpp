#include "joint_actions.hpp"

#include <cmath>
#include <cstdio>

namespace {

bool onlyTrailingWhitespace(const std::string& text, std::size_t start) {
    return text.find_first_not_of(" \t\r\n", start) == std::string::npos;
}

void setError(std::string* destination, const std::string& message) {
    if (destination != 0) {
        *destination = message;
    }
}

bool finiteControl(const VehicleControl& control) {
    return std::isfinite(control.acceleration_mps2) &&
           std::isfinite(control.steering_rate_deg_per_second) &&
           std::isfinite(control.duration_seconds);
}

}  // namespace

bool parseGemmaJointActions(const std::string& raw_output,
                            JointActions& result,
                            std::string* error_message) {
    if (raw_output.empty() || raw_output.size() > 2048U) {
        setError(error_message, "Gemma output is empty or too large");
        return false;
    }

    JointActions parsed;
    int consumed = 0;
    const int field_count = std::sscanf(
        raw_output.c_str(),
        " Car A: acceleration %lf m/s^2, steering rate %lf degrees/s, "
        "steering duration %lf seconds "
        "Car B: acceleration %lf m/s^2, steering rate %lf degrees/s, "
        "steering duration %lf seconds %n",
        &parsed.car1.acceleration_mps2,
        &parsed.car1.steering_rate_deg_per_second,
        &parsed.car1.duration_seconds,
        &parsed.car2.acceleration_mps2,
        &parsed.car2.steering_rate_deg_per_second,
        &parsed.car2.duration_seconds,
        &consumed);

    if (field_count != 6 || consumed <= 0 ||
        !onlyTrailingWhitespace(raw_output, static_cast<std::size_t>(consumed))) {
        setError(error_message,
                 "Gemma output does not match the required Car A/Car B format");
        return false;
    }
    if (!finiteControl(parsed.car1) || !finiteControl(parsed.car2)) {
        setError(error_message, "Gemma output contains a non-finite number");
        return false;
    }
    if (parsed.car1.duration_seconds < 0.0 ||
        parsed.car2.duration_seconds < 0.0) {
        setError(error_message, "Control durations cannot be negative");
        return false;
    }

    parsed.decision_path = "GEMMA_3";
    result = parsed;
    if (error_message != 0) {
        error_message->clear();
    }
    return true;
}
