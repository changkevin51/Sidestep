#ifndef V2V_JOINT_ACTIONS_HPP
#define V2V_JOINT_ACTIONS_HPP

#include <string>

struct VehicleControl {
    double acceleration_mps2;
    double steering_rate_deg_per_second;
    double duration_seconds;

    VehicleControl()
        : acceleration_mps2(0.0),
          steering_rate_deg_per_second(0.0),
          duration_seconds(0.0) {}

    VehicleControl(double acceleration,
                   double steering_rate,
                   double duration)
        : acceleration_mps2(acceleration),
          steering_rate_deg_per_second(steering_rate),
          duration_seconds(duration) {}
};

struct JointActions {
    VehicleControl car1;
    VehicleControl car2;
    std::string decision_path;
};

// Parses the strict two-line text emitted by the Gemma collision program:
// Car A: acceleration N m/s^2, steering rate N degrees/s, steering duration N seconds
// Car B: acceleration N m/s^2, steering rate N degrees/s, steering duration N seconds
bool parseGemmaJointActions(const std::string& raw_output,
                            JointActions& result,
                            std::string* error_message = 0);

#endif  // V2V_JOINT_ACTIONS_HPP
