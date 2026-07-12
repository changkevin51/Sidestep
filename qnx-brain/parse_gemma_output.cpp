#include "joint_actions.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

void printControl(const std::string& name, const VehicleControl& control) {
    std::cout << name << ".acceleration_mps2="
              << control.acceleration_mps2 << '\n'
              << name << ".steering_rate_deg_per_second="
              << control.steering_rate_deg_per_second << '\n'
              << name << ".duration_seconds="
              << control.duration_seconds << '\n';
}

}  // namespace

int main() {
    std::ostringstream input;
    input << std::cin.rdbuf();

    JointActions result;
    std::string error;
    if (!parseGemmaJointActions(input.str(), result, &error)) {
        std::cerr << "Could not parse Gemma output: " << error << std::endl;
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    printControl("car1", result.car1);
    printControl("car2", result.car2);
    std::cout << "decision_path=" << result.decision_path << std::endl;
    return 0;
}
