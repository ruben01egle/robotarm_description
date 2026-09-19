// Manual check for KinematicsCore: feeds the robot_description parameter into
// initialize(), which prints the parsed joints / DH params on success.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robotarm_kinematics/KinematicsCore.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("kinematics_core_test");

    const std::string urdf = node->declare_parameter<std::string>("robot_description", "");
    if (urdf.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'robot_description' is empty or missing");
        rclcpp::shutdown();
        return 1;
    }

    robotarm_kinematics::KinematicsCore kinematics;
    const bool ok = kinematics.initialize(urdf, node->get_node_parameters_interface(), "");

    if (ok) {
        RCLCPP_INFO(node->get_logger(), "KinematicsCore::initialize succeeded");
    } else {
        RCLCPP_ERROR(node->get_logger(), "KinematicsCore::initialize failed");
    }

    rclcpp::shutdown();
    return ok ? 0 : 1;
}
