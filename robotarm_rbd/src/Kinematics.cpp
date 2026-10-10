#include "robotarm_rbd/Kinematics.hpp"

#include "rclcpp/logging.hpp"

namespace
{
// Built on demand so there is no static-init ordering dependency on rclcpp.
rclcpp::Logger logger()
{
	return rclcpp::get_logger("robotarm_rbd");
}
}

PLUGINLIB_EXPORT_CLASS(
  	robotarm_rbd::Kinematics, 
  	kinematics_interface::KinematicsInterface
)

bool robotarm_rbd::Kinematics::initialize(
  	const std::string &robot_description,
  	std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  	const std::string &param_namespace)
{
	RobotarmRbd::Config rbd_cfg;
    // Declared only if the host node has not already declared it (or auto-declared it from overrides).
    const std::string prefix = param_namespace.empty() ? "" : param_namespace + ".";
    const std::string lambda_param = prefix + "lambda";
    const std::string jinv_method_param = prefix + "jinv_method";
    try {
        if (!parameters_interface->has_parameter(lambda_param)) {
            parameters_interface->declare_parameter(lambda_param, rclcpp::ParameterValue(0.01));
        }
        rbd_cfg.lambda = parameters_interface->get_parameter(lambda_param).as_double();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to read parameter '%s': %s", lambda_param.c_str(), e.what());
        return false;
    }

    // "svd": selective, adaptive damping (accurate), "ldlt": constant damping (fast)
    std::string jinv_method;
    try {
        if (!parameters_interface->has_parameter(jinv_method_param)) {
            parameters_interface->declare_parameter(jinv_method_param, rclcpp::ParameterValue(std::string("svd")));
        }
        jinv_method = parameters_interface->get_parameter(jinv_method_param).as_string();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to read parameter '%s': %s", jinv_method_param.c_str(), e.what());
        return false;
    }
    if (jinv_method == "svd") {
        rbd_cfg.jinv_method = RobotarmRbd::JinvMethod::SVD;
    } else if (jinv_method == "ldlt") {
        rbd_cfg.jinv_method = RobotarmRbd::JinvMethod::LDLT;
    } else {
        RCLCPP_ERROR(logger(), "Invalid parameter '%s': '%s', expected 'svd' or 'ldlt'",
            jinv_method_param.c_str(), jinv_method.c_str());
        return false;
    }

	if (!rbd_.initialize(robot_description, rbd_cfg)) {
		RCLCPP_ERROR(logger(), "%s", rbd_.last_error());
		return false;
	}

    print_joints();
    return true;
}

bool robotarm_rbd::Kinematics::convert_cartesian_deltas_to_joint_deltas(
	const Eigen::VectorXd &joint_pos,
  	const Eigen::Matrix<double, 6, 1> &delta_x,
  	const std::string &link_name,
  	Eigen::VectorXd &delta_theta)
{
	if (!rbd_.convert_cartesian_deltas_to_joint_deltas(joint_pos, delta_x, link_name, delta_theta)) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "%s", rbd_.last_error());
		return false;
	}
	return true;
}

bool robotarm_rbd::Kinematics::convert_joint_deltas_to_cartesian_deltas(
  	const Eigen::VectorXd &joint_pos,
  	const Eigen::VectorXd &delta_theta,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, 1> &delta_x)
{
	if (!rbd_.convert_joint_deltas_to_cartesian_deltas(joint_pos, delta_theta, link_name, delta_x)) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "%s", rbd_.last_error());
		return false;
	}
	return true;
}

bool robotarm_rbd::Kinematics::calculate_link_transform(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Isometry3d &transform)
{
	if (!rbd_.calculate_link_transform(joint_pos, link_name, transform)) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::calculate_jacobian(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
	if (!rbd_.calculate_jacobian(joint_pos, link_name, jacobian)) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::calculate_jacobian_inverse(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse)
{
	if (!rbd_.calculate_jacobian_inverse(joint_pos, link_name, jacobian_inverse)) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::get_joint_names(std::vector<std::string>& names)
{
    if (!rbd_.get_joint_names(names)) {
		RCLCPP_ERROR(logger(), "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::get_joint_limits(std::vector<Limits>& limits)
{
    if (!rbd_.get_joint_limits(limits)) {
		RCLCPP_ERROR(logger(), "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::get_link_names(std::vector<std::string> &names)
{
    if (!rbd_.get_link_names(names)) {
		RCLCPP_ERROR(logger(), "%s", rbd_.last_error());
		return false;
	}

	return true;
}

bool robotarm_rbd::Kinematics::get_tcp_link_name(std::string& name)
{
    if (!rbd_.get_tcp_link_name(name)) {
		RCLCPP_ERROR(logger(), "%s", rbd_.last_error());
		return false;
	}

	return true;
}

void robotarm_rbd::Kinematics::print_joints() const
{
    RCLCPP_INFO(logger(), "%s", rbd_.chain_table_log().c_str());
}


