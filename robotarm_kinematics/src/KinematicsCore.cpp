#include "robotarm_kinematics/KinematicsCore.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/logging.hpp"

namespace
{
// Built on demand so there is no static-init ordering dependency on rclcpp.
rclcpp::Logger logger()
{
	return rclcpp::get_logger("robotarm_kinematics");
}
}

PLUGINLIB_EXPORT_CLASS(
  	robotarm_kinematics::KinematicsCore, 
  	kinematics_interface::KinematicsInterface
)

bool robotarm_kinematics::KinematicsCore::initialize(
  	const std::string &robot_description,
  	std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  	const std::string &param_namespace)
{
    initialised_ = false;
    joints_.clear();

    // Declared only if the host node has not already declared it (or auto-declared it from overrides).
    const std::string lambda_param = (param_namespace.empty() ? "" : param_namespace + ".") + "lambda";
    try {
        if (!parameters_interface->has_parameter(lambda_param)) {
            parameters_interface->declare_parameter(lambda_param, rclcpp::ParameterValue(0.01));
        }
        lambda_ = parameters_interface->get_parameter(lambda_param).as_double();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to read parameter '%s': %s", lambda_param.c_str(), e.what());
        return false;
    }
    if (!std::isfinite(lambda_) || lambda_ < 0.0) {
        RCLCPP_ERROR(logger(), "Parameter '%s' must be finite and >= 0", lambda_param.c_str());
        return false;
    }

    urdf::Model model;
    if (!model.initString(robot_description)) {
        RCLCPP_ERROR(logger(), "Failed to parse robot_description");
        return false;
    }

    // Walk root -> tip so joints_ ends up in kinematic order, not map/alphabetical order.
    urdf::LinkConstSharedPtr link_urdf = model.getRoot();
	if (!link_urdf) {
		RCLCPP_ERROR(logger(), "URDF has no root link");
		return false;
	}
	std::vector<urdf::JointConstSharedPtr> joints_urdf;

	while (!link_urdf->child_joints.empty()) {
		if (link_urdf->child_joints.size() != 1) {
			RCLCPP_ERROR(logger(), "Link %s has multiple children links", link_urdf->name.c_str());
			return false;
		}
		urdf::JointConstSharedPtr joint = link_urdf->child_joints[0];
        joints_urdf.push_back(joint);
        link_urdf = model.getLink(joint->child_link_name);
		if (!link_urdf) {
			RCLCPP_ERROR(logger(), "Joint %s references unknown child link %s",
				joint->name.c_str(), joint->child_link_name.c_str());
			return false;
		}
	}

	if (joints_urdf.size() < 2) {
		RCLCPP_ERROR(logger(), "URDF chain has %zu joint(s), expected more",
			joints_urdf.size());
		return false;
	}

	urdf::JointConstSharedPtr tcp_joint_urdf = joints_urdf.back();
	if (tcp_joint_urdf->type != urdf::Joint::FIXED) {
		RCLCPP_ERROR(logger(), "TCP-Joint %s has unexpected type", tcp_joint_urdf->name.c_str());
		return false;
	}
	tcp_.tcp_name_ = tcp_joint_urdf->child_link_name;
	tcp_.parent_link_name_ = tcp_joint_urdf->parent_link_name;
	tcp_.tcp_origin_in_parent_ = joint_origin_to_isometry(tcp_joint_urdf);

    for (size_t i = 0; i < joints_urdf.size()-1; ++i) {
		urdf::JointConstSharedPtr joint_urdf = joints_urdf[i];
  
		Joint joint;
		if (joint_urdf->type != urdf::Joint::REVOLUTE) {
			RCLCPP_ERROR(logger(), "Joint %s has unexpected type", joint_urdf->name.c_str());
			return false;
		}
		
		joint.joint_name_ = joint_urdf->name;
		joint.parent_link_name_ = joint_urdf->parent_link_name;
		joint.child_link_name_ = joint_urdf->child_link_name;

		if (!joint_urdf->limits) {
			RCLCPP_ERROR(logger(), "Joint %s has no limits", joint_urdf->name.c_str());
			return false;
		}
		const auto & l = *joint_urdf->limits;

		if (!(l.lower < l.upper)) {
			RCLCPP_ERROR(logger(), "Joint %s has invalid position limits", joint_urdf->name.c_str());
			return false;
		}
		if (l.velocity <= 0.0 || l.effort <= 0.0) {
			RCLCPP_ERROR(logger(), "Joint %s has invalid velocity/effort limits", joint_urdf->name.c_str());
			return false;
		}

		joint.limits_.min = l.lower;
		joint.limits_.max = l.upper;
		joint.limits_.velocity = l.velocity;
		joint.limits_.effort = l.effort;

		// extract isometry
		joint.joint_origin_in_parent_ = joint_origin_to_isometry(joint_urdf);

		// axis
		joint.joint_axis_in_child_ = Eigen::Vector3d(joint_urdf->axis.x,
													 joint_urdf->axis.y,
													 joint_urdf->axis.z);
		if (joint.joint_axis_in_child_.isZero()) {
			RCLCPP_ERROR(logger(), "Joint %s invalid axis (all zero)", joint_urdf->name.c_str());
			return false;
		}
		joint.joint_axis_in_child_.normalize();

		joints_.push_back(joint);
    }

	// init heap member
	j_cj_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_cji_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_jd2cd_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_inv_cd2jd_ =  Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(joints_.size(), 6);
	M_ = Eigen::MatrixXd::Zero(joints_.size(), joints_.size());
	ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(joints_.size());

    print_joints();
    initialised_ = true;
    return true;
}

bool robotarm_kinematics::KinematicsCore::convert_cartesian_deltas_to_joint_deltas(
	const Eigen::VectorXd &joint_pos,
  	const Eigen::Matrix<double, 6, 1> &delta_x,
  	const std::string &link_name,
  	Eigen::VectorXd &delta_theta)
{
	if (!check_joint_pos(joint_pos)) return false;

	if (!delta_x.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "delta_x contains NaN or inf");
		return false;
	}

	// delta_theta = J⁺ delta_x
	if (!calculate_jacobian_inverse(joint_pos, link_name, j_inv_cd2jd_)) return false;
	delta_theta.noalias() = j_inv_cd2jd_ * delta_x;
    return true;
}

bool robotarm_kinematics::KinematicsCore::convert_joint_deltas_to_cartesian_deltas(
  	const Eigen::VectorXd &joint_pos,
  	const Eigen::VectorXd &delta_theta,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, 1> &delta_x)
{
	// checks initialised_ first, which the size check of delta_theta below relies on
	if (!check_joint_pos(joint_pos)) return false;

	if (delta_theta.size() != static_cast<Eigen::Index>(joints_.size())) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Unexpected delta_theta dimension");
		return false;
	}
	if (!delta_theta.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "delta_theta contains NaN or inf");
		return false;
	}

	// delta_x = J delta_theta
	if (!calculate_jacobian(joint_pos, link_name, j_jd2cd_)) return false;
	delta_x.noalias() = j_jd2cd_ * delta_theta;
    return true;
}

bool robotarm_kinematics::KinematicsCore::check_joint_pos(const Eigen::VectorXd &joint_pos)
{
	if (!initialised_) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Kinematics not initialised");
		return false;
	}

	if (joint_pos.size() != static_cast<Eigen::Index>(joints_.size())) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Unexpected joint_pos dimension");
		return false;
	}

	if (!joint_pos.allFinite()) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "joint_pos contains NaN or inf");
		return false;
	}

	return true;
}

bool robotarm_kinematics::KinematicsCore::calculate_link_transform(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Isometry3d &transform)
{
	if (!check_joint_pos(joint_pos)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		transform = Eigen::Isometry3d::Identity();
		return true;
	}

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for (size_t i = 0; i < joints_.size(); ++i) {
		T = T * joints_[i].transform(joint_pos[i]);
		if (joints_[i].child_link_name_ == link_name) {
			transform = T;
			return true;
		}
	}

	if (link_name == tcp_.tcp_name_) {  // TCP is not in joints_, so it comes after the loop
		transform = T * tcp_.tcp_origin_in_parent_;
		return true;
	}

	RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms,
		"Link %s not found in kinematic chain", link_name.c_str());
	return false;
}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
	if (!check_joint_pos(joint_pos)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		jacobian.setZero(6, joints_.size());
		return true;
	}

	j_cj_.setZero();
	Eigen::Isometry3d T_end;
	if (!calculate_link_transform(joint_pos, link_name, T_end)) return false;
	Eigen::Vector3d p_end = T_end.translation();

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for (size_t i = 0; i < joints_.size(); ++i) {

		T = T * joints_[i].transform(joint_pos[i]);
		Eigen::Vector3d joint_axis = T.linear() * joints_[i].joint_axis_in_child_;
		Eigen::Vector3d p_i = T.translation();

		j_cj_.block<3,1>(0, i) = joint_axis.cross(p_end - p_i);
    	j_cj_.block<3,1>(3, i) = joint_axis;

		if (joints_[i].child_link_name_ == link_name) {
			break;
		}
	}

	jacobian = j_cj_;
    return true;
}

bool robotarm_kinematics::KinematicsCore::calculate_jacobian_inverse(
  	const Eigen::VectorXd &joint_pos,
  	const std::string &link_name,
  	Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse)
{
	// joint_pos is validated by calculate_jacobian(), which is the first thing called here
	if (!calculate_jacobian(joint_pos, link_name, j_cji_)) return false;

	// calculate J⁻¹ as pseudo-inverse with damped least squares: f(q̇) = ‖J q̇ − ẋ‖² + λ²‖q̇‖²
	// J⁺ = (JᵀJ + λ²I)⁻¹ Jᵀ -> (JᵀJ + λ²I)J⁺ = Jᵀ as MX = Jᵀ with:
	// M = (JᵀJ + λ²I)
	// X = J⁺
	M_.noalias() = j_cji_.transpose() * j_cji_;
	M_.diagonal().array() += lambda_*lambda_;

	// factor M once, and check that it worked
	ldlt_.compute(M_);
	if (ldlt_.info() != Eigen::Success) {
		RCLCPP_ERROR_THROTTLE(logger(), clock_, log_throttle_ms, "Factorization failed");
		return false;
	}
	jacobian_inverse = ldlt_.solve(j_cji_.transpose());

    return true;
}

Eigen::Isometry3d robotarm_kinematics::KinematicsCore::joint_origin_to_isometry(urdf::JointConstSharedPtr joint)
{
    const urdf::Pose & origin = joint->parent_to_joint_origin_transform;
	double x, y, z;
	x = origin.position.x;
	y = origin.position.y;
	z = origin.position.z;
	double pitch, roll, yaw;
	origin.rotation.getRPY(roll, pitch, yaw);
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity()
						* Eigen::Translation3d(x, y, z)
						* Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) 
						* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) 
						* Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
    return T;
}

bool robotarm_kinematics::KinematicsCore::get_joint_names(std::vector<std::string>& names)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	names.clear();
	names.reserve(joints_.size());
	for (const auto & j : joints_) {
		names.push_back(j.joint_name_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_joint_limits(std::vector<Limits>& limits)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	limits.clear();
	limits.reserve(joints_.size());
	for (const auto & j : joints_) {
		limits.push_back(j.limits_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_link_names(std::vector<std::string> &names)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	names.clear();
	names.reserve(joints_.size()+1);
	names.push_back(joints_.front().parent_link_name_);
	for (const auto & j : joints_) {
		names.push_back(j.child_link_name_);
	}
    return true;
}

bool robotarm_kinematics::KinematicsCore::get_tcp_link_name(std::string& name)
{
    if (!initialised_) {
		RCLCPP_ERROR(logger(), "Not initialised");
		return false;
	}
	name = tcp_.tcp_name_;
    return true;
}

void robotarm_kinematics::KinematicsCore::print_joints() const
{
    size_t w_joint = std::string("(fixed)").size();
    size_t w_parent = std::max(std::string("parent link").size(), tcp_.parent_link_name_.size());
    for (const auto & j : joints_) {
        w_joint = std::max(w_joint, j.joint_name_.size());
        w_parent = std::max(w_parent, j.parent_link_name_.size());
    }

    auto row = [&](std::ostringstream & os, const std::string & idx, const std::string & joint,
                   const std::string & parent, const std::string & child) {
        os << std::left
           << std::setw(3) << idx << "  "
           << std::setw(static_cast<int>(w_joint)) << joint << "  "
           << std::setw(static_cast<int>(w_parent)) << parent << "  "
           << child;
    };

    std::ostringstream os;
    os << "\nKinematic chain: " << joints_.size() << " joint(s) + tcp\n";

    std::ostringstream header;
    row(header, "#", "joint", "parent link", "child link");
    os << header.str() << "\n" << std::string(header.str().size(), '-') << "\n";

    for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        row(os, std::to_string(i), j.joint_name_, j.parent_link_name_, j.child_link_name_);
        os << "\n";
    }
    row(os, "tcp", "(fixed)", tcp_.parent_link_name_, tcp_.tcp_name_);
    os << "\n";

    RCLCPP_INFO(logger(), "%s", os.str().c_str());
}


