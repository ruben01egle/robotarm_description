#include "robotarm_rbd/RobotarmRbd.hpp"

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <iomanip>
#include <sstream>



bool robotarm_rbd::RobotarmRbd::initialize(const std::string &robot_description, const Config &config)
{
    initialised_ = false;
    joints_.clear();

    if (!std::isfinite(config.lambda) || config.lambda < 0.0) {
        return fail("lambda must be finite and >= 0, got %g", config.lambda);
    }
    config_ = config;

    urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(robot_description);
    if (!model) {
        return fail("Failed to parse robot_description");
    }

    // Walk root -> tip so joints_ ends up in kinematic order, not map/alphabetical order.
    urdf::LinkConstSharedPtr link_urdf = model->getRoot();
	if (!link_urdf) {
		return fail("URDF has no root link");
	}
	std::vector<urdf::JointConstSharedPtr> joints_urdf;

	while (!link_urdf->child_joints.empty()) {
		if (link_urdf->child_joints.size() != 1) {
			return fail("Link %s has multiple children links", link_urdf->name.c_str());
		}
		urdf::JointConstSharedPtr joint = link_urdf->child_joints[0];
        joints_urdf.push_back(joint);
        link_urdf = model->getLink(joint->child_link_name);
		if (!link_urdf) {
			return fail("Joint %s references unknown child link %s",
				joint->name.c_str(), joint->child_link_name.c_str());
		}
	}

	if (joints_urdf.size() < 2) {
		return fail("URDF chain has %zu joint(s), expected more", joints_urdf.size());
	}

	urdf::JointConstSharedPtr tcp_joint_urdf = joints_urdf.back();
	if (tcp_joint_urdf->type != urdf::Joint::FIXED) {
		return fail("TCP-Joint %s has unexpected type", tcp_joint_urdf->name.c_str());
	}
	tcp_.tcp_name_ = tcp_joint_urdf->child_link_name;
	tcp_.parent_link_name_ = tcp_joint_urdf->parent_link_name;
	tcp_.tcp_origin_in_parent_ = joint_origin_to_isometry(tcp_joint_urdf);

    for (size_t i = 0; i < joints_urdf.size()-1; ++i) {
		urdf::JointConstSharedPtr joint_urdf = joints_urdf[i];

		Joint joint;
		if (joint_urdf->type != urdf::Joint::REVOLUTE) {
			return fail("Joint %s has unexpected type", joint_urdf->name.c_str());
		}

		joint.joint_name_ = joint_urdf->name;
		joint.parent_link_name_ = joint_urdf->parent_link_name;
		joint.child_link_name_ = joint_urdf->child_link_name;

		if (!joint_urdf->limits) {
			return fail("Joint %s has no limits", joint_urdf->name.c_str());
		}
		const auto & l = *joint_urdf->limits;

		if (!(l.lower < l.upper)) {
			return fail("Joint %s has invalid position limits", joint_urdf->name.c_str());
		}
		if (l.velocity <= 0.0 || l.effort <= 0.0) {
			return fail("Joint %s has invalid velocity/effort limits", joint_urdf->name.c_str());
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
			return fail("Joint %s invalid axis (all zero)", joint_urdf->name.c_str());
		}
		joint.joint_axis_in_child_.normalize();

		// inertia of the moved body, only if the child link has an <inertial>
		const urdf::LinkConstSharedPtr child_urdf = model->getLink(joint_urdf->child_link_name);
		if (child_urdf->inertial) {
			const urdf::Inertial & in = *child_urdf->inertial;
			const urdf::Rotation & r = in.origin.rotation;
			joint.inertia_.mass = in.mass;
			// vector to com in joint-frame
			joint.inertia_.com = Eigen::Vector3d(in.origin.position.x, in.origin.position.y,
												  in.origin.position.z);
			Eigen::Matrix3d R_com = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
			// inertia tensor of com in com-frame
			Eigen::Matrix3d I_com;
			I_com << in.ixx, in.ixy, in.ixz,
					 in.ixy, in.iyy, in.iyz,
					 in.ixz, in.iyz, in.izz;
			// I_joint = R * I_com * R^T: rotates tensor from com axes into joint axes (L = R I_com R^T ω)
			// reference point stays at com
			joint.inertia_.com_inertia_in_joint = R_com * I_com * R_com.transpose();
			joint.inertia_.valid = true;
		}

		joints_.push_back(joint);
    }

	// init heap member
	j_cj_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_cji_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_jd2cd_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_inv_cd2jd_ =  Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(joints_.size(), 6);
	M_ = Eigen::MatrixXd::Zero(joints_.size(), joints_.size());
	ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(joints_.size());
	data_.resize(joints_.size());

    initialised_ = true;
    return true;
}

bool robotarm_rbd::RobotarmRbd::convert_cartesian_deltas_to_joint_deltas(
	const Eigen::VectorXd &q,
	const Eigen::Matrix<double, 6, 1> &delta_x,
	const std::string &link_name,
	Eigen::VectorXd &delta_q)
{
    if (!check_input(q)) return false;

	if (!delta_x.allFinite()) {
		return fail("delta_x contains NaN or inf");
	}

	// delta_q = J⁺ delta_x
	if (!calculate_jacobian_inverse(q, link_name, j_inv_cd2jd_)) return false;
	delta_q.noalias() = j_inv_cd2jd_ * delta_x;
    return true;
}

bool robotarm_rbd::RobotarmRbd::convert_joint_deltas_to_cartesian_deltas(
	const Eigen::VectorXd &q,
	const Eigen::VectorXd &delta_q,
	const std::string &link_name,
	Eigen::Matrix<double, 6, 1> &delta_x)
{
	if (!check_input(q)) return false;

	if (delta_q.size() != static_cast<Eigen::Index>(joints_.size())) {
		return fail("Unexpected delta_q dimension");
	}
	if (!delta_q.allFinite()) {
		return fail("delta_q contains NaN or inf");
	}

	// delta_x = J delta_q
	if (!calculate_jacobian(q, link_name, j_jd2cd_)) return false;
	delta_x.noalias() = j_jd2cd_ * delta_q;
    return true;
}

bool robotarm_rbd::RobotarmRbd::calculate_link_transform(
	const Eigen::VectorXd &q,
	const std::string &link_name,
	Eigen::Isometry3d &transform)
{
    if (!check_input(q)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		transform = Eigen::Isometry3d::Identity();
		return true;
	}

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for (size_t i = 0; i < joints_.size(); ++i) {
		T = T * joints_[i].transform(q[i]);
		if (joints_[i].child_link_name_ == link_name) {
			transform = T;
			return true;
		}
	}

	if (link_name == tcp_.tcp_name_) {  // TCP is not in joints_, so it comes after the loop
		transform = T * tcp_.tcp_origin_in_parent_;
		return true;
	}

	return fail("Link %s not found in kinematic chain", link_name.c_str());
}

bool robotarm_rbd::RobotarmRbd::calculate_jacobian(
	const Eigen::VectorXd &q,
	const std::string &link_name,
	Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
    if (!check_input(q)) return false;

	if (link_name == joints_.front().parent_link_name_) {
		jacobian.setZero(6, joints_.size());
		return true;
	}

	j_cj_.setZero();
	Eigen::Isometry3d T_end;
	if (!calculate_link_transform(q, link_name, T_end)) return false;
	Eigen::Vector3d p_end = T_end.translation();

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	for (size_t i = 0; i < joints_.size(); ++i) {

		T = T * joints_[i].transform(q[i]);
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

bool robotarm_rbd::RobotarmRbd::calculate_jacobian_inverse(
	const Eigen::VectorXd &q,
	const std::string &link_name,
	Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobian_inverse)
{
    // q is validated by calculate_jacobian(), which is the first thing called here
	if (!calculate_jacobian(q, link_name, j_cji_)) return false;

	// calculate J⁻¹ as pseudo-inverse with damped least squares: f(q̇) = ‖J q̇ − ẋ‖² + λ²‖q̇‖²
	// J⁺ = (JᵀJ + λ²I)⁻¹ Jᵀ -> (JᵀJ + λ²I)J⁺ = Jᵀ as MX = Jᵀ with:
	// M = (JᵀJ + λ²I)
	// X = J⁺
	M_.noalias() = j_cji_.transpose() * j_cji_;
	M_.diagonal().array() += config_.lambda * config_.lambda;

	// factor M once, and check that it worked
	ldlt_.compute(M_);
	if (ldlt_.info() != Eigen::Success) {
		return fail("Factorization failed");
	}
	jacobian_inverse = ldlt_.solve(j_cji_.transpose());

    return true;
}

bool robotarm_rbd::RobotarmRbd::recursive_newton_euler(
	const Eigen::VectorXd &q,
	const Eigen::VectorXd &dq,
	const Eigen::VectorXd &ddq,
	Eigen::VectorXd &tau,
	const Eigen::Vector3d &F_tcp,
    const Eigen::Vector3d &M_tcp,
	const Eigen::Vector3d& gravity)
{
	if (!check_input(q)) return false;
	if (!check_input(dq)) return false;
	if (!check_input(ddq)) return false;
	tau.resize(joints_.size());				// if sized properly before call no allocation -> rt safe

	// forward pass
	Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();
	Eigen::Vector3d dot_w_prev = Eigen::Vector3d::Zero();
	// trick: add g as root_accel so it automatically gets propagated to every link and does not to be added during backwards pass
	Eigen::Vector3d a_org_prev = -gravity;

	for (size_t i=0; i<joints_.size(); ++i) {
		data_.T[i] = joints_[i].transform(q[i]);
		Eigen::Matrix3d Rt = data_.T[i].linear().transpose();		// to child system
		Eigen::Vector3d r = data_.T[i].translation();
		Eigen::Vector3d r_com = joints_[i].inertia_.com;
		Eigen::Vector3d ax = joints_[i].joint_axis_in_child_;

		data_.w[i]		= Rt*w_prev + dq[i]*ax;
		data_.dot_w[i]	= Rt*dot_w_prev + ddq[i]*ax + data_.w[i].cross(dq[i]*ax);
		data_.a_org[i]	= Rt*(a_org_prev + dot_w_prev.cross(r) + w_prev.cross(w_prev.cross(r)));
		data_.a_com[i]	= data_.a_org[i] + data_.dot_w[i].cross(r_com) + data_.w[i].cross(data_.w[i].cross(r_com));

		w_prev = data_.w[i]; dot_w_prev = data_.dot_w[i];
		a_org_prev = data_.a_org[i];
	}

	// backwards pass
	Eigen::Matrix3d R_next = tcp_.tcp_origin_in_parent_.linear();
	Eigen::Vector3d r_next = tcp_.tcp_origin_in_parent_.translation();
	Eigen::Vector3d F_next = F_tcp;
	Eigen::Vector3d M_next = M_tcp;

	for (int i=static_cast<int>(joints_.size())-1; i>=0; --i) {
		double m = joints_[i].inertia_.mass;
		Eigen::Matrix3d I_com = joints_[i].inertia_.com_inertia_in_joint;
		Eigen::Vector3d r_com = joints_[i].inertia_.com;
		Eigen::Vector3d ax  = joints_[i].joint_axis_in_child_;
		Eigen::Vector3d F_dyn = m*data_.a_com[i];

		data_.F[i] = F_dyn + R_next*F_next;
		data_.M[i] = I_com*data_.dot_w[i] + data_.w[i].cross(I_com*data_.w[i]) + r_com.cross(F_dyn)
				   + R_next*M_next + r_next.cross(R_next*F_next);
		tau[i] = data_.M[i].dot(ax);

		R_next = data_.T[i].linear();
		r_next = data_.T[i].translation();
		F_next = data_.F[i];
		M_next = data_.M[i];
	}

    return true;
}

Eigen::Isometry3d robotarm_rbd::RobotarmRbd::joint_origin_to_isometry(urdf::JointConstSharedPtr joint)
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

bool robotarm_rbd::RobotarmRbd::check_input(const Eigen::VectorXd &input)
{
    if (!initialised_) {
		return fail("Kinematics not initialised");
	}

	if (input.size() != static_cast<Eigen::Index>(joints_.size())) {
		return fail("Unexpected input dimension");
	}

	if (!input.allFinite()) {
		return fail("Input contains NaN or inf");
	}

	return true;
}

bool robotarm_rbd::RobotarmRbd::get_joint_names(std::vector<std::string>& names)
{
    if (!initialised_) {
		return fail("Not initialised");
	}
	names.clear();
	names.reserve(joints_.size());
	for (const auto & j : joints_) {
		names.push_back(j.joint_name_);
	}
    return true;
}

bool robotarm_rbd::RobotarmRbd::get_joint_limits(std::vector<Limits>& limits)
{
    if (!initialised_) {
		return fail("Not initialised");
	}
	limits.clear();
	limits.reserve(joints_.size());
	for (const auto & j : joints_) {
		limits.push_back(j.limits_);
	}
    return true;
}

bool robotarm_rbd::RobotarmRbd::get_link_names(std::vector<std::string> &names)
{
    if (!initialised_) {
		return fail("Not initialised");
	}
	names.clear();
	names.reserve(joints_.size()+1);
	names.push_back(joints_.front().parent_link_name_);
	for (const auto & j : joints_) {
		names.push_back(j.child_link_name_);
	}
    return true;
}

bool robotarm_rbd::RobotarmRbd::get_tcp_link_name(std::string& name)
{
    if (!initialised_) {
		return fail("Not initialised");
	}
	name = tcp_.tcp_name_;
    return true;
}

std::string robotarm_rbd::RobotarmRbd::chain_table_log() const
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

    return os.str();
}

bool robotarm_rbd::RobotarmRbd::fail(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(error_, sizeof(error_), fmt, args);
	va_end(args);
	return false;
}