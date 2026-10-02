#include "robotarm_rbd/RobotarmRbd.hpp"

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace
{
const char * implausible_inertial(
	double mass,
	const Eigen::Matrix3d & I,
	const Eigen::Vector3d & com,
	const Eigen::Vector4d & quat);

const char * read_inertial(const urdf::Inertial & in, robotarm_rbd::RobotarmRbd::InertialParams & out);

// Steiner (parallel axis) term: inertia of a point mass m at d about the origin, m (|d|² E − d dᵀ)
Eigen::Matrix3d steiner(double m, const Eigen::Vector3d & d)
{
	return m * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
}
}

bool robotarm_rbd::RobotarmRbd::RobotarmRbd::initialize(const std::string &robot_description, const Config &config)
{
    initialised_ = false;
    inertia_available_ = false;
    tool_mass_ = 0.0;
    joints_.clear();
    flange_ = Flange();
    tcp_ = TCP();

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
	std::vector<urdf::JointConstSharedPtr> moving_joints_urdf;
	urdf::JointConstSharedPtr flange_joint_urdf;

	// collect the moving joints root -> flange
	while (!flange_joint_urdf) {
		if (link_urdf->child_joints.empty()) {
			return fail("Chain ends at link %s without a joint %s", link_urdf->name.c_str(),
				flange_.joint_name_.c_str());
		}
		if (link_urdf->child_joints.size() != 1) {
			return fail("Link %s has multiple children links", link_urdf->name.c_str());
		}
		urdf::JointConstSharedPtr joint = link_urdf->child_joints[0];
		if (joint->name == flange_.joint_name_) {
			flange_joint_urdf = joint;
		} else {
			moving_joints_urdf.push_back(joint);
		}
		link_urdf = model->getLink(joint->child_link_name);
		if (!link_urdf) {
			return fail("Joint %s references unknown child link %s",
				joint->name.c_str(), joint->child_link_name.c_str());
		}
	}
	// link_urdf is now the flange link, the root of the tool subtree

	if (flange_joint_urdf->type != urdf::Joint::FIXED) {
		return fail("Flange joint %s is not fixed", flange_joint_urdf->name.c_str());
	}
	if (flange_joint_urdf->child_link_name != flange_.flange_name_) {
		return fail("Flange joint %s has child link %s, expected %s", flange_joint_urdf->name.c_str(),
			flange_joint_urdf->child_link_name.c_str(), flange_.flange_name_.c_str());
	}
	if (moving_joints_urdf.empty()) {
		return fail("URDF chain has no joint before %s", flange_joint_urdf->name.c_str());
	}

	// parse data of moving joints
	size_t n_inertial = 0;
	const char * link_without_inertial = nullptr;
    for (size_t i = 0; i < moving_joints_urdf.size(); ++i) {
		urdf::JointConstSharedPtr joint_urdf = moving_joints_urdf[i];

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
			const char * reason = read_inertial(*child_urdf->inertial, joint.inertia_);
			if (reason) {
				return fail("Link %s has an implausible inertial: %s", child_urdf->name.c_str(), reason);
			}
			++n_inertial;
		} else if (!link_without_inertial) {
			link_without_inertial = child_urdf->name.c_str();
		}

		joints_.push_back(joint);
    }

	// all or nothing: a model with only some inertias would give plausible looking but wrong torques
	if (n_inertial != 0 && n_inertial != joints_.size()) {
		return fail("Link %s has no inertial, but other links have one (all or none)",
			link_without_inertial);
	}
	inertia_available_ = n_inertial == joints_.size();

	// parse data of flange
	flange_.parent_link_name_ = flange_joint_urdf->parent_link_name;
	flange_.flange_origin_in_parent_ = joint_origin_to_isometry(flange_joint_urdf);

	// walk the tool subtree behind the flange and every <inertial> is
	// folded into the last moving link. Tool joints that are not fixed are taken at their zero position
	InertialParams & last = joints_.back().inertia_;
	// sums over all bodies (last moving link + tool links), about the origin of the last moving link:
	// mass, first moment of mass, inertia
	double m_sum = last.mass;
	Eigen::Vector3d mc_sum = last.mass * last.com;
	Eigen::Matrix3d I_origin = last.com_inertia_in_joint + steiner(last.mass, last.com);
	bool tcp_found = false;

	std::vector<std::pair<urdf::LinkConstSharedPtr, Eigen::Isometry3d>> todo;
	todo.emplace_back(link_urdf, flange_.flange_origin_in_parent_);  // link_urdf is the flange link
	while (!todo.empty()) {
		const auto [link, T] = todo.back();
		todo.pop_back();

		if (link->name == tcp_.tcp_name_) {
			tcp_found = true;
			tcp_.tcp_origin_in_last_mov_link_ = T;
			tcp_.tcp_origin_in_flange_ = flange_.flange_origin_in_parent_.inverse() * T;
		}

		// without inertia on the arm (kinematics only) the tool inertia is ignored, it would break all or none
		if (inertia_available_ && link->inertial) {
			InertialParams body;
			const char * reason = read_inertial(*link->inertial, body);
			if (reason) {
				return fail("Link %s has an implausible inertial: %s", link->name.c_str(), reason);
			}
			// into the axes and origin of the last moving link
			const Eigen::Matrix3d R = T.linear();
			const Eigen::Vector3d com = T * body.com;
			m_sum += body.mass;
			mc_sum += body.mass * com;
			I_origin += R * body.com_inertia_in_joint * R.transpose() + steiner(body.mass, com);
		}

		for (const auto & joint : link->child_joints) {
			const urdf::LinkConstSharedPtr child = model->getLink(joint->child_link_name);
			if (!child) {
				return fail("Joint %s references unknown child link %s",
					joint->name.c_str(), joint->child_link_name.c_str());
			}
			todo.emplace_back(child, T * joint_origin_to_isometry(joint));
		}
	}

	if (!tcp_found) {
		return fail("No link %s behind link %s", tcp_.tcp_name_.c_str(), flange_.flange_name_.c_str());
	}

	tool_mass_ = m_sum - last.mass;

	// back from the origin to the combined CoM. A massless last link with a massless tool stays all zero.
	if (m_sum > 0.0) {
		last.mass = m_sum;
		last.com = mc_sum / m_sum;
		last.com_inertia_in_joint = I_origin - steiner(m_sum, last.com);
	}

	// init heap member
	j_cj_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_cji_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_jd2cd_ = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, joints_.size());
	j_inv_cd2jd_ =  Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(joints_.size(), 6);
	jtj_damped_ = Eigen::MatrixXd::Zero(joints_.size(), joints_.size());
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
    if (!check_input(q, "q")) return false;

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
	if (!check_input(q, "q")) return false;

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
    if (!check_input(q, "q")) return false;

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

	T = T * flange_.flange_origin_in_parent_;	// Flange is not in joints_, so it comes after the loop
	if (link_name == flange_.flange_name_) {
		transform = T;
		return true;
	}

	if (link_name == tcp_.tcp_name_) {  // TCP is not in joints_, so it comes after the loop
		transform = T * tcp_.tcp_origin_in_flange_;
		return true;
	}

	return fail("Link %s not found in kinematic chain", link_name.c_str());
}

bool robotarm_rbd::RobotarmRbd::calculate_jacobian(
	const Eigen::VectorXd &q,
	const std::string &link_name,
	Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian)
{
    if (!check_input(q, "q")) return false;

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
	// M = (JᵀJ + λ²I) (called jtj_damped_)
	// X = J⁺
	jtj_damped_.noalias() = j_cji_.transpose() * j_cji_;
	jtj_damped_.diagonal().array() += config_.lambda * config_.lambda;

	// factor M once, and check that it worked
	ldlt_.compute(jtj_damped_);
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
	if (!check_input(q, "q")) return false;
	if (!check_input(dq, "dq")) return false;
	if (!check_input(ddq, "ddq")) return false;
	if (!F_tcp.allFinite() || !M_tcp.allFinite() || !gravity.allFinite()) {
		return fail("F_tcp, M_tcp or gravity contains NaN or inf");
	}
	// without inertia every link is massless: the torques would only contain the TCP wrench, which is
	// only correct if nothing else contributes
	if (!inertia_available_ && !(gravity.isZero(0.0) && dq.isZero(0.0) && ddq.isZero(0.0))) {
		return fail("Not all links have a valid inertia");
	}
	tau.resize(joints_.size());				// if sized properly before call no allocation -> rt safe

	// forward pass
	Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();
	Eigen::Vector3d dot_w_prev = Eigen::Vector3d::Zero();
	// trick: add g as root_accel so it automatically gets propagated to every link and does not to be added during backwards pass
	Eigen::Vector3d a_org_prev = -gravity;

	for (size_t i=0; i<joints_.size(); ++i) {
		data_.T[i] = joints_[i].transform(q[i]);
		const Eigen::Matrix3d Rt = data_.T[i].linear().transpose();		// to child system
		const Eigen::Vector3d r = data_.T[i].translation();
		const Eigen::Vector3d& r_com = joints_[i].inertia_.com;
		const Eigen::Vector3d& ax = joints_[i].joint_axis_in_child_;

		data_.w[i]		= Rt*w_prev + dq[i]*ax;
		data_.dot_w[i]	= Rt*dot_w_prev + ddq[i]*ax + data_.w[i].cross(dq[i]*ax);
		data_.a_org[i]	= Rt*(a_org_prev + dot_w_prev.cross(r) + w_prev.cross(w_prev.cross(r)));
		data_.a_com[i]	= data_.a_org[i] + data_.dot_w[i].cross(r_com) + data_.w[i].cross(data_.w[i].cross(r_com));

		w_prev = data_.w[i]; dot_w_prev = data_.dot_w[i];
		a_org_prev = data_.a_org[i];
	}

	// backwards pass
	Eigen::Matrix3d R_next = tcp_.tcp_origin_in_last_mov_link_.linear();
	Eigen::Vector3d r_next = tcp_.tcp_origin_in_last_mov_link_.translation();
	Eigen::Vector3d F_next = F_tcp;
	Eigen::Vector3d M_next = M_tcp;

	for (int i=static_cast<int>(joints_.size())-1; i>=0; --i) {
		const double m = joints_[i].inertia_.mass;
		const Eigen::Matrix3d& I_com = joints_[i].inertia_.com_inertia_in_joint;
		const Eigen::Vector3d& r_com = joints_[i].inertia_.com;
		const Eigen::Vector3d& ax  = joints_[i].joint_axis_in_child_;
		const Eigen::Vector3d F_dyn = m*data_.a_com[i];

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

bool robotarm_rbd::RobotarmRbd::check_input(const Eigen::VectorXd &input, const char *name)
{
    if (!initialised_) {
		return fail("Not initialised");
	}

	if (input.size() != static_cast<Eigen::Index>(joints_.size())) {
		return fail("Unexpected %s dimension", name);
	}

	if (!input.allFinite()) {
		return fail("%s contains NaN or inf", name);
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
    // the tcp row has no joint of its own: it is the path through the tool, or the tcp joint without one
    const std::string tcp_joint = tcp_.tcp_origin_in_flange_.isApprox(Eigen::Isometry3d::Identity()) ?
        "(= flange)" : "(via tool)";
    size_t w_joint = std::max({std::string("joint").size(), flange_.joint_name_.size(), tcp_joint.size()});
    size_t w_parent = std::max({std::string("parent link").size(), flange_.parent_link_name_.size(),
        tcp_.parent_link_name_.size()});
    size_t w_child = std::max({std::string("child link").size(), flange_.flange_name_.size(),
        tcp_.tcp_name_.size()});
    for (const auto & j : joints_) {
        w_joint = std::max(w_joint, j.joint_name_.size());
        w_parent = std::max(w_parent, j.parent_link_name_.size());
        w_child = std::max(w_child, j.child_link_name_.size());
    }

    auto row = [&](std::ostringstream & os, const std::string & idx, const std::string & joint,
                   const std::string & parent, const std::string & child) {
        os << std::left
           << std::setw(3) << idx << "  "
           << std::setw(static_cast<int>(w_joint)) << joint << "  "
           << std::setw(static_cast<int>(w_parent)) << parent << "  "
           << std::setw(static_cast<int>(w_child)) << child;
    };
    // translation [m] and rotation angle [deg] of a fixed frame relative to its parent in the table
    auto offset = [](std::ostringstream & os, const Eigen::Isometry3d & T) {
        const Eigen::Vector3d p = T.translation();
        os << std::fixed << std::setprecision(4) << "  xyz (" << p.x() << ", " << p.y() << ", " << p.z()
           << ") m, rotated " << std::setprecision(1)
           << Eigen::AngleAxisd(T.linear()).angle() * 180.0 / M_PI << " deg";
        os.unsetf(std::ios::floatfield);
    };

    std::ostringstream os;
    os << "\nKinematic chain: " << joints_.size() << " joint(s) + flange + tcp\n";

    std::ostringstream header;
    row(header, "#", "joint", "parent link", "child link");
    os << header.str() << "\n" << std::string(header.str().size(), '-') << "\n";

    for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        row(os, std::to_string(i), j.joint_name_, j.parent_link_name_, j.child_link_name_);
        os << "\n";
    }
    row(os, "flg", flange_.joint_name_, flange_.parent_link_name_, flange_.flange_name_);
    offset(os, flange_.flange_origin_in_parent_);
    os << "\n";
    row(os, "tcp", tcp_joint, tcp_.parent_link_name_, tcp_.tcp_name_);
    offset(os, tcp_.tcp_origin_in_flange_);
    os << "\n";

    os << "inertia: " << (inertia_available_ ? "all links (dynamics available)"
                                             : "none (kinematics only)") << "\n";
    if (inertia_available_) {
        os << "tool: " << std::fixed << std::setprecision(3) << tool_mass_ << " kg folded into "
           << flange_.parent_link_name_ << " (now " << joints_.back().inertia_.mass << " kg)\n";
    } else {
        os << "tool: inertia ignored (kinematics only)\n";
    }

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

namespace
{
// Physical plausibility of an URDF <inertial>, the tensor as given in the URDF (about the CoM, inertial
// frame axes). Returns nullptr if it is plausible, otherwise the reason.
//  - everything finite, mass >= 0
//  - tensor positive semidefinite (principal moments >= 0)
//  - triangle inequality of the principal moments, I_a + I_b >= I_c: I_a + I_b - I_c = 2∫c² dm >= 0
//    for any real mass distribution
//  - no mass means no inertia
// The tolerance is relative to the trace, CAD exports round their values.
const char * implausible_inertial(double mass, const Eigen::Matrix3d & I, const Eigen::Vector3d & com,
	const Eigen::Vector4d & quat)
{
	if (!std::isfinite(mass) || !I.allFinite() || !com.allFinite() || !quat.allFinite()) {
		return "not finite";
	}
	if (mass < 0.0) {
		return "negative mass";
	}
	const double tol = 1e-6 * std::abs(I.trace());
	if (mass == 0.0) {
		return I.isZero(0.0) ? nullptr : "inertia without mass";
	}
	const Eigen::Vector3d moments = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(
		I, Eigen::EigenvaluesOnly).eigenvalues();  // ascending
	if (moments[0] < -tol) {
		return "inertia tensor not positive semidefinite";
	}
	if (moments[0] + moments[1] < moments[2] - tol) {
		return "principal moments violate the triangle inequality";
	}
	return nullptr;
}

// URDF <inertial> -> mass, CoM in the link frame and the tensor about the CoM in link axes.
// Returns nullptr if it is plausible, otherwise the reason (out is then incomplete).
const char * read_inertial(const urdf::Inertial & in, robotarm_rbd::RobotarmRbd::InertialParams & out)
{
	const urdf::Rotation & r = in.origin.rotation;
	out.mass = in.mass;
	// vector to com in joint-frame
	out.com = Eigen::Vector3d(in.origin.position.x, in.origin.position.y, in.origin.position.z);
	const Eigen::Matrix3d R_com = Eigen::Quaterniond(r.w, r.x, r.y, r.z).toRotationMatrix();
	// inertia tensor of com in com-frame
	Eigen::Matrix3d I_com;
	I_com << in.ixx, in.ixy, in.ixz,
			 in.ixy, in.iyy, in.iyz,
			 in.ixz, in.iyz, in.izz;
	const char * reason = implausible_inertial(in.mass, I_com, out.com, Eigen::Vector4d(r.w, r.x, r.y, r.z));
	if (reason) {
		return reason;
	}
	// I_joint = R * I_com * R^T: rotates tensor from com axes into joint axes (L = R I_com R^T ω)
	// reference point stays at com
	out.com_inertia_in_joint = R_com * I_com * R_com.transpose();
	out.valid = true;
	return nullptr;
}
}  // namespace