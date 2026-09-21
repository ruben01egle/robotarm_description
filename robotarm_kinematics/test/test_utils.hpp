#ifndef ROBOTARM_KINEMATICS_TEST_UTILS_HPP
#define ROBOTARM_KINEMATICS_TEST_UTILS_HPP

// Helpers shared by the KinematicsCore tests:
//  - TestableCore:  exposes the parsed chain of KinematicsCore
//  - UrdfSpec:      builds a DH-conform URDF from a DH table, with knobs to break it on purpose
//  - ReferenceFk:   forward kinematics computed straight from the URDF joint origins. It shares no
//                   code with the DH machinery of the plugin, so it is an independent reference
//  - real_urdf():   the URDF of the real robot, processed by xacro

#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotarm_kinematics/KinematicsCore.hpp"
#include "urdf/model.h"

namespace test_utils
{

using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using JacobianInverse = Eigen::Matrix<double, Eigen::Dynamic, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

class TestableCore : public robotarm_kinematics::KinematicsCore
{
public:
    using KinematicsCore::joints_;
    using KinematicsCore::tcp_link_name_;
};

// initialize() reads the "lambda" parameter, so it needs a node that carries the override
inline std::vector<rclcpp::Parameter> lambda_param(double lambda)
{
    return {rclcpp::Parameter("lambda", lambda)};
}

inline bool initialize(
    TestableCore & core, const std::string & urdf,
    const std::vector<rclcpp::Parameter> & overrides = {})
{
    static int counter = 0;
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp::Node>("kinematics_test_" + std::to_string(counter++), options);
    return core.initialize(urdf, node->get_node_parameters_interface(), "");
}

// ---------------------------------------------------------------------------------------------
// synthetic URDF
// ---------------------------------------------------------------------------------------------

struct DhRow
{
    double a, alpha, d, theta0;
};

struct Origin
{
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
};

// A = Rz(theta0) Tz(d) Tx(a) Rx(alpha) at q = 0, written as a URDF origin (see the README)
inline Origin origin_from_dh(const DhRow & r)
{
    Origin o;
    o.xyz = Eigen::Vector3d(r.a * std::cos(r.theta0), r.a * std::sin(r.theta0), r.d);
    o.rpy = Eigen::Vector3d(r.alpha, 0.0, r.theta0);
    return o;
}

struct JointSpec
{
    std::string type = "revolute";
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    bool has_limit = true;
    double lower = -3.0;
    double upper = 3.0;
    double velocity = 1.0;
    double effort = 1.0;
};

// Joint i connects link i to link i + 1. Link 0 is "base", the last link is "tcp". The origin of
// joint k + 1 encodes DH row k, the origin of the last joint (tcp) encodes the last row.
struct UrdfSpec
{
    std::vector<Origin> origins;
    std::vector<JointSpec> joints;
    bool branch = false;  // gives link 3 a second child

    std::string link_name(size_t i) const
    {
        if (i == 0) {
            return "base";
        }
        return i == joints.size() ? "tcp" : "link" + std::to_string(i);
    }

    std::string joint_name(size_t i) const
    {
        return i + 1 == joints.size() ? "tcp_joint" : "axis" + std::to_string(i + 1);
    }

    std::string str() const
    {
        std::ostringstream os;
        os << std::setprecision(17);
        os << "<?xml version=\"1.0\"?>\n<robot name=\"test_arm\">\n";
        for (size_t i = 0; i <= joints.size(); ++i) {
            os << "  <link name=\"" << link_name(i) << "\">\n";
            os << "  </link>\n";
        }
        if (branch) {
            os << "  <link name=\"branch_link\"/>\n"
               << "  <joint name=\"branch_joint\" type=\"fixed\"><parent link=\"" << link_name(3)
               << "\"/><child link=\"branch_link\"/></joint>\n";
        }
        for (size_t i = 0; i < joints.size(); ++i) {
            const JointSpec & j = joints[i];
            os << "  <joint name=\"" << joint_name(i) << "\" type=\"" << j.type << "\">\n"
               << "    <origin xyz=\"" << origins[i].xyz.transpose() << "\" rpy=\""
               << origins[i].rpy.transpose() << "\"/>\n"
               << "    <parent link=\"" << link_name(i) << "\"/><child link=\"" << link_name(i + 1)
               << "\"/>\n";
            if (j.type != "fixed") {
                os << "    <axis xyz=\"" << j.axis.transpose() << "\"/>\n";
                if (j.has_limit) {
                    os << "    <limit effort=\"" << j.effort << "\" velocity=\"" << j.velocity
                       << "\" lower=\"" << j.lower << "\" upper=\"" << j.upper << "\"/>\n";
                }
            }
            os << "  </joint>\n";
        }
        os << "</robot>\n";
        return os.str();
    }
};

// 6 revolute joints + fixed tcp joint, DH-conform, joint limits +-3 rad
inline UrdfSpec make_spec(const std::vector<DhRow> & dh)
{
    UrdfSpec s;
    s.origins.push_back(Origin{});  // first joint: identity, root link frame == DH frame 0
    for (const DhRow & row : dh) {
        s.origins.push_back(origin_from_dh(row));
    }
    s.joints.assign(dh.size() + 1, JointSpec{});
    s.joints.back().type = "fixed";
    return s;
}

// the DH table of the real robot (twists of +-90 deg, the interesting case for the wrist)
inline std::vector<DhRow> robot_dh()
{
    return {
        {0.025, -M_PI / 2, 0.35, 0.0},
        {0.3, 0.0, 0.0185, 0.0},
        {0.015, -M_PI / 2, 0.0, -M_PI / 2},
        {0.0, M_PI / 2, 0.252, 0.0},
        {0.0, -M_PI / 2, 0.0, 0.0},
        {0.0, 0.0, 0.148, 0.0}};
}

// arbitrary twists, offsets and negative lengths: nothing special about +-90 deg
inline std::vector<DhRow> generic_dh()
{
    return {
        {-0.1, 0.7, -0.2, 0.3},
        {0.4, -0.5, 0.05, -0.8},
        {0.0, 1.2, 0.3, 0.0},
        {0.15, -M_PI / 2, -0.1, M_PI / 3},
        {-0.05, 0.4, 0.2, 0.1},
        {0.02, 0.0, 0.1, -0.4}};
}

// ---------------------------------------------------------------------------------------------
// reference forward kinematics: T_k = T_{k-1} * Origin_k * Rz(q_k), straight from the URDF
// ---------------------------------------------------------------------------------------------

class ReferenceFk
{
public:
    explicit ReferenceFk(const std::string & urdf)
    {
        urdf::Model model;
        if (!model.initString(urdf)) {
            throw std::runtime_error("reference FK: URDF does not parse");
        }
        urdf::LinkConstSharedPtr link = model.getRoot();
        root_ = link->name;
        link_names_.push_back(root_);
        while (!link->child_joints.empty()) {
            urdf::JointConstSharedPtr joint = link->child_joints[0];
            Step s;
            const urdf::Pose & p = joint->parent_to_joint_origin_transform;
            s.origin = Eigen::Isometry3d::Identity();
            s.origin.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
            s.origin.linear() =
                Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
                .normalized().toRotationMatrix();
            s.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
            s.revolute = joint->type == urdf::Joint::REVOLUTE;
            s.child = joint->child_link_name;
            if (s.revolute) {
                lower_.push_back(joint->limits->lower);
                upper_.push_back(joint->limits->upper);
            }
            steps_.push_back(s);
            link_names_.push_back(s.child);
            link = model.getLink(s.child);
        }
    }

    size_t dof() const {return lower_.size();}

    // root link first, tcp last
    const std::vector<std::string> & link_names() const {return link_names_;}

    Eigen::Isometry3d link(const Eigen::VectorXd & q, const std::string & name) const
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if (name == root_) {
            return T;
        }
        Eigen::Index qi = 0;
        for (const Step & s : steps_) {
            T = T * s.origin;
            if (s.revolute) {
                T.rotate(Eigen::AngleAxisd(q[qi++], s.axis));
            }
            if (s.child == name) {
                return T;
            }
        }
        throw std::runtime_error("reference FK: unknown link " + name);
    }

    // uniform inside the joint limits, kept away from the limits themselves
    Eigen::VectorXd random_q(std::mt19937 & rng) const
    {
        Eigen::VectorXd q(static_cast<Eigen::Index>(dof()));
        for (size_t i = 0; i < dof(); ++i) {
            const double margin = 0.02 * (upper_[i] - lower_[i]);
            std::uniform_real_distribution<double> u(lower_[i] + margin, upper_[i] - margin);
            q[static_cast<Eigen::Index>(i)] = u(rng);
        }
        return q;
    }

private:
    struct Step
    {
        Eigen::Isometry3d origin;
        Eigen::Vector3d axis;
        bool revolute = false;
        std::string child;
    };
    std::string root_;
    std::vector<Step> steps_;
    std::vector<std::string> link_names_;
    std::vector<double> lower_, upper_;
};

// ---------------------------------------------------------------------------------------------
// misc
// ---------------------------------------------------------------------------------------------

inline Eigen::VectorXd random_vector(std::mt19937 & rng, Eigen::Index n, double scale)
{
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Eigen::VectorXd v(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        v[i] = scale * u(rng);
    }
    return v;
}

// rotation vector (axis * angle) of R, the small-rotation counterpart of a delta_x angular part
inline Eigen::Vector3d rotation_vector(const Eigen::Matrix3d & R)
{
    const Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

// URDF of the real robot. Throws if robotarm_description or xacro is not available: both are
// exec_depends of this package, so that is an error of the environment, not something to skip.
inline std::string real_urdf()
{
    static const std::string cached = [] {
        const std::string share = ament_index_cpp::get_package_share_directory("robotarm_description");
        const std::string cmd = "xacro '" + share + "/urdf/robotarm.urdf.xacro' 2>/dev/null";
        FILE * pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("cannot run xacro");
        }
        std::string out;
        char buf[4096];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), pipe)) > 0) {
            out.append(buf, n);
        }
        if (pclose(pipe) != 0 || out.empty()) {
            throw std::runtime_error("xacro failed on " + share + "/urdf/robotarm.urdf.xacro");
        }
        return out;
    }();
    return cached;
}

}  // namespace test_utils

#endif  // ROBOTARM_KINEMATICS_TEST_UTILS_HPP
