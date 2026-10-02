#ifndef ROBOTARM_RBD_TEST_UTILS_HPP
#define ROBOTARM_RBD_TEST_UTILS_HPP

// Helpers shared by the Kinematics tests:
//  - TestableKinematics:  the plugin under test; its parsed chain is checked through TestableRbd
//                         (testable_rbd.hpp), since Kinematics only holds it inside RobotarmRbd
//  - UrdfSpec:      builds a serial-chain URDF ending in flange and tcp, optionally with a branched
//                   tool between them, with knobs to break it on purpose. Filled either from a DH
//                   table (realistic arm geometry) or with random origins and axes
//                   (make_random_spec, any number of joints, nothing DH-conform about it)
//  - ReferenceFk:   forward kinematics computed straight from the URDF joint origins, for every link
//                   of the tree. It shares no code with the plugin (quaternion instead of RPY,
//                   Isometry::rotate, walks from the link up to the root), so it is an independent
//                   reference
//  - real_urdf():   the URDF of the real robot, processed by xacro (real_urdf.hpp, ROS free)

#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "real_urdf.hpp"
#include "robotarm_rbd/Kinematics.hpp"
#include "testable_rbd.hpp"
#include "urdf/model.h"

namespace test_utils
{

using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using JacobianInverse = Eigen::Matrix<double, Eigen::Dynamic, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

using TestableKinematics = robotarm_rbd::Kinematics;

// tcp link name through the public getter, empty (and a test failure) before initialize()
inline std::string tcp_name(robotarm_rbd::Kinematics & core)
{
    std::string name;
    EXPECT_TRUE(core.get_tcp_link_name(name));
    return name;
}

// initialize() reads the "lambda" parameter, so it needs a node that carries the override
inline std::vector<rclcpp::Parameter> lambda_param(double lambda)
{
    return {rclcpp::Parameter("lambda", lambda)};
}

inline bool initialize(
    TestableKinematics & core, const std::string & urdf,
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

// The tool of UrdfSpec::tool, between flange and tcp. It branches (two jaws) and has a prismatic jaw,
// which the parser takes at its zero position.
inline const char * test_tool_xml()
{
    return R"(  <link name="tool_base"/>
  <link name="tool_jaw1"/>
  <link name="tool_jaw2"/>
  <joint name="tool_attach_joint" type="fixed">
    <origin xyz="0.002 -0.001 0.01" rpy="0.1 0 0.7"/>
    <parent link="flange"/><child link="tool_base"/>
  </joint>
  <joint name="tool_jaw1_joint" type="prismatic">
    <origin xyz="0 0.02 0.05" rpy="0 0.2 0"/>
    <parent link="tool_base"/><child link="tool_jaw1"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.02" velocity="0.1" effort="10"/>
  </joint>
  <joint name="tool_jaw2_joint" type="fixed">
    <origin xyz="0 -0.02 0.05" rpy="0 -0.2 0"/>
    <parent link="tool_base"/><child link="tool_jaw2"/>
  </joint>
)";
}

// Joint i connects link i to link i + 1. Link 0 is "base", the last two links are "flange" and
// "tcp", the last two joints the fixed flange joint and the fixed tcp joint. With tool, the tcp
// joint hangs on the tool (test_tool_xml) instead of the flange.
struct UrdfSpec
{
    std::vector<Origin> origins;
    std::vector<JointSpec> joints;
    bool branch = false;  // gives link 3 a second child
    bool tool = false;    // a branched tool between flange and tcp
    // names the parser relies on, changed by the tests that break them
    std::string flange_joint = "flange_joint";
    std::string flange_link = "flange";
    std::string tcp_link = "tcp";

    std::string link_name(size_t i) const
    {
        if (i == 0) {
            return "base";
        }
        if (i == joints.size()) {
            return tcp_link;
        }
        return i + 1 == joints.size() ? flange_link : "link" + std::to_string(i);
    }

    std::string joint_name(size_t i) const
    {
        if (i + 1 == joints.size()) {
            return "tcp_joint";
        }
        return i + 2 == joints.size() ? flange_joint : "axis" + std::to_string(i + 1);
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
        if (tool) {
            os << test_tool_xml();
        }
        for (size_t i = 0; i < joints.size(); ++i) {
            const JointSpec & j = joints[i];
            const bool tcp_on_tool = tool && i + 1 == joints.size();
            os << "  <joint name=\"" << joint_name(i) << "\" type=\"" << j.type << "\">\n"
               << "    <origin xyz=\"" << origins[i].xyz.transpose() << "\" rpy=\""
               << origins[i].rpy.transpose() << "\"/>\n"
               << "    <parent link=\"" << (tcp_on_tool ? "tool_base" : link_name(i))
               << "\"/><child link=\"" << link_name(i + 1) << "\"/>\n";
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

// 6 revolute joints + fixed flange joint + fixed tcp joint, joint limits +-3 rad. The origin of
// joint k + 1 encodes DH row k, the origin of the flange joint the last row, the tcp sits on the
// flange.
inline UrdfSpec make_spec(const std::vector<DhRow> & dh)
{
    UrdfSpec s;
    s.origins.push_back(Origin{});  // first joint: identity, root link frame == DH frame 0
    for (const DhRow & row : dh) {
        s.origins.push_back(origin_from_dh(row));
    }
    s.origins.push_back(Origin{});  // tcp = flange
    s.joints.assign(dh.size() + 2, JointSpec{});
    s.joints[s.joints.size() - 2].type = "fixed";
    s.joints.back().type = "fixed";
    return s;
}

// the DH robot with the branched tool and a tcp that is offset and rotated on it
inline UrdfSpec make_tool_spec(const std::vector<DhRow> & dh)
{
    UrdfSpec s = make_spec(dh);
    s.tool = true;
    s.origins.back().xyz = Eigen::Vector3d(0.01, -0.02, 0.12);
    s.origins.back().rpy = Eigen::Vector3d(0.2, -0.1, 0.4);
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

// dof revolute joints + fixed flange and tcp joints with random origins and axes: first origin not
// identity, rotations about all three axes, offsets in every direction, axes neither unit length nor
// along z. Pitch stays within +-1.2 rad, away from the RPY singularity at +-pi/2. Deterministic per
// seed.
inline UrdfSpec make_random_spec(size_t dof, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> offset(-0.3, 0.3), angle(-M_PI, M_PI),
        pitch(-1.2, 1.2), axis(-1.0, 1.0), scale(0.5, 2.0);
    UrdfSpec s;
    for (size_t i = 0; i <= dof + 1; ++i) {
        Origin o;
        o.xyz = Eigen::Vector3d(offset(rng), offset(rng), offset(rng));
        o.rpy = Eigen::Vector3d(angle(rng), pitch(rng), angle(rng));
        s.origins.push_back(o);
    }
    s.joints.assign(dof + 2, JointSpec{});
    for (size_t i = 0; i < dof; ++i) {
        Eigen::Vector3d a(axis(rng), axis(rng), axis(rng));
        s.joints[i].axis = scale(rng) * a.normalized();
    }
    s.joints[dof].type = "fixed";
    s.joints.back().type = "fixed";
    return s;
}

// URDF origin as an isometry, built independently of the plugin: Trans(xyz) * Rz(y) Ry(p) Rx(r)
inline Eigen::Isometry3d isometry(const Origin & o)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = o.xyz;
    T.linear() = (Eigen::AngleAxisd(o.rpy.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(o.rpy.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(o.rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    return T;
}

// ---------------------------------------------------------------------------------------------
// reference forward kinematics: T_k = T_{k-1} * Origin_k * R(axis_k, q_k), straight from the URDF.
// Works on the whole tree: a link's pose is the product along its path from the root. The revolute
// joints between root and flange are q (in chain order), every other joint (tool) stays at zero.
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
        root_ = model.getRoot()->name;
        // every joint, keyed by its child link: the step from the parent link to that link
        for (const auto & entry : model.joints_) {
            const urdf::Joint & joint = *entry.second;
            Step s;
            const urdf::Pose & p = joint.parent_to_joint_origin_transform;
            s.origin = Eigen::Isometry3d::Identity();
            s.origin.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
            s.origin.linear() =
                Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z)
                .normalized().toRotationMatrix();
            // the URDF axis is a direction, its length carries no meaning
            s.axis = Eigen::Vector3d(joint.axis.x, joint.axis.y, joint.axis.z).normalized();
            s.parent = joint.parent_link_name;
            s.revolute = joint.type == urdf::Joint::REVOLUTE;
            if (s.revolute) {
                s.lower = joint.limits->lower;
                s.upper = joint.limits->upper;
            }
            steps_[joint.child_link_name] = s;
        }

        // the moving chain root -> flange; its revolute joints get the entries of q
        std::vector<std::string> chain;
        for (std::string l = "flange"; l != root_; l = step(l).parent) {
            chain.push_back(l);
        }
        link_names_.push_back(root_);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
            Step & s = steps_.at(*it);
            if (s.revolute) {
                s.q_index = static_cast<Eigen::Index>(lower_.size());
                lower_.push_back(s.lower);
                upper_.push_back(s.upper);
            }
            link_names_.push_back(*it);
        }
        link_names_.push_back("tcp");
        step("tcp");  // throws if there is none
    }

    size_t dof() const {return lower_.size();}

    // the links the plugin knows: root link first, then the moving links, flange and tcp
    const std::vector<std::string> & link_names() const {return link_names_;}

    Eigen::Isometry3d link(const Eigen::VectorXd & q, const std::string & name) const
    {
        std::vector<const Step *> path;  // name -> root
        for (std::string l = name; l != root_; l = step(l).parent) {
            path.push_back(&step(l));
        }
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            T = T * (*it)->origin;
            if ((*it)->q_index >= 0) {
                T.rotate(Eigen::AngleAxisd(q[(*it)->q_index], (*it)->axis));
            }
        }
        return T;
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
        std::string parent;
        bool revolute = false;
        double lower = 0.0, upper = 0.0;
        Eigen::Index q_index = -1;  // -1: not driven by q (fixed, or a tool joint at zero)
    };

    const Step & step(const std::string & link) const
    {
        const auto it = steps_.find(link);
        if (it == steps_.end()) {
            throw std::runtime_error("reference FK: unknown link " + link);
        }
        return it->second;
    }

    std::string root_;
    std::map<std::string, Step> steps_;
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

}  // namespace test_utils

#endif  // ROBOTARM_RBD_TEST_UTILS_HPP
