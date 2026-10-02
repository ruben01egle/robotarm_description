// Tests for RobotarmRbd. Linked without rclcpp on purpose: if the library pulls in anything from ROS,
// this test stops linking. The full URDF rejection matrix runs through the Kinematics wrapper in
// kinematics_test.cpp, here the error reporting and the lifecycle of the ROS free class, the parsing
// and checks of <inertial>, and the inverse dynamics on models that can be solved by hand. The
// comparison of the inverse dynamics of the real robot with Pinocchio is in rnea_pinocchio_test.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "robotarm_rbd/RobotarmRbd.hpp"
#include "testable_rbd.hpp"

namespace
{

// two revolute joints, the flange and the tcp; axis2 is passed in so a test can break it.
// link1_inertial and link2_inertial replace the <inertial> of the links ("" leaves it out).
const std::string link1_inertial_default = R"(
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </inertial>)";
const std::string link2_inertial_default = R"(
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.2" iyy="0.25" izz="0.3" ixy="0.01" ixz="0.02" iyz="0.03"/>
      <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
    </inertial>)";

std::string two_joint_urdf(
    const std::string & axis2 = "0 1 0",
    const std::string & link1_inertial = link1_inertial_default,
    const std::string & link2_inertial = link2_inertial_default)
{
    return R"(<robot name="two_joint">
  <link name="base"/>
  <link name="link1">)" + link1_inertial + R"(
  </link>
  <link name="link2">)" + link2_inertial + R"(
  </link>
  <link name="flange"/>
  <link name="tcp"/>
  <joint name="axis1" type="revolute">
    <parent link="base"/> <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/> <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/>
  </joint>
  <joint name="axis2" type="revolute">
    <parent link="link1"/> <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/> <axis xyz=")" + axis2 + R"("/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/>
  </joint>
  <joint name="flange_joint" type="fixed">
    <parent link="link2"/> <child link="flange"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="flange"/> <child link="tcp"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>)";
}

// replaces every occurrence of from, to break a valid URDF on purpose
std::string replace_all(std::string s, const std::string & from, const std::string & to)
{
    for (size_t pos = s.find(from); pos != std::string::npos; pos = s.find(from, pos + to.size())) {
        s.replace(pos, from.size(), to);
    }
    return s;
}

}  // namespace

TEST(RobotarmRbd, AValidUrdfInitialises)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    EXPECT_TRUE(rbd.initialised());

    std::vector<std::string> names;
    std::string tcp;
    ASSERT_TRUE(rbd.get_joint_names(names));
    ASSERT_TRUE(rbd.get_tcp_link_name(tcp));
    EXPECT_EQ(names, (std::vector<std::string>{"axis1", "axis2"}));
    EXPECT_EQ(tcp, "tcp");
}

TEST(RobotarmRbd, InertiaIsParsedFromTheChildLinkAndRotatedIntoTheJointFrame)
{
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    ASSERT_EQ(rbd.joints_.size(), 2u);

    const auto & in = rbd.joints_[1].inertia_;
    EXPECT_TRUE(in.valid);
    EXPECT_DOUBLE_EQ(in.mass, 2.5);
    EXPECT_TRUE(in.com.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
    // the URDF tensor is given in the inertial frame (rpy = 0 0 pi/2), stored in joint axes:
    // I_joint = R I_urdf Rᵀ, reference point stays the CoM
    Eigen::Matrix3d I_urdf;
    I_urdf << 0.2, 0.01, 0.02,
              0.01, 0.25, 0.03,
              0.02, 0.03, 0.3;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    EXPECT_TRUE(in.com_inertia_in_joint.isApprox(R * I_urdf * R.transpose(), 1e-12));
    // by hand for Rz(90°): x_joint = -y_inertial, y_joint = x_inertial
    EXPECT_NEAR(in.com_inertia_in_joint(0, 0), 0.25, 1e-12);
    EXPECT_NEAR(in.com_inertia_in_joint(1, 1), 0.2, 1e-12);
    EXPECT_NEAR(in.com_inertia_in_joint(0, 1), -0.01, 1e-12);
    EXPECT_NEAR(in.com_inertia_in_joint(0, 2), -0.03, 1e-12);
    EXPECT_NEAR(in.com_inertia_in_joint(1, 2), 0.02, 1e-12);
}

TEST(RobotarmRbd, AModelWithoutAnyInertialInitialisesForKinematicsOnly)
{
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf("0 1 0", "", ""), {})) << rbd.last_error();
    for (const auto & joint : rbd.joints_) {
        const auto & none = joint.inertia_;
        EXPECT_FALSE(none.valid);
        EXPECT_EQ(none.mass, 0.0);
        EXPECT_TRUE(none.com.isZero());
        EXPECT_TRUE(none.com_inertia_in_joint.isZero());
    }
}

TEST(RobotarmRbd, AModelWithSomeInertialsMissingIsRejected)
{
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize(two_joint_urdf("0 1 0", ""), {}));
    EXPECT_FALSE(rbd.initialised());
    EXPECT_NE(std::string(rbd.last_error()).find("link1"), std::string::npos) << rbd.last_error();
}

TEST(RobotarmRbd, AnImplausibleInertialIsRejected)
{
    auto inertial = [](const std::string & mass, const std::string & diag) {
        return R"(<inertial><mass value=")" + mass + R"("/><inertia )" + diag +
               R"( ixy="0" ixz="0" iyz="0"/><origin xyz="0 0 0" rpy="0 0 0"/></inertial>)";
    };
    const std::vector<std::pair<std::string, std::string>> cases = {
        {"negative mass", inertial("-1", R"(ixx="0.1" iyy="0.1" izz="0.1")")},
        {"not positive semidefinite", inertial("1", R"(ixx="-0.1" iyy="0.1" izz="0.1")")},
        {"triangle inequality", inertial("1", R"(ixx="0.1" iyy="0.1" izz="0.5")")},
        {"inertia without mass", inertial("0", R"(ixx="0.1" iyy="0.1" izz="0.1")")},
        // no "nan"/"inf" case: urdfdom does not parse them as numbers, only logs an error and hands
        // out an all-zero <inertial>, which is indistinguishable from a massless link
    };
    for (const auto & c : cases) {
        SCOPED_TRACE(c.first);
        robotarm_rbd::RobotarmRbd rbd;
        EXPECT_FALSE(rbd.initialize(two_joint_urdf("0 1 0", link1_inertial_default, c.second), {}));
        EXPECT_NE(std::string(rbd.last_error()).find("link2"), std::string::npos) << rbd.last_error();
    }

    // limiting cases that are still physical: a point mass, a massless link, a thin rod (I_a + I_b = I_c)
    for (const auto & ok : {inertial("1", R"(ixx="0" iyy="0" izz="0")"),
            inertial("0", R"(ixx="0" iyy="0" izz="0")"),
            inertial("1", R"(ixx="0" iyy="0.1" izz="0.1")")})
    {
        robotarm_rbd::RobotarmRbd rbd;
        EXPECT_TRUE(rbd.initialize(two_joint_urdf("0 1 0", link1_inertial_default, ok), {}))
            << rbd.last_error();
    }
}

TEST(RobotarmRbd, AMalformedUrdfFailsWithAMessage)
{
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize("<robot", {}));
    EXPECT_FALSE(rbd.initialised());
    EXPECT_STRNE(rbd.last_error(), "");
}

TEST(RobotarmRbd, AnInvalidLambdaFailsWithAMessage)
{
    for (const double lambda : {-0.1, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity()})
    {
        SCOPED_TRACE("lambda = " + std::to_string(lambda));
        robotarm_rbd::RobotarmRbd rbd;
        EXPECT_FALSE(rbd.initialize(two_joint_urdf(), {lambda}));
        EXPECT_FALSE(rbd.initialised());
        EXPECT_NE(std::string(rbd.last_error()).find("lambda"), std::string::npos) << rbd.last_error();
    }
}

TEST(RobotarmRbd, TheMessageNamesTheOffendingJoint)
{
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize(two_joint_urdf("0 0 0"), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("axis2"), std::string::npos) << rbd.last_error();
}

TEST(RobotarmRbd, FlangeAndTcpErrorsNameWhatIsMissing)
{
    robotarm_rbd::RobotarmRbd rbd;
    // no flange joint: the chain runs out
    EXPECT_FALSE(rbd.initialize(replace_all(two_joint_urdf(), "flange_joint", "wrist_joint"), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("flange_joint"), std::string::npos) << rbd.last_error();
    // the flange joint must be fixed
    EXPECT_FALSE(rbd.initialize(replace_all(two_joint_urdf(), R"("flange_joint" type="fixed")",
        R"("flange_joint" type="continuous")"), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("not fixed"), std::string::npos) << rbd.last_error();
    // its child must be the flange link
    EXPECT_FALSE(rbd.initialize(replace_all(two_joint_urdf(), "\"flange\"", "\"mount\""), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("mount"), std::string::npos) << rbd.last_error();
    // no tcp behind the flange
    EXPECT_FALSE(rbd.initialize(replace_all(two_joint_urdf(), "\"tcp\"", "\"tool0\""), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("tcp"), std::string::npos) << rbd.last_error();
}

TEST(RobotarmRbd, AFailedReinitialiseLeavesTheObjectUnusable)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {}));
    EXPECT_FALSE(rbd.initialize("<robot", {}));
    EXPECT_FALSE(rbd.initialised());
    std::vector<std::string> names;
    EXPECT_FALSE(rbd.get_joint_names(names));
}

TEST(RobotarmRbd, GettersFailBeforeInitialiseWithAMessage)
{
    robotarm_rbd::RobotarmRbd rbd;
    std::vector<std::string> names{"keep"};
    EXPECT_FALSE(rbd.get_joint_names(names));
    EXPECT_STREQ(rbd.last_error(), "Not initialised");
    EXPECT_EQ(names, std::vector<std::string>{"keep"});
}

// ---------------------------------------------------------------------------------------------
// inverse dynamics on models that can be solved by hand
// ---------------------------------------------------------------------------------------------

namespace
{

// Pendulum: one joint about y through the base origin, the CoM at (L, 0, 0) in the link frame, the
// tcp at the CoM, rotated by Rx(90°) (tcp y = link z). The tcp pose is split into the flange at
// (L/2, 0, 0) and the tcp at (L/2, 0, 0) rotated by Rx(90°) on the flange, so the dynamics have to
// compose both. tool is added behind the flange (its links hang on "flange").
constexpr double pend_m = 2.0;
constexpr double pend_L = 0.5;
constexpr double pend_Iyy = 0.03;  // about the CoM

const std::string pendulum_inertial = R"(
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" iyy="0.03" izz="0.025" ixy="0" ixz="0" iyz="0"/>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
    </inertial>)";

std::string pendulum_urdf(const std::string & tool = "", const std::string & link1_inertial = pendulum_inertial)
{
    return R"(<robot name="pendulum">
  <link name="base"/>
  <link name="link1">)" + link1_inertial + R"(
  </link>
  <link name="flange"/>
  <link name="tcp"/>
  <joint name="axis1" type="revolute">
    <parent link="base"/> <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/> <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/>
  </joint>
  <joint name="flange_joint" type="fixed">
    <parent link="link1"/> <child link="flange"/>
    <origin xyz="0.25 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="flange"/> <child link="tcp"/>
    <origin xyz="0.25 0 0" rpy="1.5707963267948966 0 0"/>
  </joint>)" + tool + R"(
</robot>)";
}

// A tool link with an <inertial> (com in the tool link frame, tensor diagonal) on a fixed joint
// with the given origin. parent is "flange" or another tool link.
std::string tool_link(
    const std::string & name, const std::string & parent, const std::string & joint_type,
    const std::string & xyz, const std::string & rpy,
    const std::string & mass, const std::string & com, const std::string & diag)
{
    std::string inertial;
    if (!mass.empty()) {
        inertial = R"(<inertial><mass value=")" + mass + R"("/><inertia )" + diag +
                   R"( ixy="0" ixz="0" iyz="0"/><origin xyz=")" + com + R"(" rpy="0 0 0"/></inertial>)";
    }
    std::string extra;
    if (joint_type != "fixed") {
        extra = R"(<axis xyz="0 1 0"/><limit lower="0" upper="0.02" velocity="0.1" effort="10"/>)";
    }
    return R"(
  <link name=")" + name + R"(">)" + inertial + R"(</link>
  <joint name=")" + name + R"(_joint" type=")" + joint_type + R"(">
    <parent link=")" + parent + R"("/> <child link=")" + name + R"("/>
    <origin xyz=")" + xyz + R"(" rpy=")" + rpy + R"("/>)" + extra + R"(
  </joint>)";
}

// Steiner (parallel axis) term about a point at offset d from the CoM, written out on its own here
Eigen::Matrix3d parallel_axis(double m, const Eigen::Vector3d & d)
{
    Eigen::Matrix3d S;
    S << d.y() * d.y() + d.z() * d.z(), -d.x() * d.y(), -d.x() * d.z(),
         -d.x() * d.y(), d.x() * d.x() + d.z() * d.z(), -d.y() * d.z(),
         -d.x() * d.z(), -d.y() * d.z(), d.x() * d.x() + d.y() * d.y();
    return m * S;
}

Eigen::VectorXd vec1(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

}  // namespace

TEST(RecursiveNewtonEuler, PendulumMatchesTheEquationOfMotion)
{
    // Rotating by q about +y moves the CoM to (L cos q, 0, -L sin q), so the potential energy is
    // V = -m g L sin q (g = 9.81 downwards) and dV/dq = -m g L cos q. Lagrange with the inertia about
    // the axis I_yy + m L² (Steiner):   tau = (I_yy + m L²) q̈ - m g L cos q
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(), {})) << rbd.last_error();

    const double g = 9.81;
    Eigen::VectorXd tau(1);
    for (const double q : {-2.0, -0.7, 0.0, 0.4, 1.2, 2.9}) {
        for (const double dq : {0.0, -1.5, 3.0}) {
            for (const double ddq : {0.0, 2.0, -4.0}) {
                SCOPED_TRACE("q " + std::to_string(q) + " dq " + std::to_string(dq) + " ddq " +
                    std::to_string(ddq));
                ASSERT_TRUE(rbd.recursive_newton_euler(vec1(q), vec1(dq), vec1(ddq), tau))
                    << rbd.last_error();
                // one joint: no Coriolis term, dq must not change the result
                const double expected =
                    (pend_Iyy + pend_m * pend_L * pend_L) * ddq - pend_m * g * pend_L * std::cos(q);
                EXPECT_NEAR(tau[0], expected, 1e-12);
            }
        }
    }
}

TEST(RecursiveNewtonEuler, PendulumTcpWrenchIsExpressedInTheTcpFrame)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(), {})) << rbd.last_error();
    const Eigen::Vector3d no_gravity = Eigen::Vector3d::Zero();
    Eigen::VectorXd tau(1);

    // The robot pushes on the environment with F along tcp y = link z at the lever (L, 0, 0):
    // moment of that force about the joint (L,0,0) × (0,0,F) = (0, -L F, 0), which the motor has to
    // deliver. Independent of q, the wrench is carried along with the link.
    const double F = 7.0;
    for (const double q : {0.0, 0.8}) {
        ASSERT_TRUE(rbd.recursive_newton_euler(vec1(q), vec1(0), vec1(0), tau,
            Eigen::Vector3d(0, F, 0), Eigen::Vector3d::Zero(), no_gravity)) << rbd.last_error();
        EXPECT_NEAR(tau[0], -pend_L * F, 1e-12);
    }

    // a pure moment about tcp z = link -y goes straight through: tau = -M
    const double M = 1.5;
    ASSERT_TRUE(rbd.recursive_newton_euler(vec1(0.3), vec1(0), vec1(0), tau,
        Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, M), no_gravity)) << rbd.last_error();
    EXPECT_NEAR(tau[0], -M, 1e-12);

    // force along the lever (tcp x = link x) needs no torque
    ASSERT_TRUE(rbd.recursive_newton_euler(vec1(0.3), vec1(0), vec1(0), tau,
        Eigen::Vector3d(F, 0, 0), Eigen::Vector3d::Zero(), no_gravity)) << rbd.last_error();
    EXPECT_NEAR(tau[0], 0.0, 1e-12);
}

TEST(RecursiveNewtonEuler, AVerticalFirstAxisCarriesNoGravityTorque)
{
    // axis1 is vertical: gravity has no moment about it, whatever the pose
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    Eigen::VectorXd tau(2);
    for (const double q2 : {-0.9, 0.0, 0.6}) {
        ASSERT_TRUE(rbd.recursive_newton_euler(Eigen::Vector2d(0.4, q2), Eigen::Vector2d::Zero(),
            Eigen::Vector2d::Zero(), tau)) << rbd.last_error();
        EXPECT_NEAR(tau[0], 0.0, 1e-12);
        EXPECT_GT(std::abs(tau[1]), 0.1) << "link2 hangs off axis2, which is horizontal";
    }
}

// ---------------------------------------------------------------------------------------------
// tool behind the flange: its inertia is folded into the last moving link, by hand on the pendulum
// ---------------------------------------------------------------------------------------------

namespace
{

// A pendulum about y whose bodies all lie in the plane z = 0 of the link: like
// PendulumMatchesTheEquationOfMotion, tau = I_axis q̈ − (Σ m_k x_k) g cos q with
// I_axis = Σ (I_yy,k + m_k x_k²). An offset in y does not change either term.
void expect_pendulum_torques(robotarm_rbd::RobotarmRbd & rbd, double I_axis, double m_x)
{
    Eigen::VectorXd tau(1);
    for (const double q : {-2.0, 0.0, 0.4, 1.2}) {
        for (const double ddq : {0.0, 2.0, -4.0}) {
            SCOPED_TRACE("q " + std::to_string(q) + " ddq " + std::to_string(ddq));
            ASSERT_TRUE(rbd.recursive_newton_euler(vec1(q), vec1(1.5), vec1(ddq), tau)) << rbd.last_error();
            EXPECT_NEAR(tau[0], I_axis * ddq - m_x * 9.81 * std::cos(q), 1e-12);
        }
    }
}

const std::string zero_tensor = R"(ixx="0" iyy="0" izz="0")";

}  // namespace

TEST(ToolFolding, ARotatedToolBodyIsFoldedIntoTheLastMovingLink)
{
    // tool_base 0.75 m beyond the flange, so at (1, 0, 0) in link1, rotated by Rz(90°): its CoM
    // (0.1, 0, 0) is (1, 0.1, 0) in link1, its tensor diag(a, b, c) becomes diag(b, a, c)
    const std::string tool = tool_link("tool_base", "flange", "fixed", "0.75 0 0",
        "0 0 1.5707963267948966", "1.0", "0.1 0 0", R"(ixx="0.004" iyy="0.002" izz="0.003")");
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(tool), {})) << rbd.last_error();
    ASSERT_EQ(rbd.joints_.size(), 1u);
    const auto & in = rbd.joints_[0].inertia_;

    const Eigen::Vector3d c1(0.5, 0, 0), ct(1.0, 0.1, 0);
    EXPECT_TRUE(in.valid);
    EXPECT_NEAR(in.mass, 3.0, 1e-15);
    EXPECT_LT((in.com - (2.0 * c1 + 1.0 * ct) / 3.0).norm(), 1e-15) << in.com.transpose();

    // about the combined CoM: every body's own tensor in link1 axes plus its parallel axis term
    const Eigen::Matrix3d I_expected =
        Eigen::Vector3d(0.01, 0.03, 0.025).asDiagonal().toDenseMatrix() + parallel_axis(2.0, c1 - in.com) +
        Eigen::Vector3d(0.002, 0.004, 0.003).asDiagonal().toDenseMatrix() + parallel_axis(1.0, ct - in.com);
    EXPECT_LT((in.com_inertia_in_joint - I_expected).cwiseAbs().maxCoeff(), 1e-14)
        << "ours\n" << in.com_inertia_in_joint << "\nexpected\n" << I_expected;

    // and the motion it takes: I_yy of the tool is its ixx after the rotation
    expect_pendulum_torques(rbd, 0.03 + 2.0 * 0.25 + 0.004 + 1.0 * 1.0, 2.0 * 0.5 + 1.0 * 1.0);
}

TEST(ToolFolding, ABranchedToolIsFoldedWithAMovableJawAtItsZeroPosition)
{
    // massless tool_base at (1, 0, 0) in link1, two jaws as point masses 0.05 m to either side.
    // jaw1 is prismatic (along y), taken at zero: it is no joint of the model.
    const std::string tool =
        tool_link("tool_base", "flange", "fixed", "0.75 0 0", "0 0 0", "", "", "") +
        tool_link("jaw1", "tool_base", "prismatic", "0 0.05 0", "0 0 0", "0.5", "0 0 0", zero_tensor) +
        tool_link("jaw2", "tool_base", "fixed", "0 -0.05 0", "0 0 0", "0.5", "0 0 0", zero_tensor);
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(tool), {})) << rbd.last_error();

    std::vector<std::string> names;
    ASSERT_TRUE(rbd.get_joint_names(names));
    EXPECT_EQ(names, std::vector<std::string>{"axis1"});

    const auto & in = rbd.joints_[0].inertia_;
    EXPECT_NEAR(in.mass, 3.0, 1e-15);
    EXPECT_LT((in.com - Eigen::Vector3d(2.0 / 3.0, 0, 0)).norm(), 1e-15) << in.com.transpose();
    expect_pendulum_torques(rbd, 0.03 + 2.0 * 0.25 + 0.5 * 1.0 + 0.5 * 1.0, 2.0 * 0.5 + 0.5 + 0.5);
}

TEST(ToolFolding, AToolWithoutInertialChangesNothing)
{
    const std::string tool = tool_link("tool_base", "flange", "fixed", "0.75 0 0", "0 0 1", "", "", "");
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(tool), {})) << rbd.last_error();
    const auto & in = rbd.joints_[0].inertia_;
    EXPECT_EQ(in.mass, pend_m);
    EXPECT_LT((in.com - Eigen::Vector3d(pend_L, 0, 0)).norm(), 1e-15);
    EXPECT_LT((in.com_inertia_in_joint - Eigen::Vector3d(0.01, 0.03, 0.025).asDiagonal().toDenseMatrix())
        .cwiseAbs().maxCoeff(), 1e-15);
    expect_pendulum_torques(rbd, pend_Iyy + pend_m * pend_L * pend_L, pend_m * pend_L);
}

TEST(ToolFolding, OnAKinematicsOnlyArmTheToolInertiaIsIgnored)
{
    // folding it in would give the last link a mass while the others have none (all or none)
    const std::string tool = tool_link("tool_base", "flange", "fixed", "0.75 0 0", "0 0 0", "1.0",
        "0 0 0", R"(ixx="0.001" iyy="0.001" izz="0.001")");
    test_utils::TestableRbd rbd;
    ASSERT_TRUE(rbd.initialize(pendulum_urdf(tool, ""), {})) << rbd.last_error();
    EXPECT_FALSE(rbd.joints_[0].inertia_.valid);
    EXPECT_EQ(rbd.joints_[0].inertia_.mass, 0.0);
    Eigen::VectorXd tau(1);
    EXPECT_FALSE(rbd.recursive_newton_euler(vec1(0), vec1(0), vec1(0), tau));
    EXPECT_STREQ(rbd.last_error(), "Not all links have a valid inertia");
}

TEST(ToolFolding, AnImplausibleToolInertialIsRejected)
{
    const std::string tool = tool_link("tool_base", "flange", "fixed", "0.75 0 0", "0 0 0", "-1.0",
        "0 0 0", R"(ixx="0.001" iyy="0.001" izz="0.001")");
    robotarm_rbd::RobotarmRbd rbd;
    EXPECT_FALSE(rbd.initialize(pendulum_urdf(tool), {}));
    EXPECT_NE(std::string(rbd.last_error()).find("tool_base"), std::string::npos) << rbd.last_error();
}

TEST(ToolFolding, ATcpOnTheToolTakesTheWrenchAtItsLeverArm)
{
    // the tcp moved from the flange onto a massless tool_base at (1, 0, 0) in link1: it sits at
    // (1.25, 0, 0) with tcp y = link z, so a force F along tcp y needs tau = -1.25 F
    const std::string tool = tool_link("tool_base", "flange", "fixed", "0.75 0 0", "0 0 0", "", "", "");
    const std::string urdf = replace_all(pendulum_urdf(tool), R"(<parent link="flange"/> <child link="tcp"/>)",
        R"(<parent link="tool_base"/> <child link="tcp"/>)");
    ASSERT_NE(urdf, pendulum_urdf(tool)) << "test premise: the tcp joint was moved";

    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(urdf, {})) << rbd.last_error();
    Eigen::VectorXd tau(1);
    const double F = 7.0;
    ASSERT_TRUE(rbd.recursive_newton_euler(vec1(0.8), vec1(0), vec1(0), tau, Eigen::Vector3d(0, F, 0),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())) << rbd.last_error();
    EXPECT_NEAR(tau[0], -1.25 * F, 1e-12);

    // and the tcp pose itself
    Eigen::Isometry3d T;
    ASSERT_TRUE(rbd.calculate_link_transform(vec1(0.0), "tcp", T));
    EXPECT_LT((T.translation() - Eigen::Vector3d(1.25, 0, 0)).norm(), 1e-15);
    EXPECT_LT((T.linear() * Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitZ()).norm(), 1e-15);
}

TEST(RecursiveNewtonEuler, WithoutInertiaOnlyThePureTcpWrenchIsAllowed)
{
    robotarm_rbd::RobotarmRbd rbd;
    ASSERT_TRUE(rbd.initialize(two_joint_urdf("0 1 0", "", ""), {})) << rbd.last_error();
    const Eigen::VectorXd zero = Eigen::Vector2d::Zero();
    const Eigen::VectorXd one = Eigen::Vector2d::Ones();
    const Eigen::Vector3d no_gravity = Eigen::Vector3d::Zero();
    const Eigen::Vector3d F(0, 0, 0), M(0, 0, 2.0);

    // anything that needs a mass fails and leaves tau untouched
    Eigen::VectorXd tau = Eigen::Vector2d(42, 42);
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, zero, zero, tau));  // default gravity
    EXPECT_STREQ(rbd.last_error(), "Not all links have a valid inertia");
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, one, zero, tau, F, M, no_gravity));
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, zero, one, tau, F, M, no_gravity));
    EXPECT_EQ(tau, Eigen::VectorXd(Eigen::Vector2d(42, 42)));

    // the pure wrench mapping does not depend on the masses: moment about tcp z = axis1 z
    ASSERT_TRUE(rbd.recursive_newton_euler(zero, zero, zero, tau, F, M, no_gravity)) << rbd.last_error();
    EXPECT_NEAR(tau[0], 2.0, 1e-12);
    EXPECT_NEAR(tau[1], 0.0, 1e-12);
}

TEST(RecursiveNewtonEuler, InvalidInputFailsWithAMessageNamingIt)
{
    robotarm_rbd::RobotarmRbd rbd;
    Eigen::VectorXd tau = Eigen::Vector2d(42, 42);
    const Eigen::VectorXd zero = Eigen::Vector2d::Zero();
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, zero, zero, tau));
    EXPECT_STREQ(rbd.last_error(), "Not initialised");

    ASSERT_TRUE(rbd.initialize(two_joint_urdf(), {})) << rbd.last_error();
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, Eigen::Vector3d::Zero(), zero, tau));
    EXPECT_STREQ(rbd.last_error(), "Unexpected dq dimension");
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, zero, Eigen::Vector2d(0, NAN), tau));
    EXPECT_STREQ(rbd.last_error(), "ddq contains NaN or inf");
    EXPECT_FALSE(rbd.recursive_newton_euler(zero, zero, zero, tau, Eigen::Vector3d(INFINITY, 0, 0)));
    EXPECT_EQ(tau, Eigen::VectorXd(Eigen::Vector2d(42, 42)));
}
