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

// two revolute joints and the tcp; axis2 is passed in so a test can break it. link1_inertial and
// link2_inertial replace the <inertial> of the links ("" leaves it out).
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
  <joint name="tcp_joint" type="fixed">
    <parent link="link2"/> <child link="tcp"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>)";
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
// tcp at the CoM, rotated by Rx(90°) (tcp y = link z).
constexpr double pend_m = 2.0;
constexpr double pend_L = 0.5;
constexpr double pend_Iyy = 0.03;  // about the CoM

std::string pendulum_urdf()
{
    return R"(<robot name="pendulum">
  <link name="base"/>
  <link name="link1">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" iyy="0.03" izz="0.025" ixy="0" ixz="0" iyz="0"/>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
    </inertial>
  </link>
  <link name="tcp"/>
  <joint name="axis1" type="revolute">
    <parent link="base"/> <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/> <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="link1"/> <child link="tcp"/>
    <origin xyz="0.5 0 0" rpy="1.5707963267948966 0 0"/>
  </joint>
</robot>)";
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
