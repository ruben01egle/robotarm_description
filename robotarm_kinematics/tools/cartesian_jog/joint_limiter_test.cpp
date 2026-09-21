// Tests for limit_joint_step(): the joint limit policy of the Cartesian jog node.

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "JointLimiter.hpp"

namespace
{

using robotarm_kinematics::JointLimit;
using robotarm_kinematics::limit_joint_step;
using robotarm_kinematics::StepLimit;

// three joints: [-1, 1] rad, 2 rad/s / [-2, 0.5] rad, 1 rad/s / [0, 3] rad, 0.5 rad/s
std::vector<JointLimit> limits()
{
    return {{-1.0, 1.0, 2.0}, {-2.0, 0.5, 1.0}, {0.0, 3.0, 0.5}};
}

Eigen::VectorXd vec(double a, double b, double c)
{
    Eigen::VectorXd v(3);
    v << a, b, c;
    return v;
}

constexpr double kDt = 0.01;

TEST(JointLimiter, ASmallStepInsideAllLimitsIsUnchanged)
{
    const auto r = limit_joint_step(vec(0, 0, 1), vec(0.001, -0.001, 0.001), limits(), kDt);
    EXPECT_EQ(r.scale, 1.0);
    EXPECT_EQ(r.reason, StepLimit::None);
    EXPECT_EQ(r.joint, -1);
}

TEST(JointLimiter, ZeroStepIsUnchanged)
{
    const auto r = limit_joint_step(vec(0, 0, 1), vec(0, 0, 0), limits(), kDt);
    EXPECT_EQ(r.scale, 1.0);
    EXPECT_EQ(r.reason, StepLimit::None);
}

TEST(JointLimiter, VelocityLimitScalesTheWholeStepAndKeepsItsDirection)
{
    // joint 3 wants 0.02 rad in 0.01 s = 2 rad/s but may only do 0.5 rad/s -> factor 0.25
    const Eigen::VectorXd dq = vec(0.004, -0.002, 0.02);
    const auto r = limit_joint_step(vec(0, 0, 1), dq, limits(), kDt);
    EXPECT_NEAR(r.scale, 0.25, 1e-15);
    EXPECT_EQ(r.reason, StepLimit::Velocity);
    EXPECT_EQ(r.joint, 2);

    // the limiting joint runs exactly at its limit, all others are within theirs
    const Eigen::VectorXd limited = r.scale * dq;
    EXPECT_NEAR(std::abs(limited[2]) / kDt, 0.5, 1e-12);
    EXPECT_LE(std::abs(limited[0]) / kDt, 2.0);
    EXPECT_LE(std::abs(limited[1]) / kDt, 1.0);
}

TEST(JointLimiter, ATargetBeyondAPositionLimitEndsExactlyAtTheLimit)
{
    // joint 1 is 0.005 rad below its upper limit and wants 0.02 rad
    const Eigen::VectorXd q = vec(0.995, 0, 1);
    const Eigen::VectorXd dq = vec(0.02, -0.001, 0.001);
    const auto r = limit_joint_step(q, dq, limits(), 1.0);  // large dt: velocity does not limit
    EXPECT_EQ(r.reason, StepLimit::Position);
    EXPECT_EQ(r.joint, 0);
    const Eigen::VectorXd after = q + r.scale * dq;
    EXPECT_NEAR(after[0], 1.0, 1e-12);
    // nothing else left its range
    EXPECT_GE(after[1], -2.0);
    EXPECT_LE(after[2], 3.0);
}

TEST(JointLimiter, ALowerLimitWorksTheSameWay)
{
    const Eigen::VectorXd q = vec(0, -1.99, 1);
    const Eigen::VectorXd dq = vec(0, -0.05, 0);
    const auto r = limit_joint_step(q, dq, limits(), 1.0);
    EXPECT_EQ(r.reason, StepLimit::Position);
    EXPECT_EQ(r.joint, 1);
    EXPECT_NEAR((q + r.scale * dq)[1], -2.0, 1e-12);
}

TEST(JointLimiter, AJointAtItsLimitMovingFurtherOutStopsTheWholeStep)
{
    const auto r = limit_joint_step(vec(1.0, 0, 1), vec(0.001, 0.001, 0.001), limits(), kDt);
    EXPECT_EQ(r.scale, 0.0);
    EXPECT_EQ(r.reason, StepLimit::Position);
    EXPECT_EQ(r.joint, 0);
}

TEST(JointLimiter, MovingBackAwayFromALimitIsAllowed)
{
    const auto r = limit_joint_step(vec(1.0, 0, 1), vec(-0.001, 0.001, 0.001), limits(), kDt);
    EXPECT_EQ(r.scale, 1.0);
    EXPECT_EQ(r.reason, StepLimit::None);
}

TEST(JointLimiter, ABeyondTheLimitJointMayOnlyMoveBackIn)
{
    // joint 1 is outside (1.05 > 1.0), e.g. after a bad initial pose
    EXPECT_EQ(limit_joint_step(vec(1.05, 0, 1), vec(0.001, 0, 0), limits(), kDt).scale, 0.0);
    EXPECT_EQ(limit_joint_step(vec(1.05, 0, 1), vec(-0.001, 0, 0), limits(), kDt).scale, 1.0);
}

TEST(JointLimiter, TheTightestOfSeveralLimitsWins)
{
    // joint 1 would allow 0.5, joint 2 only 0.1 of the step, joint 3 has plenty of room
    const Eigen::VectorXd q = vec(0.9, 0.45, 1.0);
    const Eigen::VectorXd dq = vec(0.2, 0.5, 0.01);
    const auto r = limit_joint_step(q, dq, limits(), 1.0);
    EXPECT_NEAR(r.scale, 0.1, 1e-12);
    EXPECT_EQ(r.reason, StepLimit::Position);
    EXPECT_EQ(r.joint, 1);
}

TEST(JointLimiter, NonFiniteInputsAndBadArgumentsGiveAZeroStep)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Eigen::VectorXd q = vec(0, 0, 1), dq = vec(0.001, 0.001, 0.001);

    for (const auto & r : {
            limit_joint_step(q, vec(nan, 0, 0), limits(), kDt),
            limit_joint_step(q, vec(0, inf, 0), limits(), kDt),
            limit_joint_step(vec(0, nan, 1), dq, limits(), kDt),
            limit_joint_step(q, dq, limits(), 0.0),
            limit_joint_step(q, dq, limits(), -0.01),
            limit_joint_step(q, dq, limits(), nan),
            limit_joint_step(Eigen::VectorXd::Zero(2), dq, limits(), kDt),
            limit_joint_step(q, Eigen::VectorXd::Zero(4), limits(), kDt)})
    {
        EXPECT_EQ(r.scale, 0.0);
        EXPECT_EQ(r.reason, StepLimit::Invalid);
    }
}

TEST(JointLimiter, InvalidLimitsGiveAZeroStepInsteadOfNoLimiting)
{
    const Eigen::VectorXd q = vec(0, 0, 1), dq = vec(0.001, 0.001, 0.001);
    auto bad = limits();
    bad[1].velocity = 0.0;
    EXPECT_EQ(limit_joint_step(q, dq, bad, kDt).scale, 0.0);
    bad = limits();
    bad[0].min = 2.0;  // min > max
    EXPECT_EQ(limit_joint_step(q, dq, bad, kDt).scale, 0.0);
    bad = limits();
    bad[2].max = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(limit_joint_step(q, dq, bad, kDt).scale, 0.0);
}

}  // namespace
