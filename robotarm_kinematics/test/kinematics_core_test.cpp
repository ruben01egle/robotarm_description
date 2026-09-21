// Tests for KinematicsCore, in order of importance:
//   1. URDF -> DH parsing: recovered DH parameters, and every link transform against an
//      independent reference FK built from the URDF itself (real robot + synthetic robots)
//   2. initialize() rejects every URDF that is not DH-conform / valid
//   3. Jacobian against finite differences of the reference FK
//   4. delta conversions: consistency with FK, round-trip accuracy of the damped inverse,
//      behaviour at a singularity, input validation
// The allocation check lives in kinematics_core_malloc_test.cpp (it needs a special build).

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_utils.hpp"

namespace
{

using test_utils::Jacobian;
using test_utils::JacobianInverse;
using test_utils::Vector6;
using test_utils::TestableCore;
using test_utils::UrdfSpec;

// The real URDF stores pi/2 as 1.5707963, which makes some translations a few 1e-10 m "not in the
// DH plane"; the plugin drops that part, the reference FK keeps it. 1e-8 m is far below any real
// error (a wrong frame or sign is off by millimetres or more).
constexpr double kPositionTol = 1e-8;
constexpr double kRotationTol = 1e-9;

std::string urdf_for(const std::string & robot)
{
    if (robot == "real") {
        return test_utils::real_urdf();
    }
    if (robot == "baseline") {
        return test_utils::make_spec(test_utils::robot_dh()).str();
    }
    if (robot == "generic") {
        return test_utils::make_spec(test_utils::generic_dh()).str();
    }
    throw std::invalid_argument("unknown test robot " + robot);
}

// ---------------------------------------------------------------------------------------------
// tests that run on all robots: the real one, one with the same DH table, and a generic one
// ---------------------------------------------------------------------------------------------

class RobotTest : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override
    {
        try {
            urdf_ = urdf_for(GetParam());
            ref_ = std::make_unique<test_utils::ReferenceFk>(urdf_);
        } catch (const std::exception & e) {
            FAIL() << e.what();
        }
        ASSERT_TRUE(test_utils::initialize(core_, urdf_));
        rng_.seed(12345);
    }

    Eigen::VectorXd random_q() {return ref_->random_q(rng_);}

    TestableCore core_;
    std::string urdf_;
    std::unique_ptr<test_utils::ReferenceFk> ref_;
    std::mt19937 rng_;
};

INSTANTIATE_TEST_SUITE_P(
    Robots, RobotTest, ::testing::Values("real", "baseline", "generic"),
    [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

// --- 1. forward kinematics ---------------------------------------------------------------------

TEST_P(RobotTest, LinkTransformsMatchTheUrdfReference)
{
    // root link, all six links and the tcp, at q = 0 and at random configurations
    for (int n = 0; n < 50; ++n) {
        const Eigen::VectorXd q = n == 0 ? Eigen::VectorXd::Zero(6) : random_q();
        for (const std::string & link : ref_->link_names()) {
            SCOPED_TRACE("link " + link + ", q = " + std::to_string(n));
            Eigen::Isometry3d T;
            ASSERT_TRUE(core_.calculate_link_transform(q, link, T));
            const Eigen::Isometry3d R = ref_->link(q, link);
            EXPECT_LT((T.translation() - R.translation()).norm(), kPositionTol);
            EXPECT_LT(Eigen::AngleAxisd(R.linear().transpose() * T.linear()).angle(), kRotationTol);
        }
    }
}

TEST_P(RobotTest, BaseLinkIsTheIdentityWithAZeroJacobian)
{
    const Eigen::VectorXd q = random_q();
    const std::string base = ref_->link_names().front();
    Eigen::Isometry3d T;
    ASSERT_TRUE(core_.calculate_link_transform(q, base, T));
    EXPECT_TRUE(T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-15));
    Jacobian J;
    ASSERT_TRUE(core_.calculate_jacobian(q, base, J));
    EXPECT_EQ(J.cols(), 6);
    EXPECT_EQ(J.norm(), 0.0);
}

TEST_P(RobotTest, UnknownLinkFailsAndLeavesOutputsUntouched)
{
    const Eigen::VectorXd q = random_q();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(1, 2, 3);
    const Eigen::Isometry3d T0 = T;
    EXPECT_FALSE(core_.calculate_link_transform(q, "no_such_link", T));
    EXPECT_TRUE(T.matrix() == T0.matrix());

    Jacobian J = Jacobian::Constant(6, 6, 42.0);
    EXPECT_FALSE(core_.calculate_jacobian(q, "no_such_link", J));
    EXPECT_TRUE((J.array() == 42.0).all());

    JacobianInverse Ji = JacobianInverse::Constant(6, 6, 42.0);
    EXPECT_FALSE(core_.calculate_jacobian_inverse(q, "no_such_link", Ji));
    EXPECT_TRUE((Ji.array() == 42.0).all());

    Vector6 dx = Vector6::Constant(42.0);
    EXPECT_FALSE(core_.convert_joint_deltas_to_cartesian_deltas(q, Eigen::VectorXd::Zero(6), "no_such_link", dx));
    EXPECT_TRUE((dx.array() == 42.0).all());

    Eigen::VectorXd dq = Eigen::VectorXd::Constant(6, 42.0);
    EXPECT_FALSE(core_.convert_cartesian_deltas_to_joint_deltas(q, Vector6::Zero(), "no_such_link", dq));
    EXPECT_TRUE((dq.array() == 42.0).all());
}

// --- 3. Jacobian -------------------------------------------------------------------------------

TEST_P(RobotTest, JacobianMatchesFiniteDifferencesOfTheReferenceFk)
{
    // Central differences of the reference FK for every link (also the base link, whose Jacobian
    // is zero, and links in the middle of the chain, whose later columns must be zero).
    // Linear rows: d position / dq_i. Angular rows: rotation vector of R(q + h) R(q - h)^T / 2h.
    const double h = 1e-6;
    for (int n = 0; n < 20; ++n) {
        const Eigen::VectorXd q = random_q();
        for (const std::string & link : ref_->link_names()) {
            Jacobian J;
            ASSERT_TRUE(core_.calculate_jacobian(q, link, J));
            ASSERT_EQ(J.rows(), 6);
            ASSERT_EQ(J.cols(), 6);
            for (Eigen::Index i = 0; i < 6; ++i) {
                SCOPED_TRACE("link " + link + ", column " + std::to_string(i));
                Eigen::VectorXd qp = q, qm = q;
                qp[i] += h;
                qm[i] -= h;
                const Eigen::Isometry3d Tp = ref_->link(qp, link);
                const Eigen::Isometry3d Tm = ref_->link(qm, link);
                const Eigen::Vector3d linear = (Tp.translation() - Tm.translation()) / (2 * h);
                const Eigen::Vector3d angular =
                    test_utils::rotation_vector(Tp.linear() * Tm.linear().transpose()) / (2 * h);
                EXPECT_LT((J.col(i).head<3>() - linear).norm(), 1e-6);
                EXPECT_LT((J.col(i).tail<3>() - angular).norm(), 1e-6);
            }
        }
    }
}

// --- 4. delta conversions ----------------------------------------------------------------------

TEST_P(RobotTest, JointDeltasToCartesianAreTheJacobianProductAndFollowTheFkToFirstOrder)
{
    const std::string tcp = core_.tcp_link_name_;
    for (int n = 0; n < 20; ++n) {
        const Eigen::VectorXd q = random_q();
        const Eigen::VectorXd dq = test_utils::random_vector(rng_, 6, 1e-6);

        Vector6 dx;
        ASSERT_TRUE(core_.convert_joint_deltas_to_cartesian_deltas(q, dq, tcp, dx));

        Jacobian J;
        ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J));
        EXPECT_LT((dx - J * dq).norm(), 1e-12);

        // FK(q + dq) - FK(q): the error of the linearisation is O(|dq|^2) ~ 1e-12
        const Eigen::Isometry3d T0 = ref_->link(q, tcp);
        const Eigen::Isometry3d T1 = ref_->link(q + dq, tcp);
        EXPECT_LT((dx.head<3>() - (T1.translation() - T0.translation())).norm(), 1e-10);
        EXPECT_LT(
            (dx.tail<3>() - test_utils::rotation_vector(T1.linear() * T0.linear().transpose())).norm(),
            1e-10);
    }
}

TEST_P(RobotTest, RoundTripErrorStaysWithinTheDampingBound)
{
    // The damped inverse is J+ = V S (S^2 + l^2)^-1 U^T for J = U S V^T, so
    //   J+ J = V diag(s^2 / (s^2 + l^2)) V^T   and   J J+ = U diag(s^2 / (s^2 + l^2)) U^T.
    // Hence for both round trips (dq -> dx -> dq and dx -> dq -> dx, J is 6x6):
    //   |result - input| <= l^2 / (s_min^2 + l^2) * |input|
    // with s_min the smallest singular value of the Jacobian at q. A bug in the Jacobian, its
    // inverse or the products breaks this; a wrong / ignored lambda does as well.
    const std::string tcp = core_.tcp_link_name_;
    for (const double lambda : {1e-6, 1e-2, 5e-2}) {
        SCOPED_TRACE("lambda = " + std::to_string(lambda));
        ASSERT_TRUE(test_utils::initialize(core_, urdf_, test_utils::lambda_param(lambda)));

        int accepted = 0;
        for (int n = 0; n < 1000 && accepted < 30; ++n) {
            const Eigen::VectorXd q = random_q();
            Jacobian J;
            ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J));
            const double s_min = Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues().minCoeff();
            if (s_min < 0.03) {
                continue;  // near a singularity, the bound is loose and says little
            }
            ++accepted;
            const double bound = lambda * lambda / (s_min * s_min + lambda * lambda);
            const double slack = 1e-6 * bound + 1e-10;  // floating point noise of the LDLT solve

            // dq -> dx -> dq
            const Eigen::VectorXd dq = test_utils::random_vector(rng_, 6, 1e-3);
            Vector6 dx;
            Eigen::VectorXd dq_back;
            ASSERT_TRUE(core_.convert_joint_deltas_to_cartesian_deltas(q, dq, tcp, dx));
            ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_back));
            const double joint_error = (dq_back - dq).norm() / dq.norm();
            EXPECT_LE(joint_error, bound + slack);
            if (lambda == 1e-6) {
                EXPECT_LT(joint_error, 1e-8);  // practically exact without damping
            }
            if (lambda == 5e-2) {
                EXPECT_GT(joint_error, 1e-6);  // the damping is really applied
            }

            // dx -> dq -> dx
            const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);
            Vector6 dx_back;
            ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx_in, tcp, dq_back));
            ASSERT_TRUE(core_.convert_joint_deltas_to_cartesian_deltas(q, dq_back, tcp, dx_back));
            EXPECT_LE((dx_back - dx_in).norm() / dx_in.norm(), bound + slack);
        }
        EXPECT_GE(accepted, 30) << "too few well-conditioned configurations found";
    }
}

TEST_P(RobotTest, InvalidInputsFailAndLeaveOutputsUntouched)
{
    const std::string tcp = core_.tcp_link_name_;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    const Eigen::VectorXd q = random_q();
    Eigen::VectorXd q_nan = q;
    q_nan[2] = nan;
    const Eigen::VectorXd q_short = Eigen::VectorXd::Zero(5);
    const Eigen::VectorXd q_long = Eigen::VectorXd::Zero(7);

    const Eigen::VectorXd dq = test_utils::random_vector(rng_, 6, 1e-3);
    Eigen::VectorXd dq_inf = dq;
    dq_inf[0] = inf;
    Eigen::VectorXd dq_nan = dq;
    dq_nan[5] = nan;
    const Eigen::VectorXd dq_short = Eigen::VectorXd::Zero(5);
    const Eigen::VectorXd dq_long = Eigen::VectorXd::Zero(7);

    const Vector6 dx = test_utils::random_vector(rng_, 6, 1e-3);
    Vector6 dx_nan = dx;
    dx_nan[4] = nan;
    Vector6 dx_inf = dx;
    dx_inf[1] = -inf;

    // joint_pos is validated by every function
    for (const Eigen::VectorXd & bad_q : {q_nan, q_short, q_long}) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation().setConstant(42.0);
        const Eigen::Isometry3d T0 = T;
        EXPECT_FALSE(core_.calculate_link_transform(bad_q, tcp, T));
        EXPECT_TRUE(T.matrix() == T0.matrix());

        Jacobian J = Jacobian::Constant(6, 6, 42.0);
        EXPECT_FALSE(core_.calculate_jacobian(bad_q, tcp, J));
        EXPECT_TRUE((J.array() == 42.0).all());

        JacobianInverse Ji = JacobianInverse::Constant(6, 6, 42.0);
        EXPECT_FALSE(core_.calculate_jacobian_inverse(bad_q, tcp, Ji));
        EXPECT_TRUE((Ji.array() == 42.0).all());

        Vector6 out_x = Vector6::Constant(42.0);
        EXPECT_FALSE(core_.convert_joint_deltas_to_cartesian_deltas(bad_q, dq, tcp, out_x));
        EXPECT_TRUE((out_x.array() == 42.0).all());

        Eigen::VectorXd out_q = Eigen::VectorXd::Constant(6, 42.0);
        EXPECT_FALSE(core_.convert_cartesian_deltas_to_joint_deltas(bad_q, dx, tcp, out_q));
        EXPECT_TRUE((out_q.array() == 42.0).all());
    }

    // delta_theta: size and finiteness
    for (const Eigen::VectorXd & bad_dq : {dq_inf, dq_nan, dq_short, dq_long}) {
        Vector6 out_x = Vector6::Constant(42.0);
        EXPECT_FALSE(core_.convert_joint_deltas_to_cartesian_deltas(q, bad_dq, tcp, out_x));
        EXPECT_TRUE((out_x.array() == 42.0).all());
    }

    // delta_x: finiteness
    for (const Vector6 & bad_dx : {dx_nan, dx_inf}) {
        Eigen::VectorXd out_q = Eigen::VectorXd::Constant(6, 42.0);
        EXPECT_FALSE(core_.convert_cartesian_deltas_to_joint_deltas(q, bad_dx, tcp, out_q));
        EXPECT_TRUE((out_q.array() == 42.0).all());
    }
}

TEST_P(RobotTest, UnsizedOutputsAreResizedAndGiveTheSameResultAsPresizedOnes)
{
    const std::string tcp = core_.tcp_link_name_;
    const Eigen::VectorXd q = random_q();
    const Vector6 dx = test_utils::random_vector(rng_, 6, 1e-3);

    Jacobian J_sized = Jacobian::Zero(6, 6), J_empty;
    ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J_sized));
    ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J_empty));
    EXPECT_TRUE(J_empty.rows() == 6 && J_empty.cols() == 6);
    EXPECT_TRUE(J_empty == J_sized);

    JacobianInverse Ji_sized = JacobianInverse::Zero(6, 6), Ji_empty;
    ASSERT_TRUE(core_.calculate_jacobian_inverse(q, tcp, Ji_sized));
    ASSERT_TRUE(core_.calculate_jacobian_inverse(q, tcp, Ji_empty));
    EXPECT_TRUE(Ji_empty.rows() == 6 && Ji_empty.cols() == 6);
    EXPECT_TRUE(Ji_empty == Ji_sized);

    Eigen::VectorXd dq_sized = Eigen::VectorXd::Zero(6), dq_empty, dq_wrong = Eigen::VectorXd::Zero(3);
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_sized));
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_empty));
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_wrong));
    EXPECT_EQ(dq_empty.size(), 6);
    EXPECT_EQ(dq_wrong.size(), 6);
    EXPECT_TRUE(dq_empty == dq_sized);
    EXPECT_TRUE(dq_wrong == dq_sized);
}

// --- damping at a singularity (baseline robot: wrist singular at q5 = 0) -------------------------

TEST(Singularity, DampingKeepsTheJointDeltasBoundedAtTheWristSingularity)
{
    const double lambda = 1e-2;
    TestableCore core;
    ASSERT_TRUE(test_utils::initialize(core, urdf_for("baseline"), test_utils::lambda_param(lambda)));

    Eigen::VectorXd q(6);
    q << 0.1, -0.5, 0.8, 0.2, 0.0, -0.3;  // q5 = 0 aligns axes 4 and 6
    Jacobian J;
    ASSERT_TRUE(core.calculate_jacobian(q, "tcp", J));
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU);
    const double s_min = svd.singularValues().minCoeff();
    ASSERT_LT(s_min, 1e-9) << "test premise: this configuration must be singular";

    // A Cartesian delta the arm cannot follow: an undamped inverse would give |dq| ~ |dx| / s_min.
    const Vector6 dx_lost = 1e-3 * svd.matrixU().col(5);
    Eigen::VectorXd dq;
    ASSERT_TRUE(core.convert_cartesian_deltas_to_joint_deltas(q, dx_lost, "tcp", dq));
    ASSERT_TRUE(dq.allFinite());
    EXPECT_NEAR(dq.norm(), dx_lost.norm() * s_min / (s_min * s_min + lambda * lambda), 1e-12);

    // In general |dq| <= |dx| * max_s s / (s^2 + l^2) <= |dx| / (2 l), for any direction.
    std::mt19937 rng(7);
    for (int n = 0; n < 200; ++n) {
        const Vector6 dx = test_utils::random_vector(rng, 6, 1e-3);
        ASSERT_TRUE(core.convert_cartesian_deltas_to_joint_deltas(q, dx, "tcp", dq));
        ASSERT_TRUE(dq.allFinite());
        EXPECT_LE(dq.norm(), dx.norm() / (2 * lambda) * (1 + 1e-9));
    }
}

// ---------------------------------------------------------------------------------------------
// 1. initialize(): parsed values
// ---------------------------------------------------------------------------------------------

TEST(Initialize, RealUrdfIsParsedIntoSixJointsAndTheTcp)
{
    // replaces the old manual check: initialize() prints the joint table on success
    std::string urdf;
    try {
        urdf = test_utils::real_urdf();
    } catch (const std::exception & e) {
        FAIL() << e.what();
    }
    TestableCore core;
    ASSERT_TRUE(test_utils::initialize(core, urdf));

    ASSERT_EQ(core.joints_.size(), 6u);
    EXPECT_EQ(core.tcp_link_name_, "tcp");
    for (size_t i = 0; i < 6; ++i) {
        const auto & joint = core.joints_[i];
        EXPECT_EQ(joint.joint_name_, "axis" + std::to_string(i + 1));
        EXPECT_LT(joint.limits_.min, joint.limits_.max);
        EXPECT_GT(joint.limits_.velocity, 0.0);
        EXPECT_GT(joint.limits_.effort, 0.0);
        if (i > 0) {
            EXPECT_EQ(joint.parent_link_name_, core.joints_[i - 1].child_link_name_) << "chain order";
        }
    }

    std::vector<std::string> names;
    std::vector<robotarm_kinematics::KinematicsCore::Limits> limits;
    std::string tcp;
    ASSERT_TRUE(core.get_joint_names(names));
    ASSERT_TRUE(core.get_joint_limits(limits));
    ASSERT_TRUE(core.get_tcp_link_name(tcp));
    ASSERT_EQ(names.size(), 6u);
    ASSERT_EQ(limits.size(), 6u);
    EXPECT_EQ(tcp, "tcp");
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(names[i], "axis" + std::to_string(i + 1));
        EXPECT_LT(limits[i].min, limits[i].max);
        EXPECT_GT(limits[i].velocity, 0.0);
    }
}

TEST(Initialize, DhParametersAreRecoveredFromTheUrdfTheyWereGeneratedInto)
{
    for (const auto & table : {test_utils::robot_dh(), test_utils::generic_dh()}) {
        UrdfSpec spec = test_utils::make_spec(table);
        spec.joints[0].lower = -1.0;
        spec.joints[0].upper = 2.0;
        spec.joints[0].velocity = 0.7;
        spec.joints[0].effort = 5.0;

        TestableCore core;
        ASSERT_TRUE(test_utils::initialize(core, spec.str()));
        ASSERT_EQ(core.joints_.size(), 6u);
        EXPECT_EQ(core.tcp_link_name_, "tcp");
        for (size_t i = 0; i < 6; ++i) {
            SCOPED_TRACE("row " + std::to_string(i + 1));
            const auto & dh = core.joints_[i].dhparams_;
            EXPECT_NEAR(dh.a, table[i].a, 1e-12);
            EXPECT_NEAR(dh.alpha, table[i].alpha, 1e-12);
            EXPECT_NEAR(dh.d, table[i].d, 1e-12);
            EXPECT_NEAR(dh.theta_0, table[i].theta0, 1e-12);
            EXPECT_EQ(core.joints_[i].joint_name_, spec.joint_name(i));
            EXPECT_EQ(core.joints_[i].parent_link_name_, spec.link_name(i));
            EXPECT_EQ(core.joints_[i].child_link_name_, spec.link_name(i + 1));
        }
        EXPECT_EQ(core.joints_[0].limits_.min, -1.0);
        EXPECT_EQ(core.joints_[0].limits_.max, 2.0);
        EXPECT_EQ(core.joints_[0].limits_.velocity, 0.7);
        EXPECT_EQ(core.joints_[0].limits_.effort, 5.0);

        // the getters hand out the same, one entry per joint in chain order
        std::vector<std::string> names;
        std::vector<robotarm_kinematics::KinematicsCore::Limits> limits;
        std::string tcp;
        ASSERT_TRUE(core.get_joint_names(names));
        ASSERT_TRUE(core.get_joint_limits(limits));
        ASSERT_TRUE(core.get_tcp_link_name(tcp));
        ASSERT_EQ(names.size(), 6u);
        ASSERT_EQ(limits.size(), 6u);
        EXPECT_EQ(tcp, "tcp");
        for (size_t i = 0; i < 6; ++i) {
            EXPECT_EQ(names[i], spec.joint_name(i));
        }
        EXPECT_EQ(limits[0].min, -1.0);
        EXPECT_EQ(limits[0].max, 2.0);
        EXPECT_EQ(limits[0].velocity, 0.7);
        EXPECT_EQ(limits[0].effort, 5.0);
        EXPECT_EQ(limits[5].velocity, core.joints_[5].limits_.velocity);
    }
}

TEST(Initialize, RobotModelGettersFailBeforeInitializeAndLeaveTheOutputUntouched)
{
    TestableCore core;
    std::vector<std::string> names{"keep"};
    std::vector<robotarm_kinematics::KinematicsCore::Limits> limits(2);
    std::string tcp = "keep";
    EXPECT_FALSE(core.get_joint_names(names));
    EXPECT_FALSE(core.get_joint_limits(limits));
    EXPECT_FALSE(core.get_tcp_link_name(tcp));
    EXPECT_EQ(names, std::vector<std::string>{"keep"});
    EXPECT_EQ(limits.size(), 2u);
    EXPECT_EQ(tcp, "keep");
}

TEST(Initialize, LambdaIsReadFromTheParameter)
{
    // the values accepted by the check ">= 0 and finite"; the rejected ones are in InvalidUrdfTest
    const std::string urdf = urdf_for("baseline");
    for (const double lambda : {0.0, 1e-6, 0.5}) {
        TestableCore core;
        EXPECT_TRUE(test_utils::initialize(core, urdf, test_utils::lambda_param(lambda))) << lambda;
    }
}

// ---------------------------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------------------------

TEST(Lifecycle, EveryFunctionRefusesBeforeInitialize)
{
    TestableCore core;
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Isometry3d T;
    Jacobian J;
    JacobianInverse Ji;
    Vector6 dx = Vector6::Zero();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
    EXPECT_FALSE(core.calculate_link_transform(q, "tcp", T));
    EXPECT_FALSE(core.calculate_jacobian(q, "tcp", J));
    EXPECT_FALSE(core.calculate_jacobian_inverse(q, "tcp", Ji));
    EXPECT_FALSE(core.convert_joint_deltas_to_cartesian_deltas(q, dq, "tcp", dx));
    EXPECT_FALSE(core.convert_cartesian_deltas_to_joint_deltas(q, dx, "tcp", dq));
}

TEST(Lifecycle, AFailedInitializeLeavesTheObjectUnusableAndAValidOneRevivesIt)
{
    const std::string valid = urdf_for("baseline");
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Isometry3d T;

    TestableCore core;
    ASSERT_TRUE(test_utils::initialize(core, valid));
    EXPECT_TRUE(core.calculate_link_transform(q, "tcp", T));

    EXPECT_FALSE(test_utils::initialize(core, "<robot"));
    EXPECT_FALSE(core.calculate_link_transform(q, "tcp", T)) << "stale state of the previous URDF";

    ASSERT_TRUE(test_utils::initialize(core, valid));
    EXPECT_TRUE(core.calculate_link_transform(q, "tcp", T));
}

// ---------------------------------------------------------------------------------------------
// 2. initialize() rejects everything that is not DH-conform / valid. Each case is one deviation
// from an URDF that is accepted (see AcceptsTheUnmodifiedUrdf...), so a failure has exactly one cause.
// ---------------------------------------------------------------------------------------------

// What a case changes, one of: a mutation of the valid URDF, a raw text instead of the URDF
// (use_raw), or an invalid parameter next to the valid URDF.
struct InvalidCase
{
    std::string name;
    std::function<void(UrdfSpec &)> mutate;
    bool use_raw;
    std::string raw_urdf;
    std::vector<rclcpp::Parameter> params;
};

void remove_joint(UrdfSpec & s, size_t i)
{
    s.origins.erase(s.origins.begin() + static_cast<long>(i));
    s.joints.erase(s.joints.begin() + static_cast<long>(i));
}

void insert_joint(UrdfSpec & s, size_t i)
{
    s.origins.insert(s.origins.begin() + static_cast<long>(i), test_utils::Origin{});
    s.joints.insert(s.joints.begin() + static_cast<long>(i), test_utils::JointSpec{});
}

std::vector<InvalidCase> invalid_cases()
{
    using test_utils::JointSpec;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<InvalidCase> c;
    auto add = [&c](std::string name, std::function<void(UrdfSpec &)> mutate) {
        c.push_back({std::move(name), std::move(mutate), false, "", {}});
    };

    // chain structure
    add("FiveJoints", [](UrdfSpec & s) {remove_joint(s, 3);});
    add("EightJoints", [](UrdfSpec & s) {insert_joint(s, 3);});
    add("BranchingChain", [](UrdfSpec & s) {s.branch = true;});

    // the first joint must not move the root link frame (DH frame 0)
    add("FirstOriginTranslated", [](UrdfSpec & s) {s.origins[0].xyz.x() = 0.01;});
    add("FirstOriginRotated", [](UrdfSpec & s) {s.origins[0].rpy.z() = 0.1;});

    // joints rotate about +z
    add("AxisX", [](UrdfSpec & s) {s.joints[2].axis = Eigen::Vector3d::UnitX();});
    add("AxisMinusZ", [](UrdfSpec & s) {s.joints[4].axis = -Eigen::Vector3d::UnitZ();});

    // origins that cannot be written as Rz(theta0) Tz(d) Tx(a) Rx(alpha)
    add("RotationAboutY", [](UrdfSpec & s) {s.origins[2].rpy.y() = 0.2;});
    add("TranslationOutOfTheDhPlane", [](UrdfSpec & s) {
        // 1 cm along the y axis of the theta0-rotated frame: x_i does not intersect z_(i-1)
        const double yaw = s.origins[3].rpy.z();
        s.origins[3].xyz += Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(0, 0.01, 0);
    });

    // joint types and limits
    add("ContinuousJoint", [](UrdfSpec & s) {s.joints[1].type = "continuous";});
    add("PrismaticJoint", [](UrdfSpec & s) {s.joints[1].type = "prismatic";});
    add("FixedJointInsideTheChain", [](UrdfSpec & s) {s.joints[2].type = "fixed";});
    add("TcpJointNotFixed", [](UrdfSpec & s) {s.joints.back().type = "revolute";});
    add("MissingLimits", [](UrdfSpec & s) {s.joints[0].has_limit = false;});
    add("LowerEqualsUpper", [](UrdfSpec & s) {s.joints[3].lower = s.joints[3].upper = 0.5;});
    add("LowerAboveUpper", [](UrdfSpec & s) {s.joints[3].lower = 1.0; s.joints[3].upper = -1.0;});
    add("ZeroVelocityLimit", [](UrdfSpec & s) {s.joints[1].velocity = 0.0;});
    add("ZeroEffortLimit", [](UrdfSpec & s) {s.joints[1].effort = 0.0;});

    // not an URDF at all
    c.push_back({"MalformedXml", nullptr, true, "<robot name=\"x\"><link", {}});
    c.push_back({"EmptyString", nullptr, true, "", {}});

    // the valid URDF with an invalid parameter
    c.push_back({"LambdaNegative", nullptr, false, "", {rclcpp::Parameter("lambda", -0.1)}});
    c.push_back({"LambdaNan", nullptr, false, "", {rclcpp::Parameter("lambda", nan)}});
    c.push_back({"LambdaInfinite", nullptr, false, "",
        {rclcpp::Parameter("lambda", std::numeric_limits<double>::infinity())}});
    c.push_back({"LambdaWrongType", nullptr, false, "",
        {rclcpp::Parameter("lambda", std::string("abc"))}});
    return c;
}

class InvalidUrdfTest : public ::testing::TestWithParam<InvalidCase> {};

TEST(InvalidUrdf, AcceptsTheUnmodifiedUrdfTheCasesAreDerivedFrom)
{
    TestableCore core;
    EXPECT_TRUE(test_utils::initialize(core, test_utils::make_spec(test_utils::robot_dh()).str()));
}

TEST_P(InvalidUrdfTest, InitializeReturnsFalseAndTheObjectStaysUnusable)
{
    const InvalidCase & c = GetParam();
    const std::string valid = test_utils::make_spec(test_utils::robot_dh()).str();

    std::string urdf = valid;
    if (c.use_raw) {
        urdf = c.raw_urdf;
    } else if (c.mutate) {
        UrdfSpec spec = test_utils::make_spec(test_utils::robot_dh());
        c.mutate(spec);
        urdf = spec.str();
        EXPECT_NE(urdf, valid) << "the case does not change the URDF, it tests nothing";
    }

    TestableCore core;
    EXPECT_FALSE(test_utils::initialize(core, urdf, c.params));
    Eigen::Isometry3d T;
    EXPECT_FALSE(core.calculate_link_transform(Eigen::VectorXd::Zero(6), "tcp", T));
}

INSTANTIATE_TEST_SUITE_P(
    Rules, InvalidUrdfTest, ::testing::ValuesIn(invalid_cases()),
    [](const ::testing::TestParamInfo<InvalidCase> & info) {return info.param.name;});

}  // namespace

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
