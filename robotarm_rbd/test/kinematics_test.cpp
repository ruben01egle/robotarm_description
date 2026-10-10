// Tests for Kinematics, in order of importance:
//   1. URDF parsing: joint origins, axes and limits, and every link transform against an
//      independent reference FK built from the URDF itself (real robot + synthetic robots of
//      different lengths, DH-like and random geometry)
//   2. initialize() rejects every invalid URDF, and accepts the geometry the old DH parser refused
//   3. Jacobian against finite differences of the reference FK
//   4. delta conversions: consistency with FK, round-trip accuracy of both damped inverses (ldlt
//      and svd), the svd inverse against its formula, behaviour at a singularity, input validation
// The allocation check lives in kinematics_malloc_test.cpp (it needs a special build).

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
using test_utils::TestableKinematics;
using test_utils::UrdfSpec;

// Plugin and reference use the same URDF numbers, they only differ in floating point noise
// (RPY vs quaternion). A wrong frame, axis or sign is off by millimetres or more.
constexpr double kPositionTol = 1e-12;
constexpr double kRotationTol = 1e-12;

// "real", "baseline" (DH table of the robot), "tool" (baseline with a branched tool between flange
// and tcp), "generic" (arbitrary DH table) or "random<dof>" (random origins and axes, see
// make_random_spec)
std::string urdf_for(const std::string & robot)
{
    if (robot == "real") {
        return test_utils::real_urdf();
    }
    if (robot == "baseline") {
        return test_utils::make_spec(test_utils::robot_dh()).str();
    }
    if (robot == "tool") {
        return test_utils::make_tool_spec(test_utils::robot_dh()).str();
    }
    if (robot == "generic") {
        return test_utils::make_spec(test_utils::generic_dh()).str();
    }
    if (robot.rfind("random", 0) == 0) {
        const size_t dof = std::stoul(robot.substr(6));
        return test_utils::make_random_spec(dof, 100 + static_cast<unsigned>(dof)).str();
    }
    throw std::invalid_argument("unknown test robot " + robot);
}

// every link transform at q against the reference FK
void expect_fk_matches_reference(
    TestableKinematics & core, const test_utils::ReferenceFk & ref, const Eigen::VectorXd & q)
{
    for (const std::string & link : ref.link_names()) {
        SCOPED_TRACE("link " + link);
        Eigen::Isometry3d T;
        ASSERT_TRUE(core.calculate_link_transform(q, link, T));
        const Eigen::Isometry3d R = ref.link(q, link);
        EXPECT_LT((T.translation() - R.translation()).norm(), kPositionTol);
        EXPECT_LT(Eigen::AngleAxisd(R.linear().transpose() * T.linear()).angle(), kRotationTol);
    }
}

// The Jacobian of every link at q against central differences of the reference FK (also the base
// link, whose Jacobian is zero, and links in the middle of the chain, whose later columns must be
// zero). Linear rows: d position / dq_i. Angular rows: rotation vector of R(q + h) R(q - h)^T / 2h.
void expect_jacobian_matches_reference(
    TestableKinematics & core, const test_utils::ReferenceFk & ref, const Eigen::VectorXd & q)
{
    const double h = 1e-6;
    const Eigen::Index n = q.size();
    for (const std::string & link : ref.link_names()) {
        Jacobian J;
        ASSERT_TRUE(core.calculate_jacobian(q, link, J));
        ASSERT_EQ(J.rows(), 6);
        ASSERT_EQ(J.cols(), n);
        for (Eigen::Index i = 0; i < n; ++i) {
            SCOPED_TRACE("link " + link + ", column " + std::to_string(i));
            Eigen::VectorXd qp = q, qm = q;
            qp[i] += h;
            qm[i] -= h;
            const Eigen::Isometry3d Tp = ref.link(qp, link);
            const Eigen::Isometry3d Tm = ref.link(qm, link);
            const Eigen::Vector3d linear = (Tp.translation() - Tm.translation()) / (2 * h);
            const Eigen::Vector3d angular =
                test_utils::rotation_vector(Tp.linear() * Tm.linear().transpose()) / (2 * h);
            EXPECT_LT((J.col(i).head<3>() - linear).norm(), 1e-6);
            EXPECT_LT((J.col(i).tail<3>() - angular).norm(), 1e-6);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// tests that run on all robots: the real one, one with the same DH table (also with a tool), a
// generic DH one, and random non-DH chains of 1, 3, 6 and 7 joints. The links checked are the root,
// every moving link, the flange and the tcp.
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
    Eigen::Index n() const {return static_cast<Eigen::Index>(ref_->dof());}

    TestableKinematics core_;
    std::string urdf_;
    std::unique_ptr<test_utils::ReferenceFk> ref_;
    std::mt19937 rng_;
};

INSTANTIATE_TEST_SUITE_P(
    Robots, RobotTest,
    ::testing::Values("real", "baseline", "tool", "generic", "random1", "random3", "random6", "random7"),
    [](const ::testing::TestParamInfo<std::string> & info) {return info.param;});

// --- 1. forward kinematics ---------------------------------------------------------------------

TEST_P(RobotTest, LinkTransformsMatchTheUrdfReference)
{
    // root link, all links and the tcp, at q = 0 and at random configurations
    for (int k = 0; k < 50; ++k) {
        SCOPED_TRACE("q = " + std::to_string(k));
        const Eigen::VectorXd q = k == 0 ? Eigen::VectorXd::Zero(n()) : random_q();
        expect_fk_matches_reference(core_, *ref_, q);
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
    EXPECT_EQ(J.cols(), n());
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

    Jacobian J = Jacobian::Constant(6, n(), 42.0);
    EXPECT_FALSE(core_.calculate_jacobian(q, "no_such_link", J));
    EXPECT_TRUE((J.array() == 42.0).all());

    JacobianInverse Ji = JacobianInverse::Constant(n(), 6, 42.0);
    EXPECT_FALSE(core_.calculate_jacobian_inverse(q, "no_such_link", Ji));
    EXPECT_TRUE((Ji.array() == 42.0).all());

    Vector6 dx = Vector6::Constant(42.0);
    EXPECT_FALSE(core_.convert_joint_deltas_to_cartesian_deltas(q, Eigen::VectorXd::Zero(n()), "no_such_link", dx));
    EXPECT_TRUE((dx.array() == 42.0).all());

    Eigen::VectorXd dq = Eigen::VectorXd::Constant(n(), 42.0);
    EXPECT_FALSE(core_.convert_cartesian_deltas_to_joint_deltas(q, Vector6::Zero(), "no_such_link", dq));
    EXPECT_TRUE((dq.array() == 42.0).all());
}

// --- 3. Jacobian -------------------------------------------------------------------------------

TEST_P(RobotTest, JacobianMatchesFiniteDifferencesOfTheReferenceFk)
{
    for (int k = 0; k < 20; ++k) {
        SCOPED_TRACE("q = " + std::to_string(k));
        expect_jacobian_matches_reference(core_, *ref_, random_q());
    }
}

// --- 4. delta conversions ----------------------------------------------------------------------

TEST_P(RobotTest, JointDeltasToCartesianAreTheJacobianProductAndFollowTheFkToFirstOrder)
{
    const std::string tcp = test_utils::tcp_name(core_);
    for (int k = 0; k < 20; ++k) {
        const Eigen::VectorXd q = random_q();
        const Eigen::VectorXd dq = test_utils::random_vector(rng_, n(), 1e-6);

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
    // Both inverses are J+ = V diag(g_i) U^T for J = U S V^T, so
    //   J+ J = V diag(s_i g_i) V^T   and   J J+ = U diag(s_i g_i) U^T,
    // and both round trips (dq -> dx -> dq and dx -> dq -> dx) are off by at most
    //   |result - input| <= max_i (1 - s_i g_i) * |input|.
    // ldlt: g = s / (s^2 + l^2), so 1 - s g = l^2 / (s^2 + l^2), largest at s_min.
    // svd:  g = s / (s^2 + l_i^2), l_i^2 <= l^2 and 0 for s >= eps = sqrt(2) l, so exact for
    //       s_min >= eps and otherwise largest at s_min as well.
    // A bug in the Jacobian, its inverse or the products breaks this; a wrong / ignored lambda or
    // method does as well.
    // With fewer than 6 joints J (6xN) cannot reach every dx, so only dq -> dx -> dq is bounded;
    // with more than 6, J has a null space that dq -> dx loses, so only dx -> dq -> dx is.
    const bool check_joint_round_trip = n() <= 6;
    const bool check_cartesian_round_trip = n() >= 6;
    const std::string tcp = test_utils::tcp_name(core_);
    for (const std::string method : {"ldlt", "svd"}) {
        for (const double lambda : {1e-6, 1e-2, 5e-2}) {
            SCOPED_TRACE(method + ", lambda = " + std::to_string(lambda));
            ASSERT_TRUE(test_utils::initialize(core_, urdf_, test_utils::inverse_params(lambda, method)));
            const bool ldlt = method == "ldlt";

            int accepted = 0;
            for (int k = 0; k < 1000 && accepted < 30; ++k) {
                const Eigen::VectorXd q = random_q();
                Jacobian J;
                ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J));
                const double s_min = Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues().minCoeff();
                if (s_min < 0.03) {
                    continue;  // near a singularity, the bound is loose and says little
                }
                ++accepted;
                const double bound = ldlt ?
                    lambda * lambda / (s_min * s_min + lambda * lambda) :
                    1.0 - s_min * test_utils::svd_gain(s_min, lambda);
                const double slack = 1e-6 * bound + 1e-10;  // floating point noise of the solve
                // svd leaves every direction with s >= eps undamped
                const bool svd_undamped = !ldlt && s_min >= std::sqrt(2.0) * lambda;

                // dq -> dx -> dq
                Eigen::VectorXd dq_back;
                if (check_joint_round_trip) {
                    const Eigen::VectorXd dq = test_utils::random_vector(rng_, n(), 1e-3);
                    Vector6 dx;
                    ASSERT_TRUE(core_.convert_joint_deltas_to_cartesian_deltas(q, dq, tcp, dx));
                    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_back));
                    const double joint_error = (dq_back - dq).norm() / dq.norm();
                    EXPECT_LE(joint_error, bound + slack);
                    if (lambda == 1e-6 || svd_undamped) {
                        EXPECT_LT(joint_error, 1e-8);  // practically exact without damping
                    }
                    if (lambda == 5e-2 && ldlt) {
                        EXPECT_GT(joint_error, 1e-6);  // the damping is really applied, to every direction
                    }
                }

                // dx -> dq -> dx
                if (check_cartesian_round_trip) {
                    const Vector6 dx_in = test_utils::random_vector(rng_, 6, 1e-3);
                    Vector6 dx_back;
                    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx_in, tcp, dq_back));
                    ASSERT_TRUE(core_.convert_joint_deltas_to_cartesian_deltas(q, dq_back, tcp, dx_back));
                    const double cartesian_error = (dx_back - dx_in).norm() / dx_in.norm();
                    EXPECT_LE(cartesian_error, bound + slack);
                    if (lambda == 1e-6 || svd_undamped) {
                        EXPECT_LT(cartesian_error, 1e-8);
                    }
                }
            }
            EXPECT_GE(accepted, 30) << "too few well-conditioned configurations found";
        }
    }
}

TEST_P(RobotTest, SvdInverseMatchesTheSelectiveDampingFormula)
{
    // against the formula evaluated from an independent SVD of the Jacobian (test_utils), for every
    // link, at a lambda where some directions are damped and some are not
    const double lambda = 5e-2;
    ASSERT_TRUE(test_utils::initialize(core_, urdf_, test_utils::inverse_params(lambda, "svd")));
    for (int k = 0; k < 20; ++k) {
        const Eigen::VectorXd q = random_q();
        for (const std::string & link : ref_->link_names()) {
            SCOPED_TRACE("q = " + std::to_string(k) + ", link " + link);
            Jacobian J;
            JacobianInverse Ji;
            ASSERT_TRUE(core_.calculate_jacobian(q, link, J));
            ASSERT_TRUE(core_.calculate_jacobian_inverse(q, link, Ji));
            ASSERT_EQ(Ji.rows(), n());
            const Eigen::MatrixXd Ji_ref = test_utils::reference_svd_inverse(J, lambda);
            // the gain is at most 1 / eps ~ 14 here
            EXPECT_LT((Ji - Ji_ref).cwiseAbs().maxCoeff(), 1e-10);
        }
    }
}

TEST_P(RobotTest, BothMethodsAgreeAwayFromSingularities)
{
    // with a tiny lambda both are the undamped pseudo inverse wherever J is well conditioned.
    // Not for more than 6 joints: there the n×n JᵀJ of the ldlt has a null space, JᵀJ + l²I is
    // conditioned ~ 1 / l², and the ldlt solve loses ~ 4 digits at l = 1e-6.
    if (n() > 6) {
        GTEST_SKIP() << "ldlt is ill conditioned for a tiny lambda on a redundant chain";
    }
    const double lambda = 1e-6;
    const std::string tcp = test_utils::tcp_name(core_);
    TestableKinematics ldlt, svd;
    ASSERT_TRUE(test_utils::initialize(ldlt, urdf_, test_utils::inverse_params(lambda, "ldlt")));
    ASSERT_TRUE(test_utils::initialize(svd, urdf_, test_utils::inverse_params(lambda, "svd")));
    int accepted = 0;
    for (int k = 0; k < 1000 && accepted < 20; ++k) {
        const Eigen::VectorXd q = random_q();
        Jacobian J;
        ASSERT_TRUE(ldlt.calculate_jacobian(q, tcp, J));
        if (Eigen::JacobiSVD<Eigen::MatrixXd>(J).singularValues().minCoeff() < 0.03) {
            continue;
        }
        ++accepted;
        JacobianInverse Ji_ldlt, Ji_svd;
        ASSERT_TRUE(ldlt.calculate_jacobian_inverse(q, tcp, Ji_ldlt));
        ASSERT_TRUE(svd.calculate_jacobian_inverse(q, tcp, Ji_svd));
        // ldlt still damps by l^2 / s^2 <= 1e-12 / 0.03^2 ~ 1e-9 relative, svd not at all
        EXPECT_LT((Ji_ldlt - Ji_svd).norm() / Ji_svd.norm(), 1e-8);
    }
    EXPECT_GE(accepted, 20) << "too few well-conditioned configurations found";
}

TEST_P(RobotTest, InvalidInputsFailAndLeaveOutputsUntouched)
{
    const std::string tcp = test_utils::tcp_name(core_);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    const Eigen::VectorXd q = random_q();
    Eigen::VectorXd q_nan = q;
    q_nan[n() - 1] = nan;
    const Eigen::VectorXd q_short = Eigen::VectorXd::Zero(n() - 1);
    const Eigen::VectorXd q_long = Eigen::VectorXd::Zero(n() + 1);

    const Eigen::VectorXd dq = test_utils::random_vector(rng_, n(), 1e-3);
    Eigen::VectorXd dq_inf = dq;
    dq_inf[0] = inf;
    Eigen::VectorXd dq_nan = dq;
    dq_nan[n() - 1] = nan;
    const Eigen::VectorXd dq_short = Eigen::VectorXd::Zero(n() - 1);
    const Eigen::VectorXd dq_long = Eigen::VectorXd::Zero(n() + 1);

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

        Jacobian J = Jacobian::Constant(6, n(), 42.0);
        EXPECT_FALSE(core_.calculate_jacobian(bad_q, tcp, J));
        EXPECT_TRUE((J.array() == 42.0).all());

        JacobianInverse Ji = JacobianInverse::Constant(n(), 6, 42.0);
        EXPECT_FALSE(core_.calculate_jacobian_inverse(bad_q, tcp, Ji));
        EXPECT_TRUE((Ji.array() == 42.0).all());

        Vector6 out_x = Vector6::Constant(42.0);
        EXPECT_FALSE(core_.convert_joint_deltas_to_cartesian_deltas(bad_q, dq, tcp, out_x));
        EXPECT_TRUE((out_x.array() == 42.0).all());

        Eigen::VectorXd out_q = Eigen::VectorXd::Constant(n(), 42.0);
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
        Eigen::VectorXd out_q = Eigen::VectorXd::Constant(n(), 42.0);
        EXPECT_FALSE(core_.convert_cartesian_deltas_to_joint_deltas(q, bad_dx, tcp, out_q));
        EXPECT_TRUE((out_q.array() == 42.0).all());
    }
}

TEST_P(RobotTest, UnsizedOutputsAreResizedAndGiveTheSameResultAsPresizedOnes)
{
    const std::string tcp = test_utils::tcp_name(core_);
    const Eigen::VectorXd q = random_q();
    const Vector6 dx = test_utils::random_vector(rng_, 6, 1e-3);

    Jacobian J_sized = Jacobian::Zero(6, n()), J_empty;
    ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J_sized));
    ASSERT_TRUE(core_.calculate_jacobian(q, tcp, J_empty));
    EXPECT_TRUE(J_empty.rows() == 6 && J_empty.cols() == n());
    EXPECT_TRUE(J_empty == J_sized);

    JacobianInverse Ji_sized = JacobianInverse::Zero(n(), 6), Ji_empty;
    ASSERT_TRUE(core_.calculate_jacobian_inverse(q, tcp, Ji_sized));
    ASSERT_TRUE(core_.calculate_jacobian_inverse(q, tcp, Ji_empty));
    EXPECT_TRUE(Ji_empty.rows() == n() && Ji_empty.cols() == 6);
    EXPECT_TRUE(Ji_empty == Ji_sized);

    Eigen::VectorXd dq_sized = Eigen::VectorXd::Zero(n()), dq_empty, dq_wrong = Eigen::VectorXd::Zero(n() + 2);
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_sized));
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_empty));
    ASSERT_TRUE(core_.convert_cartesian_deltas_to_joint_deltas(q, dx, tcp, dq_wrong));
    EXPECT_EQ(dq_empty.size(), n());
    EXPECT_EQ(dq_wrong.size(), n());
    EXPECT_TRUE(dq_empty == dq_sized);
    EXPECT_TRUE(dq_wrong == dq_sized);
}

// --- damping at a singularity (baseline robot: wrist singular at q5 = 0) -------------------------

TEST(Singularity, DampingKeepsTheJointDeltasBoundedAtTheWristSingularity)
{
    const double lambda = 1e-2;
    for (const std::string method : {"ldlt", "svd"}) {
        SCOPED_TRACE(method);
        const bool ldlt = method == "ldlt";
        TestableKinematics core;
        ASSERT_TRUE(test_utils::initialize(
            core, urdf_for("baseline"), test_utils::inverse_params(lambda, method)));

        Eigen::VectorXd q(6);
        q << 0.1, -0.5, 0.8, 0.2, 0.0, -0.3;  // q5 = 0 aligns axes 4 and 6
        Jacobian J;
        ASSERT_TRUE(core.calculate_jacobian(q, "tcp", J));
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU);
        const double s_min = svd.singularValues().minCoeff();
        ASSERT_LT(s_min, 1e-9) << "test premise: this configuration must be singular";

        // A Cartesian delta the arm cannot follow: an undamped inverse would give |dq| ~ |dx| / s_min.
        const Vector6 dx_lost = 1e-3 * svd.matrixU().col(5);
        const double gain = ldlt ?
            s_min / (s_min * s_min + lambda * lambda) : test_utils::svd_gain(s_min, lambda);
        Eigen::VectorXd dq;
        ASSERT_TRUE(core.convert_cartesian_deltas_to_joint_deltas(q, dx_lost, "tcp", dq));
        ASSERT_TRUE(dq.allFinite());
        EXPECT_NEAR(dq.norm(), dx_lost.norm() * gain, 1e-12);

        // In general |dq| <= |dx| * max_s g(s), for any direction:
        //   ldlt: s / (s^2 + l^2), largest at s = l:            1 / (2 l)
        //   svd:  s / (l^2 + s^2 / 2) below eps = sqrt(2) l, 1 / s above, largest at s = eps: 1 / (sqrt(2) l)
        const double max_gain = ldlt ? 1.0 / (2 * lambda) : 1.0 / (std::sqrt(2.0) * lambda);
        std::mt19937 rng(7);
        for (int n = 0; n < 200; ++n) {
            const Vector6 dx = test_utils::random_vector(rng, 6, 1e-3);
            ASSERT_TRUE(core.convert_cartesian_deltas_to_joint_deltas(q, dx, "tcp", dq));
            ASSERT_TRUE(dq.allFinite());
            EXPECT_LE(dq.norm(), dx.norm() * max_gain * (1 + 1e-9));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// 1. initialize(): parsed values
// ---------------------------------------------------------------------------------------------

TEST(Initialize, RealUrdfIsParsedIntoSixJointsTheFlangeAndTheTcp)
{
    // replaces the old manual check: initialize() prints the joint table on success
    std::string urdf;
    try {
        urdf = test_utils::real_urdf();
    } catch (const std::exception & e) {
        FAIL() << e.what();
    }
    test_utils::TestableRbd core;
    ASSERT_TRUE(core.initialize(urdf, {})) << core.last_error();

    ASSERT_EQ(core.joints_.size(), 6u);
    EXPECT_EQ(core.flange_.parent_link_name_, core.joints_.back().child_link_name_);
    EXPECT_EQ(core.tcp_.tcp_name_, "tcp");
    EXPECT_EQ(core.tcp_.parent_link_name_, "flange");
    for (size_t i = 0; i < 6; ++i) {
        const auto & joint = core.joints_[i];
        EXPECT_NEAR(joint.joint_axis_in_child_.norm(), 1.0, 1e-15);
        EXPECT_EQ(joint.joint_name_, "axis" + std::to_string(i + 1));
        EXPECT_LT(joint.limits_.min, joint.limits_.max);
        EXPECT_GT(joint.limits_.velocity, 0.0);
        EXPECT_GT(joint.limits_.effort, 0.0);
        if (i > 0) {
            EXPECT_EQ(joint.parent_link_name_, core.joints_[i - 1].child_link_name_) << "chain order";
        }
    }

    std::vector<std::string> names;
    std::vector<robotarm_rbd::Kinematics::Limits> limits;
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

TEST(Initialize, OriginsAxesAndLimitsAreParsedFromTheUrdf)
{
    // a DH-like and a random chain: the parser does not care which one it gets
    for (const UrdfSpec & base : {test_utils::make_spec(test_utils::robot_dh()),
            test_utils::make_random_spec(6, 1)})
    {
        UrdfSpec spec = base;
        spec.joints[0].lower = -1.0;
        spec.joints[0].upper = 2.0;
        spec.joints[0].velocity = 0.7;
        spec.joints[0].effort = 5.0;
        const size_t dof = spec.joints.size() - 2;  // minus flange and tcp joint

        test_utils::TestableRbd core;
        ASSERT_TRUE(core.initialize(spec.str(), {})) << core.last_error();
        ASSERT_EQ(core.joints_.size(), dof);
        for (size_t i = 0; i < dof; ++i) {
            SCOPED_TRACE("joint " + std::to_string(i));
            const auto & joint = core.joints_[i];
            EXPECT_TRUE(joint.joint_origin_in_parent_.matrix().isApprox(
                test_utils::isometry(spec.origins[i]).matrix(), 1e-12));
            // stored normalised, whatever length the URDF axis has
            EXPECT_TRUE(joint.joint_axis_in_child_.isApprox(spec.joints[i].axis.normalized(), 1e-12));
            EXPECT_EQ(joint.joint_name_, spec.joint_name(i));
            EXPECT_EQ(joint.parent_link_name_, spec.link_name(i));
            EXPECT_EQ(joint.child_link_name_, spec.link_name(i + 1));
        }
        // flange on the last moving link, tcp on the flange, and both composed for the dynamics
        const Eigen::Isometry3d flange = test_utils::isometry(spec.origins[dof]);
        const Eigen::Isometry3d tcp_in_flange = test_utils::isometry(spec.origins[dof + 1]);
        EXPECT_EQ(core.flange_.parent_link_name_, spec.link_name(dof));
        EXPECT_TRUE(core.flange_.flange_origin_in_parent_.matrix().isApprox(flange.matrix(), 1e-12));
        EXPECT_EQ(core.tcp_.tcp_name_, "tcp");
        EXPECT_EQ(core.tcp_.parent_link_name_, "flange");
        EXPECT_TRUE(core.tcp_.tcp_origin_in_flange_.matrix().isApprox(tcp_in_flange.matrix(), 1e-12));
        EXPECT_TRUE(core.tcp_.tcp_origin_in_last_mov_link_.matrix().isApprox(
            (flange * tcp_in_flange).matrix(), 1e-12));
        EXPECT_EQ(core.joints_[0].limits_.min, -1.0);
        EXPECT_EQ(core.joints_[0].limits_.max, 2.0);
        EXPECT_EQ(core.joints_[0].limits_.velocity, 0.7);
        EXPECT_EQ(core.joints_[0].limits_.effort, 5.0);

        // the getters hand out the same, one entry per joint in chain order
        std::vector<std::string> names, links;
        std::vector<robotarm_rbd::Kinematics::Limits> limits;
        std::string tcp;
        ASSERT_TRUE(core.get_joint_names(names));
        ASSERT_TRUE(core.get_joint_limits(limits));
        ASSERT_TRUE(core.get_link_names(links));
        ASSERT_TRUE(core.get_tcp_link_name(tcp));
        ASSERT_EQ(names.size(), dof);
        ASSERT_EQ(limits.size(), dof);
        ASSERT_EQ(links.size(), dof + 1);
        EXPECT_EQ(tcp, "tcp");
        for (size_t i = 0; i < dof; ++i) {
            EXPECT_EQ(names[i], spec.joint_name(i));
        }
        for (size_t i = 0; i <= dof; ++i) {
            EXPECT_EQ(links[i], spec.link_name(i));
        }
        EXPECT_EQ(limits[0].min, -1.0);
        EXPECT_EQ(limits[0].max, 2.0);
        EXPECT_EQ(limits[0].velocity, 0.7);
        EXPECT_EQ(limits[0].effort, 5.0);
        EXPECT_EQ(limits[dof - 1].velocity, core.joints_[dof - 1].limits_.velocity);
    }
}

TEST(Initialize, TheTcpOfAToolIsFoundBehindTheFlangeAndComposedThroughIt)
{
    // the tool branches and has a prismatic jaw: neither adds a joint, the tcp hangs on the tool
    const UrdfSpec spec = test_utils::make_tool_spec(test_utils::robot_dh());
    test_utils::TestableRbd core;
    ASSERT_TRUE(core.initialize(spec.str(), {})) << core.last_error();
    const test_utils::ReferenceFk ref(spec.str());

    std::vector<std::string> names;
    ASSERT_TRUE(core.get_joint_names(names));
    EXPECT_EQ(names, (std::vector<std::string>{"axis1", "axis2", "axis3", "axis4", "axis5", "axis6"}));

    // relative poses are the same for every q, the reference takes the jaw at zero
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    const Eigen::Isometry3d last = ref.link(q, spec.link_name(6));
    const Eigen::Isometry3d flange = ref.link(q, "flange");
    const Eigen::Isometry3d tcp = ref.link(q, "tcp");
    EXPECT_TRUE(core.tcp_.tcp_origin_in_last_mov_link_.matrix().isApprox(
        (last.inverse() * tcp).matrix(), 1e-12));
    EXPECT_TRUE(core.tcp_.tcp_origin_in_flange_.matrix().isApprox((flange.inverse() * tcp).matrix(), 1e-12));
    EXPECT_FALSE(core.tcp_.tcp_origin_in_flange_.isApprox(Eigen::Isometry3d::Identity()))
        << "test premise: the tool moves the tcp away from the flange";

    // the tool links are folded away, they are no link of the model
    Eigen::Isometry3d T;
    EXPECT_FALSE(core.calculate_link_transform(q, "tool_base", T));
    EXPECT_FALSE(core.calculate_link_transform(q, "tool_jaw1", T));
}

TEST(Initialize, TheNumberOfJointsFollowsTheUrdf)
{
    for (const size_t dof : {1u, 2u, 5u, 8u, 12u}) {
        SCOPED_TRACE("dof = " + std::to_string(dof));
        test_utils::TestableRbd core;
        ASSERT_TRUE(core.initialize(test_utils::make_random_spec(dof, 7).str(), {})) << core.last_error();
        EXPECT_EQ(core.joints_.size(), dof);
        Eigen::Isometry3d T;
        EXPECT_TRUE(core.calculate_link_transform(Eigen::VectorXd::Zero(dof), "tcp", T));
        EXPECT_FALSE(core.calculate_link_transform(Eigen::VectorXd::Zero(6), "tcp", T)) << "fixed to 6";
    }
}

TEST(Initialize, RobotModelGettersFailBeforeInitializeAndLeaveTheOutputUntouched)
{
    TestableKinematics core;
    std::vector<std::string> names{"keep"};
    std::vector<robotarm_rbd::Kinematics::Limits> limits(2);
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
    // the values accepted by the check "> 0 and finite"; the rejected ones are in InvalidUrdfTest
    const std::string urdf = urdf_for("baseline");
    for (const double lambda : {1e-6, 0.5}) {
        TestableKinematics core;
        EXPECT_TRUE(test_utils::initialize(core, urdf, test_utils::lambda_param(lambda))) << lambda;
    }
}

TEST(Initialize, JinvMethodIsReadFromTheParameter)
{
    // both values accepted, the rejected ones are in InvalidUrdfTest. The methods differ far from a
    // singularity at a large lambda: ldlt damps every direction, svd none of the well conditioned ones.
    const std::string urdf = urdf_for("baseline");
    Eigen::VectorXd q(6);
    q << 0.1, -0.5, 0.8, 0.2, 0.7, -0.3;
    JacobianInverse Ji_ldlt, Ji_svd;
    TestableKinematics ldlt, svd;
    ASSERT_TRUE(test_utils::initialize(ldlt, urdf, test_utils::inverse_params(0.5, "ldlt")));
    ASSERT_TRUE(test_utils::initialize(svd, urdf, test_utils::inverse_params(0.5, "svd")));
    ASSERT_TRUE(ldlt.calculate_jacobian_inverse(q, "tcp", Ji_ldlt));
    ASSERT_TRUE(svd.calculate_jacobian_inverse(q, "tcp", Ji_svd));
    EXPECT_GT((Ji_ldlt - Ji_svd).cwiseAbs().maxCoeff(), 1e-3);

    // without the parameter: svd
    TestableKinematics dflt;
    ASSERT_TRUE(test_utils::initialize(dflt, urdf, test_utils::lambda_param(0.5)));
    JacobianInverse Ji_default;
    ASSERT_TRUE(dflt.calculate_jacobian_inverse(q, "tcp", Ji_default));
    EXPECT_TRUE(Ji_default == Ji_svd);
}

// ---------------------------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------------------------

TEST(Lifecycle, EveryFunctionRefusesBeforeInitialize)
{
    TestableKinematics core;
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

    TestableKinematics core;
    ASSERT_TRUE(test_utils::initialize(core, valid));
    EXPECT_TRUE(core.calculate_link_transform(q, "tcp", T));

    EXPECT_FALSE(test_utils::initialize(core, "<robot"));
    EXPECT_FALSE(core.calculate_link_transform(q, "tcp", T)) << "stale state of the previous URDF";

    ASSERT_TRUE(test_utils::initialize(core, valid));
    EXPECT_TRUE(core.calculate_link_transform(q, "tcp", T));
}

// ---------------------------------------------------------------------------------------------
// 2a. geometry the old DH parser rejected is accepted now, and computed correctly. Each case is
// one deviation from the baseline robot, checked against the reference FK and its derivative.
// ---------------------------------------------------------------------------------------------

struct GeometryCase
{
    std::string name;
    std::function<void(UrdfSpec &)> mutate;
};

std::vector<GeometryCase> non_dh_cases()
{
    return {
        // the first joint moves the root link frame
        {"FirstOriginTranslated", [](UrdfSpec & s) {s.origins[0].xyz.x() = 0.01;}},
        {"FirstOriginRotated", [](UrdfSpec & s) {s.origins[0].rpy.z() = 0.1;}},
        // axes other than +z (the Jacobian must rotate the actual axis, not the z column)
        {"AxisX", [](UrdfSpec & s) {s.joints[2].axis = Eigen::Vector3d::UnitX();}},
        {"AxisMinusZ", [](UrdfSpec & s) {s.joints[4].axis = -Eigen::Vector3d::UnitZ();}},
        {"AxisOblique", [](UrdfSpec & s) {s.joints[1].axis = Eigen::Vector3d(0.3, -0.5, 0.8);}},
        {"AxisNotUnitLength", [](UrdfSpec & s) {s.joints[3].axis = Eigen::Vector3d(0.0, 0.0, 3.0);}},
        // origins that cannot be written as Rz(theta0) Tz(d) Tx(a) Rx(alpha)
        {"RotationAboutY", [](UrdfSpec & s) {s.origins[2].rpy.y() = 0.2;}},
        {"TranslationOutOfTheDhPlane", [](UrdfSpec & s) {
            const double yaw = s.origins[3].rpy.z();
            s.origins[3].xyz += Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(0, 0.01, 0);
        }},
    };
}

// gtest prints the case name in failure messages instead of a byte dump
void PrintTo(const GeometryCase & c, std::ostream * os) {*os << c.name;}

class NonDhGeometryTest : public ::testing::TestWithParam<GeometryCase> {};

TEST_P(NonDhGeometryTest, IsAcceptedAndMatchesTheReference)
{
    UrdfSpec spec = test_utils::make_spec(test_utils::robot_dh());
    GetParam().mutate(spec);
    const std::string urdf = spec.str();
    ASSERT_NE(urdf, test_utils::make_spec(test_utils::robot_dh()).str()) << "the case changes nothing";

    TestableKinematics core;
    ASSERT_TRUE(test_utils::initialize(core, urdf));
    const test_utils::ReferenceFk ref(urdf);
    std::mt19937 rng(99);
    for (int k = 0; k < 5; ++k) {
        SCOPED_TRACE("q = " + std::to_string(k));
        const Eigen::VectorXd q = ref.random_q(rng);
        expect_fk_matches_reference(core, ref, q);
        expect_jacobian_matches_reference(core, ref, q);
    }
}

INSTANTIATE_TEST_SUITE_P(
    FormerDhRestrictions, NonDhGeometryTest, ::testing::ValuesIn(non_dh_cases()),
    [](const ::testing::TestParamInfo<GeometryCase> & info) {return info.param.name;});

// ---------------------------------------------------------------------------------------------
// 2b. initialize() rejects everything that is not valid. Each case is one deviation from an URDF
// that is accepted (see AcceptsTheUnmodifiedUrdf...), so a failure has exactly one cause.
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

std::vector<InvalidCase> invalid_cases()
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<InvalidCase> c;
    auto add = [&c](std::string name, std::function<void(UrdfSpec &)> mutate) {
        c.push_back({std::move(name), std::move(mutate), false, "", {}});
    };

    // chain structure: serial up to the flange, at least one revolute joint in front of it
    add("BranchingChain", [](UrdfSpec & s) {s.branch = true;});
    add("OnlyTheFlangeAndTcpJoints", [](UrdfSpec & s) {
        while (s.joints.size() > 2) {remove_joint(s, 0);}
    });
    add("NoJoints", [](UrdfSpec & s) {
        while (!s.joints.empty()) {remove_joint(s, 0);}
    });

    // flange and tcp: fixed names, the flange joint fixed
    add("NoFlangeJoint", [](UrdfSpec & s) {s.flange_joint = "wrist_joint";});
    add("FlangeJointWithAnotherChild", [](UrdfSpec & s) {s.flange_link = "mount";});
    add("FlangeJointNotFixed", [](UrdfSpec & s) {s.joints[s.joints.size() - 2].type = "revolute";});
    add("NoTcpBehindTheFlange", [](UrdfSpec & s) {s.tcp_link = "tool0";});

    // a revolute joint needs a direction to rotate about
    add("ZeroAxis", [](UrdfSpec & s) {s.joints[2].axis = Eigen::Vector3d::Zero();});

    // joint types and limits
    add("ContinuousJoint", [](UrdfSpec & s) {s.joints[1].type = "continuous";});
    add("PrismaticJoint", [](UrdfSpec & s) {s.joints[1].type = "prismatic";});
    add("FixedJointInsideTheChain", [](UrdfSpec & s) {s.joints[2].type = "fixed";});
    // (a tcp joint that is not fixed is valid now: tool joints are taken at zero, see the "tool" robot)
    add("MissingLimits", [](UrdfSpec & s) {s.joints[0].has_limit = false;});
    add("LowerEqualsUpper", [](UrdfSpec & s) {s.joints[3].lower = s.joints[3].upper = 0.5;});
    add("LowerAboveUpper", [](UrdfSpec & s) {s.joints[3].lower = 1.0; s.joints[3].upper = -1.0;});
    add("ZeroVelocityLimit", [](UrdfSpec & s) {s.joints[1].velocity = 0.0;});
    add("ZeroEffortLimit", [](UrdfSpec & s) {s.joints[1].effort = 0.0;});

    // not an URDF at all
    c.push_back({"MalformedXml", nullptr, true, "<robot name=\"x\"><link", {}});
    c.push_back({"EmptyString", nullptr, true, "", {}});

    // the valid URDF with an invalid parameter
    c.push_back({"LambdaZero", nullptr, false, "", {rclcpp::Parameter("lambda", 0.0)}});
    c.push_back({"LambdaNegative", nullptr, false, "", {rclcpp::Parameter("lambda", -0.1)}});
    c.push_back({"LambdaNan", nullptr, false, "", {rclcpp::Parameter("lambda", nan)}});
    c.push_back({"LambdaInfinite", nullptr, false, "",
        {rclcpp::Parameter("lambda", std::numeric_limits<double>::infinity())}});
    c.push_back({"LambdaWrongType", nullptr, false, "",
        {rclcpp::Parameter("lambda", std::string("abc"))}});
    c.push_back({"JinvMethodUnknown", nullptr, false, "",
        {rclcpp::Parameter("jinv_method", std::string("qr"))}});
    c.push_back({"JinvMethodUpperCase", nullptr, false, "",
        {rclcpp::Parameter("jinv_method", std::string("SVD"))}});
    c.push_back({"JinvMethodWrongType", nullptr, false, "", {rclcpp::Parameter("jinv_method", 1)}});
    return c;
}

void PrintTo(const InvalidCase & c, std::ostream * os) {*os << c.name;}

class InvalidUrdfTest : public ::testing::TestWithParam<InvalidCase> {};

TEST(InvalidUrdf, AcceptsTheUnmodifiedUrdfTheCasesAreDerivedFrom)
{
    TestableKinematics core;
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

    TestableKinematics core;
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
