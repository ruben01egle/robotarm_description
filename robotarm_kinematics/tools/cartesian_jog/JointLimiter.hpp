#ifndef ROBOTARM_KINEMATICS_JOINTLIMITER_HPP
#define ROBOTARM_KINEMATICS_JOINTLIMITER_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace robotarm_kinematics
{

struct JointLimit
{
    double min = 0.0;       // rad
    double max = 0.0;       // rad
    double velocity = 0.0;  // rad/s
};

enum class StepLimit
{
    None,      // the step is used as it is
    Velocity,  // scaled down: a joint would exceed its velocity limit
    Position,  // scaled down: a joint would leave its position range
    Invalid    // scale is 0: non-finite input, wrong sizes, dt <= 0 or invalid limits
};

struct StepLimitResult
{
    double scale = 1.0;       // factor in [0, 1] to apply to the whole joint step
    StepLimit reason = StepLimit::None;
    Eigen::Index joint = -1;  // joint that determined the scale, -1 if none
};

// Limits one joint step dq (rad, taken during dt seconds) that starts at q.
//
// The WHOLE step is scaled by one common factor, never single joints. The joints keep their ratio,
// so the Cartesian direction of the motion is preserved: the tool only gets slower, or stops, but
// does not drift sideways. The tightest joint decides:
//  - velocity: |dq_i| / dt <= velocity_i
//  - position: q_i + dq_i stays inside [min_i, max_i]. A step towards a limit ends exactly at the
//    limit. A joint that is at (or, from outside, beyond) a limit and moves further out stops the
//    whole step (scale 0), while moving back in is always allowed.
// Does not allocate.
inline StepLimitResult limit_joint_step(
    const Eigen::VectorXd & q, const Eigen::VectorXd & dq,
    const std::vector<JointLimit> & limits, double dt)
{
    StepLimitResult result;
    const auto invalid = [&result]() {
        result.scale = 0.0;
        result.reason = StepLimit::Invalid;
        result.joint = -1;
        return result;
    };

    const Eigen::Index n = static_cast<Eigen::Index>(limits.size());
    if (q.size() != n || dq.size() != n || !std::isfinite(dt) || !(dt > 0.0) ||
        !q.allFinite() || !dq.allFinite())
    {
        return invalid();
    }
    for (const JointLimit & l : limits) {
        if (!std::isfinite(l.min) || !std::isfinite(l.max) || !(l.min <= l.max) ||
            !std::isfinite(l.velocity) || !(l.velocity > 0.0))
        {
            return invalid();
        }
    }

    for (Eigen::Index i = 0; i < n; ++i) {
        const double step = dq[i];
        if (step == 0.0) {
            continue;
        }

        const double velocity_scale = limits[i].velocity * dt / std::abs(step);
        if (velocity_scale < result.scale) {
            result.scale = velocity_scale;
            result.reason = StepLimit::Velocity;
            result.joint = i;
        }

        // signed distance to the limit in the direction of the step: >= 0 inside the range,
        // < 0 if the joint is already beyond that limit, which gives a negative ratio -> 0
        const double room = (step > 0.0 ? limits[i].max : limits[i].min) - q[i];
        const double position_scale = std::max(0.0, room / step);
        if (position_scale < result.scale) {
            result.scale = position_scale;
            result.reason = StepLimit::Position;
            result.joint = i;
        }
    }
    return result;
}

}  // namespace robotarm_kinematics

#endif  // ROBOTARM_KINEMATICS_JOINTLIMITER_HPP
