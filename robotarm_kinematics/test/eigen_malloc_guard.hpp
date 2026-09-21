#ifndef ROBOTARM_KINEMATICS_EIGEN_MALLOC_GUARD_HPP
#define ROBOTARM_KINEMATICS_EIGEN_MALLOC_GUARD_HPP

// Force-included (-include) into kinematics_core_malloc_test only, before any Eigen header.
//
// EIGEN_RUNTIME_NO_MALLOC lets Eigen::internal::set_is_malloc_allowed(false) turn every heap
// allocation made by Eigen into a failed eigen_assert. The default eigen_assert is the plain
// assert(), which is compiled out with NDEBUG (Release builds), and the guard would silently do
// nothing. So eigen_assert is replaced by one that always throws.

#include <stdexcept>

#define EIGEN_RUNTIME_NO_MALLOC
#define eigen_assert(x) \
    do {if (!(x)) {throw std::runtime_error(#x);}} while (0)

#endif  // ROBOTARM_KINEMATICS_EIGEN_MALLOC_GUARD_HPP
