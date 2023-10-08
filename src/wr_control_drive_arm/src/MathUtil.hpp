/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Declarations for math utilities used in @ref wr_control_drive_arm
 *
 */

#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>

/**
 * @brief Namespace for math utilies in @ref wr_control_drive_arm
 *
 */
namespace MathUtil {
/**
 * @brief Mathematically correct modulus functions over R, to double precision
 *
 * @param dividend x in x % y
 * @param divisor y in x % y
 * @return double x % y, bounded on [0,abs(y))
 */
auto corrMod(double dividend, double divisor) -> double;
/// 2*pi
static constexpr double RADIANS_PER_ROTATION{2 * M_PI};
} // namespace MathUtil

#endif

/// @}