/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Definitions for math utilities used in @ref wr_control_drive_arm
 *
 */

#include "MathUtil.hpp"
#include <cmath>

namespace MathUtil {
auto corrMod(double dividend, double divisor) -> double {
    return std::fmod(std::fmod(dividend, divisor) + divisor, std::abs(divisor));
}
} // namespace MathUtil

/// @}
