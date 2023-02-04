#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>

namespace MathUtil {
auto corrMod(double dividend, double divisor) -> double;
static constexpr double RADIANS_PER_ROTATION{2 * M_PI};
} // namespace MathUtil

#endif