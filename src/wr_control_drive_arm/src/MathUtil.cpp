#include "MathUtil.hpp"
#include <cmath>

namespace MathUtil {
auto corrMod(double dividend, double divisor) -> double {
    return std::fmod(std::fmod(dividend, divisor) + divisor, std::abs(divisor));
}
} // namespace MathUtil