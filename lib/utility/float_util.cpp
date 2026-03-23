#include "float_util.hpp"

#include <cmath>

namespace utility
{

bool isAlmostEqualRelative(const double a, const double b, const double epsilon)
{
    // Relative epsilon comparison - https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
    // Using a+b is more lenient but faster than first finding the larger value and then using that
    if (std::fabs(a - b) <= (std::fabs(a + b) * epsilon))
        return true;
    return false;
}

bool isAlmostEqual(const double a, const double b, const double epsilon)
{

    // Absolute epsilon comparison - https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
    // Assumes the user can reasonably set the epsilon
    if (std::fabs(a - b) <= epsilon)
        return true;
    return false;
}

bool isAlmostEqualToZero(const double a, const double epsilon)
{
    if (a == 0.0)
        return true;

    // Absolute epsilon comparison - https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
    // Assumes the user can reasonably set the epsilon
    if (std::fabs(a) <= epsilon)   
        return true;
    return false;
}

} // namespace utility