#pragma once

#include <iostream>
#include "VPE.hpp"

namespace std
{
    inline ostream &operator<<(ostream &os, const glmvec4 &vec)
    {
        os << "(" << vec.x << ',' << vec.y << ',' << vec.z << ',' << vec.w << ")"; // output quaternion
        return os;
    }
}

inline bool equal_within_ulps(real x, real y)
{
    size_t n_ulps = 2;
    const real m = std::min(std::fabs(x), std::fabs(y));
    const int exp = m < std::numeric_limits<real>::min()
                        ? std::numeric_limits<real>::min_exponent - 1
                        : std::ilogb(m);
    return std::fabs(x - y) <= n_ulps * std::ldexp(std::numeric_limits<real>::epsilon(), exp);
}

inline bool isVectorEqualTo(const glmvec3 &a, const glmvec3 &b)
{
    return equal_within_ulps(a.x, b.x) && equal_within_ulps(a.y, b.y) && equal_within_ulps(a.z, b.z) ||
           equal_within_ulps(a.x, b.y) && equal_within_ulps(a.y, b.z) && equal_within_ulps(a.z, b.x) ||
           equal_within_ulps(a.x, b.z) && equal_within_ulps(a.y, b.x) && equal_within_ulps(a.z, b.y);
}
