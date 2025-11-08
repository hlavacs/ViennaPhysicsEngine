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
    return std::fabs(x - y) < std::numeric_limits<real>::min() ||
           std::fabs(x - y) <= std::numeric_limits<real>::epsilon() * std::max(std::fabs(x), std::fabs(y));
}

inline bool isVectorEqualTo(const glmvec3 &a, const glmvec3 &b)
{
    return equal_within_ulps(a.x, b.x) && equal_within_ulps(a.y, b.y) && equal_within_ulps(a.z, b.z);
}
