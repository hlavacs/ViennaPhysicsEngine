#pragma once

#include <iostream>
#include "VPE.hpp"

#include "Point.hpp"

namespace std
{
    inline ostream &operator<<(ostream &os, const glmvec4 &vec)
    {
        os << "(" << vec.x << ',' << vec.y << ',' << vec.z << ',' << vec.w << ")"; // output quaternion
        return os;
    }
}

namespace VD
{
    /**
     * Comparison accounting for floating point precision errors
     */
    inline bool equal_within_ulps(real x, real y)
    {
        return std::fabs(x - y) < std::numeric_limits<real>::min() ||
               std::fabs(x - y) <= std::numeric_limits<real>::epsilon() * std::max(std::fabs(x), std::fabs(y));
    }

    /**
     * Equal Comparison between Vectors/Points
     */
    inline bool isVectorEqualTo(const Point &a, const Point &b)
    {
        return equal_within_ulps(a.getX(), b.getX()) && equal_within_ulps(a.getY(), b.getY());
    }

    inline Point getAveragePoint(std::vector<Point> points)
    {
        real x = 0.0_real;
        real y = 0.0_real;

        real size = real(points.size());
        for (Point each_point : points)
        {
            x += each_point.getX();
            y += each_point.getY();
        }

        return Point(x / size, y / size);
    }
}
