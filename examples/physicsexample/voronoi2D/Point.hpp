#pragma once

#include "VPE.hpp"

namespace VD
{
    /**
     * 2D Point
     */
    struct Point
    {
    private:
        real x, y;

    public:
        Point(real x, real y) : x{x}, y{y} {}
        Point() {}

        real getX() const
        {
            return x;
        }

        real getY() const
        {
            return y;
        }

        real length() const
        {
            return std::sqrt(x * x + y * y);
        }

        Point normalized()
        {
            return Point(x / length(), y / length());
        }

        real dot(Point p) const
        {
            return x * p.x + y * p.y;
        }

        real cross(Point p) const
        {
            return x * p.y - p.x * y;
        }

        Point operator+(Point p)
        {
            return Point(x + p.x, y + p.y);
        }

        Point operator-(Point p)
        {
            return Point(x - p.x, y - p.y);
        }

        Point operator*(double d)
        {
            return Point(real(x * d), real(y * d));
        }

        Point operator*(float f)
        {
            return Point(x * f, y * f);
        }

        Point operator*(int i)
        {
            return Point(x * i, y * i);
        }

        glmvec2 vec2()
        {
            return glmvec2{x, y};
        }

        friend bool operator<(const Point &p1, const Point &p2)
        {
            return std::tie(p1.x, p2.y) < std::tie(p2.x, p2.y);
        }

        bool operator==(const Point &p) const
        {
            return (x == p.x) && (y == p.y);
        }

        friend std::ostream &operator<<(std::ostream &o, const Point &p)
        {
            return o << "Point (" << p.x << ", " << p.y << ")";
        }
    };
}

/**
 *  Hash function for set
 */
namespace std
{
    template <>
    struct std::hash<VD::Point>
    {
        std::size_t operator()(const VD::Point &p) const
        {
            size_t h0 = std::hash<real>{}(p.getX());
            size_t h1 = std::hash<real>()(p.getY()) + 0x9e3779b9 + (h0 << 6) + (h0 >> 2);
            return h1;
        }
    };
}
