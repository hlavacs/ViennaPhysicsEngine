#pragma once

#include "VPE.hpp"

struct Point
{
private:
    real x, y, z;

public:
    Point(real x, real y, real z) : x{x}, y{y}, z{z} {}
    Point(glmvec3 point)
    {
        x = point.x;
        y = point.y;
        z = point.z;
    }
    Point() {}

    real getX() const
    {
        return x;
    }

    real getY() const
    {
        return y;
    }

    real getZ() const
    {
        return z;
    }

    real length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Point normalized()
    {
        return Point(x / length(), y / length(), z / length());
    }

    real dot(Point p) const
    {
        return x * p.x + y * p.y + z * p.z;
    }

    Point cross(Point p) const
    {
        return Point(y * p.z - z * p.y, z * p.x - x * p.z, p.x * p.y - y * p.x);
    }

    Point operator+(Point p)
    {
        return Point(x + p.x, y + p.y, z + p.z);
    }

    Point operator-(Point p)
    {
        return Point(x - p.x, y - p.y, z - p.z);
    }

    Point operator*(double d)
    {
        return Point(real(x * d), real(y * d), real(z * d));
    }

    Point operator*(float f)
    {
        return Point(x * f, y * f, z * f);
    }

    Point operator*(int i)
    {
        return Point(x * i, y * i, z * i);
    }

    glmvec3 vec3()
    {
        return glmvec3{x, y, z};
    }

    bool operator<(const Point &p) const
    {
        return std::tie(x, y, z) < std::tie(p.x, p.y, p.z);
    }

    bool operator=(const Point &p) const
    {
        return std::tie(x, y, z) == std::tie(p.x, p.y, p.z);
    }

    friend std::ostream &operator<<(std::ostream &o, const Point &p)
    {
        return o << "Point (" << p.x << ", " << p.y << ", " << p.z << ")";
    }
};