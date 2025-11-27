#pragma once

#include "Point.hpp"
using Vector = Point;

struct Plane
{
private:
    Vector normal;
    Point p1, p2, p3;
    real d;

public:
    Plane(Vector normal, real d) : normal{normal}, d{d} {}

    Plane(Point p1, Point p2, Point p3) : p1{p1}, p2{p2}, p3{p3}
    {
        normal = (p2 - p1).cross(p3 - p1).normalized();
        d = -normal.dot(p1);
    }
    Plane() {}

    Vector getNormal() const
    {
        return normal;
    }
    void invert()
    {
        normal = normal * (-1);
    }

    real getDistance(Point p) const
    {
        return normal.dot(p) + d;
    }
};