#pragma once

#include <iostream>
#include <vector>

#include "Point.hpp"
#include "Edge.hpp"

namespace VD
{
    struct Voronoi
    {
    private:
        Point center;
        std::vector<Point> vertices;

    public:
        Voronoi(Point center, std::vector<Point> vertices) : center{center}, vertices{vertices} {}
        Voronoi() {}

        friend std::ostream &operator<<(std::ostream &o, const Voronoi &v)
        {
            o << "Voronoi Diagram (" << v.center.getX() << v.center.getY() << ")\n";
            for (auto vertice : v.vertices)
            {
                o << vertice << "\n";
            }
            o << "\n";
            return o;
        }
    };
}