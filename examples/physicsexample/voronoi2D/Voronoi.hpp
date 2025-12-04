#pragma once

#include <iostream>
#include <vector>

#include "Point.hpp"
#include "Edge.hpp"

#include "VPE.hpp"

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

        std::vector<Point> getVertices()
        {
            return vertices;
        }

        std::vector<glmvec2> getVec2Vertices()
        {
            std::vector<glmvec2> result = {};
            for (auto each_vertice : vertices)
            {
                result.push_back(glmvec2(each_vertice.getX(), each_vertice.getY()));
            }
            std::reverse(result.begin(), result.end());
            return result;
        }

        void setVec2Vertices(std::vector<glmvec2> vec2_vertices)
        {
            vertices.clear();
            vec2_vertices.pop_back();
            for (auto each_vertice : vec2_vertices)
            {
                Point p = Point(each_vertice.x, each_vertice.y);
                vertices.push_back(p);
            }
            std::reverse(vertices.begin(), vertices.end());
        }

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