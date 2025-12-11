#pragma once

#include <iostream>
#include "VPE.hpp"

#include "Point.hpp"
#include "Voronoi.hpp"

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

    /**
     * Get average point of a list of points
     */
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

    /**
     * translate point by the average of the points
     */
    inline void translateVertices(std::vector<Point> &points, Point translation)
    {
        for (auto &each_point : points)
        {
            each_point = each_point - translation;
        }
    }

    /**
     * Translate voronoi by the average of their vertices and return this vector of translations
     */
    inline std::vector<glmvec3> center_voronois(std::vector<Voronoi> &voronois)
    {
        std::vector<glmvec3> translationVector = {};
        for (auto &each_voronoi : voronois)
        {
            std::vector<Point> voronoi_vertice = each_voronoi.getVertices();
            Point average = getAveragePoint(voronoi_vertice);
            translateVertices(voronoi_vertice, average);
            translationVector.push_back({average.getX(), average.getY(), 0});
            each_voronoi.setVertices(voronoi_vertice);
        }

        return translationVector;
    }

    /**
     * Find minimum and maximum x/y values of given vertices
     */
    inline std::vector<real> getBoundaryValues(std::vector<vpe::VPEWorld::Vertex> &poly_vertices)
    {
        real min_x = std::numeric_limits<real>::max();
        real min_y = std::numeric_limits<real>::max();

        real max_x = std::numeric_limits<real>::min();
        real max_y = std::numeric_limits<real>::min();
        for (auto &each_vertice : poly_vertices)
        {
            min_x = std::min(min_x, each_vertice.m_positionL.x);
            min_y = std::min(min_y, each_vertice.m_positionL.y);

            max_x = std::max(max_x, each_vertice.m_positionL.x);
            max_y = std::max(max_y, each_vertice.m_positionL.y);
        }

        return {min_x, max_x, min_y, max_y};
    }
}
