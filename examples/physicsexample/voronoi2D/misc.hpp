#pragma once

#include <iostream>
#include "VPE.hpp"

#include "Voronoi.hpp"

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
    inline bool isVectorEqualTo(const glmvec2 &a, const glmvec2 &b)
    {
        return equal_within_ulps(a.x, b.x) && equal_within_ulps(a.y, b.y);
    }

    /**
     * Get average point of a list of points
     */
    inline glmvec2 getAveragePoint(const std::vector<glmvec2> &points)
    {
        real x = 0.0_real;
        real y = 0.0_real;

        real size = real(points.size());
        for (const auto &each_point : points)
        {
            x += each_point.x;
            y += each_point.y;
        }

        return {x / size, y / size};
    }

    /**
     * translate point by the average of the points
     */
    inline void translateVertices(std::vector<glmvec2> &points, const glmvec2 &translation)
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
            std::vector<glmvec2> voronoi_vertice = each_voronoi.getVertices();
            glmvec2 average = getAveragePoint(voronoi_vertice);
            translateVertices(voronoi_vertice, average);
            translationVector.push_back({average.x, average.y, 0});
            each_voronoi.setVertices(voronoi_vertice);
        }

        return translationVector;
    }

    /**
     * Compare angles between two point around the center point
     */
    inline static bool angle_comparison(const glmvec2 &p1, const glmvec2 &p2, const glmvec2 &center)
    {
        double angle1 = std::atan2(p1.y - center.y, p1.x - center.x);
        double angle2 = std::atan2(p2.y - center.y, p2.x - center.x);
        return angle1 < angle2;
    }
    /**
     * Sort vertices in counter clockwise order by angle
     */
    inline void sort_vertices_ccw(std::vector<glmvec2> &unordered_vertices)
    {
        glmvec2 center = getAveragePoint(unordered_vertices);
        std::sort(unordered_vertices.begin(), unordered_vertices.end(), [&center](const glmvec2 &p1, const glmvec2 &p2)
                  { return angle_comparison(p1, p2, center); });

        // Remove duplicate values in vector
        auto duplicates = std::unique(unordered_vertices.begin(), unordered_vertices.end());
        unordered_vertices.erase(duplicates, unordered_vertices.end());
    }

    struct PolytopeInfo
    {
        std::vector<glmvec2> vertices;
        real min_x;
        real max_x;
        real min_y;
        real max_y;
        real min_z;
        real max_z;

        PolytopeInfo(std::vector<glmvec2> vertices, real min_x, real max_x, real min_y, real max_y, real min_z, real max_z)
            : vertices{vertices}, min_x{min_x}, max_x{max_x}, min_y{min_y}, max_y{max_y}, min_z{min_z}, max_z{max_z} {}
    };
    /**
     * Find minimum and maximum x/y values of given vertices
     */
    inline PolytopeInfo getBoundaryValues(const std::vector<vpe::VPEWorld::Vertex> &poly_vertices)
    {
        std::vector<glmvec2> front_polytope_face = {};
        real min_x = std::numeric_limits<real>::max();
        real min_y = std::numeric_limits<real>::max();
        real min_z = std::numeric_limits<real>::max();

        real max_x = std::numeric_limits<real>::min();
        real max_y = std::numeric_limits<real>::min();
        real max_z = std::numeric_limits<real>::min();

        for (const auto &each_vertice : poly_vertices)
        {
            min_x = std::min(min_x, each_vertice.m_positionL.x);
            min_y = std::min(min_y, each_vertice.m_positionL.y);
            min_z = std::min(min_z, each_vertice.m_positionL.z);

            max_x = std::max(max_x, each_vertice.m_positionL.x);
            max_y = std::max(max_y, each_vertice.m_positionL.y);
            max_z = std::max(max_z, each_vertice.m_positionL.z);
        }

        for (const auto &each_vertice : poly_vertices)
        {
            if (each_vertice.m_positionL.z == max_z)
            {
                front_polytope_face.push_back({each_vertice.m_positionL.x, each_vertice.m_positionL.y});
            }
        }

        return PolytopeInfo(front_polytope_face, min_x, max_x, min_y, max_y, min_z, max_z);
    }
}
