#pragma once

#include <iostream>
#include "VPE.hpp"

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
    inline PolytopeInfo getPolytopeInfo(const std::vector<vpe::VPEWorld::Vertex> &poly_vertices)
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
