#pragma once

#include "VPE.hpp"
#include "tetrahedra.hpp"

namespace poly_transform
{

    vpe::VPEWorld::Polytope transformToPolytope(Tetrahedra tetrahedra)
    {
        std::vector<glmvec3> points = tetrahedra.getPoints();
        glmvec3 a = points[0];
        glmvec3 b = points[1];
        glmvec3 c = points[2];
        glmvec3 d = points[3];

        return vpe::VPEWorld::Polytope{

            {a, b, c, d},

            {{0, 1}, {1, 2}, {2, 3}, {0, 3}, {1, 3}, {2, 3}},

            {{{0, 1}, {1, 2}, {2, 0}},

             {{0, 1}, {1, 3}, {3, 0}},

             {{1, 2}, {2, 3}, {3, 2}},

             {{0, 2}, {2, 3}, {3, 0}}},

            [](real mass, glmvec3 &s)
            {
                return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
            }};
    }

    std::vector<vpe::VPEWorld::Polytope> transformMultiToPolytopes(std::vector<Tetrahedra> tetrahedras)
    {
        std::vector<vpe::VPEWorld::Polytope> polytopes;
        for (auto it = tetrahedras.begin(); it != tetrahedras.end(); ++it)
        {
            vpe::VPEWorld::Polytope polytope = transformToPolytope(*it);
            polytopes.push_back(polytope);
        }
        return polytopes;
    }

}
