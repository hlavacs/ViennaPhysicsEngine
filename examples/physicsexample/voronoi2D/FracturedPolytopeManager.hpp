#pragma once

#include "VPE.hpp"

#include "triangulation.hpp"
#include "voronoi_generation.hpp"
#include "writeToOBJ.hpp"
#include "transformToPolytopes.hpp"

namespace VD
{

    inline static vpe::VPEWorld::Polytope g_wall{
        {{-1.0_real, -1.0_real, -0.25_real}, {-1.0_real, -1.0_real, 0.25_real}, {-1.0_real, 1.0_real, 0.25_real}, {-1.0_real, 1.0_real, -0.25_real}, {1.0_real, -1.0_real, 0.25_real}, {1.0_real, -1.0_real, -0.25_real}, {1.0_real, 1.0_real, -0.25_real}, {1.0_real, 1.0_real, 0.25_real}},
        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {5, 0}, {1, 4}, {3, 6}, {7, 2}}, // edges
        {
            {{0, 1}, {1, 1}, {2, 1}, {3, 1}},       // face 0
            {{4, 1}, {5, 1}, {6, 1}, {7, 1}},       // face 1
            {{0, -1}, {8, -1}, {4, -1}, {9, -1}},   // face 2
            {{2, -1}, {11, -1}, {6, -1}, {10, -1}}, // face 3
            {{3, -1}, {10, 1}, {5, -1}, {8, 1}},    // face 4
            {{1, -1}, {9, 1}, {7, -1}, {11, 1}}     // face 5
        },
        [](real mass, glmvec3 &s) { // callback for calculating the inertia tensor of this polytope
            return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
        }};

    struct FracturedPolytope
    {
    private:
        std::string name;
        vpe::VPEWorld::Polytope polytope;
        std::vector<vpe::VPEWorld::Polytope> fractured_polytopes;
        std::vector<glmvec3> translation;

    public:
        FracturedPolytope() : polytope{g_wall}
        {
            auto boundary_values = VD::getBoundaryValues(polytope.m_vertices);
            auto result = VD::fracture_polytope(boundary_values, 12); // default 12

            std::vector<VD::Voronoi> clipped_voronoi = VD::transformToVoronoi(result.first, result.second);
            VD::clipVoronoiToBoundingBox(boundary_values.vertices, clipped_voronoi);

            this->translation = VD::center_voronois(clipped_voronoi);
            VD::transformToPolytopes(this->fractured_polytopes, clipped_voronoi, boundary_values);
            VD::writeToOBJ("wall", clipped_voronoi, {boundary_values.min_z, boundary_values.max_z});
        }

        vpe::VPEWorld::Polytope getPolytope() const
        {
            return polytope;
        }

        std::vector<vpe::VPEWorld::Polytope> getFracturedPolytopes() const
        {
            return fractured_polytopes;
        }

        const std::vector<glmvec3> getTranslationVectors() const
        {
            return translation;
        }
    };

}