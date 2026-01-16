#pragma once

#include <iostream>
#include <vector>

#include "Voronoi.hpp"
#include "VPE.hpp"

namespace VD
{
    /// <summary>
    /// Transform 2D Voronoi to 3D polytopes by extruding them into z-dimension.
    /// </summary>
    /// <param name="polytopes"> vector in which polytopes are saved in .</param>
    /// <param name="voronois"> 2D Voronoi .</param>
    /// <param name="min_z"> negative z dimension of polytope.</param>
    /// <param name="max_z"> positive z dimension of polytope.</param>
    void transformToPolytopes(std::vector<vpe::VPEWorld::Polytope> &polytopes, const std::vector<Voronoi> &voronois, const real &min_z, const real &max_z)
    {
        for (const Voronoi &eachVoronoi : voronois)
        {
            /** Fetch voronoi vertices */
            std::vector<glmvec2> voronoi_vertices = eachVoronoi.getVertices();
            uint32_t vertices_size = uint32_t(voronoi_vertices.size());

            /** Polytope variables */
            std::vector<glmvec3> vertices = {};
            std::vector<std::pair<uint_t, uint_t>> edges = {};
            std::vector<std::vector<vpe::VPEWorld::signed_edge_t>> faces = {};

            /** Add z dimension to vertices */
            /** Add Front vertices */
            for (const auto &each_vertice : voronoi_vertices)
            {
                vertices.push_back({each_vertice.x, each_vertice.y, max_z});
            }
            /** Add back vertices */
            for (const auto &each_vertice : voronoi_vertices)
            {
                vertices.push_back({each_vertice.x, each_vertice.y, min_z});
            }

            /**Add Front edges */
            for (uint32_t index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    edges.push_back({index, 0});
                }
                else
                {
                    edges.push_back({index, index + 1});
                }
            }
            /**Add Back edges */
            for (uint32_t index = vertices_size; index < vertices_size * 2; ++index)
            {
                if (index == vertices_size * 2 - 1)
                {
                    edges.push_back({index, vertices_size});
                }
                else
                {
                    edges.push_back({index, index + 1});
                }
            }

            /**Add Side edges */
            for (uint32_t index = 0; index < vertices_size; ++index)
            {
                edges.push_back({index, vertices_size + index});
            }

            /**Add faces */

            /**Front Face */
            std::vector<vpe::VPEWorld::signed_edge_t> front_face;
            for (uint32_t index = 0; index < voronoi_vertices.size(); ++index)
            {
                front_face.push_back({index, 1});
            }
            faces.push_back(front_face);

            /**Back Face */
            std::vector<vpe::VPEWorld::signed_edge_t> back_face;
            for (uint32_t index = vertices_size; index < vertices.size(); ++index)
            {
                back_face.push_back({index, -1});
            }
            std::reverse(back_face.begin(), back_face.end());
            faces.push_back(back_face);

            /**Side Faces */
            for (uint32_t index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    faces.push_back({{index + vertices_size * 2, 1}, {index + vertices_size, 1}, {index + vertices_size + 1, -1}, {index, -1}});
                }
                else
                {
                    faces.push_back({{index + vertices_size * 2, 1}, {index + vertices_size, 1}, {index + vertices_size * 2 + 1, -1}, {index, -1}});
                }
            }

            /** Create polytope */
            vpe::VPEWorld::Polytope polytope = vpe::VPEWorld::Polytope(
                std::move(vertices), std::move(edges), std::move(faces), [](real mass, glmvec3 &s) { // callback for calculating the inertia tensor of this polytope
                    return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
                });

            polytopes.push_back(std::move(polytope));
        }
        // return std::move(polytopes);
    }
}