#pragma once

#include "VPE.hpp"
#include "voronoi3D/delaunay3d/tetrahedra.hpp"
#include "voronoi3D/VoronoiEdge.hpp"
#include "voronoi3D/VoronoiFace.hpp"
#include "voronoi3D/Voronoi3D.hpp"
#include "voronoi3D/Point.hpp"
#include "voronoi3D/Plane.hpp"

inline real signedVolume(std::vector<glmvec3> points)
{

    return glm::dot((points[1] - points[0]), glm::cross((points[2] - points[0]), (points[3] - points[0]))) / 6.0_real;
}

inline glmvec3 translationVector(std::vector<glmvec3> points)
{
    glmvec3 translation = glmvec3{0, 0, 0};
    for (const auto &eachpoint : points)
    {
        translation += eachpoint;
    }
    return translation * 0.25_real;
}

inline std::vector<vpe::VPEWorld::Polytope> convertToPolytope(std::vector<Tetrahedra> tetrahedras)
{

    std::vector<vpe::VPEWorld::Polytope> result = {};
    for (Tetrahedra eachTetrahedra : tetrahedras)
    {
        std::vector<glmvec3> points = eachTetrahedra.getPoints();
        glmvec3 translation = translationVector(points);
        glmvec3 p0, p1, p2, p3;
        if (signedVolume(points) < 0)
        {
            p0 = points[1] - translation;
            p1 = points[0] - translation;
            p2 = points[2] - translation;
            p3 = points[3] - translation;
        }
        else
        {
            p0 = points[0] - translation;
            p1 = points[1] - translation;
            p2 = points[2] - translation;
            p3 = points[3] - translation;
        }

        vpe::VPEWorld::Polytope poly{
            {p0, p1, p2, p3},
            {
                {0, 1}, // edge 0
                {1, 2}, // edge 1
                {2, 0}, // edge 2
                {0, 3}, // edge 3
                {1, 3}, // edge 4
                {2, 3}  // edge 5
            },
            {
                {{2, -1}, {1, -1}, {0, -1}}, // face 0
                {{0, 1}, {4, 1}, {3, -1}},   // face 1
                {{1, 1}, {5, 1}, {4, -1}},   // face 2
                {{2, 1}, {3, 1}, {5, -1}}    // face 3
            },
            [](real mass, glmvec3 &s) { // callback for calculating the inertia tensor of this polytope
                return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
            }};
        result.push_back(poly);
    }
    return result;
}

inline std::vector<Voronoi> transformDelaunayToVoronoi(std::vector<Tetrahedra> tetrahedras, std::vector<glmvec3> starting_points)
{
    /*
    Delaunay vertex -> Voronoi cell
    Delaunay edge -> Voronoi face
    Delaunay triangular face -> Voronoi edge
    delaunay tetrahedron -> voronoi vertex
    */
    std::vector<Voronoi> voronois;
    std::vector<glmvec3> voronoiCenters = starting_points;
    for (auto &eachVoronoiCenter : voronoiCenters)
    {
        std::vector<glmvec3> voronoiVertices = {};
        std::vector<VoronoiEdge> voronoiEdges = {};

        std::vector<Tetrahedra> relevantTetra = {};
        std::copy_if(tetrahedras.begin(), tetrahedras.end(), std::back_inserter(relevantTetra), [eachVoronoiCenter](Tetrahedra &t)
                     { return t.containsPoint(eachVoronoiCenter); });

        for (auto first_iter = relevantTetra.begin(); first_iter != relevantTetra.end(); first_iter++)
        {
            for (auto second_iter = first_iter + 1; second_iter != relevantTetra.end(); second_iter++)
            {
                std::vector<Face> firstFaces = first_iter->getFaces();
                std::vector<Face> secondFaces = second_iter->getFaces();

                for (Face eachFirstFace : firstFaces)
                {
                    for (Face eachSecondFace : secondFaces)
                    {
                        if (eachFirstFace.isFaceEqualTo(eachSecondFace) && first_iter != second_iter)
                        {
                            std::cout << "TETRA CHECK \n";
                            std::cout << *first_iter << "\n"
                                      << *second_iter << "\n";
                            std::cout << "FACE CHECK\n";
                            std::cout << eachFirstFace << "\n is equal to \n " << eachSecondFace << "\n";
                            std::cout << "POINT CHECK";
                            std::cout << first_iter->getCenter() << "to " << second_iter->getCenter() << '\n';
                            voronoiEdges.push_back(VoronoiEdge(first_iter->getCenter(), second_iter->getCenter()));
                        }
                    }
                }
            }
        }

        voronois.push_back(Voronoi(eachVoronoiCenter, voronoiEdges));
    }

    return voronois;
}
