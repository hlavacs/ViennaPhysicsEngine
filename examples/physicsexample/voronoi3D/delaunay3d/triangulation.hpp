#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <type_traits>

#include "VPE.hpp"
#include "face.hpp"
#include "tetrahedra.hpp"
#include "misc.hpp"

inline std::vector<glmvec3> getInitRandomPoints(size_t n)
{
    /*
    TODO: Prevent duplicate points from generating
    TODO: Prevent 4 points being on a sphere
    TODO: Calculate points based on dimension of convex polyhedron
    */
    std::default_random_engine rnd_gen{12345};
    std::uniform_real_distribution<> rnd_unif{-1, 1};

    std::vector<glmvec3> startings_points;
    for (size_t idx = 0; idx < n; ++idx)
    {
        real x = (real)rnd_unif(rnd_gen);
        real y = (real)rnd_unif(rnd_gen);
        real z = (real)rnd_unif(rnd_gen);

        startings_points.push_back(glmvec3{x, y, z});
    }

    return startings_points;
}

inline Tetrahedra getSuperTetrahedron(const std::vector<glmvec3> &points)
{
    real min_x = std::numeric_limits<real>::max();
    real min_y = std::numeric_limits<real>::max();
    real min_z = std::numeric_limits<real>::max();

    real max_x = std::numeric_limits<real>::min();
    real max_y = std::numeric_limits<real>::min();
    real max_z = std::numeric_limits<real>::min();

    for (const glmvec3 &forEachPoint : points)
    {
        min_x = std::min(min_x, forEachPoint.x);
        min_y = std::min(min_y, forEachPoint.y);
        min_z = std::min(min_z, forEachPoint.z);

        max_x = std::max(max_x, forEachPoint.x);
        max_y = std::max(max_y, forEachPoint.y);
        max_z = std::max(max_z, forEachPoint.z);
    }

    real mid_x = (min_x + max_x) / 2.0_real;
    real mid_y = (min_y + max_y) / 2.0_real;
    real mid_z = (min_z + max_z) / 2.0_real;

    real deltaMax = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    real scale = 10;

    return Tetrahedra({mid_x - deltaMax * scale, mid_y - deltaMax * scale, mid_z - deltaMax},
                      {mid_x, mid_y + deltaMax * scale, mid_z - deltaMax * scale},
                      {mid_x + deltaMax * scale, mid_y - deltaMax, mid_z - deltaMax},
                      {mid_x, mid_y + deltaMax, mid_z + deltaMax * scale});
}

inline bool arePointsCoplanar(std::vector<glmvec3> points, glmvec3 point)
{
    // https://www.cuemath.com/geometry/coplanar/
    glmvec3 a = points[0];
    glmvec3 b = points[1];
    glmvec3 c = points[2];
    glmvec3 d = point;

    glmvec3 AB = b - a;
    glmvec3 BC = c - b;
    glmvec3 CD = d - c;

    glmmat3 m(AB, BC, CD);

    return glm::determinant(m) == 0;
}

inline std::vector<Tetrahedra> BowyerWatson(std::vector<glmvec3> pointList)
{
    std::vector<Tetrahedra> triangulation = {};
    Tetrahedra superTetrahedron = getSuperTetrahedron(pointList);
    triangulation.push_back(superTetrahedron);

    for (const glmvec3 &eachPoint : pointList)
    {
        std::vector<Face> badTetrahedronFaces = {};
        for (Tetrahedra &eachTetrahedron : triangulation)
        {
            if (eachTetrahedron.isInCircumSphere(eachPoint))
            {
                eachTetrahedron.setBad(true);
                std::vector<Face> faces = eachTetrahedron.getFaces();
                badTetrahedronFaces.insert(badTetrahedronFaces.end(), faces.begin(), faces.end());
            }
        }

        for (size_t first_idx = 0; first_idx < badTetrahedronFaces.size(); first_idx++)
        {
            for (size_t second_idx = first_idx + 1; second_idx < badTetrahedronFaces.size(); second_idx++)
            {
                if (badTetrahedronFaces[first_idx].isFaceEqualTo(badTetrahedronFaces[second_idx]))
                {
                    badTetrahedronFaces[first_idx].setBad(true);
                    badTetrahedronFaces[second_idx].setBad(true);
                }
            }
        }

        badTetrahedronFaces.erase(std::remove_if(badTetrahedronFaces.begin(), badTetrahedronFaces.end(), [](Face &face)
                                                 { return face.getBad(); }),
                                  badTetrahedronFaces.end());

        triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [](Tetrahedra &t)
                                           { return t.getBad(); }),
                            triangulation.end());

        for (Face eachFace : badTetrahedronFaces)
        {
            if (!arePointsCoplanar(eachFace.getPoints(), eachPoint))
            {
                triangulation.push_back(Tetrahedra(eachFace, eachPoint));
            }
        }
    }

    std::vector<glmvec3> vertices = superTetrahedron.getPoints();
    triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [vertices](Tetrahedra &t)
                                       { return t.containsPoint(vertices[0]) ||
                                                t.containsPoint(vertices[1]) ||
                                                t.containsPoint(vertices[2]) ||
                                                t.containsPoint(vertices[3]); }),
                        triangulation.end());

    return triangulation;
}

inline std::pair<std::vector<Tetrahedra>, std::vector<glmvec3>> fracture_polytope(vpe::VPEWorld::Polytope *polytope, size_t amount)
{
    std::vector<glmvec3> pointList = getInitRandomPoints(amount);
    std::vector<Tetrahedra> tetrahedrons = BowyerWatson(pointList);
    return {tetrahedrons, pointList};
}