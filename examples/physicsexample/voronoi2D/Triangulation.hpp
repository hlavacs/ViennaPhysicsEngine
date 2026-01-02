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

#include "Edge.hpp"

#include "Triangle.hpp"
#include "misc.hpp"

namespace VD
{
    /**
     * Returns a list of n amount of 2D points
     */
    inline std::set<glmvec2> getInitRandomPoints(const size_t &n, const PolytopeInfo &boundary_values)
    {
        std::default_random_engine rnd_gen{12345};
        std::uniform_real_distribution<> rnd_unif_x{boundary_values.min_x, boundary_values.max_x};
        std::uniform_real_distribution<> rnd_unif_y{boundary_values.min_y, boundary_values.max_y};

        std::set<glmvec2> startings_points = {};
        while (startings_points.size() < n)
        {
            real x = (real)rnd_unif_x(rnd_gen);
            real y = (real)rnd_unif_y(rnd_gen);
            startings_points.insert({x, y});
        }

        return startings_points;
    }
    /**
     * Forms a Triangle that encloses all given points
     */
    inline Triangle getSuperTriangle(const std::set<glmvec2> &points)
    {
        real min_x = std::numeric_limits<real>::max();
        real min_y = std::numeric_limits<real>::max();

        real max_x = std::numeric_limits<real>::min();
        real max_y = std::numeric_limits<real>::min();

        for (const auto &forEachPoint : points)
        {
            min_x = std::min(min_x, forEachPoint.x);
            min_y = std::min(min_y, forEachPoint.y);

            max_x = std::max(max_x, forEachPoint.x);
            max_y = std::max(max_y, forEachPoint.y);
        }

        real mid_x = (min_x + max_x) / 2.0_real;
        real mid_y = (min_y + max_y) / 2.0_real;

        real deltaMax = std::max({max_x - min_x, max_y - min_y});
        real scale = 10;

        return Triangle({mid_x, mid_y + deltaMax * scale},
                        {mid_x - deltaMax * scale, mid_y - deltaMax},
                        {mid_x + deltaMax * scale * scale, mid_y - deltaMax});
    }

    /**
     *  Slope Method to check if 3 points are collinear
     */
    inline bool arePointsColinear(const Edge &e, const glmvec2 &c)
    {
        glmvec2 a = e.getA();
        glmvec2 b = e.getB();
        real slopeAB = (b.y - a.y) / (b.x - a.x);
        real slopeBC = (c.y - b.y) / (c.x - b.x);
        return slopeAB == slopeBC;
    }

    /**
     * Bowyer-Watson Algorithm to construct delaunay triangles from a given set of points
     */
    inline std::vector<Triangle> BowyerWatson(const std::set<glmvec2> &pointList)
    {
        /**Start with empty triangles */
        std::vector<Triangle> triangulation = {};
        /**Create Super Triangle that encompasses all points */
        Triangle superTriangle = getSuperTriangle(pointList);
        triangulation.push_back(superTriangle);

        /** For each point */
        for (const glmvec2 &eachPoint : pointList)
        {
            std::vector<Edge> badTriangleEdges = {};
            /** If point is in triangle, mark it and save it */
            for (Triangle &eachTriangle : triangulation)
            {
                if (eachTriangle.isInCircumcircle(eachPoint))
                {
                    eachTriangle.setBad(true);
                    std::vector<Edge> edges = eachTriangle.getEdges();
                    badTriangleEdges.insert(badTriangleEdges.end(), edges.begin(), edges.end());
                }
            }

            /**Iterate through the edges and mark the duplicate edges as bad */
            for (size_t first_idx = 0; first_idx < badTriangleEdges.size(); first_idx++)
            {
                for (size_t second_idx = first_idx + 1; second_idx < badTriangleEdges.size(); second_idx++)
                {
                    if (badTriangleEdges[first_idx] == (badTriangleEdges[second_idx]))
                    {
                        badTriangleEdges[first_idx].setBad(true);
                        badTriangleEdges[second_idx].setBad(true);
                    }
                }
            }
            /** Remove bad edges */
            badTriangleEdges.erase(std::remove_if(badTriangleEdges.begin(), badTriangleEdges.end(), [](const Edge &edge)
                                                  { return edge.isBad(); }),
                                   badTriangleEdges.end());
            /** Remove bad triangles*/
            triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [](const Triangle &t)
                                               { return t.isBad(); }),
                                triangulation.end());

            /** Create new triangles from bad triangle edges */
            for (const Edge &eachEdge : badTriangleEdges)
            {
                if (!arePointsColinear(eachEdge, eachPoint))
                {
                    triangulation.push_back(Triangle(eachEdge, eachPoint));
                }
            }
        }
        /** Remove all triangles containing vertices of the super triangle */
        std::vector<glmvec2> vertices = superTriangle.getPoints();
        triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [vertices](const Triangle &t)
                                           { return t.containsPoint(vertices[0]) ||
                                                    t.containsPoint(vertices[1]) ||
                                                    t.containsPoint(vertices[2]); }),
                            triangulation.end());

        return triangulation;
    }

    /**
     * Construct delaunay triangles from a given set of points
     */
    inline std::pair<std::vector<Triangle>, std::set<glmvec2>> fracture_polytope(const PolytopeInfo &boundary_values, const size_t &amount)
    {
        std::set<glmvec2> pointList = getInitRandomPoints(amount, boundary_values);
        std::vector<Triangle> triangles = BowyerWatson(pointList);
        return {triangles, pointList};
    }
}