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
#include "Point.hpp"
#include "Triangle.hpp"
#include "misc.hpp"

namespace VD
{
    /**
     * Returns a list of n 2D points
     */
    inline std::set<Point> getInitRandomPoints(size_t n)
    {
        /*
        TODO: Prevent 4 points being on a sphere
        TODO: Calculate points based on dimension of convex polyhedron
        */
        std::default_random_engine rnd_gen{12345};
        std::uniform_real_distribution<> rnd_unif{-1, 1};

        std::set<Point> startings_points;
        for (size_t idx = 0; idx < n; ++idx)
        {
            real x = (real)rnd_unif(rnd_gen);
            real y = (real)rnd_unif(rnd_gen);

            startings_points.insert(Point(x, y));
        }

        return startings_points;
    }
    /**
     * Forms a Triangle that encloses all given points
     */
    inline Triangle getSuperTriangle(const std::set<Point> &points)
    {
        real min_x = std::numeric_limits<real>::max();
        real min_y = std::numeric_limits<real>::max();

        real max_x = std::numeric_limits<real>::min();
        real max_y = std::numeric_limits<real>::min();

        for (const Point &forEachPoint : points)
        {
            min_x = std::min(min_x, forEachPoint.getX());
            min_y = std::min(min_y, forEachPoint.getY());

            max_x = std::max(max_x, forEachPoint.getX());
            max_y = std::max(max_y, forEachPoint.getY());
        }

        real mid_x = (min_x + max_x) / 2.0_real;
        real mid_y = (min_y + max_y) / 2.0_real;

        real deltaMax = std::max({max_x - min_x, max_y - min_y});
        real scale = 10;

        return Triangle(Point(mid_x, mid_y + deltaMax * scale),
                        Point(mid_x - deltaMax * scale, mid_y - deltaMax),
                        Point(mid_x + deltaMax * scale * scale, mid_y - deltaMax));
    }

    /* Checks if points are collinear*/
    inline bool arePointsColinear(Edge e, Point c)
    {
        // TODO
        return false;
    }

    /**
     * Bowyer-Watson Algorithm to construct delaunay triangles from a given set of points
     */
    inline std::vector<Triangle> BowyerWatson(std::set<Point> pointList)
    {
        /**Start with empty triangles */
        std::vector<Triangle> triangulation = {};
        /**Create Super Triangle that encompasses all points */
        Triangle superTriangle = getSuperTriangle(pointList);
        triangulation.push_back(superTriangle);

        /** For each point */
        for (const Point &eachPoint : pointList)
        {
            std::vector<Edge> badTriangleEdges = {};
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
            badTriangleEdges.erase(std::remove_if(badTriangleEdges.begin(), badTriangleEdges.end(), [](Edge &edge)
                                                  { return edge.isBad(); }),
                                   badTriangleEdges.end());
            /** Remove bad triangles*/
            triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [](Triangle &t)
                                               { return t.isBad(); }),
                                triangulation.end());

            for (Edge eachEdge : badTriangleEdges)
            {
                if (!arePointsColinear(eachEdge, eachPoint))
                {
                    triangulation.push_back(Triangle(eachEdge, eachPoint));
                }
            }
        }
        /** Remove all triangles containing vertices of the super triangle */
        std::vector<Point> vertices = superTriangle.getPoints();
        triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(), [vertices](Triangle &t)
                                           { return t.containsPoint(vertices[0]) ||
                                                    t.containsPoint(vertices[1]) ||
                                                    t.containsPoint(vertices[2]); }),
                            triangulation.end());

        return triangulation;
    }

    /**
     * Construct delaunay triangles from a given set of points
     */
    inline std::pair<std::vector<Triangle>, std::set<Point>> fracture_polytope(vpe::VPEWorld::Polytope *polytope, size_t amount)
    {
        std::set<Point> pointList = getInitRandomPoints(amount);
        std::vector<Triangle> triangles = BowyerWatson(pointList);
        return {triangles, pointList};
    }
}