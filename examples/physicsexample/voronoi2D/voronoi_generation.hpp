#pragma once

#include <vector>
#include <iostream>
#include <unordered_set>
#include <functional>
#include <cmath>

#include "Voronoi.hpp"
#include "Edge.hpp"
#include "Triangle.hpp"
#include "VPE.hpp"
#include "misc.hpp"

namespace VD
{
    /**
     * Calculate the intersection of a perpendicular line from point c on the line segment of AB
     */
    inline glmvec2 intersectionOnLineFromPoint(const glmvec2 &A, const glmvec2 &B, const glmvec2 &C)
    {
        glmvec2 NAB = glm::normalize(B - A);
        glmvec2 AC = C - A;
        real cos_theta = glm::dot(NAB, AC);
        glmvec2 AD_length = NAB * cos_theta;
        glmvec2 D = A + AD_length;
        return D;
    }
    /**
     * Check if points a,b and c are in clockwise order
     */
    inline bool counterclockwise_order(const glmvec2 &A, const glmvec2 &B, const glmvec2 &C)
    {
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
    }

    /**
     * Check if line segements intersect
     */
    inline bool checkIntersection(const Edge &e1, const Edge &e2)
    {
        glmvec2 A = e1.getA();
        glmvec2 B = e1.getB();

        glmvec2 C = e2.getA();
        glmvec2 D = e2.getB();

        return counterclockwise_order(A, C, D) != counterclockwise_order(B, C, D) && counterclockwise_order(A, B, C) != counterclockwise_order(A, B, D);
    }
    /**
     * get intersection point of Two line segments AB and CD
     */
    inline glmvec2 getIntersection(const glmvec2 &A, const glmvec2 &B, const glmvec2 &C, const glmvec2 &D)
    {
        glmvec2 AC = C - A;
        glmvec2 CD = D - C;
        glmvec2 AB = B - A;
        real t = (AC.x * CD.y - CD.x * AC.y) / (AB.x * CD.y - CD.x * AB.y);
        glmvec2 I = A + AB * t;
        return I;
    }

    /**
     * Transform the delaunay triangles to voronoi diagram
     */
    inline std::vector<Voronoi> transformToVoronoi(const std::vector<Triangle> &delaunayTriangles, const std::set<glmvec2> &starting_points)
    {
        std::vector<Voronoi> voronois = {};
        /* Iterate through every starting points/voronoi center */
        for (const auto &eachPoints : starting_points)
        {
            /**Save triangles that contain current point */
            std::vector<Triangle> relevantTriangles = {};
            /** keep Track on which edge belongs to which triangle*/
            std::unordered_map<Edge, std::vector<Triangle>> edgeTriangleMap;
            for (const Triangle &eachTriangle : delaunayTriangles)
            {
                if (eachTriangle.containsPoint(eachPoints))
                {
                    relevantTriangles.push_back(eachTriangle);
                    /** Only leave the two other points to make edges to them*/
                    std::vector<glmvec2> trianglePoints = eachTriangle.getPoints();
                    trianglePoints.erase(std::remove_if(trianglePoints.begin(), trianglePoints.end(), [&eachPoints](const glmvec2 &trianglePoint)
                                                        { return isVectorEqualTo(eachPoints, trianglePoint); }),
                                         trianglePoints.end());
                    /**
                     * Construct Edges from current Point to the other points and save corresponding triangle
                     */
                    for (const auto &eachPoint : trianglePoints)
                    {
                        edgeTriangleMap[Edge(eachPoints, eachPoint)].push_back(eachTriangle);
                    }
                }
            }

            /**
             * Keep track of vertices and unbounded edges
             */
            std::vector<glmvec2> voronoi_vertices = {};
            std::vector<Edge> unbounded_edges = {};

            for (const auto &[edge, triangle] : edgeTriangleMap)
            {

                /** If only 1 triangle contains this edge, it means that the edge is unbounded. Create artificial boundary for voronoi diagram for clipping later*/
                if (triangle.size() == 1)
                {
                    if (unbounded_edges.size() == 0)
                    {
                        /**Construct the voronoi edge from the perpendicular line segment from the delaunay edge to the circumcircle center */
                        glmvec2 A = edge.getA();
                        glmvec2 B = edge.getB();
                        glmvec2 C = triangle[0].getCircumcircleCenter();
                        glmvec2 D = intersectionOnLineFromPoint(A, B, C);

                        glmvec2 DC = C - D;

                        glmvec2 P1 = C + glm::normalize(DC) * 10.0_real;
                        glmvec2 P2 = C - glm::normalize(DC) * 10.0_real;

                        /**Check orientation of edge, should go away from the voronoi */
                        glm::length(P1) < glm::length(P2) ? unbounded_edges.push_back(Edge(C, P2)) : unbounded_edges.push_back(Edge(C, P1));
                    }
                    else
                    {
                        /**Construct the voronoi edge from the perpendicular line segment from the delaunay edge to the circumcircle center */
                        glmvec2 A = edge.getA();
                        glmvec2 B = edge.getB();
                        glmvec2 C = triangle[0].getCircumcircleCenter();
                        glmvec2 D = intersectionOnLineFromPoint(A, B, C);

                        glmvec2 DC = C - D;
                        glmvec2 P1 = C + glm::normalize(DC) * 10.0_real;
                        glmvec2 P2 = C - glm::normalize(DC) * 10.0_real;

                        /**Check orientation of edge, should go away from the voronoi */
                        glm::length(P1) < glm::length(P2) ? unbounded_edges.push_back(Edge(C, P2)) : unbounded_edges.push_back(Edge(C, P1));

                        Edge e1 = unbounded_edges[0];
                        Edge e2 = unbounded_edges[1];
                        /**Check if the two outside voronoi edges intersect  */
                        if (checkIntersection(e1, e2))
                        {
                            /**If intersection ocurrs save the intersection point a voronoi vertice */
                            glmvec2 I = getIntersection(e1.getA(), e1.getB(), e2.getA(), e2.getB());
                            voronoi_vertices.push_back(I);
                        }
                        else
                        {
                            /**If no intersection, stop after some distance and then connect the two edges with another edge */
                            voronoi_vertices.push_back(e1.getB());
                            voronoi_vertices.push_back(e2.getB());
                        }
                    }
                }
                else
                {
                    /**If an edge shares two triangle, connect the circumcircle of each delaunay triangle to form a voronoi edge */
                    voronoi_vertices.push_back(triangle[0].getCircumcircleCenter());
                    voronoi_vertices.push_back(triangle[1].getCircumcircleCenter());
                }
                if (edgeTriangleMap.size() == 2)
                {
                    /**Edge case if only one triangle exist, connect the center of circumcircle with the constructed edges */
                    voronoi_vertices.push_back(triangle[0].getCircumcircleCenter());
                }
            }
            /**Sort the voronoi vertices in counter clockwise order for later visualizing*/
            sort_vertices_ccw(voronoi_vertices);
            Voronoi voronoi = Voronoi(voronoi_vertices);
            voronois.push_back(voronoi);
        }
        return voronois;
    }
    /**
     * Clip Voronoi vertices to a bounding box
     */
    inline void clipVoronoiToBoundingBox(const std::vector<glmvec2> &boundary_values, std::vector<Voronoi> &voronois)
    {
        std::vector<glmvec2> boundingBox = boundary_values;
        sort_vertices_ccw(boundingBox);
        std::reverse(boundingBox.begin(), boundingBox.end());

        for (Voronoi &each_voronoi : voronois)
        {
            std::vector<glmvec2> newPolygon = {};
            std::vector<glmvec2> vertices = each_voronoi.getReversedVertices();

            geometry::SutherlandHodgman(vertices, boundingBox, newPolygon);
            std::reverse(newPolygon.begin(), newPolygon.end());
            each_voronoi.setVertices(newPolygon);
        }

        voronois.erase(std::remove_if(voronois.begin(), voronois.end(), [](const Voronoi &v)
                                      { return v.getVertices().size() == 0; }),
                       voronois.end());
    }
}