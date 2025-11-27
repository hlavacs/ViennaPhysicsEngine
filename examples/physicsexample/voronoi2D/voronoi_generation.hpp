#pragma once

#include <vector>
#include <iostream>
#include <unordered_set>
#include <functional>
#include <cmath>

#include "Voronoi.hpp"
#include "Edge.hpp"
#include "Triangle.hpp"
#include "Point.hpp"
#include "VPE.hpp"

namespace VD
{
    using Vector = Point;
    /**
     * Calculate the intersection of a perpendicular line from point c on the line segment of AB
     */
    inline Point intersectionOnLineFromPoint(Point A, Point B, Point C)
    {
        Vector NAB = (B - A).normalized();
        Vector AC = C - A;
        real cos_theta = NAB.dot(AC);
        Vector AD_length = NAB * cos_theta;
        Point D = A + AD_length;

        return D;
    }
    /**
     * Check if points a,b and c are in clockwise order
     */
    inline bool counterclockwise_order(Point A, Point B, Point C)
    {
        return (C.getY() - A.getY()) * (B.getX() - A.getX()) > (B.getY() - A.getY()) * (C.getX() - A.getX());
    }

    /**
     * Check if line segements intersect
     */
    inline bool checkIntersection(Edge e1, Edge e2)
    {
        Point A = e1.getA();
        Point B = e1.getB();

        Point C = e2.getA();
        Point D = e2.getB();

        return counterclockwise_order(A, C, D) != counterclockwise_order(B, C, D) && counterclockwise_order(A, B, C) != counterclockwise_order(A, B, D);
    }
    /**
     * get intersection point of Two line segments AB and CD
     */
    inline Point getIntersection(Point A, Point B, Point C, Point D)
    {
        Vector AC = C - A;
        Vector CD = D - C;
        Vector AB = B - A;

        real t = AC.cross(CD) / AB.cross(CD);
        Point I = A + AB * t;
        return I;
    }
    /**
     * Get center point from list of points
     */
    inline Point getCenterPoint(std::vector<Point> &vertices)
    {
        real mean_x = 0;
        real mean_y = 0;
        for (Point each_vertice : vertices)
        {
            mean_x += each_vertice.getX();
            mean_y += each_vertice.getY();
        }

        mean_x /= vertices.size();
        mean_y /= vertices.size();

        return Point(mean_x, mean_y);
    }
    /**
     * Compare angles between two point around the center point
     */
    inline static bool angle_comparison(Point p1, Point p2, Point center)
    {
        double angle1 = std::atan2(p1.getY() - center.getY(), p1.getX() - center.getX());
        double angle2 = std::atan2(p2.getY() - center.getY(), p2.getX() - center.getX());
        return angle1 < angle2;
    }
    /**
     * Sort vertices in counter clockwise order
     */
    inline void sort_vertices_ccw(std::vector<Point> &unordered_vertices)
    {
        Point center = getCenterPoint(unordered_vertices);
        std::sort(unordered_vertices.begin(), unordered_vertices.end(), [&center](Point &p1, Point &p2)
                  { return angle_comparison(p1, p2, center); });
        // Remove duplicate values in vector
        auto it = std::unique(unordered_vertices.begin(), unordered_vertices.end());
        unordered_vertices.erase(it, unordered_vertices.end());
    }
    /**
     * Transform the delaunay triangles to voronoi diagram
     */
    inline std::vector<Voronoi> transformToVoronoi(std::vector<Triangle> delaunayTriangles, std::set<Point> starting_points)
    {
        std::vector<Voronoi> voronois = {};
        /* Iterate through every starting points/voronoi center */
        for (Point eachPoints : starting_points)
        {
            /**Save triangles that contain current point */
            std::vector<Triangle> relevantTriangles = {};
            /** keep Track on which edge belongs to which triangle*/
            std::unordered_map<Edge, std::vector<Triangle>> edgeTriangleMap;
            for (Triangle eachTriangle : delaunayTriangles)
            {

                if (eachTriangle.containsPoint(eachPoints))
                {
                    relevantTriangles.push_back(eachTriangle);
                    /** Only leave the two other points to make edges to them*/
                    std::vector<Point> trianglePoints = eachTriangle.getPoints();
                    trianglePoints.erase(std::remove_if(trianglePoints.begin(), trianglePoints.end(), [&eachPoints](Point &trianglePoint)
                                                        { return isVectorEqualTo(eachPoints, trianglePoint); }),
                                         trianglePoints.end());
                    /**
                     * Construct Edges from current Point to the other points and save corresponding triangle
                     */
                    for (Point eachPoint : trianglePoints)
                    {
                        edgeTriangleMap[Edge(eachPoints, eachPoint)].push_back(eachTriangle);
                    }
                }
            }

            /**
             * Keep track of vertices and unbounded edges
             */
            std::vector<Point> voronoi_vertices = {};
            std::vector<Edge> unbounded_edges = {};

            for (auto [edge, triangle] : edgeTriangleMap)
            {
                /** If only 1 triangle contains this edge, it means that the edge is unbounded. Create artificial boundary for voronoi diagram for clipping later*/
                if (triangle.size() == 1)
                {
                    if (unbounded_edges.size() == 0)
                    {
                        /**Construct the voronoi edge from the perpendicular line segment from the delaunay edge to the circumcircle center */
                        Point A = edge.getA();
                        Point B = edge.getB();
                        Point C = triangle[0].getCircumcircleCenter();
                        Point D = intersectionOnLineFromPoint(A, B, C);

                        Vector DC = C - D;
                        Point P1 = C + DC.normalized() * 3;
                        Point P2 = C - DC.normalized() * 3;

                        /**Check orientation of edge, should go away from the voronoi */
                        P1.length() < P2.length() ? unbounded_edges.push_back(Edge(C, P2)) : unbounded_edges.push_back(Edge(C, P1));
                    }
                    else
                    {
                        /**Construct the voronoi edge from the perpendicular line segment from the delaunay edge to the circumcircle center */
                        Point A = edge.getA();
                        Point B = edge.getB();
                        Point C = triangle[0].getCircumcircleCenter();
                        Vector D = intersectionOnLineFromPoint(A, B, C);

                        Vector DC = C - D;
                        Point P1 = C + DC.normalized() * 3;
                        Point P2 = C - DC.normalized() * 3;
                        /**Check orientation of edge, should go away from the voronoi */
                        P1.length() < P2.length() ? unbounded_edges.push_back(Edge(C, P2)) : unbounded_edges.push_back(Edge(C, P1));

                        Edge e1 = unbounded_edges[0];
                        Edge e2 = unbounded_edges[1];
                        /**Check if the two outside voronoi edges intersect  */
                        if (checkIntersection(e1, e2))
                        {
                            /**If intersection ocurrs save the intersection point a voronoi vertice */
                            Point I = getIntersection(e1.getA(), e1.getB(), e2.getA(), e2.getB());
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
            }
            /**Sort the voronoi vertices in counter clockwise order for later visualizing*/
            sort_vertices_ccw(voronoi_vertices);
            Voronoi voronoi = Voronoi(eachPoints, voronoi_vertices);
            voronois.push_back(voronoi);
        }
        return voronois;
    }

    /**
     * Clip Voronoi to bounding box
     * TODO: Add bounding box parameters
     */
    inline void clipVoronoiToBoundingBox(std::vector<Voronoi> &voronois)
    {
        for (Voronoi &each_voronoi : voronois)
        {
            std::vector<glmvec2> newPolygon;
            std::vector<glmvec2> boundingBox = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
            std::vector<glmvec2> vertices = each_voronoi.getVec2Vertices();

            geometry::SutherlandHodgman(vertices, boundingBox, newPolygon);
            each_voronoi.setVec2Vertices(newPolygon);
        }
    }
}