
#pragma once

#include <iostream>

#include "VPE.hpp"
#include "Edge.hpp"
#include "misc.hpp"

namespace VD
{
    /**
     * Delaunay Triangle structure
     */
    struct Triangle
    {
    private:
        Point a, b, c;            /* Points that define Triangle*/
        std::vector<Edge> edges;  /* Edges connecting the points*/
        bool badTriangle = false; /* boolean value to determine bad triangle*/

    public:
        Triangle(Point a, Point b, Point c) : a{a}, b{b}, c{c}
        {
            edges.push_back(Edge(a, b));
            edges.push_back(Edge(b, c));
            edges.push_back(Edge(c, a));
        }

        Triangle(Edge e, Point c)
        {
            this->a = e.getA();
            this->b = e.getB();
            this->c = c;

            edges.push_back(Edge(a, b));
            edges.push_back(Edge(b, c));
            edges.push_back(Edge(c, a));
        }

        Triangle() {}

        std::vector<Point> getPoints() const
        {
            return std::vector<Point>{a, b, c};
        }

        std::vector<Edge> getEdges()
        {
            return edges;
        }

        bool isBad()
        {
            return badTriangle;
        }
        
        void setBad(bool value)
        {
            badTriangle = value;
        }

        /**
         * Check if Triangle contains Point p
         */
        bool containsPoint(Point p)
        {
            return isVectorEqualTo(a, p) || isVectorEqualTo(b, p) || isVectorEqualTo(c, p);
        }

        /**
         * Checks if Point d is in the circle defined by the points of the triangle
         */
        bool isInCircumcircle(Point d)
        {
            real m11 = a.getX() - d.getX();
            real m12 = a.getY() - d.getY();
            real m13 = m11 * m11 + m12 * m12;

            real m21 = b.getX() - d.getX();
            real m22 = b.getY() - d.getY();
            real m23 = m21 * m21 + m22 * m22;

            real m31 = c.getX() - d.getX();
            real m32 = c.getY() - d.getY();
            real m33 = m31 * m31 + m32 * m32;

            return (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32) - (m31 * m22 * m13 + m32 * m23 * m11 + m33 * m21 * m12) >= 0;
        }
        /*
         * Calculates the center point of the circumcircle of the triangle using determinant method
         */
        Point getCircumcircleCenter()
        {
            real m11 = a.getX() * a.getX() + a.getY() * a.getY();
            real m21 = b.getX() * b.getX() + b.getY() * b.getY();
            real m31 = c.getX() * c.getX() + c.getY() * c.getY();

            real M11 = (a.getX() * b.getY() + a.getY() * c.getX() + b.getX() * c.getY()) - (c.getX() * b.getY() + c.getY() * a.getX() + b.getX() * a.getY());
            real M12 = (m11 * b.getY() + a.getY() * m31 + m21 * c.getY()) - (m31 * b.getY() + c.getY() * m11 + m21 * a.getY());
            real M13 = (m11 * b.getX() + a.getX() * m31 + m21 * c.getX()) - (m31 * b.getX() + c.getX() * m11 + m21 * a.getX());

            real x_0 = 0.5_real * M12 / M11;
            real y_0 = -0.5_real * M13 / M11;
            return Point(x_0, y_0);
        }

        /**
         * Compares two triangles, equal if all 3 points are the same
         */
        bool isTriangleEqualTo(const Triangle &other) const
        {
            return isVectorEqualTo(a, other.a) && isVectorEqualTo(b, other.b) && isVectorEqualTo(c, other.c) ||
                   isVectorEqualTo(a, other.b) && isVectorEqualTo(b, other.c) && isVectorEqualTo(c, other.a) ||
                   isVectorEqualTo(a, other.c) && isVectorEqualTo(b, other.a) && isVectorEqualTo(c, other.b);
        }

        friend bool operator<(const Triangle &t1, const Triangle &t2)
        {
            return std::tie(t1.a, t1.b, t1.c) < std::tie(t2.a, t2.b, t2.c);
        }

        friend std::ostream &operator<<(std::ostream &o, const Triangle &t)
        {
            o << "Triangle\n";
            for (auto &eachPoint : t.getPoints())
            {
                o << eachPoint << "\n";
            }
            return o;
        }
    };
}
