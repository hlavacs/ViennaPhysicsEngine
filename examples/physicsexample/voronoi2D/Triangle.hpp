
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
        glmvec2 a, b, c;          /* Points that define Triangle*/
        std::vector<Edge> edges;  /* Edges connecting the points*/
        bool badTriangle = false; /* boolean value to determine bad triangle*/

    public:
        Triangle(glmvec2 a, glmvec2 b, glmvec2 c) : a{a}, b{b}, c{c}
        {
            edges.push_back(Edge(a, b));
            edges.push_back(Edge(b, c));
            edges.push_back(Edge(c, a));
        }

        Triangle(Edge e, glmvec2 c)
        {
            this->a = e.getA();
            this->b = e.getB();
            this->c = c;

            edges.push_back(Edge(a, b));
            edges.push_back(Edge(b, c));
            edges.push_back(Edge(c, a));
        }

        Triangle() {}

        std::vector<glmvec2> getPoints() const
        {
            return {a, b, c};
        }

        std::vector<Edge> getEdges() const
        {
            return edges;
        }

        bool isBad() const
        {
            return badTriangle;
        }

        void setBad(bool value)
        {
            badTriangle = value;
        }

        /**
         * Check if Triangle contains glmvec2 p
         */
        bool containsPoint(glmvec2 p) const
        {
            return isVectorEqualTo(a, p) || isVectorEqualTo(b, p) || isVectorEqualTo(c, p);
        }

        /**
         * Checks if glmvec2 d is in the circle defined by the points of the triangle
         */
        bool isInCircumcircle(glmvec2 d) const
        {
            real m11 = a.x - d.x;
            real m12 = a.y - d.y;
            real m13 = m11 * m11 + m12 * m12;

            real m21 = b.x - d.x;
            real m22 = b.y - d.y;
            real m23 = m21 * m21 + m22 * m22;

            real m31 = c.x - d.x;
            real m32 = c.y - d.y;
            real m33 = m31 * m31 + m32 * m32;

            return (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32) - (m31 * m22 * m13 + m32 * m23 * m11 + m33 * m21 * m12) >= 0;
        }
        /*
         * Calculates the center point of the circumcircle of the triangle using determinant method
         */
        glmvec2 getCircumcircleCenter() const
        {
            real m11 = a.x * a.x + a.y * a.y;
            real m21 = b.x * b.x + b.y * b.y;
            real m31 = c.x * c.x + c.y * c.y;

            real M11 = (a.x * b.y + a.y * c.x + b.x * c.y) - (c.x * b.y + c.y * a.x + b.x * a.y);
            real M12 = (m11 * b.y + a.y * m31 + m21 * c.y) - (m31 * b.y + c.y * m11 + m21 * a.y);
            real M13 = (m11 * b.x + a.x * m31 + m21 * c.x) - (m31 * b.x + c.x * m11 + m21 * a.x);

            real x_0 = 0.5_real * M12 / M11;
            real y_0 = -0.5_real * M13 / M11;
            return glmvec2(x_0, y_0);
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
