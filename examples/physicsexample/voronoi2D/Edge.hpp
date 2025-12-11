#pragma once

#include "Point.hpp"
#include "misc.hpp"

namespace VD
{
    /**
     * Edge class for Voronoi/Delaunay Triangle
     */
    struct Edge
    {
    private:
        Point a, b;           /* start and end points */
        bool badEdge = false; /* boolean for bad edges in delaunay algorithm */

    public:
        Edge(Point a, Point b) : a{a}, b{b} {}

        Point getA() const
        {
            return a;
        }

        Point getB() const
        {
            return b;
        }

        bool isBad()
        {
            return badEdge;
        }

        void setBad(bool value)
        {
            badEdge = value;
        }

        bool operator==(const Edge &other) const
        {
            return isVectorEqualTo(a, other.a) && isVectorEqualTo(b, other.b) ||
                   isVectorEqualTo(a, other.b) && isVectorEqualTo(b, other.a);
        }

        friend std::ostream &operator<<(std::ostream &o, const Edge &e)
        {
            o << "Edge\n"
              << e.a << "\n"
              << e.b << "\n";
            return o;
        }
    };
}
/**
 *  Hash function for map,sets
 */
namespace std
{
    template <>
    struct std::hash<VD::Edge>
    {
        std::size_t operator()(const VD::Edge &e) const
        {
            std::hash<VD::Point> hasher;
            size_t h0 = std::hash<size_t>{}(hasher(e.getA()));
            size_t h1 = std::hash<size_t>()(hasher(e.getB())) + 0x9e3779b9 + (h0 << 6) + (h0 >> 2);
            return h1;
        }
    };
}
