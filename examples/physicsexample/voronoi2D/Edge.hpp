#pragma once

#include "misc.hpp"

namespace VD
{
    /**
     * Edge class for Voronoi/Delaunay Triangle
     */
    struct Edge
    {
    private:
        glmvec2 a, b;
        bool badEdge = false; /* boolean for bad edges in delaunay algorithm */

    public:
        Edge(glmvec2 a, glmvec2 b) : a{a}, b{b} {}

        glmvec2 getA() const
        {
            return a;
        }

        glmvec2 getB() const
        {
            return b;
        }

        bool isBad() const
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
            size_t seed = std::hash<glmvec2>{}(e.getA());
            hash_combine(seed, e.getB());
            return seed;
        }
    };
}
