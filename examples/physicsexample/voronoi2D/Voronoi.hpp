#pragma once

#include <iostream>
#include <vector>

#include "Edge.hpp"
#include "misc.hpp"

#include "VPE.hpp"

namespace VD
{
    /**
     * Voronoi class
     */
    struct Voronoi
    {
    private:
        std::vector<glmvec2> vertices; /** voronoi defined by vertices in counter clockwise order */

    public:
        Voronoi(std::vector<glmvec2> vertices) : vertices{vertices} {}
        Voronoi() {}

        std::vector<glmvec2> getVertices() const
        {
            return vertices;
        }

        std::vector<glmvec2> getReversedVertices() const
        {
            std::vector<glmvec2> reversed_vertices = vertices;
            std::reverse(reversed_vertices.begin(), reversed_vertices.end());
            return reversed_vertices;
        }

        void setVertices(std::vector<glmvec2> vertices)
        {
            this->vertices = vertices;
        }

        friend std::ostream &operator<<(std::ostream &o, const Voronoi &v)
        {
            o << "Voronoi Diagram \n";
            for (auto vertice : v.vertices)
            {
                o << vertice << "\n";
            }
            o << "\n";
            return o;
        }
    };
}