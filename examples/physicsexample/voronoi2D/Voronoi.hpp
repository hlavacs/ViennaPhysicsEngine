#pragma once

#include <iostream>
#include <vector>

#include "Point.hpp"
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
        /**TODO: Replace with glmvec2 */
        std::vector<Point> vertices; /** voronoi defined by vertices in counter clockwise order */

    public:
        Voronoi(std::vector<Point> vertices) : vertices{vertices} {}
        Voronoi() {}

        std::vector<Point> getVertices()
        {
            return vertices;
        }

        void setVertices(std::vector<Point> vertices)
        {
            this->vertices = vertices;
        }

        /**
         * Convert into reversed vector of glm::vec2 values and returns it
         */
        std::vector<glmvec2> getVec2Vertices()
        {
            std::vector<glmvec2> result = {};
            for (auto each_vertice : vertices)
            {
                result.push_back(glmvec2(each_vertice.getX(), each_vertice.getY()));
            }
            std::reverse(result.begin(), result.end());
            return result;
        }

        /**
         * Convert reversed glmvec2 vector and sets it as vertices 
         */
        void setVec2Vertices(std::vector<glmvec2> vec2_vertices)
        {
            vertices.clear();
            for (auto each_vertice : vec2_vertices)
            {
                Point p = Point(each_vertice.x, each_vertice.y);
                vertices.push_back(p);
            }
            std::reverse(vertices.begin(), vertices.end());
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