#pragma once

#include <iostream>
#include <vector>

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

    /**
     * Get average point of a list of points
     */
    inline glmvec2 getAveragePoint(const std::vector<glmvec2> &points)
    {
        real x = 0.0_real;
        real y = 0.0_real;

        real size = real(points.size());
        for (const auto &each_point : points)
        {
            x += each_point.x;
            y += each_point.y;
        }

        return {x / size, y / size};
    }

    /**
     * translate point by the average of the points
     */
    inline void translateVertices(std::vector<glmvec2> &points, const glmvec2 &translation)
    {
        for (auto &each_point : points)
        {
            each_point = each_point - translation;
        }
    }

    /**
     * Translate voronoi by the average of their vertices and return this vector of translations
     */
    inline std::vector<glmvec3> center_voronois(std::vector<Voronoi> &voronois)
    {
        std::vector<glmvec3> translationVector = {};
        for (auto &each_voronoi : voronois)
        {
            std::vector<glmvec2> voronoi_vertice = each_voronoi.getVertices();
            glmvec2 average = getAveragePoint(voronoi_vertice);
            translateVertices(voronoi_vertice, average);
            translationVector.push_back({average.x, average.y, 0});
            each_voronoi.setVertices(voronoi_vertice);
        }

        return translationVector;
    }

    /**
     * Compare angles between two point around the center point
     */
    inline static bool angle_comparison(const glmvec2 &p1, const glmvec2 &p2, const glmvec2 &center)
    {
        double angle1 = std::atan2(p1.y - center.y, p1.x - center.x);
        double angle2 = std::atan2(p2.y - center.y, p2.x - center.x);
        return angle1 < angle2;
    }
    /**
     * Sort vertices in counter clockwise order by angle
     */
    inline void sort_vertices_ccw(std::vector<glmvec2> &unordered_vertices)
    {
        glmvec2 center = getAveragePoint(unordered_vertices);
        std::sort(unordered_vertices.begin(), unordered_vertices.end(), [&center](const glmvec2 &p1, const glmvec2 &p2)
                  { return angle_comparison(p1, p2, center); });

        // Remove duplicate values in vector
        auto duplicates = std::unique(unordered_vertices.begin(), unordered_vertices.end());
        unordered_vertices.erase(duplicates, unordered_vertices.end());
    }

}