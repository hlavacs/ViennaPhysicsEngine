#pragma once

#include "VPE.hpp"
#include "Point.hpp"
#include "VoronoiFace.hpp"
#include "VoronoiEdge.hpp"

struct Voronoi
{
private:
    std::vector<Point> vertices;
    std::vector<VoronoiFace> faces;
    std::vector<VoronoiEdge> edges;
    glmvec3 center;

public:
    Voronoi(glmvec3 center, std::vector<VoronoiEdge> edges) : center{center}, edges{edges} {}

    friend std::ostream &operator<<(std::ostream &o, const Voronoi &v)
    {
        o << "Voronoi " << v.center << "\n";
        for (VoronoiEdge edge : v.edges)
        {
            o << edge;
        }
        o << "\n";
        return o;
    }
};
