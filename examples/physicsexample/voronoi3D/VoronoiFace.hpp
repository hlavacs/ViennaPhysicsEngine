#pragma once

#include "VPE.hpp"
#include "Point.hpp"
#include "VoronoiEdge.hpp"

struct VoronoiFace
{
private:
    std::vector<Point> vertices;
    std::vector<VoronoiEdge> edges;

public:
    VoronoiFace(std::vector<Point> vertices, std::vector<VoronoiEdge> edges) : vertices{vertices}, edges{edges} {}
    VoronoiFace() {}

    void addEdge(VoronoiEdge edge)
    {
        edges.push_back(edge);
    }

    void addVertices(glmvec3 vertice)
    {
        vertices.push_back(vertice);
    }
};
