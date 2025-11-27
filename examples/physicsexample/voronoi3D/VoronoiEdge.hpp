#pragma once

#include "VPE.hpp"
#include "Point.hpp"
#include "VoronoiFace.hpp"

struct VoronoiEdge
{
private:
    glmvec3 a, b;
    bool bounded = false;

public:
    VoronoiEdge(glmvec3 a, glmvec3 b) : a{a}, b{b}
    {
        bounded = true;
    }

    VoronoiEdge() {}

    friend std::ostream &operator<<(std::ostream &o, const VoronoiEdge &ve)
    {
        return o << "Edge (" << ve.a << ", " << ve.b << ")\n";
    }
};
