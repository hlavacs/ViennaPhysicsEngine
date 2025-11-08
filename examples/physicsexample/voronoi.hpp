#pragma once

#include "VPE.hpp"
#include "tetrahedra.hpp"

struct Voronoi
{
    std::vector<glmvec3> vertices;
    std::vector<Face> faces;
    glmvec3 center;
};
