#pragma once

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace glm;

//Pluecker coordinates for point, line and plane (Eric Lengyel, Foundations of Game Engine Development, Vol 1: Mathematics, 2016)
using pluecker_point = vec4;    // homogeneous 4D coordinates (p | w) where the original point q is 1/w p

using pluecker_plane = vec4;    // 4D plane coordinates [normal | d] where d = -dot(normal,q) for q point on the plane
 
struct pluecker_line {          // 6D line coordinates { direction vector | p0 x p1 }, where p0 and p1 are points on the line
    vec3 v;
    vec3 m;
};

pluecker_plane make_pluecker_plane( pluecker_line line, pluecker_point point);

pluecker_point make_pluecker_point( vec3 p) {
    return {p, 1.0f};
}

pluecker_line make_pluecker_line( vec3 p0, vec3 p1 ) {
    return { {p1 - p0}, { cross(p0, p1) } };
}

pluecker_plane make_pluecker_plane( vec3 p0, vec3 p1, vec3 normal) {
    return make_pluecker_plane( make_pluecker_line(p0, p1), make_pluecker_point(p0 + normal) );
}

pluecker_plane make_pluecker_plane( pluecker_line line, pluecker_point point) {
    vec3 v = line.v;
    vec3 m = line.m;
    vec3 p = point;
    float w = point.w;
    return { vec3{cross(v,p) + w*m}, -1.0f*dot(p,m) };
}

float distance_point_plane( vec3 &p, pluecker_plane &plane ) {
    vec3 n = plane;
    float d = plane.w;
    return std::abs(dot(n,p) + d) / length(n);
}

pluecker_point intersect_line_plane( pluecker_line &line, pluecker_plane & plane ) {
    vec3 v = line.v;
    vec3 m = line.m;
    vec3 n = plane;
    float d = plane.w;
    return { vec3{cross( m, n ) + d*v}, -1.0f*dot(n,v) };
}


