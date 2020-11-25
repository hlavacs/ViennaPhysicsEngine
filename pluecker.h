#ifndef PLUECKER_H
#define PLUECKER_H

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace glm;


namespace vpe {


    //Pluecker coordinates for point, line and plane (Eric Lengyel, Foundations of Game Engine Development, Vol 1: Mathematics, 2016)

    // homogeneous 4D coordinates (p | w) where the original point q is 1/w p
    struct pluecker_point {
        private:
        vec4 m_o;

        public:
        pluecker_point( vec3 p, float w = 1.0f ) : m_o( vec4{p,w} ) {};
        operator vec4 () { return m_o; } 
        vec3  p3D() const { return { m_o*(1.0f / m_o.w) }; }
        vec3  p() const { return { m_o }; }
        float w() const { return m_o.w; }
    };

    // 6D line coordinates { direction vector | p0 x p1 }, where p0 and p1 are points on the line
    struct pluecker_line {
        private:     
        vec3 m_v;
        vec3 m_m;

        public:
        pluecker_line( vec3 p0, vec3 p1) {
            m_v = {p1 - p0};
            m_m = cross(p0, p1);
        };
        vec3 v() const { return m_v;}
        vec3 m() const { return m_m;}
    };

    // 4D plane coordinates [normal | d] where d = -dot(normal,q) for q point on the plane
    struct pluecker_plane {
        private:
        vec4 m_p;

        public:
        pluecker_plane( vec3 n, float d ) : m_p( vec4{n,d} ) {};
        pluecker_plane( vec3 n, vec3 q ) : m_p( vec4{n, -1.0f*dot(n,q)} ) {};
        pluecker_plane( pluecker_point point, pluecker_line line ) {
            vec3 p = point.p();
            float w = point.w();
            vec3 v = line.v();    
            vec3 m = line.m();
            m_p = vec4( vec3(cross(v,p) + w*m), -1.0f*dot(p,m) );
        }
        pluecker_plane( vec3 p0, vec3 p1, vec3 p2) :
            pluecker_plane( pluecker_point(p2), pluecker_line( p0, p1 ) ) {}
        operator vec4 () const { return m_p; } 
        vec3  n() const { return {m_p}; }
        float d() const { return m_p.w; }
    };




}

#endif