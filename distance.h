#ifndef DISTANCE_H
#define DISTANCE_H

#include "pluecker.h"
#include "collider.h"

namespace vpe {

    //--------------------------------------------------------------------------

    inline float distance_point_line( vec3 p, pluecker_line line ) {
        vec3 v = line.v();
        vec3 m = line.m();
        return length(cross(v,p) + m) / length(v);
    }

    inline float distance_point_line( pluecker_point point, pluecker_line line ) {
        return distance_point_line( point.p3D(), line );
    }

    inline float distance_line_line(pluecker_line line1, pluecker_line line2) {
        vec3 v1 = line1.v();
        vec3 m1 = line1.m();
        vec3 v2 = line2.v();
        vec3 m2 = line2.m();
        return std::abs(dot(v1,m2) + dot(v2,m1)) / length(cross(v1,v2));
    }

    inline float distance_point_plane( vec3 p, pluecker_plane plane ) {
        vec3  n = plane.n();
        float d = plane.d();
        return std::abs(dot(n,p) + d) / length(n);
    }

    //--------------------------------------------------------------------------

    //use http://mathforum.org/library/drmath/view/62814.html to find the intersection of two line segments
    inline pluecker_point intersect_segment_segment( Line l1, Line l2 ) {
        if( distance_line_line(l1.plueckerW(), l2.plueckerW()) > EPS ) return { {0,0,0}, 0};

        vec3 cp = cross(l1.m_dir, l2.m_dir);
        if( cp == vec3{0,0,0}) return { {0,0,0}, 0}; //if edges are parallel there is no intersection
        vec3 diff = l2.pos() - l1.pos();
        vec3 rs = cross( diff, l2.m_dir );

        float t = length(rs) / length(cp);      //get line parameter t
        t = dot(cp,rs)>=0 ? t : -t;             //might have to invert sign
        if( t<0 || t>1) return { {0,0,0}, 0};   //test if in segment

        vec3 ls = t*l1.m_dir - diff;
        float u = length( ls ) / length( l2.m_dir); //get line parameter u
        u = dot(ls, l2.m_dir)>=0 ? u : -u;          //might have to invert sign
        if( u<0 || u>1) return { {0,0,0}, 0};       //test if in segment
        return l1.pos() + t * l1.m_dir;
    }

    inline pluecker_point intersect_line_plane( pluecker_line line, pluecker_plane plane ) {
        vec3  v = line.v();
        vec3  m = line.m();
        vec3  n = plane.n();
        float d = plane.d();
        return { {cross(m,n) + d*v}, -1.0f*dot(n,v) };
    }

}


#endif

