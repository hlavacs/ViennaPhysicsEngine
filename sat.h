#pragma once

#include <iostream>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#include "Collider.h"
#include "hash.h"


using v3pair = std::pair<vec3,vec3>;


bool sat_axis_test( Collider *obj1, Collider *obj2, vec3 *dir, vec3 *r) {
    vec3 p = obj1->support( *dir );
    vec3 q = obj2->support( -1.0f* (*dir) );
    *r = normalize(q - p); 
    return dot( *r, *dir ) > 0.0f;
}

bool sat_chung_wang_test( Collider *obj1, Collider *obj2, vec3 *dir, int max_loops = 100 ) {
    vec3 r;
    int loop = 0;
    while( loop < max_loops && sat_axis_test(obj1, obj2, dir, &r) ) {
        *dir = reflect( *dir, r); //dir = dir - 2*dot(r, dir)*r
        ++loop;
    }
    return sat_axis_test(obj1, obj2, dir, &r);
}

bool sat_faces_test( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    auto &faces = obj1->faces2;
    vec3 r;
    bool found = false;
    for( int i=0; !found && i<faces.size(); ++i) {
        *dir = obj1->get_normal_of_face(i);
        found = sat_axis_test(obj1, obj2, dir, &r);
    }
    return found;
}

bool sat_edges_test( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    std::vector<vec3> edges1;
    std::vector<vec3> edges2;

    obj1->get_edge_vectors( edges1 );
    obj2->get_edge_vectors( edges2 );

    vec3 r;
    bool found = false;
    for( int i=0; !found && i<edges1.size(); ++i) {
        for( int j=0; !found && i<edges2.size(); ++j) {
            *dir = cross( edges1[i], edges2[j] );
            found = sat_axis_test(obj1, obj2, dir, &r);
        }
    }
    return found;
}

bool sat( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);
    bool found = sat_faces_test( obj1, obj2, dir );
    if( !found ) found = sat_edges_test( obj1, obj2, dir );
    return !found;
}

bool sat( Collider *obj1, Collider *obj2, vec3 *dir ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);
    return !sat_chung_wang_test( obj1, obj2, dir );
}


