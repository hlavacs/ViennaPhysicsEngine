#pragma once

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#include "Collider.h"
#include <random>


constexpr int NUM_RANDOM_DIR = 64;


//test whether a direction is a separating axis
bool sat_axis_test( ICollider *obj1, ICollider *obj2, vec3 *dir, vec3 *r, float *d) {
    vec3 p = obj1->support( *dir );
    vec3 q = obj2->support( -1.0f* (*dir) );
    *r = normalize(q - p); 
    *d = dot( *r, *dir );
    return  *d > 1.0e-6f;
}

//choose N random directions to find SA
bool sat_random_test( ICollider *obj1, ICollider *obj2, vec3 *dir) {
    static std::vector<vec3> random_axes;   //Fibonacci sphere
    if( random_axes.empty()) {
        float phi = (float)(M_PI * (3. - std::sqrt(5.)));  // golden angle in radians

        for(int i=0; i<NUM_RANDOM_DIR; ++i ) {
            float y = 1 - (i / float(NUM_RANDOM_DIR - 1)) * 2;  // y goes from 1 to -1
            float radius = std::sqrt(1 - y * y);         // radius at y

            float theta = phi * i;                       // golden angle increment

            float x = std::cos(theta) * radius;
            float z = std::sin(theta) * radius;
            random_axes.emplace_back( x, y, z );
        }
    }

    vec3 r;
    float d;
    float max = -1.0e6;
    bool found = false;
    for( int i=0; i<random_axes.size() && !found; ++i ) {        
        found = sat_axis_test(obj1, obj2, &random_axes[i], &r, &d);
        if( d>max ) { 
            max = d;
            *dir = random_axes[i];
        }
    }
    return found;
}

//Chung Wang Test with damping
bool sat_chung_wang_test( ICollider *obj1, ICollider *obj2, vec3 *dir, int max_loops = 50 ) {
    vec3 r;
    float d;
    int loop = 0;
    float f = 1.0f;
    while( loop < max_loops && !sat_axis_test(obj1, obj2, dir, &r, &d) ) {
        *dir = *dir - (1.0f + f)*dot(r, *dir)*r;
        ++loop;
        f = std::max( f * 0.97f, 0.5f );
    }
    std::cout << "Loops: " << loop << std::endl;
    return sat_axis_test(obj1, obj2, dir, &r, &d);
}

//SAT using the normals of polytope faces
bool sat_faces_test( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    auto &faces = obj1->m_faces;
    vec3 r;
    float d;
    bool found = false;
    for( int i=0; !found && i<faces.size(); ++i) {
        *dir = obj1->get_face_normal(i);
        found = sat_axis_test(obj1, obj2, dir, &r, &d);
    }

    faces = obj2->m_faces;
    for( int i=0; !found && i<faces.size(); ++i) {
        *dir = obj1->get_face_normal(i);
        found = sat_axis_test(obj1, obj2, dir, &r, &d);
    }

    return found;
}

//SAT using the cross products of edge pairs from two polytopes
bool sat_edges_test( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    std::vector<vec3> edges1;
    std::vector<vec3> edges2;

    obj1->get_edge_vectors( edges1 );
    obj2->get_edge_vectors( edges2 );

    vec3 r;
    float d;
    bool found = false;
    for( int i=0; !found && i<edges1.size(); ++i) {
        for( int j=0; !found && i<edges2.size(); ++j) {
            *dir = cross( edges1[i], edges2[j] );
            found = sat_axis_test(obj1, obj2, dir, &r, &d);
        }
    }
    return found; 
}


//entry points for SAT
//returns true of the objects are in contact
//else false
bool sat( Polytope *obj1, Polytope *obj2, vec3 *dir ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);
    if( sat_faces_test( obj1, obj2, dir ) ) return false;

    if( obj1->m_edges.size() * obj2->m_edges.size() > NUM_RANDOM_DIR ) {
        if( sat_random_test( obj1, obj2, dir ) ) return false;
        if( sat_edges_test( obj1, obj2, dir ) ) return false;
    } else {
        if( sat_edges_test( obj1, obj2, dir ) ) return false;
    }
    return true;
}

//entry points for SAT
//returns true of the objects are in contact
//else false
bool sat( ICollider *obj1, ICollider *obj2, vec3 *dir ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);
    if( sat_random_test( obj1, obj2, dir ) ) return false;
    if( sat_chung_wang_test( obj1, obj2, dir ) ) return false;
    return true;
}





