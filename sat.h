#pragma once

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <random>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#include "collider.h"


constexpr int NUM_RANDOM_DIR = 32;


//test whether a direction is a separating axis
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
bool sat_axis_test( ICollider &obj1, ICollider &obj2, vec3 &dir, vec3 &r, float &d) {
    vec3 p = obj1.support( dir );
    vec3 q = obj2.support( -1.0f* (dir) );
    r = normalize(q - p); 
    d = dot( r, dir );
    return  d > EPS;       //allow for numerical inaccuracies
}

bool sat_axis_test( ICollider &obj1, ICollider &obj2, vec3 &dir ) {
    vec3 r;
    float d;
    return sat_axis_test( obj1, obj2, dir, r, d);
}



//choose N random directions to find SA
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
bool sat_random_test( ICollider &obj1, ICollider &obj2, vec3 &dir) {
    static vvec3 random_axes;   //Fibonacci sphere
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
        found = sat_axis_test(obj1, obj2, random_axes[i], r, d);
        if( d>max ) { 
            max = d;
            dir = random_axes[i];
        }
    }
    return found;
}

//Chung Wang Test with damping
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
bool sat_chung_wang_test( ICollider &obj1, ICollider &obj2, vec3 &dir, int max_loops = 50 ) {
    vec3 r;
    float d;
    int loop = 0;
    float f = 1.0f;
    while( loop < max_loops && !sat_axis_test(obj1, obj2, dir, r, d) ) {
        dir = dir - (1.0f + f)*dot(r, dir)*r;
        ++loop;
        f = std::max( f * 0.97f, 0.5f );   //apply damping to prevent small gaps from oscillating forever
    }
    std::cout << "Loops: " << loop << std::endl;
    return sat_axis_test(obj1, obj2, dir, r, d);
}


//SAT using the normals of 2 faces
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
bool sat_faces_test( Face &face1, Face &face2, vec3 &dir ) {
    dir = face1.normalW();
    if( sat_axis_test(face1, face2, dir ) ) return true;
    dir = face2.normalW();
    if( sat_axis_test(face1, face2, dir ) ) return true;
    return false;
}


//SAT using the normals of polytope faces
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
bool sat_faces_test( Polytope &obj1, Polytope &obj2, vec3 &dir ) {
    for( auto& face : obj1.m_faces ) {
        dir = face.normalW();
        if( sat_axis_test(obj1, obj2, dir) ) return true;
    }

    for( auto& face : obj2.m_faces ) {
        dir = face.normalW();
        if( sat_axis_test(obj1, obj2, dir) ) return true;
    }

    return false;
}


//SAT using the cross products of edge pairs from two Faces/polytopes
//returns true if a separating axis was found (i.e. objects are NOT in contact), else false
template<typename T>
bool sat_edges_test( T &obj1, T &obj2, vec3 &dir ) {
    vvec3 edges1;
    vvec3 edges2;
    obj1.edge_vectorsW( edges1 );
    obj2.edge_vectorsW( edges2 );

    for( auto& v1 : edges1 )  {
        for( auto& v2: edges2 ) {
            vec3 axis = cross( v1, v2 );
            dir = axis;
            if( sat_axis_test(obj1, obj2, dir) ) return true;
        }
    }
    return false; 
}

//---------------------------------------------------------------------------


//entry points for SAT
//returns true of the objects are in contact
//else false
bool sat( Face &face1, Face &face2, vec3 &dir ) {
    if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
    if( sat_faces_test( face1, face2, dir ) ) return false;
    if( sat_edges_test( face1, face2, dir ) ) return false;
    return true;
}


//entry points for SAT
//returns true of the objects are in contact
//else false
bool sat( Polytope &obj1, Polytope &obj2, vec3 &dir ) {
    if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
    if( sat_faces_test( obj1, obj2, dir ) ) return false;
    if( sat_random_test( obj1, obj2, dir ) ) return false;
    if( sat_edges_test( obj1, obj2, dir ) ) return false;
    return true;
}

//entry points for SAT
//returns true of the objects are in contact
//else false
bool sat( ICollider &obj1, ICollider &obj2, vec3 &dir ) {
    if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
    if( sat_chung_wang_test( obj1, obj2, dir ) ) return false;
    return true;
}





