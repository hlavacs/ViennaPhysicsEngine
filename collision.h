#ifndef COLLISION_H
#define COLLISION_H

#include <iostream>
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#include "collider.h"

namespace vpe {


    constexpr int NUM_RANDOM_DIR = 32;


    //test whether a direction is a separating axis
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    bool sat_axis_test1( ICollider &obj1, ICollider &obj2, vec3 &dir, vec3 &r, float &d) {
        vec3 dir2 = -1.0f*dir;
        vec3 p = obj1.support( dir );
        vec3 q = obj2.support( dir2 );
        r = normalize(q - p); 
        d = dot( r, dir );
        return( d > EPS);       //allow for numerical inaccuracies
    }

    bool sat_axis_test1( ICollider &obj1, ICollider &obj2, vec3 &dir ) {
        vec3 r;
        float d;
        return sat_axis_test1( obj1, obj2, dir, r, d);
    }

    //test whether a direction (or its reverse) is a separating axis
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    bool sat_axis_test2( ICollider &obj1, ICollider &obj2, vec3 &dir, vec3 &r, float &d) {

        vec3 dir2 = -1.0f*dir;
        {
            vec3 p = obj1.support( dir );
            vec3 q = obj2.support( dir2 );
            r = normalize(q - p); 
            d = dot( r, dir );
            if( d > EPS) return true;       //allow for numerical inaccuracies
        }
        {
            vec3 p2 = obj1.support( dir2 );
            vec3 q2 = obj2.support( dir );
            vec3 r2 = normalize(q2 - p2); 
            float d2 = dot( r2, dir2 );
            if( d2 > EPS ) {
                dir = dir2;
                r = r2;
                d = d2;
                return true;
            }
        }
        return false;
    }

    bool sat_axis_test2( ICollider &obj1, ICollider &obj2, vec3 &dir ) {
        vec3 r;
        float d;
        return sat_axis_test2( obj1, obj2, dir, r, d);
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
            found = sat_axis_test1(obj1, obj2, random_axes[i], r, d);
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
        while( loop < max_loops && !sat_axis_test1(obj1, obj2, dir, r, d) ) {
            dir = dir - (1.0f + f)*dot(r, dir)*r;
            ++loop;
            f = std::max( f * 0.97f, 0.5f );   //apply damping to prevent small gaps from oscillating forever
        }
        //std::cout << "Loops: " << loop << std::endl;
        return sat_axis_test1(obj1, obj2, dir, r, d);
    }

    //SAT using the normals of 2 faces
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    bool sat_faces_test2( Face &face1, Face &face2, vec3 &dir ) {
        vec3 n = face1.normalW();
        dir = n;
        if( sat_axis_test2(face1, face2, dir ) ) return true;
        std::vector<Line> edges1;
        face1.edgesW( edges1 );
        for( auto &edge : edges1 ) {
            dir = cross(edge.m_dir, n);
            if( sat_axis_test2(face1, face2, dir ) ) return true;
        }
        return false;
    }

    //SAT using the normals of 2 faces
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    bool sat_faces_test( Face &face1, Face &face2, vec3 &dir ) {
        if( sat_faces_test2( face1, face2, dir) ) return true;
        return sat_faces_test2( face2, face1, dir);
    }

    //SAT using the normals of polytope faces
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    bool sat_faces_test( Polytope &obj1, Polytope &obj2, vec3 &dir ) {
        for( auto& face : obj1.faces() ) {
            dir = face.normalW();
            if( sat_axis_test1(obj1, obj2, dir) ) return true;
        }

        for( auto& face : obj2.faces() ) {
            dir = face.normalW();
            if( sat_axis_test1(obj1, obj2, dir) ) return true;
        }

        return false;
    }


    //SAT using the cross products of edge pairs from two Faces/polytopes
    //returns true if a separating axis was found (i.e. objects are NOT in contact), else false
    template<typename T>
    bool sat_edges_test( T &obj1, T &obj2, vec3 &dir ) {
        std::set<std::pair<int,int>> pairs1;    //each edge should be included only once
        std::set<std::pair<int,int>> pairs2;
        std::vector<Line> edges1;
        std::vector<Line> edges2;
        obj1.edgesW( edges1, &pairs1 );
        obj2.edgesW( edges2, &pairs2 );

        for( auto& l1 : edges1 )  {
            for( auto& l2: edges2 ) {
                vec3 axis = cross( l1.m_dir, l2.m_dir );
                dir = axis;
                if( sat_axis_test2(obj1, obj2, dir) ) return true;
            }
        }
        return false; 
    }

    //---------------------------------------------------------------------------


    //test for collision between a vertex and a face
    //returns true of the objects are in contact
    //else false
    bool collision( Vertex &vertex, Face &face, vec3 &dir ) {
        vec3 p = vertex.pointW();
        return distance_point_plane( p, face.plueckerW() ) < EPS && face.voronoi(p);
    }

    //test for collision between two faces
    //returns true of the objects are in contact
    //else false
    bool collision( Face &face1, Face &face2, vec3 &dir ) {
        if( dot(dir, dir) < EPS ) {
            dir = vec3(0.0f, 1.0f, 0.0f);
        } else {
            sat_axis_test1( face1, face2, dir );
        }
        if( sat_faces_test( face1, face2, dir ) ) return false;
        return true;
    }

    //test for collision between two polytopes
    //returns true of the objects are in contact
    //else false
    bool collision( Polytope &obj1, Polytope &obj2, vec3 &dir ) {
        if( dot(dir, dir) < EPS ) {
            dir = vec3(0.0f, 1.0f, 0.0f);
        } else {
            sat_axis_test1( obj1, obj2, dir );
        }
        if( sat_faces_test( obj1, obj2, dir ) ) return false;
        if( sat_chung_wang_test( obj1, obj2, dir ) ) return false;
        return true;
    }

    //test for collision between two colliders
    //returns true of the objects are in contact
    //else false
    bool collision( ICollider &obj1, ICollider &obj2, vec3 &dir ) {
        if( dot(dir, dir) < EPS ) {
            dir = vec3(0.0f, 1.0f, 0.0f);
        } else {
            sat_axis_test1( obj1, obj2, dir );
        }
        if( sat_chung_wang_test( obj1, obj2, dir ) ) return false;
        return true;
    }


}


#endif

