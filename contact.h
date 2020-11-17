#pragma once

#include <iostream>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "hash.h"
#include <random>
#include <set>

#include "Collider.h"
#include "sat.h"

using vec3pair = std::pair<vec3,vec3>;



struct face_face_contact {
    Polytope *obj1;    
    Polytope *obj2;
    int f1;
    int f2;

    face_face_contact(Polytope *o1, Polytope *o2, int fi1, int fi2 = -1) : obj1(o1), obj2(o2), f1(fi1), f2(fi2) {};
};

struct contact {
    Polytope *obj1;    
    Polytope *obj2;    
    vec3 pos;
    vec3 normal;
};


 /*template<>
 struct std::hash<face_contact> {
    size_t operator()(const face_contact& c) {
        return std::hash<std::pair<int,int>>()(c.f1, c.f2);
    }
 };*/

void process_vertex_face_contact( Polytope *obj1, Polytope *obj2, int v1, int f2, std::set<contact> & contacts) {

}

void process_edge_edge_contact( Polytope *obj1, Polytope *obj2, int e1, int e2, std::set<contact> & contacts) {

}



void process_face_face_contact(    Polytope *obj1, Polytope *obj2, vec3 *dir
                                ,  int f1, int f2, std::set<contact> & contacts ) {

     for( int v1 : obj1->m_faces[f1].vertices ) {      //go through all vertices of face 1
        if( sat( &obj1->m_vertices[v1], &obj2->m_faces[f2], dir) ) {
            process_vertex_face_contact( obj1, obj2, v1, f2, contacts );
        }
     }

     for( int v2 : obj2->m_faces[f2].vertices ) {      //go through all vertices of face 2
        if( sat( &obj2->m_vertices[v2], &obj1->m_faces[f1], dir) ) {
            process_vertex_face_contact( obj2, obj1, v2, f1, contacts );
        }
     }

     for( int e1 : obj1->m_faces[f1].edges ) {      //go through all edge pairs
        for( int e2 : obj2->m_faces[f2].edges) {
            if( sat( &obj1->m_edges[e1], &obj2->m_edges[e2], dir) ) {
                process_edge_edge_contact( obj1, obj2, e1, e2, contacts );
            }
        }
     }
}

void process_face_obj_contacts(     Polytope *obj1, Polytope *obj2, vec3 *dir
                                ,   std::set<int>& obj1_faces, std::set<int>& obj2_faces
                                ,   std::set<contact> & contacts ) {
    
    for( int f1 : obj1_faces) {
        for( int f2 : obj2_faces) {
            //if( sat( &obj1->m_faces[f1], &obj2->m_faces[f2], dir) ) {         //can apply another filter?
            process_face_face_contact( obj1, obj2, dir, f1, f2, contacts );
            //}
        }
    }
}

void get_face_obj_contacts( Polytope *obj1, Polytope *obj2, vec3 *dir, std::set<int>& obj_faces ) {
    for( int f = 0; f < obj1->m_faces.size(); ++f ) {
        Face *face = &obj1->m_faces[f];
        if( sat( face, obj2, dir ) ) {              //find the first face from obj1 that touches obj2
            obj_faces.insert(f);                    //insert into result list
            auto& neighbors = obj1->get_face_neighbors(f);
            std::copy( std::begin(neighbors), std::end(neighbors), std::back_inserter(obj_faces) );   //also insert its neighbors
            return;
        }
    }
}

void neighboring_faces( Polytope *obj1, Polytope *obj2, vec3 *dir, std::set<contact> & contacts ) {
    std::set<int> obj1_faces;
    std::set<int> obj2_faces;

    get_face_obj_contacts(obj1, obj2, dir, obj1_faces );    
    get_face_obj_contacts(obj2, obj1, dir, obj2_faces );    
    process_face_obj_contacts( obj1, obj2, dir, obj1_faces, obj2_faces, contacts );
}


void  contacts( Polytope *obj1, Polytope *obj2, vec3 *dir, std::set<contact> & contacts ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);
    neighboring_faces( obj1, obj2, dir, contacts);
}


