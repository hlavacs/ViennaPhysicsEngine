#pragma once

#include <iostream>

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "hash.h"
#include <random>
#include <set>

#include "define.h"
#include "collider.h"
#include "sat.h"

using vec3pair = std::pair<vec3,vec3>;


struct contact {
    Polytope *obj1;    
    Polytope *obj2;    
    vec3 pos;
    vec3 normal;
};


 /*template<>
 struct std::hash<face_contact> {
    size_t operator()(const face_contact& c) {
        return std::hash<std::pair<int,int>>()( std::make_pair(c.f1, c.f2));
    }
 };*/


//point contacts face if the distance point-plane is smaller than EPS AND
//the point is inside the face Voronoi region, i.e. it lies on the left of all planes
//defined by a face edge and the face normal vector
void process_vertex_face_contact( Vertex &vertex, Face &face, std::set<contact> & contacts) {

}

void process_edge_edge_contact( Line &edge1, Line &edge2, std::set<contact> & contacts) {
}

//test a pair of faces against each other.
//collide each vertex from one face with the other face
//collide each edge from one face with all edges from the other face
void process_face_face_contact(    Polytope &obj1, Polytope &obj2, vec3 &dir
                                ,  int f1, int f2, std::set<contact> & contacts ) {
    
    Face &face1 = obj1.face(f1);
    Face &face2 = obj1.face(f2);

    for( int v1 : face1.vertices() ) {      //go through all vertices of face 1
        if( sat( obj1.vertex(v1), face2, dir) ) {
            process_vertex_face_contact( obj1.vertex(v1), face1, contacts );
        }
    }

    for( int v2 : face2.vertices() ) {      //go through all vertices of face 2
        if( sat( obj2.vertex(v2), face1, dir) ) {
            process_vertex_face_contact( obj2.vertex(v2), face2, contacts );
        }
    }

    std::vector<Line> edges1;
    std::vector<Line> edges2;
    face1.edgesW( edges1 );
    face2.edgesW( edges2 );

    for( auto& edge1 : edges1 ) {      //go through all edge pairs
        for( auto& edge2 : edges2 ) {
            if( sat( edge1, edge2, dir) ) {
                process_edge_edge_contact( edge1, edge2, contacts );
            }
        }
    }
}


//find a list of face-pairs that touch each other
//process these pairs by colliding a face agains vertices and edges of the other face
void process_face_obj_contacts(     Polytope &obj1, Polytope &obj2, vec3 &dir
                                ,   vint& obj1_faces, vint& obj2_faces
                                ,   std::set<contact> & contacts ) {
    
    for( int f1 : obj1_faces) {             // go through all face-face pairs
        for( int f2 : obj2_faces) {
            if( sat( obj1.face(f1), obj2.face(f2), dir) ) {           //only if the faces actually touch - can also drop this if statement
                process_face_face_contact( obj1, obj2, dir, f1, f2, contacts ); //compare all vertices and edges in the faces
            }
        }
    }
}


//find a face of obj1 that touches obj2
//return it and its neighbors by adding their indices to a list
void get_face_obj_contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, vint& obj_faces ) {
    for( int f = 0; f < obj1.faces().size(); ++f ) {
        Face &face = obj1.face(f);
        if( sat( face, obj2, dir ) ) {              //find the first face from obj1 that touches obj2
            obj_faces.push_back(f);                    //insert into result list
            auto& neighbors = face.neighbors();
            std::copy( std::begin(neighbors), std::end(neighbors), std::back_inserter(obj_faces) );   //also insert its neighbors
            return;
        }
    }
}


//neighboring faces algorithm
void neighboring_faces( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
    vint obj1_faces;
    vint obj2_faces;

    get_face_obj_contacts(obj1, obj2, dir, obj1_faces );    //get list of faces from obj1 that touch obj2
    get_face_obj_contacts(obj2, obj1, dir, obj2_faces );    //get list of faces from obj2 that touch obj1
    process_face_obj_contacts( obj1, obj2, dir, obj1_faces, obj2_faces, contacts ); //collide them pairwise
}


//compute a list of contact points between two objects
void  contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
    if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
    neighboring_faces( obj1, obj2, dir, contacts);
}


