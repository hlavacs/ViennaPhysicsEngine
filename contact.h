#ifndef CONTACT_H
#define CONTACT_H

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
#include "collision.h"



//a contact stores a contact point between two objects
//normal is the contact normal pointing away from obj2
struct contact {
    Polytope *obj1;    
    Polytope *obj2;    
    vec3 pos;
    vec3 normal;

    bool operator <(const contact& c) const; //need this for std::set
};

//define a hash for struct contact
 template<>
 struct std::hash<contact> {
    size_t operator()(const contact& c) {
        return std::hash<std::tuple<Polytope*,Polytope*,vec3,vec3>>()( 
            std::make_tuple( c.obj1, c.obj2, c.pos, c.normal) );
    }
 };

//operator< uses hash values for contacts
bool contact::operator <(const contact& c) const {
    return std::hash<contact>()(*this) < std::hash<contact>()(c);
}

//point contacts face if the distance point-plane is smaller than EPS AND 
//the point is inside the face Voronoi region.
void process_vertex_face_contact( Vertex &vertex, Face &face, std::set<contact> & contacts) {
    vec3 p = vertex.pointW();
    if( distance_point_plane( p, face.plueckerW() ) < EPS &&  face.voronoi(p) ) {
        contacts.insert( { vertex.polytope(), face.polytope(), p, face.normalW() }  );
    }
}

//process a possible edge-edge contact
//first test whether both lines are in contact
//then get the intersection between the edge face and the other edge line -> point
//Finally test whether the contact point is inside both line segments
void process_edge_edge_contact( Face &face1, Line &edge1, Face &face2, Line &edge2, std::set<contact> & contacts) {
    if( distance_line_line(edge1.plueckerW(), edge2.plueckerW()) < EPS ) {
        pluecker_point point = intersect_line_plane( edge2.plueckerW(), face1.plueckerW()).p3D();
        if( std::abs(point.w()) < EPS ) return; // if 0 then both are parallel
        vec3 p = point.p3D();
        if( edge1.in_segment(p) && edge2.in_segment(p) ) {
            contacts.insert( { face1.polytope(), face2.polytope(), p, face2.normalW() }  );
        }
    }
}

//test a pair of faces against each other.
//collide each vertex from one face with the other face
//collide each edge from one face with all edges from the other face
void process_face_face_contact(    Face &face1, Face &face2, vec3 &dir
                                ,  std::set<contact> & contacts ) {
    
    for( int v1 : face1.vertices() ) {      //go through all vertices of face 1
        Vertex &vertex1 = face1.polytope()->vertex(v1);
        if( collision( vertex1, face2, dir) ) {
            process_vertex_face_contact( vertex1, face2, contacts );
        }
    }

    for( int v2 : face2.vertices() ) {      //go through all vertices of face 2
        Vertex &vertex2 = face2.polytope()->vertex(v2);
        if( collision( vertex2, face1, dir) ) {
            process_vertex_face_contact( vertex2, face1, contacts );
        }
    }

    std::vector<Line> edges1;
    std::vector<Line> edges2;
    face1.edgesW( edges1 );
    face2.edgesW( edges2 );

    for( auto& edge1 : edges1 ) {      //go through all edge pairs
        for( auto& edge2 : edges2 ) {
            if( collision( edge1, edge2, dir) ) {
                process_edge_edge_contact( face1, edge1, face2, edge2, contacts );
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
        Face & face1 = obj1.face(f1);
        for( int f2 : obj2_faces) {
            Face & face2 = obj2.face(f2);
            if( collision( face1, face2, dir) ) {           //only if the faces actually touch - can also drop this if statement
                process_face_face_contact( face1, face2, dir, contacts ); //compare all vertices and edges in the faces
            }
        }
    }
}


//find a face of obj1 that touches obj2
//return it and its neighbors by adding their indices to a list
void get_face_obj_contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, vint& obj_faces ) {
    for( auto & face : obj1.faces() ) {
        if( collision( face, obj2, dir ) ) {
            obj_faces.push_back(face.index());
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

    get_face_obj_contacts( obj1, obj2, dir, obj1_faces );    //get list of faces from obj1 that touch obj2
    get_face_obj_contacts( obj2, obj1, dir, obj2_faces );    //get list of faces from obj2 that touch obj1
    process_face_obj_contacts( obj1, obj2, dir, obj1_faces, obj2_faces, contacts ); //collide them pairwise
}


//compute a list of contact points between two objects
void  contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
    if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
    neighboring_faces( obj1, obj2, dir, contacts);
}


#endif