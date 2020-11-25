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

namespace vpe {

    //a contact stores a contact point between two objects
    //normal is the contact normal pointing away from obj2
    struct contact {
        Polytope *obj1;    
        Polytope *obj2;    
        vec3 pos;
        vec3 normal;

        int v1 = -1;
        int f1 = -1;
        int f2 = -1;
        int e1 = -1;
        int e2 = -1;

        bool operator <(const contact& c) const; //need this for std::set
    };

}


//define a hash for struct contact
//must be defined in namespace std
template<>
struct std::hash<vpe::contact> {
    size_t operator()(const vpe::contact& c) {
        return std::hash<std::tuple<vpe::Polytope*,vpe::Polytope*,vec3>>()( 
            std::make_tuple( c.obj1, c.obj2, c.pos ) );
    }
};


namespace vpe {

    //operator< uses hash values for contacts
    inline bool contact::operator <(const contact& c) const {
        return std::hash<contact>()(*this) < std::hash<contact>()(c);
    }

    inline void add_contact( contact c, std::set<contact> & contacts) {
            if( reinterpret_cast<std::uintptr_t>(c.obj1) > reinterpret_cast<std::uintptr_t>(c.obj2) ) {
                contacts.insert( c );
            } else {
                contacts.insert( {  c.obj2, c.obj1, c.pos, -c.normal, c.v1, c.f2, c.f1 }  );
            }
    }

    //point contacts face if the distance point-plane is smaller than EPS AND 
    //the point is inside the face Voronoi region.
    //since contacts can be symmetric, as convention, sort the polytopes by larger pointer first
    inline void process_vertex_face_contact( Vertex &vertex, Face &face1, Face &face2, std::set<contact> & contacts) {
        if( collision( vertex, face2 ) ) {
            add_contact( {    vertex.polytope(), face2.polytope(), vertex.pointW(), face2.normalW()
                            , vertex.index(), face1.index(), face2.index() }
                            , contacts );
        }
    }

    //process a possible edge-edge contact
    //first test whether both lines are in contact
    //then test that they are not parallel
    //then use http://mathforum.org/library/drmath/view/62814.html to find the intersection
    //Finally test whether the contact point is inside both line segments
    inline void process_edge_edge_contact( Face &face1, Line &edge1, Face &face2, Line &edge2, std::set<contact> & contacts) {
        if( distance_line_line(edge1.plueckerW(), edge2.plueckerW()) < EPS ) {
            vec3 cp = cross(edge1.m_dir, edge2.m_dir);
            if( cp == vec3{0,0,0}) return; //if edges are parallel there is no intersection
            vec3 diff = edge1.pos() - edge2.pos();
            vec3 rs = cross( diff, edge2.m_dir );
            float t = length(rs) / length(cp);
            if( t<0 || t>1) return;
            float u = length( t*edge1.m_dir - diff ) / length( edge2.m_dir);
            if( u<0 || u>1) return;

            vec3 p = edge1.pos() + t * edge1.m_dir;
            vec3 outwards2 = cross( edge2.m_dir, face2.normalW());  //points outwards of face2
            if( dot( cp, outwards2 ) <0  ) cp *= -1.0f;    //contact normal should point outwards of face2
            add_contact( {  face1.polytope(), face2.polytope(), p, cp, -1, face1.index(), face2.index() }
                            , contacts  );
        }
    }

    //test a pair of faces against each other.
    //collide each vertex from one face with the other face
    //collide each edge from one face with all edges from the other face
    inline void process_face_face_contact( Face &face1, Face &face2, vec3 &dir,  std::set<contact> & contacts ) {
        
        if( !collision( face1, face2, dir) || dot(face1.normalW(), face2.normalW()) >= 0.0f ) return;   //only if the faces actually touch 

        for( int v1 : face1.face_vertices() ) {      //go through all vertices of face 1
            process_vertex_face_contact( face1.polytope()->vertex(v1), face1, face2, contacts );
        }

        for( int v2 : face2.face_vertices() ) {      //go through all vertices of face 2
            process_vertex_face_contact( face2.polytope()->vertex(v2), face2, face1, contacts );
        }

        std::vector<Line> edges1;
        std::vector<Line> edges2;
        face1.edgesW( edges1 );
        face2.edgesW( edges2 );

        for( auto& edge1 : edges1 ) {      //go through all edge pairs
            for( auto& edge2 : edges2 ) {
                process_edge_edge_contact( face1, edge1, face2, edge2, contacts );
            }
        }
    }


    //find a list of face-pairs that touch each other
    //process these pairs by colliding a face agains vertices and edges of the other face
    inline void process_face_obj_contacts(     Polytope &obj1, Polytope &obj2, vec3 &dir
                                    ,   vint& obj1_faces, vint& obj2_faces
                                    ,   std::set<contact> & contacts ) {
        
        for( int f1 : obj1_faces) {             // go through all face-face pairs
            Face & face1 = obj1.face(f1);
            for( int f2 : obj2_faces) {
                Face & face2 = obj2.face(f2);
                //if( f1==2 && f2==3)
                process_face_face_contact( face1, face2, dir, contacts ); //compare all vertices and edges in the faces
            }
        }
    }


    //find the first face of obj1 that touches obj2
    //return it and its neighbors by adding their indices to a list
    inline void find_face_obj_contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, vint& obj_faces ) {
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
    inline void neighboring_faces( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
        vint obj1_faces;
        vint obj2_faces;

        find_face_obj_contacts( obj1, obj2, dir, obj1_faces );    //get list of faces from obj1 that touch obj2
        find_face_obj_contacts( obj2, obj1, dir, obj2_faces );    //get list of faces from obj2 that touch obj1
        process_face_obj_contacts( obj1, obj2, dir, obj1_faces, obj2_faces, contacts ); //collide them pairwise
    }


    //-------------------------------------------------------------------------------------------


    //compute a list of contact points between two objects
    inline void  contacts( Polytope &obj1, Polytope &obj2, vec3 &dir, std::set<contact> & contacts ) {
        if( dot(dir, dir) < EPS ) dir = vec3(0.0f, 1.0f, 0.0f);
        neighboring_faces( obj1, obj2, dir, contacts);
    }

}


#endif