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


struct face_contact {
    Polytope *obj1;
    Polytope *obj2;
    int f;
    ICollider *collider;

    face_contact(Polytope *o1, Polytope *o2, int fi, ICollider *c) 
    : obj1(o1), obj2(o2), f(fi), collider(c) {};
};

 template<>
 struct std::hash<face_contact> {
    size_t operator()(const face_contact& c) {
        return std::hash<int>()(c.f);
    }
 };


void process_face_contacts( std::vector<face_contact>& faces, std::set<vec3> & contacts ) {
    
}

void neighboring_faces( Polytope *obj1, Polytope *obj2, vec3 *dir, std::set<vec3> & contacts ) {
    if( dot(*dir, *dir) < 1.0e-6 ) *dir = vec3(0.0f, 1.0f, 0.0f);

    std::vector<face_contact> face_contacts;

    for( int f = 0; f < obj1->m_faces.size(); ++f ) {
        ICollider *collider = &obj1->m_faces[f];
        if( sat( collider, obj2, dir ) ) {
            face_contacts.emplace_back( obj1, obj2, f, collider );
        }
    }
    for( int f = 0; f < obj2->m_faces.size(); ++f ) {
        ICollider *collider = &obj1->m_faces[f];
        if( sat( collider, obj1, dir ) ) {
            face_contacts.emplace_back( obj2, obj1, f, collider );
        }
    }
    process_face_contacts( face_contacts, contacts);
}


void  contacts( Polytope *obj1, Polytope *obj2, vec3 *dir, std::set<vec3> & contacts ) {
    neighboring_faces( obj1, obj2, dir, contacts);
}
