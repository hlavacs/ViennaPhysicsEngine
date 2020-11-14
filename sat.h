#pragma once


#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

#include "Collider.h"
#include "hash.h"


using v3pair = std::pair<vec3,vec3>;

bool sat( Collider *obj1, Collider *obj2, vec3 *dir ) {

    if( dot(*dir, *dir) < 1.0e-6 ) {
        *dir = vec3(1.0f, 1.0f, 1.0f);
    }

    vec3 p = obj1->support( *dir );
    vec3 q = obj2->support( -1.0f* (*dir) );
    vec3 r = normalize(q - p); 

    std::set<size_t> cache;

    while( dot( r, *dir ) <= 0.0f ) {
        auto pair = std::hash<v3pair>()(std::make_pair(p,q));
        if( cache.contains(pair)) return true;
        cache.insert(pair);

        *dir = reflect( *dir, r); //dir = dir - 2*dot(r, dir)*r

        p = obj1->support( *dir );
        q = obj2->support( -1.0f* (*dir) );
        r = normalize(q - p);
    }
    return false;
}


