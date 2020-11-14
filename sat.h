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
    vec3 r = q - p; 

    std::set<size_t> cache;
    cache.insert(std::hash<v3pair>()(std::make_pair(p,q)));
    cache.insert(std::hash<v3pair>()(std::make_pair(q,p)));

    while( dot( r, *dir ) <= 0.0f ) {
        *dir = reflect( *dir, r); 

        p = obj1->support( *dir );
        q = obj2->support( -1.0f* (*dir) );

        auto pair1 = std::hash<v3pair>()(std::make_pair(p,q));
        auto pair2 = std::hash<v3pair>()(std::make_pair(q,p));

        if( cache.contains(pair1) || cache.contains(pair2)) return true;

        cache.insert(pair1);
        cache.insert(pair2);

        r = q - p; 
    }
    return false;

}


