#ifndef DEFINE_H
#define DEFINE_H

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

using namespace glm;

#include <vector>

namespace vpe { 

    constexpr float SMALL = 1.0e-3f;
    constexpr float EPS = 1.0e-6f;

    inline bool equal(const float &f1, const float &f2) {
        float max = std::abs(f1) > std::abs(f2) ? std::abs(f1) : std::abs(f2);
        if( max < EPS ) return true;
        return std::abs(f2 - f1)/max < EPS;
    }

    inline bool equal(const vec3 &v1, const vec3 &v2) {
        return equal( v1.x, v2.x) && equal( v1.y, v2.y) && equal( v1.z, v2.z); 
    }

    bool operator==(const vec3 &v1, const vec3 &v2) {
        return equal(v1, v2);
    }

    bool operator!=(const vec3 &v1, const vec3 &v2) {
        return !equal(v1, v2);
    }

    using vint = std::vector<int>;
    using vfloat = std::vector<float>;
    using vvec3 = std::vector<vec3>;
    using vvec4 = std::vector<vec4>;
    using pint = std::pair<int,int>;
    using pfloat = std::pair<int,int>;

    inline bool equal(const vfloat & v1, const vfloat & v2 ) {
        if( v1.size() != v2.size() ) return false;
        for( int i=0; i<v1.size(); ++i) {
            if( !equal(v1[i], v2[i]) ) return false;
        }
        return true;
    }

}

#endif
