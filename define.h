#ifndef DEFINE_H
#define DEFINE_H

#include "glm/glm/glm.hpp"
#include "glm/glm/ext.hpp"

using namespace glm;

#include <vector>

constexpr float SMALL = 1.0e-3f;
constexpr float EPS = 1.0e-6f;

using vint = std::vector<int>;
using vvec3 = std::vector<vec3>;
using vvec4 = std::vector<vec4>;

using ipair = std::pair<int,int>;


#endif
