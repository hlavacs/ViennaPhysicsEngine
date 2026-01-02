#pragma once

#include "VPE.hpp"
namespace VD
{

    inline static vpe::VPEWorld::Polytope wall{
        {{-0.5_real, -0.5_real, -0.25_real},
         {-0.5_real, -0.5_real, 0.25_real},
         {-0.5_real, 0.5_real, 0.25_real},
         {-0.5_real, 0.5_real, -0.25_real},
         {0.5_real, -0.5_real, 0.25_real},
         {0.5_real, -0.5_real, -0.25_real},
         {0.5_real, 0.5_real, -0.25_real},
         {0.5_real, 0.5_real, 0.25_real}},
        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {5, 0}, {1, 4}, {3, 6}, {7, 2}}, // edges
        {
            {{0, 1}, {1, 1}, {2, 1}, {3, 1}},       // face 0
            {{4, 1}, {5, 1}, {6, 1}, {7, 1}},       // face 1
            {{0, -1}, {8, -1}, {4, -1}, {9, -1}},   // face 2
            {{2, -1}, {11, -1}, {6, -1}, {10, -1}}, // face 3
            {{3, -1}, {10, 1}, {5, -1}, {8, 1}},    // face 4
            {{1, -1}, {9, 1}, {7, -1}, {11, 1}}     // face 5
        },
        [](real mass, glmvec3 &s) { // callback for calculating the inertia tensor of this polytope
            return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
        }};

    inline static vpe::VPEWorld::Polytope ceiling{
        {{-3.0_real, -0.125_real, -2.5_real},
         {-3.0_real, -0.125_real, 2.5_real},
         {-3.0_real, 0.125_real, 2.5_real},
         {-3.0_real, 0.125_real, -2.5_real},
         {3.0_real, -0.125_real, 2.5_real},
         {3.0_real, -0.125_real, -2.5_real},
         {3.0_real, 0.125_real, -2.5_real},
         {3.0_real, 0.125_real, 2.5_real}},
        {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {5, 0}, {1, 4}, {3, 6}, {7, 2}}, // edges
        {
            {{0, 1}, {1, 1}, {2, 1}, {3, 1}},       // face 0
            {{4, 1}, {5, 1}, {6, 1}, {7, 1}},       // face 1
            {{0, -1}, {8, -1}, {4, -1}, {9, -1}},   // face 2
            {{2, -1}, {11, -1}, {6, -1}, {10, -1}}, // face 3
            {{3, -1}, {10, 1}, {5, -1}, {8, 1}},    // face 4
            {{1, -1}, {9, 1}, {7, -1}, {11, 1}}     // face 5
        },
        [](real mass, glmvec3 &s) { // callback for calculating the inertia tensor of this polytope
            return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
        }};
}
