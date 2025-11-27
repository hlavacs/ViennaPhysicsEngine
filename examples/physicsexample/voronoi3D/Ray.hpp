#pragma once

#include "VPE.hpp"

struct Ray
{
private:
    glmvec3 origin;
    glmvec3 direction;

public:
    Ray(glmvec3 origin, glmvec3 direction) : origin{origin}, direction{direction} {}

    glmvec3 getOrigin()
    {
        return origin;
    }

    glmvec3 getDirection()
    {
        return direction;
    }

    void setOrigin(glmvec3 origin)
    {
        this->origin = origin;
    }

    void getDirection(glmvec3 direction)
    {
        this->direction = direction;
    }
};