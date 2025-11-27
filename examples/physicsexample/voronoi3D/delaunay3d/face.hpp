
#pragma once

#include <iostream>

#include "VPE.hpp"
#include "misc.hpp"

struct Face
{
private:
    glmvec3 a, b, c; 
    bool isBad = false; 

public:
    Face(glmvec3 a, glmvec3 b, glmvec3 c) : a{a}, b{b}, c{c} {}
    Face() {}

    std::vector<glmvec3> getPoints() const
    {
        return {a, b, c};
    }

    bool getBad()
    {
        return isBad;
    }

    void setBad(bool bad)
    {
        isBad = bad;
    }

    bool isFaceEqualTo(const Face &other) const
    {
        return isVectorEqualTo(a, other.a) && isVectorEqualTo(b, other.b) && isVectorEqualTo(c, other.c) ||
               isVectorEqualTo(a, other.b) && isVectorEqualTo(b, other.c) && isVectorEqualTo(c, other.a) ||
               isVectorEqualTo(a, other.c) && isVectorEqualTo(b, other.a) && isVectorEqualTo(c, other.b);
    }

    friend std::ostream &operator<<(std::ostream &o, const Face &f)
    {
        o << "Face\n";
        for (auto &eachPoint : f.getPoints())
        {
            o << "Point" << eachPoint << "\n";
        }
        return o;
    }
};