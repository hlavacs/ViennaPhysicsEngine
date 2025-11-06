
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

    const real getArea() const
    {
        glmvec3 AB = b - a;
        glmvec3 AC = c - a;

        glmvec3 cross = glm::cross(AB, AC);
        return real(glm::length(cross) * 0.5);
    }

    bool operator<(const Face &other) const
    {
        return this->getArea() < other.getArea();
    }

    bool operator==(const Face &other) const
    {
        return a == other.a && b == other.b && c == other.c ||
               a == other.b && b == other.c && c == other.a ||
               a == other.c && b == other.a && c == other.b;
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