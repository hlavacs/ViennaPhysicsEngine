
#pragma once

#include <iostream>

#include "VPE.hpp"
#include "face.hpp"
#include "misc.hpp"

struct Tetrahedra
{
private:
    glmvec3 a, b, c, d; // Points
    std::vector<Face> faces;
    glmvec3 center;
    real radius;

    bool isBad = false;

public:
    Tetrahedra(glmvec3 a, glmvec3 b, glmvec3 c, glmvec3 d) : a{a}, b{b}, c{c}, d{d}
    {
        faces.push_back(Face({a, b, c}));
        faces.push_back(Face({a, b, d}));
        faces.push_back(Face({a, c, d}));
        faces.push_back(Face({b, c, d}));
        calculateCircumSphere();
    }

    Tetrahedra(Face face, glmvec3 d)
    {
        std::vector<glmvec3> face_points = face.getPoints();
        this->a = face_points[0];
        this->b = face_points[1];
        this->c = face_points[2];
        this->d = d;

        faces.push_back(Face({a, b, c}));
        faces.push_back(Face({a, b, d}));
        faces.push_back(Face({a, c, d}));
        faces.push_back(Face({b, c, d}));
        calculateCircumSphere();
    }

    Tetrahedra() {}

    std::vector<glmvec3> getPoints() const
    {
        return std::vector<glmvec3>{a, b, c, d};
    }

    std::vector<Face> getFaces() const
    {
        return faces;
    }

    glmvec3 getCenter() const
    {
        return center;
    }

    real getRadius() const
    {
        return radius;
    }

    bool getBad()
    {
        return isBad;
    }

    void setBad(bool bad)
    {
        isBad = bad;
    }

    void calculateCircumSphere()
    {
        // http://www.convertalot.com/sphere_solver.html
        glmvec4 c1 = glmvec4(a.x * a.x + a.y * a.y + a.z * a.z,
                             b.x * b.x + b.y * b.y + b.z * b.z,
                             c.x * c.x + c.y * c.y + c.z * c.z,
                             d.x * d.x + d.y * d.y + d.z * d.z);

        glmvec4 c2 = glmvec4(a.x, b.x, c.x, d.x);
        glmvec4 c3 = glmvec4(a.y, b.y, c.y, d.y);
        glmvec4 c4 = glmvec4(a.z, b.z, c.z, d.z);
        glmvec4 c5 = glmvec4(1.0, 1.0, 1.0, 1.0);

        glmmat4 m11(c2, c3, c4, c5);
        glmmat4 m12(c1, c3, c4, c5);
        glmmat4 m13(c1, c2, c4, c5);
        glmmat4 m14(c1, c2, c3, c5);
        glmmat4 m15(c1, c2, c3, c4);

        real x_0 = real(0.5 * glm::determinant(m12) / glm::determinant(m11));
        real y_0 = real(-0.5 * glm::determinant(m13) / glm::determinant(m11));
        real z_0 = real(0.5 * glm::determinant(m14) / glm::determinant(m11));

        this->center = glmvec3(x_0, y_0, z_0);
        this->radius = std::sqrt(x_0 * x_0 + y_0 * y_0 + z_0 * z_0 - glm::determinant(m15) / glm::determinant(m11));
    }
    bool isInCircumSphere(glmvec3 point)
    {
        real distance = real(std::pow(point.x - this->center.x, 2) + std::pow(point.y - this->center.y, 2) + std::pow(point.z - this->center.z, 2));
        real epsilon_value = std::numeric_limits<real>::epsilon() * distance;
        return distance <= std::pow(this->radius, 2) + epsilon_value;
    }

    bool containsPoint(glmvec3 point)
    {
        return isVectorEqualTo(a, point) || isVectorEqualTo(b, point) || isVectorEqualTo(c, point) || isVectorEqualTo(d, point);
    }

    bool isTetrahedraEqualTo(const Tetrahedra &other) const
    {
        return isVectorEqualTo(a, other.a) && isVectorEqualTo(b, other.b) && isVectorEqualTo(c, other.c) ||
               isVectorEqualTo(a, other.b) && isVectorEqualTo(b, other.c) && isVectorEqualTo(c, other.a) ||
               isVectorEqualTo(a, other.c) && isVectorEqualTo(b, other.a) && isVectorEqualTo(c, other.b);
    }

    friend std::ostream &operator<<(std::ostream &o, const Tetrahedra &t)
    {
        o << "Tetrahedra\n";
        for (auto &eachPoint : t.getPoints())
        {
            o << "Point " << eachPoint << "\n";
        }
        o << "Sphere Center " << t.getCenter() << "\n";
        o << "Radius " << t.getRadius() << "\n";
        return o;
    }
};