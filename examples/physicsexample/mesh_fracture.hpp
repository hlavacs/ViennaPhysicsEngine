#ifndef MESH_FRACTURE_HPP
#define MESH_FRACTURE_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "VPE.hpp"

struct Face
{
private:
    std::vector<glmvec3> points;

public:
    Face(std::vector<glmvec3> given_points) : points{given_points} {}
    Face() {}

    const std::vector<glmvec3> getPoints() const
    {
        return points;
    }

    bool operator==(const Face &other) const
    {
        auto this_points = points;
        auto other_points = other.points;
        std::sort(this_points.begin(), this_points.end(), [](const glm::vec3 &a, const glm::vec3 &b)
                  { return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z); });
        std::sort(other_points.begin(), other_points.end(), [](const glm::vec3 &a, const glm::vec3 &b)
                  { return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z); });
        return this_points == other_points;
    }
};

namespace std
{
    template <>
    struct hash<Face>
    {
        std::size_t operator()(const Face &f) const
        {
            auto points = f.getPoints();
            std::sort(points.begin(), points.end(), [](const glm::vec3 &a, const glm::vec3 &b)
                      { return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z); });

            std::size_t hash = 0;
            for (const auto &eachPoint : points)
            {
                size_t hv = std::hash<real>()(eachPoint.x) ^ std::hash<real>()(eachPoint.y) ^ std::hash<real>()(eachPoint.z);
                hash ^= hv + 0x9e3779b9 + (hash << 6) + (hash << 2);
            }
            return hash;
        }
    };
}

struct Tetrahedra
{
private:
    glmvec3 a, b, c, d; // Points
    std::vector<Face> faces;
    glmvec3 center;
    real radius;

public:
    Tetrahedra(glmvec3 a, glmvec3 b, glmvec3 c, glmvec3 d) : a{a}, b{b}, c{c}, d{d}
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

        real x_0 = 0.5 * glm::determinant(m12) / glm::determinant(m11);
        real y_0 = -0.5 * glm::determinant(m13) / glm::determinant(m11);
        real z_0 = 0.5 * glm::determinant(m14) / glm::determinant(m11);

        center = glmvec3(x_0, y_0, z_0);
        radius = x_0 * x_0 + y_0 * y_0 + z_0 * z_0 - glm::determinant(m15) / glm::determinant(m11);

        faces.push_back(Face({a, b, c}));
        faces.push_back(Face({b, c, d}));
        faces.push_back(Face({a, c, d}));
        faces.push_back(Face({b, c, d}));
    }
    Tetrahedra(std::set<Face> faces)
    {
        std::set<glmvec3> points;
        for (auto eachFace : faces)
        {
            std::vector<glmvec3> face_points = eachFace.getPoints();
            points.insert(face_points.begin(), face_points.end());
        }

        if (points.size() != 4)
        {
            throw std::runtime_error("Something went wrong!");
        }
        auto it = points.begin();
        a = *it++;
        b = *it++;
        c = *it++;
        d = *it++;
    }
    Tetrahedra() {}

    std::vector<glmvec3> getPoints()
    {
        return {a, b, c, d};
    }

    std::vector<Face> getFaces()
    {
        return faces;
    }

    bool isInCircumSphere(glmvec3 point)
    {
        return (std::pow(center.x - point.x, 2) + std::pow(center.y - point.y, 2) + std::pow(center.z - point.z, 2) - std::pow(radius, 2)) <= 0;
    }

    bool operator==(const Tetrahedra &other) const
    {
        return a == other.a && b == other.b && c == other.c && d == other.d;
    }
};

inline std::vector<glmvec3> getInitRandomPoints(size_t amount)
{
    std::default_random_engine rnd_gen{12345};
    std::uniform_real_distribution<> rnd_unif{-0.5_real, 0.5_real};

    std::vector<glmvec3> startings_points;
    startings_points.reserve(amount);

    for (size_t idx = 0; idx < amount; ++idx)
    {
        real x = (real)rnd_unif(rnd_gen);
        real y = (real)rnd_unif(rnd_gen);
        real z = (real)rnd_unif(rnd_gen);

        startings_points.push_back(glmvec3{x, y, z});
    }

    return startings_points;
}

inline Tetrahedra getSuperTetrahedron(const std::vector<glmvec3> &points)
{
    real min_x = std::numeric_limits<real>::max();
    real min_y = std::numeric_limits<real>::max();
    real min_z = std::numeric_limits<real>::max();

    real max_x = std::numeric_limits<real>::min();
    real max_y = std::numeric_limits<real>::min();
    real max_z = std::numeric_limits<real>::min();

    for (const glmvec3 &forEachPoint : points)
    {
        min_x = std::min(min_x, forEachPoint.x);
        min_y = std::min(min_y, forEachPoint.y);
        min_z = std::min(min_y, forEachPoint.z);

        max_x = std::min(max_x, forEachPoint.x);
        max_y = std::min(max_y, forEachPoint.y);
        max_z = std::min(max_z, forEachPoint.z);
    }

    real mid_x = (min_x + max_x) / 2.0;
    real mid_y = (min_y + max_y) / 2.0;
    real mid_z = (min_z + max_z) / 2.0;

    real deltaMax = std::max({max_x - min_x, max_y - min_y, max_z - min_z});
    real scale = deltaMax * 20.0;

    return Tetrahedra({mid_x - scale, mid_y - scale, mid_z - scale}, {mid_x + scale, mid_y - scale, mid_z - scale}, {mid_x, mid_y + scale, mid_z - scale}, {mid_x, mid_y, mid_z - scale});
    // return vpe::VPEWorld::Polytope{
    //     {{mid_x - scale, mid_y - scale, mid_z - scale}, {mid_x + scale, mid_y - scale, mid_z - scale}, {mid_x, mid_y + scale, mid_z - scale}, {mid_x, mid_y, mid_z - scale}},
    //     {{0, 1}, {1, 2}, {2, 3}, {0, 3}, {1, 3}, {2, 3}},
    //     {{{0, 1}, {1, 2}, {2, 0}},
    //      {{0, 1}, {1, 3}, {3, 0}},
    //      {{1, 2}, {2, 3}, {3, 2}},
    //      {{0, 2}, {2, 3}, {3, 0}}},
    //     [](real mass, glmvec3 &s)
    //     {
    //         return mass * glmmat3{{s.y * s.y + s.z * s.z, 0, 0}, {0, s.x * s.x + s.z * s.z, 0}, {0, 0, s.x * s.x + s.y * s.y}} / 12.0_real;
    //     }};
}

inline std::vector<Tetrahedra> BowyerWatson(std::vector<glmvec3> pointList)
{
    std::list<Tetrahedra> triangulation;
    Tetrahedra superTetrahedron = getSuperTetrahedron(pointList);
    triangulation.push_back(superTetrahedron);

    for (const auto &eachPoint : pointList)
    {
        std::set<Tetrahedra> badTetrahedron;
        for (auto &eachTetrahedron : triangulation)
        {
            if (eachTetrahedron.isInCircumSphere(eachPoint))
            {
                badTetrahedron.insert(eachTetrahedron);
            }

            std::map<Face, size_t> face_map;
            for (Tetrahedra eachBadTetrahedron : badTetrahedron)
            {
                for (Face eachFace : eachBadTetrahedron.getFaces())
                {
                    ++face_map[eachFace];
                }
            }

            std::set<Face> polygon;
            for (const auto &[face, count] : face_map)
            {
                if (count == 1)
                {
                    polygon.insert(face);
                }
            }

            for (Tetrahedra eachBadTetrahedron : badTetrahedron)
            {
                triangulation.remove(eachBadTetrahedron);
            }
            triangulation.push_back(Tetrahedra(polygon));
        }
    }
    std::vector<glmvec3> vertices = superTetrahedron.getPoints();
    for (Tetrahedra eachTetrahedra : triangulation)
    {
        std::vector<glmvec3> points = eachTetrahedra.getPoints();
        for (glmvec3 eachPoint : points)
        {
            if (std::find(vertices.begin(), vertices.end(), eachPoint) != vertices.end())
            {
                triangulation.remove(eachTetrahedra);
                break;
            }
        }
    }
    triangulation.remove(superTetrahedron);
}

inline vpe::VPEWorld::Polytope convertToVPEPolytope(Tetrahedra tetrahedra)
{
    return;
}

inline vpe::VPEWorld::Polytope fracture_polytope(vpe::VPEWorld::Polytope *polytope, size_t amount)
{
    std::vector<glmvec3> pointList = getInitRandomPoints(amount);
    std::vector<Tetrahedra> tetrahedrons = BowyerWatson(pointList);
    return;
}

#endif