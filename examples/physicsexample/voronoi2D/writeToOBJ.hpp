#pragma once

#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <filesystem>

#include "Voronoi.hpp"
#include "VPE.hpp"

namespace VD
{
    namespace fs = std::filesystem;

    inline void writeHeader(std::ofstream &file, const std::string &name, int &index)
    {
        file << "# " + name + std::to_string(index) + ".obj\n";
        file << "#\n\n";
        file << "mtllib " + name + ".mtl\n";
        file << "o " + name + "\n";
        file << "usemtl " + name + "\n\n";
    }

    inline void writeVertices(std::ofstream &file, real x, real y, real z)
    {
        file << std::setprecision(6) << std::fixed << "v " << x << " " << y << " " << z << "\n";
    }

    inline void writeTexture(std::ofstream &file)
    {
        file << "vt 0.000000 0.000000\n";
        file << "vt 1.000000 0.000000\n";
        file << "vt 0.000000 1.000000\n";
        file << "vt 1.000000 1.000000\n ";
    }

    inline void writeNormal(std::ofstream &file, real x, real y, real z)
    {
        file << std::setprecision(6) << std::fixed << "vn " << x << " " << y << " " << z << "\n";
    }

    inline void writeFace(std::ofstream &file, int x, int y, int z, int normal_index)
    {
        file << "f " << x << "/1/" << normal_index << " " << y << "/2/" << normal_index << " " << z << "/4/" << normal_index << "\n";
    }

    inline void writeNewLine(std::ofstream &file)
    {
        file << "\n";
    }

    inline void writeToOBJ(const std::string &name, const std::vector<VD::Voronoi> &voronois, const std::pair<real, real> &z_coords)
    {
        int polytope_index = 0;
        fs::path parent_path = fs::current_path().parent_path().parent_path();
        fs::path path = fs::path(parent_path.string() + "\\media\\models\\test\\destruction\\");

        for (const Voronoi &eachVoronoi : voronois)
        {
            std::string filename = path.string() + name + std::to_string(polytope_index) + ".obj";
            std::ofstream file(filename);

            if (!file.is_open())
            {
                std::cerr << "Error: Could not open the file " << filename << std::endl;
                return;
            }
            writeHeader(file, name, polytope_index);

            /** Fetch voronoi vertices */
            std::vector<glmvec2> voronoi_vertices = eachVoronoi.getVertices();
            int vertices_size = int(voronoi_vertices.size());
            glmvec2 center = getAveragePoint(voronoi_vertices);

            /** variables for calculating vertex normals */
            std::vector<glmvec3> faces = {};
            std::vector<glmvec3> vertex_normals = {};
            int normal_index = 1;

            /** Add z dimension to vertices */
            /** Add Front vertices */
            for (auto each_vertice : voronoi_vertices)
            {
                faces.push_back({each_vertice.x, each_vertice.y, z_coords.second});
                writeVertices(file, each_vertice.x, each_vertice.y, z_coords.second);
            }
            /** Add back vertices */
            for (auto each_vertice : voronoi_vertices)
            {
                faces.push_back({each_vertice.x, each_vertice.y, z_coords.first});
                writeVertices(file, each_vertice.x, each_vertice.y, z_coords.first);
            }

            faces.push_back({center.x, center.y, z_coords.second});
            faces.push_back({center.x, center.y, z_coords.first});

            writeVertices(file, center.x, center.y, z_coords.second);
            writeVertices(file, center.x, center.y, z_coords.first);
            writeNewLine(file);

            writeTexture(file);
            writeNewLine(file);

            /** Create vertex normals */
            // front face normal
            glmvec3 AB = faces[0] - faces[vertices_size * 2];
            glmvec3 AC = faces[1] - faces[vertices_size * 2];
            glmvec3 normal = glm::normalize(glm::cross(AB, AC));
            writeNormal(file, normal.x, normal.y, normal.z);

            // back face normal
            AB = faces[vertices_size + 1] - faces[vertices_size * 2 + 1];
            AC = faces[vertices_size] - faces[vertices_size * 2 + 1];
            normal = glm::normalize(glm::cross(AB, AC));
            writeNormal(file, normal.x, normal.y, normal.z);

            // side face normals
            for (int index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    AB = faces[vertices_size] - faces[index];
                    AC = faces[0] - faces[index];
                    normal = glm::normalize(glm::cross(AB, AC));
                    writeNormal(file, normal.x, normal.y, normal.z);
                }
                else
                {
                    AB = faces[index + vertices_size + 1] - faces[index];
                    AC = faces[index + 1] - faces[index];
                    normal = glm::normalize(glm::cross(AB, AC));
                    writeNormal(file, normal.x, normal.y, normal.z);
                }
            }
            writeNewLine(file);

            /**Add faces */
            /**Front Face */
            for (int index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    writeFace(file, vertices_size * 2 + 1, 1, index + 1, normal_index);
                }
                else
                {
                    writeFace(file, vertices_size * 2 + 1, index + 2, index + 1, normal_index);
                }
            }
            ++normal_index;

            /**Back Face */
            for (int index = vertices_size; index < vertices_size * 2; ++index)
            {
                if (index == vertices_size * 2 - 1)
                {
                    writeFace(file, vertices_size * 2 + 2, index + 1, vertices_size + 1, normal_index);
                }
                else
                {
                    writeFace(file, vertices_size * 2 + 2, index + 1, index + 2, normal_index);
                }
            }
            ++normal_index;

            /**Side Faces */
            for (int index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    writeFace(file, index + 1, vertices_size + 1, index + vertices_size + 1, normal_index);
                    writeFace(file, index + 1, 1, vertices_size + 1, normal_index);
                    ++normal_index;
                }
                else
                {
                    writeFace(file, index + 1, index + vertices_size + 2, index + vertices_size + 1, normal_index);
                    writeFace(file, index + 1, index + 2, index + vertices_size + 2, normal_index);
                    ++normal_index;
                }
            }

            ++polytope_index;
        }
    }
}
