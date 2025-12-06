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

    inline void writeVertices(std::ofstream &file, real x, real y, real z)
    {
        file << std::setprecision(6) << std::fixed << "v " << x << " " << y << " " << z << "\n";
    }

    inline void writeFace(std::ofstream &file, int x, int y, int z)
    {
        file << "f " << x << " " << y << " " << z << "\n";
    }

    inline void writeNewLine(std::ofstream &file)
    {
        file << "\n";
    }

    inline void writeHeader(std::ofstream &file, int &index)
    {
        file << "# polytope" + std::to_string(index) + ".obj\n";
        file << "#\n\n";
    }

    inline void writeToOBJ(std::vector<VD::Voronoi> &voronois)
    {
        int polytope_index = 0;
        fs::path parent_path = fs::current_path().parent_path().parent_path();
        fs::path path = fs::path(parent_path.string() + "\\media\\models\\test\\destruction\\polytopes\\");

        for (Voronoi &eachVoronoi : voronois)
        {
            std::string filename = path.string() + "polytope" + std::to_string(polytope_index) + ".obj";
            std::ofstream file(filename);

            if (!file.is_open())
            {
                std::cerr << "Error: Could not open the file " << filename << std::endl;
                return;
            }
            writeHeader(file, polytope_index);

            /** Fetch voronoi vertices */
            std::vector<Point> voronoi_vertices = eachVoronoi.getVertices();
            int vertices_size = int(voronoi_vertices.size());
            Point center = getAveragePoint(voronoi_vertices);

            /** Add z dimension to vertices */
            /** Add Front vertices */

            for (auto each_vertice : voronoi_vertices)
            {
                writeVertices(file, each_vertice.getX(), each_vertice.getY(), 0.25_real);
            }
            /** Add back vertices */

            for (auto each_vertice : voronoi_vertices)
            {
                writeVertices(file, each_vertice.getX(), each_vertice.getY(), -0.25_real);
            }
            writeVertices(file, center.getX(), center.getY(), 0.25_real);
            writeVertices(file, center.getX(), center.getY(), -0.25_real);
            writeNewLine(file);

            /**Add faces */
            /**Front Face */
            for (int index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    writeFace(file, vertices_size * 2 + 1, index + 1, 1);
                }
                else
                {
                    writeFace(file, vertices_size * 2 + 1, index + 1, index + 2);
                }
            }

            /**Back Face */
            for (int index = vertices_size; index < vertices_size * 2; ++index)
            {
                if (index == vertices_size * 2 - 1)
                {
                    writeFace(file, vertices_size * 2 + 2, vertices_size + 1, index + 1);
                }
                else
                {
                    writeFace(file, vertices_size * 2 + 2, index + 2, index + 1);
                }
            }

            /**Side Faces */
            for (int index = 0; index < vertices_size; ++index)
            {
                if (index == vertices_size - 1)
                {
                    writeFace(file, index + 1, index + vertices_size + 1, vertices_size + 1);
                    writeFace(file, index + 1, vertices_size + 1, 1);
                }
                else
                {
                    writeFace(file, index + 1, index + vertices_size + 1, index + vertices_size + 2);
                    writeFace(file, index + 1, index + vertices_size + 2, index + 2);
                }
            }

            ++polytope_index;
        }
    }
}
