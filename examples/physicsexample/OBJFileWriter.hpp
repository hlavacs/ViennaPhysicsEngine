#pragma once

#include <iostream>
#include <fstream>
#include <array>
#include <string>
#include <filesystem>

#include "VPE.hpp"
namespace fs = std::filesystem;

inline void writeToOBJ(std::vector<vpe::VPEWorld::Polytope> polytopes)
{
    std::size_t polytope_index = 0;
    fs::path parent_path = fs::current_path().parent_path().parent_path();
    fs::path path = fs::path(parent_path.string() + "\\media\\models\\test\\destruction\\");

    if (fs::exists(path))
    {
        fs::remove_all(path);
    }
    fs::create_directories(path);
    if (!fs::exists(path))
    {
        std::cerr << "Failed to create directory: " << path << std::endl;
        return;
    }

    for (vpe::VPEWorld::Polytope &each : polytopes)
    {
        std::string filename = path.string() + "polytope" + std::to_string(polytope_index) + ".obj";
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open the file " << filename << std::endl;
            return;
        }

        file << "# polytope" + std::to_string(polytope_index) + ".obj\n";
        file << "#\n\n";

        // file << "mtllib polytope.mtl\n";
        // file << "o polytope\n";
        // file << "usemtl polytope\n\n";

        for (const auto &vertex : each.m_vertices)
        {
            file << "v " << vertex.m_positionL.x << " " << vertex.m_positionL.y << " " << vertex.m_positionL.z << "\n";
        }

        file << "\n";

        file << "vt 0.000000 0.000000\n";
        file << "vt 1.000000 0.000000\n";
        file << "vt 0.000000 1.000000\n";
        file << "vt 1.000000 1.000000\n\n";

        file << "vn 0.000000 0.000000 1.000000\n";
        file << "vn 0.000000 1.000000 0.000000\n";
        file << "vn 0.000000 0.000000 -1.000000\n";
        file << "vn 0.000000 -1.000000 0.000000\n";
        file << "vn 1.000000 0.000000 0.000000\n";
        file << "vn -1.000000 0.000000 0.000000\n\n";

        for (const auto &face : each.m_faces)
        {
            file << "f ";
            for (const auto &vertex : face.m_face_vertex_ptrs)
            {
                file << vertex->m_id + 1 << "/" << std::to_string(polytope_index % 4 + 1) << "/" << std::to_string(polytope_index % 4 + 1) << " ";
            }
            file << "\n";
        }
        ++polytope_index;
        file.close();
    }
}