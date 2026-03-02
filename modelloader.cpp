#include "ModelLoader.h"
#include <fstream>
#include <sstream>

bool loadOBJModel(const std::string& path, std::vector<Vertex>& out_vertices, std::vector<TextureCoord>& out_textures, std::vector<Normal>& out_normals, std::vector<Face>& out_faces) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            Vertex v;
            iss >> v.x >> v.y >> v.z;
            out_vertices.push_back(v);
        } else if (prefix == "vt") {
            TextureCoord vt;
            iss >> vt.u >> vt.v;
            out_textures.push_back(vt);
        } else if (prefix == "vn") {
            Normal vn;
            iss >> vn.nx >> vn.ny >> vn.nz;
            out_normals.push_back(vn);
        } else if (prefix == "f") {
            Face f;
            std::string vertexData;
            while (iss >> vertexData) {
                std::istringstream viss(vertexData);
                std::string vIndex;
                std::getline(viss, vIndex, '/');
                if (!vIndex.empty()) {
                    f.vertexIndices.push_back(std::stoi(vIndex));
                }
            }
            out_faces.push_back(f);
        }
    }
    return true;
}