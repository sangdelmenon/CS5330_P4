#pragma once
#include <vector>
#include <string>

struct Vertex { float x, y, z; };
struct TextureCoord { float u, v; };
struct Normal { float nx, ny, nz; };
struct Face { std::vector<int> vertexIndices; };

bool loadOBJModel(const std::string& path, std::vector<Vertex>& out_vertices, std::vector<TextureCoord>& out_textures, std::vector<Normal>& out_normals, std::vector<Face>& out_faces);