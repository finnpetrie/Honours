#include "stdafx.h"
#include "Geometry.h"

Geometry::Geometry()
{
}


void Geometry::updateIndicesOffset(uint32_t offset) {
    for (int i = 0; i < indices.size(); i++) {
        indices[i] += offset;
    }
}

std::vector<Vertex> Geometry::getVertices() {
    return vertices;
}

std::vector<uint32_t> Geometry::getIndices() {
    return indices;
}


void Geometry::initPlane()
{
    // Plane indices.
    indices =
    {
        3,1,0,
        2,1,3,

    };

    // Cube vertices positions and corresponding triangle normals.
    vertices = 
    {
        { XMFLOAT3(0.0f, -5.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(1.0f, -5.0f, 0.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(1.0f, -5.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(0.0f, -5.0f, 1.0f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
    };
}

void Geometry::LoadModel(std::string filepath)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err, warn;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filepath.c_str()))
    {
        throw std::runtime_error(err);
    }
    uint32_t i = 0;
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            Vertex v{};
            v.position = {
                attrib.vertices[3 * (float)index.vertex_index + 2],
                attrib.vertices[3 * (float)index.vertex_index + 1],
                attrib.vertices[3 * (float)index.vertex_index + 0]
            };
            v.normal = {
                attrib.normals[3 * (float)index.normal_index + 2],
                attrib.normals[3 * (float)index.normal_index + 1],
                attrib.normals[3 * (float)index.normal_index + 0]
            };

            vertices.push_back(v);
            indices.push_back(i);
            i++;
        }
    }
}
