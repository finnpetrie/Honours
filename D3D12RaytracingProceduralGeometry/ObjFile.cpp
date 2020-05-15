#include "stdafx.h"
#include "ObjFile.h"
#include <iostream>
#include <unordered_map>



ObjFile::ObjFile(std::string path) : MODEL_PATH(path)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn, err;
	//std::unordered_map<Vertex, uint32_t> uniqueVertices = {};
	if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, MODEL_PATH.c_str())) {
		throw std::runtime_error(warn + err);
	}

	for (const auto& shape : shapes) {
		for (const auto& index : shape.mesh.indices) {
			size_t i = (size_t)3 * index.vertex_index;
			XMFLOAT3 vertex = XMFLOAT3(attrib.vertices[i + 0], attrib.vertices[i + 1], attrib.vertices[i + 2]);
			XMFLOAT3 normal = XMFLOAT3(attrib.normals[i + 0], attrib.normals[i + 1], attrib.normals[i + 2]);
			Vertex v = { vertex, normal };
			vertices.push_back(v);
			indices.push_back(indices.size());

		}
	}
}
