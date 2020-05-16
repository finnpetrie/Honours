#include "stdafx.h"
#include "ObjFile.h"
#include <iostream>
#include <unordered_map>
#define TINYOBJLOADER_IMPLEMENTATION



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
			Vertex vertex{};
			vertex.position = {
			attrib.vertices[3 * index.vertex_index + 2],
			attrib.vertices[3 * index.vertex_index + 1],
			attrib.vertices[3 * index.vertex_index + 0]
			};
			vertex.normal = {
				attrib.normals[3 * index.normal_index + 2],
				attrib.normals[3 * index.normal_index + 1],
				attrib.normals[3 * index.normal_index + 0]
			};
			//XMFLOAT3 vertex = XMFLOAT3(attrib.vertices[3*index.vertex_index + 0], attrib.vertices[3*index.vertex_index + 1], attrib.vertices[3*index.vertex_index + 2]);
			
			//Vertex v = { vertex, normal };
			vertices.push_back(vertex);
			indices.push_back(indices.size());

		}
	}
}
