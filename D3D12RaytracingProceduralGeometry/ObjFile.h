#pragma once
#include <tiny_obj_loader.h>
#include <DirectXMath.h>
#include "stdafx.h"
#include "RaytracingHlslCompat.h"

#define TINYOBJLOADER_IMPLEMENTATION

using namespace DirectX;
class ObjFile
{
public:
	std::string MODEL_PATH;
	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
	
	ObjFile(std::string path);
};

