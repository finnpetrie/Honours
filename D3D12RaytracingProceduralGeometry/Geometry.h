#pragma once
#include "stdafx.h"
#include "DirectXRaytracingHelper.h"
#include "RaytracingHlslCompat.h"
#define TINY_OBJ_IMPLEMENTATION
#include "tiny_obj_loader.h"
class Geometry
{

private:
    D3DBuffer m_indexBuffer;
    D3DBuffer m_geometryBuffer;


    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

public:

    Geometry();

    void updateIndicesOffset(uint32_t offset);

    std::vector<Vertex> getVertices();

    std::vector<uint32_t> getIndices();

    void initPlane();
    void LoadModel(std::string filepath);

};

