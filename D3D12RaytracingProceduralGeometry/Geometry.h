#pragma once
#include "stdafx.h"
#include "DirectXRaytracingHelper.h"
class Geometry
{
    D3DBuffer m_indexBuffer;
    D3DBuffer m_geometryBuffer;


    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
};

