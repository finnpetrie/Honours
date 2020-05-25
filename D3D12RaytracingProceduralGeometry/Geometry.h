#pragma once
#include "stdafx.h"
#include "DirectXRaytracingHelper.h"
class Geometry
{
    D3DBuffer m_indexBuffer;
    D3DBuffer m_vertexBuffer;
    D3DBuffer m_aabbBuffer;
   // std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
};

