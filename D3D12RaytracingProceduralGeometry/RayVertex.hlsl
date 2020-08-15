#ifndef RAY_VERTEX
#define RAY_VERTEX
#define HLSL
#include "Rasterisation.hlsli"


struct Photon {
	float4 pos;
	float4 direction;
	float4 colour;
};

cbuffer mvp : register(b0)
{
	float4x4 mvp;
};

RWStructuredBuffer<Photon> photonBuffer : register(u0);
PSInput main(float4 pos : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{

	PSInput result;
	float4 projectedPos = mul(mvp, pos);
	result.position = projectedPos;
	result.color = color;
	return result;
}

#endif