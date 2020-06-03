#ifndef RAY_VERTEX
#define RAY_VERTEX
#define HLSL
#include "Rasterisation.hlsli"


cbuffer mvp : register(b0)
{
	float4x4 mvp;
};


PSInput main(float4 pos : POSITION, float4 color : COLOR)
{

	PSInput result;
	float4 projectedPos = mul(mvp, pos);
	result.position = projectedPos;
	result.color = color;
	return result;
}

#endif