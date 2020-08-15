#ifndef TILING_HLSL
#define TILING_HLSL

struct ComputeConstantBuffer {
	float3 pos;
	float3 dir;
	float3 up;
	float4x4 reprojection;
};

struct Photon {
	float4 pos;
	float4 direction;
	float4 colour;
};
//#define HLSL
//#include "RayTracingHlslCompat.h"
#define MAX_PHOTONS 10000
RWTexture2D<float4> g_renderTarget : register(u0);
//note to self - do the register addresses need to be the same as in our other shader?
RWStructuredBuffer<Photon> photonBuffer : register(u1);

RWTexture2D<uint> photonBucket: register(u2);
//RWTexture1D<float3> photonPosition : register(u8);
//RWTexture1D<float3> photonColour : register(u9);
//RWTexture1D<float3> photonDirection : register(u10);
//RWTexture1D<float> photonCount : register(u11);

groupshared uint tilePhotonCount;
groupshared uint photonBufferIndex;
//groupshared uint tilePhotonIndices[16000];

ConstantBuffer<ComputeConstantBuffer> computeInfo : register(b0);


float3 CreatePlane(in float4 corner, in float4 nextCorner) {
	float3 c1 = corner.xyz - computeInfo.pos;
	float3 c2 = nextCorner.xyz - computeInfo.pos;
	float3 normal = cross(c1, c2);

	return normal;

}



uint classifyIndex(uint3 DTid) {
	uint x = DTid.x;
	uint y = DTid.y;
	uint width;
	uint height;
	g_renderTarget.GetDimensions(width, height);
	//screenSpacePhotonMap[0].GetDimensions(width, height);
	uint cell = 0;

	for (int dx = width / 10; dx < width; dx += width / 10) {
		for (int dy = height / 10; dy < height; dy += height / 10) {
			if ((x <= dx) && (y <= dy)){
				return cell;
			}
			cell++;
		}
	}

	return cell;
}

bool SphereIntersectsFrustrum(in float3 photon, in float radius, in float4 frustrum[4]) {
	//photon = centre of sphere
	//signed distance to plane
	for (int i = 0; i < 4; i++) {
		float distance = dot(photon, frustrum[i].xyz) / sqrt(dot(frustrum[i], frustrum[i]));
		if (distance <= radius) {
			return true;
		}
	}
	return false;
}

//the purpose of this compute shader is to take in the photon buffer, and its contents into tile buckets.

[numthreads(128, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	float width, height;
	//screenSpacePhotonMap[0].GetDimensions(width, height);
	//screenSpacePhotonMap[0][DTid.xy] = float4(DTid.xy, DTid.x, 0);

	uint index = classifyIndex(DTid);
	//classify index
	uint MAX_WORK_GROUP_SIZE = 16 * 16;
	uint photonCount = 0;
	uint minX = MAX_WORK_GROUP_SIZE * DTid.x;
	uint minY = MAX_WORK_GROUP_SIZE * DTid.y;
	uint maxX = MAX_WORK_GROUP_SIZE * (DTid.x + 1);
	uint maxY = MAX_WORK_GROUP_SIZE * (DTid.y + 1);

	//create tiled frustrum
	//top left
	float4 tiledCorners[4];
	tiledCorners[0] = mul(float4(float(minX) / width * 2.0f - 1.0f, (float(minY) / height) * 2.0 - 1.0f, 1.0f, 1.0f), computeInfo.reprojection);
	tiledCorners[1] = mul(float4(float(maxX) / width * 2.0f - 1.0f, (float(minY) / height) * 2.0 - 1.0f, 1.0f, 1.0f), computeInfo.reprojection);
	tiledCorners[2] = mul(float4(float(maxX) / width * 2.0f - 1.0f, float(maxY) / height * 2.0 - 1.0f, 1.0f, 1.0f), computeInfo.reprojection);
	tiledCorners[3] = mul(float4(float(minX) / width * 2.0f - 1.0f, float(maxY) / height * 2.0f - 1.0f, 1.0f, 1.0f), computeInfo.reprojection);
	
	float4 frustrumEquation[4];



	for (int i = 0; i < 4; i++) {
		frustrumEquation[i] = float4(CreatePlane(tiledCorners[i], tiledCorners[i + 1]), 0);
	}
	//GroupMemoryBarrierWithSync();
	//project to world space using the inverse of the view matrix.  
	//Create the frustum planes by using the cross product between these points

	//GroupMemoryBarrierWithSync();

	uint photonBufferWidth;
	//photonPosition.GetDimensions(photonBufferWidth);
	for (uint j = 0; j < photonBufferWidth; j++) {
		Photon p = photonBuffer[j];
		if (SphereIntersectsFrustrum(p.pos.xyz, 1, frustrumEquation)) {
			uint photonIndex = 0;
			InterlockedAdd(photonBufferIndex, 1, photonIndex);
			if (photonIndex < MAX_PHOTONS) {
				//add photon j to corresponding photon bucket
				photonBucket[float2(index, photonIndex)] = j;
			}
		}
	}

}

#endif