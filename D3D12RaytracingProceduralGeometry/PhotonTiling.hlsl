#ifndef TILING_HLSL
#define TILING_HLSL

struct ComputeConstantBuffer {
	float3 pos;
	float3 dir;
	float3 up;
	float4x4 reprojection;
};
//#define HLSL
//#include "RayTracingHlslCompat.h"
RWTexture2D<float4> screenSpacePhotonMap[5]: register(u1);
RWTexture2D<float2> photonBucket: register(u7);
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
	screenSpacePhotonMap[0].GetDimensions(width, height);
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


[numthreads(1, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	float width, height;
	screenSpacePhotonMap[0].GetDimensions(width, height);

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
	//project to world space using the inverse of the view matrix.  
	//Create the frustum planes by using the cross product between these points

	uint photonIndex = 0;
	screenSpacePhotonMap[3][float2(0,1)] = float4(1, 1, 0, 0);
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			//retrieve photon at i,j.
			float3 photon = screenSpacePhotonMap[0][float2(i, j)].xyz;
			//assume constant radius for now
			if (SphereIntersectsFrustrum(photon, 1, frustrumEquation)) {
				//store the corresponding index in the texture.
				if (photonIndex < 700) {
					float2 src = float2(0, 0);
					//InterlockedAdd(photonBucket[float2(index, photonIndex)], float2(i, j), old);
					//uint2 i = uint2(index, photonIndex);
					//photonIndex++;
					//probably best to do this with an interlocked add.
					//screenSpacePhotonMap[3][float2(0, 1)] = float4(photon, 0);
				}
				//photonIndex += 1;

			}
		}
	}
}

#endif