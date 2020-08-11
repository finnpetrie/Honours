#ifndef TILING_HLSL
#define TILING_HLSL

#define HLSL
#include "RayTracingHlslCompat.h"
RWTexture2D<float4> screenSpacePhotonMap[5]: register(u1);
RWTexture2D<float4> photonBucket: register(u7);
ConstantBuffer<SceneConstantBuffer> kernelParams : register(b0);

[numthreads(10, 10, 6)]
void main( uint3 DTid : SV_DispatchThreadID )
{
}

#endif