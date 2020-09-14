RWTexture2D<uint> stagingTarget_R : register(u8);
RWTexture2D<uint> stagingTarget_G : register(u9);
RWTexture2D<uint> stagingTarget_B : register(u10);

RWTexture2D<float4> lightTracingPhotons[MAX_RAY_RECURSION_DEPTH * 4]: register(u13);


[numthreads(1, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
}