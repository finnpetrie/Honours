RWTexture2D<float4> g_renderTarget : register(u0);
RWTexture2D<float4> g_indirectIllumination : register(u1);

[numthreads(1, 1, 1)]

//want to take in two render targets, add them together, and return
void main( uint3 DTid : SV_DispatchThreadID )
{
	//float4 rayT = g_renderTarget[DTid.xy];
	//float4 raster = g_indirectIllumination[DTid.xy];
	//float4 output = rayT + raster;
	//g_renderTarget[DTid.xy] = output;
}