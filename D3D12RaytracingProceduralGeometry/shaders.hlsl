


cbuffer MVP : register(b0)
{
    float4x4 view;
    float4x4 proj;
};
//ConstantBuffer<float4x4> mvp : register(b0);

struct PSInput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
    float4 direction : DIRECTION;
   float distance : DISTANCE;
};

struct Photon {
    float4 position;
    float4 direction;
    float4 colour;

};
RWStructuredBuffer<Photon> photons : register(u2);
RWTexture2D<float4> GBufferBRDF : register(u4);
RWTexture2D<float4> GBufferPosition : register(u5);
RWTexture2D<float4> GBufferNormal : register(u6);
//RWTexture1D<float4> photons : register(u0);


PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{
    PSInput result;
    //float4x4 mvp = proj * view;
//    Photon photon = photons[instanceID];
    //data is certainly getting in! WooHoo!
    Photon photon = photons[instanceID];
   uint decr = photons.DecrementCounter();
 


    //for some reason the raster and ray-tracer are mirrored projections - just mirror the corresponding point.
    float4 p = position - photon.position;
     p.y = -p.y;
     p.z = -p.z;
     p.x = -p.x;
    float4 c = mul(proj, p);
  

    //lookup position for coordinate .xy 
   float4 fragmentPosition = GBufferPosition[c.xy];
    float4 rel = fragmentPosition - photon.position;
    float distance = sqrt(dot(rel, rel));
    result.position = c;
    result.color = photon.colour;
    result.direction = photon.direction;
    result.distance = distance;
    //need to compress the kernel along the photon's point of intersection's normal

    return result;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    float4 normal = GBufferNormal[input.position.xy];
     float4 BRDF = GBufferBRDF[input.position.xy];
     float maxima = max(0, dot(input.direction.xyz, normal.xyz));
     float colourThroughput = dot(input.color.xyz, float3(1.0f, 1.0f, 1.0f));
     float4 color = float4(BRDF.x * input.color.x, BRDF.y * input.color.y, BRDF.z * input.color.z, 1);//divided by distance?
    float totalPower = dot(color.xyz , float3(1.0f, 1.0f, 1.0f));
    float3 weighted_direction = totalPower * input.direction.xyz;
   
    return float4(color.xyz, weighted_direction.x);
     //return float4(color.xyz, weighted_direction.x)
   // return input.position;
}