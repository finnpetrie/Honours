


cbuffer MVP : register(b0)
{
    float4x4 mvp;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
   // float4 Direction : DIRECTION;
};


RWTexture1D<float4> photons : register(u0);


PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{
    PSInput result;
    float4 photon = photons[instanceID];

    //transform via model view;
    float4 pos = mul(mvp, photon.xyz);
    result.position = mul(mvp, position);
   // result.position = position;
    result.color = color;

    return result;
}

[earlydepthstencil]
float4 PSMain(PSInput input) : SV_TARGET
{
   return input.color;
}