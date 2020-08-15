


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

struct Photon {
    float4 position;
    float4 direction;
    float4 colour;

};
RWStructuredBuffer<Photon> photons : register(u1);
//RWTexture1D<float4> photons : register(u0);


PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{
    PSInput result;
    float4x4 view = float4x4(1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
//    Photon photon = photons[instanceID];
    Photon photon = photons[instanceID];
    result.position = mul(view, photon.position);
    //transform via model view;
    //float4 pos = mul(mvp, photon.position);
   //result.position = pos;
   // result.position = mul(mvp, position);
   // result.position = position;
    result.color = photon.colour;
   // result.color = float4(1, 1, 1, 0);

    return result;
}

[earlydepthstencil]
float4 PSMain(PSInput input) : SV_TARGET
{
   return input.color;
}