


cbuffer MVP : register(b0)
{
    float4x4 mvp;
};
//ConstantBuffer<float4x4> mvp : register(b0);

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
RWStructuredBuffer<Photon> photons : register(u2);
//RWTexture1D<float4> photons : register(u0);


PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{
    PSInput result;
    float4x4 view = float4x4(1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
//    Photon photon = photons[instanceID];
    //data is certainly getting in! WooHoo!
    Photon photon = photons[instanceID];
    //result.position = mul(view, photon.position);
    //transform via model view;
    //float4 pos = mul(mvp, photon.position);
   //result.position = pos;
   // result.position = mul(mvp, position);
   // result.position = position;
   // result.color = photon.colour;
   // result.color = float4(1, 1, 1, 0);
    float4 p = position - photon.position;
    float4 c = mul(mvp, p);
    //result.position = position - photon.position;
    result.position = c;
    result.color = photon.position;

    return result;
}

float4 PSMain(PSInput input) : SV_TARGET
{
   return input.color;
}