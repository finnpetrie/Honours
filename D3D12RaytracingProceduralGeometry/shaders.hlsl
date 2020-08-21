


cbuffer MVP : register(b0)
{
    float4x4 view;
    float4x4 proj;
};
//ConstantBuffer<float4x4> mvp : register(b0);

struct PSInput
{
      float4 position : SV_POSITION;
      nointerpolation float4 color : COLOR;
      nointerpolation float4 direction : DIRECTION;
    nointerpolation float4 originalPosition : ORG_POSITION;
    nointerpolation float4 normal : PHOTON_NORMAL;
    nointerpolation float majKernelRadius : KERNEL_MAJOR;
    nointerpolation float invMajKernelRadius : KERNEL_MINOR;
};

struct Photon {
    float4 position;
    float4 direction;
    float4 colour;
    float4 normal;

};

RWStructuredBuffer<Photon> photons : register(u2);
RWTexture2D<float4> GBufferBRDF : register(u4);
RWTexture2D<float4> GBufferPosition : register(u5);
RWTexture2D<float4> GBufferNormal : register(u6);
//RWTexture1D<float4> photons : register(u0);



PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{

    float lMax = 50;

    PSInput result;

    float maxMajorKernelRadius = 10;
    float minMajKernelRadius = 0.1;
    float pi = 3.1415926535897932384626422832795028841971f;


    //data is certainly getting in! WooHoo!
    Photon photon = photons[instanceID];
   uint decr = photons.DecrementCounter();
   float raySize = photon.position.w;
   float scale = min(raySize / lMax, 1);
   //scale position
   float4 scaledPos = scale * float4(position.xyz, 0);
   //squish position
   //multiplied by some scaling factor
     float4 sP = scaledPos - photon.normal * dot(scaledPos, photon.normal);
    //for some reason the raster and ray-tracer are mirrored projections - just mirror the corresponding point.

    // p.y = -p.y;
     //p.z = -p.z;
     //p.x = -p.x;

    // p *= scale;
     float cos_theta = saturate(dot(photon.normal, normalize(-photon.direction)));
     float3 u = normalize(-photon.direction) - cos_theta * photon.normal;
     float3 toU = dot(u, scaledPos)*u;
     float3 toT = scaledPos - toU;
     toT -= dot(toT, photon.normal) * photon.normal;
     float scaling = min(1.0f / cos_theta, 50);
     float3 scaled_u = toU * scaling;
    float k = sqrt(dot(-photon.direction, -photon.direction));
     
    float inv = 1.0 / k;
    //float kSquash = 1 - minMajKerne
    float pathDensity = k - 1.0;

    float majKernelRadius = lerp(maxMajorKernelRadius, minMajKernelRadius, pathDensity);
    float invMajorKernelR = 1.0 / majKernelRadius;
    //multiplied by 1.1 to avoid undersampling
    float4 p = sP * majKernelRadius*1.1;
    float4 p_i = p + photon.position + float4(scaled_u, 0) + float4(toT, 0);
    //float4 p_i = float4(p.xyz * majKernelRadius, 1);
    float4  c = mul(proj, float4(p_i.xyz, 1));
    //loat ellipse_area = pi*

    float majKernel2 = majKernelRadius * majKernelRadius;
    result.majKernelRadius = majKernel2;
    result.invMajKernelRadius = invMajorKernelR;
    result.position = c;
    //colour, i.e., power, is divided by kernel radius * pi to normalize
    result.color = photon.colour*1/ (majKernelRadius*pi);
    result.direction = photon.direction/inv;
    result.originalPosition = photon.position;
    result.normal = photon.normal;
    //need to compress the kernel along the photon's point of intersection's normal

    return result;
}

float4 PSMain(PSInput input) : SV_TARGET
{

     float pi = 3.1415926535897932384626422832795028841971f;

        float4 pos = GBufferPosition[input.position.xy];
        //max distance between photon and position
        float r = 1000;

        float4 axis = pos - input.originalPosition;

        float beta = 1.953;
        float alpha = 0.918;
        //float exp = 1 - exp()
        float distance2 = dot(axis, axis);
        float p_a = -beta * (distance2 / (2 * r * r));
        float gauss = alpha*(1 - ((1 - exp(p_a)) / (1 - exp(-beta))));
        
        
    // float4 pos = GBufferPosition[input.position.xy];
     //float dist = sqrt(dot(pos - original))
    
    float4 normal = GBufferNormal[input.position.xy];

    
    float k_L = dot(normal, -input.direction);
    //this is meant to be a means to discard any comparisons, i.e, these points are outside of the kernel's domain
    if ( (k_L < 0.0)) {
        //backface
       return float4(0.0, 0.0, 0.0, 0.0);
    }
        
    //cone filter
    float k = 1.1;
    //falloff
    float wp = 1 - sqrt(distance2) / (k * r);
    

        float4 BRDF = GBufferBRDF[input.position.xy];

        float maxima = max(0, dot(input.direction.xyz, normal.xyz));
        float colourThroughput = dot(input.color.xyz, float3(1.0f, 1.0f, 1.0f));
        float normalize = (1 - (2 / 3 * k)) * pi * r * r;
        // float4 color = BRDF * kernel
        float n_o = wp / normalize;
           //float4 color = (float4(BRDF.x * input.color.x, BRDF.y * input.color.y, BRDF.z * input.color.z, 1));// *maxima * wp) / (1 - 2 / 3 * k) * pi * r * r * r * r;//divided by distance?
        float4 color = (BRDF * input.color)*max(0, k_L);
        float totalPower = dot(color.xyz, float3(1.0f, 1.0f, 1.0f));
        float3 weighted_direction = totalPower * input.direction.xyz;
        //return input.direction;
        //return  input.originalPosition;;
       // return float4(color.xyz, weighted_direction.x); / float4(wp, wp, wp, 1);
       // return normal;
        //return float4(gauss, gauss, gauss, 1.0);
        return float4(color.xyz, weighted_direction.x);
    
     //return float4(color.xyz, weighted_direction.x)
   // return input.position;
}