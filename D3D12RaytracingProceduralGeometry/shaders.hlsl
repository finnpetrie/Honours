


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
    nointerpolation float majKernelRadiusSquared : KERNEL_MAJOR;
    nointerpolation float invMajKernelRadius : KERNEL_MINOR;
    nointerpolation float kernelMinor : SMALL_RADIUS;
    nointerpolation float u_radius : U_AXIS;
    nointerpolation float t_radius : T_AXIS;
    nointerpolation float3x3 eInverse : ELLIPSOID_INVERSE;

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
RWTexture2D<float4> rasterTarget : register(u8);
//RWTexture1D<float4> photons : register(u0);



PSInput VSMain(float4 position : POSITION, uint instanceID : SV_InstanceID, float4 color : COLOR)
{
        float lMax = 100;
        float maxMajorKernelRadius = 20;
        float minMajKernelRadius = 0.01;
        float pi = 3.1415926535897932384626422832795028841971f;

        PSInput result;
        
        Photon photon = photons[instanceID];
        uint decr = photons.DecrementCounter();
        //photons[instanceID] = { float4(0,0,0,0), float4(0,0,0,0), float4(0,0,0,0), float4(0,0,0,0) };
        float raySize = photon.position.w;
        float scale = min(raySize / lMax, 1);
       //scale position
       //squish position
       //multiplied by some scaling factor
        float radius = minMajKernelRadius + (maxMajorKernelRadius - minMajKernelRadius) * sqrt(1 - photon.normal.w);
        float4 scaledPos = scale * float4(position.xyz, 0);

       float k = sqrt(dot(-photon.direction, -photon.direction));

       float inv = 1.0 / k;
       //float kSquash = 1 - minMajKerne
       float pathDensity = k - 1.0;
        
       float r = minMajKernelRadius + (maxMajorKernelRadius - minMajKernelRadius) * sqrt(1 - pathDensity);
       float majKernelRadius = lerp(maxMajorKernelRadius, minMajKernelRadius, pathDensity);
       majKernelRadius = r;
       float kSquash = 1 - length(photon.normal) / majKernelRadius;
        kSquash = 1;

        float4 sP = scaledPos - photon.normal * dot(scaledPos, photon.normal)*kSquash;
    //for some reason the raster and ray-tracer are mirrored projections - just mirror the corresponding point.

        //exapnd in directions u and t on the tanget plane (projects the photon in its direction, shearing)
        float cos_theta = saturate(dot(photon.normal, normalize(-photon.direction)));
        float3 u = normalize(-photon.direction) - cos_theta * photon.normal;
        float3 toU = dot(u, scaledPos)*u;
        float3 toT = scaledPos - toU;
        toT -= dot(toT, photon.normal) * photon.normal;
        float scaling = min(1.0f / cos_theta, lMax);
        float3 scaled_u = toU * scaling;

        float invMajorKernelR = 1.0 / majKernelRadius;

        float oversize = 1.1;
        //multiplied by 1.1 to avoid undersampling
        float4 p = sP * majKernelRadius;
        float4 p_i = p + photon.position;//float4(scaled_u, 0) + float4(toT, 0);
        //float4 p_i = float4(p.xyz * majKernelRadius, 1);
        float4  c = mul(proj, float4(p_i.xyz, 1));
        //loat ellipse_area = pi*
      //  float ellipse_area = pi * length(scaled_u) * length(toT);
        float majKernel2 = majKernelRadius * majKernelRadius;

        float4 projectedPos = mul(proj, float4(photon.position.xyz, 1));
    
        float3 uN = normalize(u);
        float3 pN = normalize(photon.normal.xyz);
        float3 tN = normalize(toT);

        float3x3 ellipsoidBasis = float3x3(uN.x, pN.x, tN.x,uN.y, pN.y, tN.y,
                                            uN.z, pN.z, tN.z);


        //we know the ellipsoid basis is orthnormal (at least we can make it orthonormal), therefore its tranpose is its inverse
        float3x3 eInverse = transpose(ellipsoidBasis);
        
        result.majKernelRadiusSquared = majKernel2;
        result.invMajKernelRadius = invMajorKernelR;
        result.position = c;
        result.kernelMinor = radius;
        //colour, i.e., power, is divided by kernel radius * pi to normalize
        //UNSURE ABOUT THIS - HOW DO WE COMPUTE THE COLOUR -
        result.color = photon.colour;
        result.direction = photon.direction;
        result.originalPosition = photon.position;
        result.normal = photon.normal;
        result.u_radius = length(scaled_u) *majKernelRadius * oversize;
        result.t_radius = length(toT) * majKernelRadius * oversize;
        result.eInverse = ellipsoidBasis;
        return result;

}

float4 PSMain(PSInput input) : SV_TARGET
{
 
     float pi = 3.1415926535897932384626422832795028841971f;

     float4 pos = GBufferPosition[input.position.xy];
     //max distance between photon and position
     float3x3 eInverse = input.eInverse;

     float4 axis = pos - input.originalPosition;
     //map axis to the standard basis
     float3 axisE = mul(eInverse, axis.xyz);

     float beta = 1.953;
     float alpha = 0.918;
     //float exp = 1 - exp()
     float distance2 = dot(axis.xyz, axis.xyz);
    // float relativeDistance = sqrt(distance2 * dot(axis.xyz, input.normal.xyz) * dot(axis.xyz, input.normal.xyz));
     float probability = input.normal.w;
     float r_1 = input.u_radius;
     float r_3 = input.t_radius;
     float r_2 = 1;

     float r = 0.8;
     // float r = input.kernelMinor + (sqrt(input.majKernelRadius) - )
     //float r = input.majKernelRadiusSquared;


     float p_a = -beta * (distance2 / (2 * r * r));
     float exponentCoeff = -((axisE.x * axisE.x) / (2 * r_1 * r_1) + (axisE.y * axisE.y) / (2 *r_2 * r_2) + (axisE.z * axisE.z) / (2 * r_3 * r_3));
     
     float expOther = -((axis.x * axis.x) / (2 * r_1 * r_1) + (axis.y * axis.y) / (2 * r_2 * r_2) + (axis.z * axis.z) / (2 * r * r));
     float symExponent = -(distance2) / (2 * r * r);
     // float gaussianExponent = exp(-())
    // float gauss = 1/(2*pi)*r
     float gauss = 1 / (r_1*r_2*r_3* sqrt(2 * pi)) * exp(exponentCoeff);
     float symmetricGauss = 1 / (r * r * sqrt(2 * pi)) * exp(symExponent);
     float JensenGauss = alpha*(1 - ((1 - exp(p_a)) / (1 - exp(-p_a))));

     // float4 pos = GBufferPosition[input.position.xy];
      //float dist = sqrt(dot(pos - original)) 

     float4 normal = GBufferNormal[input.position.xy];


     float k_L = dot(normal, -input.direction);
     //this is meant to be a means to discard any comparisons, i.e, these points are outside of the kernel's domain
     if ( (k_L < 0.0) || (distance2 >= input.majKernelRadiusSquared)) {
         //backface
         discard;
     }

     //cone filter
     float k = 0.8;
     //falloff
     float wp = 1 - sqrt(distance2) / (k * r);
     //wp = min(1 / 5, wp);

     float4 BRDF = GBufferBRDF[input.position.xy];

     float nor = (1 - (2 / 3 * k))*pi*r*r;
     // float4 color = BRDF * kernel
     float n_o = wp / nor;
         //float4 color = (float4(BRDF.x * input.color.x, BRDF.y * input.color.y, BRDF.z * input.color.z, 1));// *maxima * wp) / (1 - 2 / 3 * k) * pi * r * r * r * r;//divided by distance?
     float4 color = (BRDF*input.color) * max(0, k_L)* n_o;
     float totalPower = dot(color.xyz, float3(1.0f, 1.0f, 1.0f));
     float3 weighted_direction = totalPower * input.direction.xyz;
    
  

    // rasterTarget[input.position.xy] += color;
    // return  input.color;
     // return  float4(distance2, distance2, distance2, 0);
     return 1.5*color;
     // return 22*float4(n_o, n_o, n_o, n_o);//float4(gauss, gauss, gauss, gauss);//float4(gauss, gauss, gauss, gauss);
      //return float4(color.xyz, weighted_direction.x)
    // return input.position;*/

}