
//**********************************************************************************************
//
// SignedDistanceFieldLibrary.h
//
// A list of useful distance function to simple primitives, and an example on how to 
// do some interesting boolean operations, repetition and displacement.
// More info here: http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
//
//**********************************************************************************************

#ifndef SIGNEDDISTANCEPRIMITIVES_H
#define SIGNEDDISTANCEPRIMITIVES_H


#include "RaytracingShaderHelper.hlsli"

//------------------------------------------------------------------
float GetDistanceFromSignedDistancePrimitive(in float3 position, in SignedDistancePrimitive::Enum sdPrimitive);

//------------------------------------------------------------------

// Subtract: Obj1 - Obj2
float opS(float d1, float d2)
{
    return max(d1, -d2);
}

// Union: Obj1 + Obj2
float opU(float d1, float d2)
{
    return min(d1, d2);
}

// Intersection: Obj1 & Obj2
float opI(float d1, float d2)
{
    return max(d1, d2);
}

// Repetitions
float3 opRep(float3 p, float3 c)
{
    return fmod(p, c) - 0.5 * c;
} 

// Polynomial smooth min/union (k = 0.1)
// Ref: http://www.iquilezles.org/www/articles/smin/smin.htm
float smin(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5*(b - a) / k, 0.0, 1.0);
    return lerp(b, a, h) - k * h*(1.0 - h);
}


// Polynomial smooth min/union (k = 0.1)
float smax(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5*(b - a) / k, 0.0, 1.0);
    return lerp(a, b, h) + k * h*(1.0 - h);
}

// Smooth blend as union 
float opBlendU(float d1, float d2)
{
    return smin(d1, d2, 0.1);
}

// Smooth blend as intersect 
float opBlendI(float d1, float d2)
{
    return smax(d1, d2, 0.1);
}


// Twist
float3 opTwist(float3 p)
{
    float c = cos(3.0 * p.y);
    float s = sin(3.0 * p.y);
    float2x2 m = float2x2(c, -s, s, c);
    return float3(mul(m, p.xz), p.y);
}


//------------------------------------------------------------------

float sdPlane(float3 p)
{
    return p.y;
}

float sdSphere(float3 p, float s)
{
    return length(p) - s;
}

// Box extents: <-b,b>
float sdBox(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    return min(max(d.x, max(d.y, d.z)), 0.0) + length(max(d, 0.0));
}

float sdEllipsoid(in float3 p, in float3 r)
{
    return (length(p / r) - 1.0) * min(min(r.x, r.y), r.z);
}

float udRoundBox(float3 p, float3 b, float r)
{
    return length(max(abs(p) - b, 0.0)) - r;
}

// t: {radius, tube radius}
float sdTorus(float3 p, float2 t)
{
    float2 q = float2(length(p.xz) - t.x, p.y);
    return length(q) - t.y;
}



float sdHexPrism(float3 p, float2 h)
{
    float3 q = abs(p);
    float d1 = q.z - h.y;
    float d2 = max((q.x * 0.866025 + q.y * 0.5), q.y) - h.x;
    return length(max(float2(d1, d2), 0.0)) + min(max(d1, d2), 0.);
}

float sdCapsule(float3 p, float3 a, float3 b, float r)
{
    float3 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h) - r;
}

float sdEquilateralTriangle(in float2 p)
{
    const float k = 1.73205;    //sqrt(3.0);
    p.x = abs(p.x) - 1.0;
    p.y = p.y + 1.0 / k;
    if (p.x + k * p.y > 0.0) p = float2(p.x - k * p.y, -k * p.x - p.y) / 2.0;
    p.x += 2.0 - 2.0 * clamp((p.x + 2.0) / 2.0, 0.0, 1.0);
    return -length(p) * sign(p.y);
}

float sdTriPrism(float3 p, float2 h)
{
    float3 q = abs(p);
    float d1 = q.z - h.y;
#if 1
    // distance bound
    float d2 = max(q.x * 0.866025 + p.y * 0.5, -p.y) - h.x * 0.5;
#else
    // correct distance
    h.x *= 0.866025;
    float d2 = sdEquilateralTriangle(p.xy / h.x) * h.x;
#endif
    return length(max(float2(d1, d2), 0.0)) + min(max(d1, d2), 0.);
}

float sdCylinder(float3 p, float2 h)
{
    float2 d = abs(float2(length(p.xz), p.y)) - h;
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

float sdCone(in float3 p, in float3 c)
{
    float2 q = float2(length(p.xz), p.y);
    float d1 = -q.y - c.z;
    float d2 = max(dot(q, c.xy), q.y);
    return length(max(float2(d1, d2), 0.0)) + min(max(d1, d2), 0.);
}

float sdConeSection(in float3 p, in float h, in float r1, in float r2)
{
    float d1 = -p.y - h;
    float q = p.y - h;
    float si = 0.5 * (r1 - r2) / h;
    float d2 = max(sqrt(dot(p.xz, p.xz) * (1.0 - si * si)) + q * si - r2, q);
    return length(max(float2(d1, d2), 0.0)) + min(max(d1, d2), 0.);
}


// h = { sin a, cos a, height of a pyramid }
// a = pyramid's inner angle between its side plane and a ground plane.
// Octahedron position - ground plane intersecting in the middle.
float sdOctahedron(float3 p, float3 h)
{
    float d = 0.0;

    // Get distance against pyramid's sides going through origin.
    // Test: d = p.x * sin a + p.y * cos a
    d = dot(float2(max(abs(p.x), abs(p.z)), abs(p.y)), 
            float2(h.x, h.y));

    // Subtract distance to a side when at height h.z from the origin.
    return d - h.y * h.z;
}

// h = { sin a, cos a, height of a pyramid}
// a = pyramid's inner angle between its side plane and a ground plane.
// Pyramid position - sitting on a ground plane.
float sdPyramid(float3 p, float3 h) // h = { sin a, cos a, height }
{
    float octa = sdOctahedron(p, h);

    // Subtract bottom half
    return opS(octa, p.y);
}


float length_toPowNegative6(float2 p)
{
    p = p * p * p; 
    p = p * p;
    return pow(p.x + p.y, 1.0 / 6.0);
}

float length_toPowNegative8(float2 p)
{
    p = p * p; p = p * p; p = p * p;
    return pow(p.x + p.y, 1.0 / 8.0);
}

float sdTorus82(float3 p, float2 t)
{
    float2 q = float2(length(p.xz) - t.x, p.y);
    return length_toPowNegative8(q) - t.y;
}

float sdTorus88(float3 p, float2 t)
{
    float2 q = float2(length_toPowNegative8(p.xz) - t.x, p.y);
    return length_toPowNegative8(q) - t.y;
}

float sdCylinder6(float3 p, float2 h)
{
    return max(length_toPowNegative6(p.xz) - h.x, abs(p.y) - h.y);
}

float3 sdCalculateNormal(in float3 pos, in SignedDistancePrimitive::Enum sdPrimitive)
{
    float2 e = float2(1.0, -1.0) * 0.5773 * 0.0001;
    return normalize(
        e.xyy * GetDistanceFromSignedDistancePrimitive(pos + e.xyy, sdPrimitive) +
        e.yyx * GetDistanceFromSignedDistancePrimitive(pos + e.yyx, sdPrimitive) +
        e.yxy * GetDistanceFromSignedDistancePrimitive(pos + e.yxy, sdPrimitive) +
        e.xxx * GetDistanceFromSignedDistancePrimitive(pos + e.xxx, sdPrimitive));
}

/*float intersectQuaternionJuliaSet(in Ray ray, out float4 res, in float4 c) {
    float4 tmp;
    float resT = -1.0;
    float maxd = 10.0;
    float h = 1.0;
    float t = 0.0;
    for (uint i = 0; i < 300; i++) {
        if (h <0.0001 || t >maxd)break;
        //map quaternion julia set
        t += h;
    }
    
}*/

float4 qsqr(in float4 a) // square a quaterion
{
    return float4(a.x * a.x - a.y * a.y - a.z * a.z - a.w * a.w,
        2.0 * a.x * a.y,
        2.0 * a.x * a.z,
        2.0 * a.x * a.w);
}

float qlength2(in float4 q)
{
    return dot(q, q);
}

float4 qconj(in float4 a)
{
    return float4(a.x, -a.yzw);
}

float map(in float3 p, out float4 oTrap, in float4 c)
{
    float4 z = float4(p, 0.0);
    float md2 = 1.0;
    float mz2 = dot(z, z);

    float4 trap = float4(abs(z.xyz), dot(z, z));

    float n = 1.0;
    int numIterations = 11;
    for (int i = 0; i < numIterations; i++)
    {
        // dz -> 2·z·dz, meaning |dz| -> 2·|z|·|dz|
        // Now we take the 2.0 out of the loop and do it at the end with an exp2
        md2 *= 4.0 * mz2;
        // z  -> z^2 + c
        z = qsqr(z) + c;

        trap = min(trap, float4(abs(z.xyz), dot(z, z)));

        mz2 = qlength2(z);
        if (mz2 > 4.0) break;
        n += 1.0;
    }

    oTrap = trap;

    return 0.25 * sqrt(mz2 / md2) * log(mz2);  // d = 0.5·|z|·log|z|/|z'|
}
float3 calcNormal(in float3 p, in float4 c)
{
    float4 z = float4(p, 0.0);

    // identity derivative
    float4 J0 = float4(1, 0, 0, 0);
    float4 J1 = float4(0, 1, 0, 0);
    float4 J2 = float4(0, 0, 1, 0);
    uint numIterations = 11;
    for (int i = 0; i < numIterations; i++)
    {
        float4 cz = qconj(z);

        // chain rule of jacobians (removed the 2 factor)
        J0 = float4(dot(J0, cz), dot(J0.xy, z.yx), dot(J0.xz, z.zx), dot(J0.xw, z.wx));
        J1 = float4(dot(J1, cz), dot(J1.xy, z.yx), dot(J1.xz, z.zx), dot(J1.xw, z.wx));
        J2 = float4(dot(J2, cz), dot(J2.xy, z.yx), dot(J2.xz, z.zx), dot(J2.xw, z.wx));

        // z -> z2 + c
        z = qsqr(z) + c;

        if (qlength2(z) > 4.0) break;
    }

    float3 v = float3(dot(J0, z),
        dot(J1, z),
        dot(J2, z));

    return normalize(v);
}

float intersect(Ray r, out float4 res, in float4 c) {
    float4 tmp;
    float resT = -1.0;
    float maxD = 10.0;
    float h = 1.0;
    float t = 0.0;
    for (int i = 0; i < 300; i++) {
        if (h < 0.0001 || t > maxD)break;
        h = map(r.origin + r.direction * t, tmp, c);
        t += h;
    }
    if (t < maxD) {
        resT = t;
        res = tmp;
    }
}

bool RaySignedDistanceQuatTest(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr) {
    float4 tra;
    float4 c = float4(0.6, 0.6, 0.6, 0);
    float t = intersect(ray, tra, c);
    if (t < 0.0) {
        return false;
    }
    else {
        float3 pos = ray.origin + t * ray.direction;
        float3 normal = calcNormal(pos, c);
        thit = t;
        attr.normal = normal;
        return true;
        //calculate surface normal
        //return true
    }
}
// Test ray against a signed distance primitive.
// Ref: https://www.scratchapixel.com/lessons/advanced-rendering/rendering-distance-fields/basic-sphere-tracer
bool RaySignedDistancePrimitiveTest(in Ray ray, in SignedDistancePrimitive::Enum sdPrimitive, out float thit, out ProceduralPrimitiveAttributes attr, in float stepScale = 1.0f)
{
    const float threshold = 0.0001;
    float t = RayTMin();
    const UINT MaxSteps = 300;
    float4 tmp;
    float4 c = float4(0.1, 0.7, 0.12, 0.12);
    float maxD = 10.0;
    // Do sphere tracing through the AABB.
    UINT i = 0;
    while (i++ < MaxSteps && t <= RayTCurrent())
    {
        float3 position = ray.origin + t * ray.direction;
        float distance = map(position, tmp, c);//GetDistanceFromSignedDistancePrimitive(position, sdPrimitive);
        if (distance < threshold) {
            break;
        }
        // Has the ray intersected the primitive? 
        if (distance <= threshold * t)
        {
            float3 hitSurfaceNormal = calcNormal(position, c);
           // float3 hitSurfaceNormal = sdCalculateNormal(position, sdPrimitive);
            if (IsAValidHit(ray, t, hitSurfaceNormal))
            {
                thit = t;
                attr.normal = hitSurfaceNormal;
                return true;
            }
        }

        // Since distance is the minimum distance to the primitive, 
        // we can safely jump by that amount without intersecting the primitive.
        // We allow for scaling of steps per primitive type due to any pre-applied 
        // transformations that don't preserve true distances.
        t += stepScale * distance;
    }
    return false;
}

// Analytically integrated checkerboard grid (box filter).
// Ref: http://iquilezles.org/www/articles/filterableprocedurals/filterableprocedurals.htm
// ratio - Center fill to border ratio.
float CheckersTextureBoxFilter(in float2 uv, in float2 dpdx, in float2 dpdy, in UINT ratio)
{
    float2 w = max(abs(dpdx), abs(dpdy));   // Filter kernel
    float2 a = uv + 0.5*w;
    float2 b = uv - 0.5*w;

    // Analytical integral (box filter).
    float2 i = (floor(a) + min(frac(a)*ratio, 1.0) -
        floor(b) - min(frac(b)*ratio, 1.0)) / (ratio*w);
    return (1.0 - i.x)*(1.0 - i.y);
}


#endif // SIGNEDDISTANCEPRIMITIVES_H