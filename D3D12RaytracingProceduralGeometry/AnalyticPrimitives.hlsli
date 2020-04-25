//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

//**********************************************************************************************
//
// AnalyticPrimitives.hlsli
//
// Set of ray vs analytic primitive intersection tests.
//
//**********************************************************************************************

#ifndef ANALYTICPRIMITIVES_H
#define ANALYTICPRIMITIVES_H


#include "RaytracingShaderHelper.hlsli"

// Solve a quadratic equation.
// Ref: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
bool SolveQuadraticEqn(float a, float b, float c, out float x0, out float x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = -0.5 * b / a;
    else {
        float q = (b > 0) ?
            -0.5 * (b + sqrt(discr)) :
            -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) swap(x0, x1);

    return true;
}

// Calculate a normal for a hit point on a sphere.
float3 CalculateNormalForARaySphereHit(in Ray ray, in float thit, float3 center)
{
    float3 hitPosition = ray.origin + thit * ray.direction;
    return normalize(hitPosition - center);
}

float3 CalculateNormalForRayConeHit(in Ray ray, in float thit, float3 centre,float3 axis) {
    float3 cp = ray.origin + thit * ray.direction - centre;
    float3 n = normalize(cp * dot(axis, cp) / dot(cp, cp) - axis);
    return n;
}
// Analytic solution of an unbounded ray sphere intersection points.
// Ref: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
bool SolveRaySphereIntersectionEquation(in Ray ray, out float tmin, out float tmax, in float3 center, in float radius)
{
    float3 L = ray.origin - center;
    float a = dot(ray.direction, ray.direction);
    float b = 2 * dot(ray.direction, L);
    float c = dot(L, L) - radius * radius;

    return SolveQuadraticEqn(a, b, c, tmin, tmax);
}


bool AltRayCone(in Ray ray, out float tmin, out float tmax, in float3 tip, in float3 axis, in float theta) {
    float3 co = ray.origin - tip;
    float cosa = 0.95;
    float a = dot(ray.direction, axis) * dot(ray.direction, axis) - cosa * cosa; 
    float b = 2 * (dot(ray.direction, axis) * dot(co, axis) - dot(ray.direction, co) * cosa * cosa);
    float c = dot(co, axis) * dot(co, axis) - dot(co, co) * cosa * cosa;
    return SolveQuadraticEqn(a, b, c, tmin, tmax);
}
bool SolveRayConeIntersection(in Ray ray, out float tmin, out float tmax, in float3 tip, in float3 axis, in float theta) {
    float dv = dot(ray.direction, axis);
    float3 co = ray.origin - tip;
    float dcoV = dot(co, axis);
    float dco = dot(co, co);
    float dirCo = dot(ray.direction, co);
    float cosTheta = cos(theta);


    float a = dv * dv - cosTheta * cosTheta;
    float b = 2 * (dv * dcoV - dirCo * cosTheta * cosTheta);
    float c = (dcoV * dcoV - dirCo * cosTheta * cosTheta);

    return SolveQuadraticEqn(a, b, c, tmin, tmax);
}


bool SolveRayQuadricInteresection(in Ray ray, out float tmin, out float tmax, in float4x4 Q) {

    float4 AD = mul(Q, float4(ray.direction, 0));
    float4 AC = mul(Q, float4(ray.origin, 1));

    float a = dot(float4(ray.direction, 0), AD);
    float b = dot(float4(ray.origin, 1), AD) + dot(float4(ray.direction, 0), AC);
    float c = dot(float4(ray.origin, 1), AC);
    if (abs(a) < (0.0001f)) {
        float t = -c / b;
        tmin = t;
        tmax = t;
        return true;
    }

    return SolveQuadraticEqn(a, b, c, tmin, tmax);
}

bool QuadricRayIntersectionTest(in Ray r, out float thit, out ProceduralPrimitiveAttributes attr) {
    float4x4 Q = {1, 0, 0,0,
                   0, 1, 0, 0,
                    0, 0, 1 , 0,
                    0, 0, 0, -1}; 
    float tmin, tmax;
    float n_x, n_y, n_z;
    if (!SolveRayQuadricInteresection(r, tmin, tmax, Q)) {
        return false;
    }
    if (tmin < 0) {
        if (tmax < 0) {
            return false;
        }
        thit = tmax;
     }
    else {

        thit = tmin;
    }
    float3 intersectionPoint = r.origin + thit * r.direction;
    float r_x = intersectionPoint.x;
    float r_y = intersectionPoint.y;
    float r_z = intersectionPoint.z;

    float4 Q_X = mul(Q, float4(intersectionPoint, 1));
    n_x = dot(float4(2, 0, 0, 0), Q_X);
    n_y = dot(float4(0, 2, 0, 0), Q_X);
    n_z = dot(float4(0, 0, 2, 0), Q_X);
    float3 norm = float3(n_x, n_y, n_z);
    
    attr.normal = normalize(norm);
    if (IsAValidHit(r, thit, attr.normal)) {
        return true;
    }
    else {
        return false;
    }
}

bool RayConeIntersectionTest(in Ray r, out float thit,out ProceduralPrimitiveAttributes attr) {
    float3 tip = float3(0, 0, 0);
    float3 axis = float3(0, 1, 0);
    float3 co = r.origin - tip;
    float cosa = 0.95;
    float a = dot(r.direction, axis) * dot(r.direction, axis) - cosa * cosa;
    float b = 2. * (dot(r.direction, axis) * dot(co, axis) - dot(r.direction, co) * cosa * cosa);
    float c = dot(co, axis) * dot(co, axis) - dot(co, co) * cosa * cosa;

    float det = b * b - 4. * a * c;
    if (det < 0.) return false;

    det = sqrt(det);
    float t1 = (-b - det) / (2. * a);
    float t2 = (-b + det) / (2. * a);

    // This is a bit messy; there ought to be a more elegant solution.
    float t = t1;
    if (t < 0. || (t2 > 0.&& t2 < t) )t = t2;
    if (t < 0.) return false;

    float3 cp = r.origin + t * r.direction - tip;
    float h = dot(cp, axis);
    if (h < 0. || h > 1) return false;

    float3 n = normalize(cp * dot(axis, cp) / dot(cp, cp) - axis);
    thit = t;
    attr.normal = n;
    return true;
    /*float t0, t1;
    float tmax;
    if (!SolveRayConeIntersection (ray, t0, t1, float3(0, 0, 0), float3(0, 1, 0), 45)) return false;
    if (t0 < 0 || (t1 > 0  && t1 < t0)) {
        t0 = t1;
   }
    if (t0 < 0) {
        return false;
   }
    float3 v = float3(0, 1, 0);
    float3 cp = ray.origin + t0 * ray.direction;
    float h = dot(cp, v);
    if (h < 0 || 0 > 1) {
        return false;
    }

    float3 norm = CalculateNormalForRayConeHit(ray, t0, float3(0, 0, 0), float3(0, 1, 0));
    attr.normal = norm;
    thit = t0;
    return true;
    /**(tmax = t1;
    if (t0 < RayTMin()) {
        if (t1 < RayTMin()) {
            return false;
      }
        if (IsAValidHit(ray, t1, attr.normal)) {
            thit = t1;
            attr.normal = CalculateNormalForRayConeHit(ray, thit, float3(0, 0, 0), float3(0, 1, 0));

            return true;
        }
    }
    else
    {
        if (IsAValidHit(ray, t0, attr.normal))
        {
            thit = t0;
            attr.normal = CalculateNormalForRayConeHit(ray, thit, float3(0, 0, 0), float3(0, 1, 0));

            return true;
        }

        if (IsAValidHit(ray, t1, attr.normal))
        {
            thit = t1;
            attr.normal = CalculateNormalForRayConeHit(ray, thit, float3(0, 0, 0), float3(0, 1, 0));

            return true;
        }
    }
    return false;*/
}
// Test if a ray with RayFlags and segment <RayTMin(), RayTCurrent()> intersects a hollow sphere.
bool RaySphereIntersectionTest(in Ray ray, out float thit, out float tmax, out ProceduralPrimitiveAttributes attr, in float3 center = float3(0, 0, 0), in float radius = 1)
{
    float t0, t1; // solutions for t if the ray intersects 

    if (!SolveRaySphereIntersectionEquation(ray, t0, t1, center, radius)) return false;
    tmax = t1;

    if (t0 < RayTMin())
    {
        // t0 is before RayTMin, let's use t1 instead .
        if (t1 < RayTMin()) return false; // both t0 and t1 are before RayTMin

        attr.normal = CalculateNormalForARaySphereHit(ray, t1, center);
        if (IsAValidHit(ray, t1, attr.normal))
        {
            thit = t1;
            return true;
        }
    }
    else
    {
        attr.normal = CalculateNormalForARaySphereHit(ray, t0, center);
        if (IsAValidHit(ray, t0, attr.normal))
        {
        thit = t0;
        return true;
        }

        attr.normal = CalculateNormalForARaySphereHit(ray, t1, center);
        if (IsAValidHit(ray, t1, attr.normal))
        {
        thit = t1;
        return true;
        }
    }
    return false;
}

// Test if a ray segment <RayTMin(), RayTCurrent()> intersects a solid sphere.
// Limitation: this test does not take RayFlags into consideration and does not calculate a surface normal.
bool RaySolidSphereIntersectionTest(in Ray ray, out float thit, out float tmax, in float3 center = float3(0, 0, 0), in float radius = 1)
{
    float t0, t1; // solutions for t if the ray intersects 

    if (!SolveRaySphereIntersectionEquation(ray, t0, t1, center, radius)) 
        return false;

    // Since it's a solid sphere, clip intersection points to ray extents.
    thit = max(t0, RayTMin());
    tmax = min(t1, RayTCurrent());

    return true;
}

// Test if a ray with RayFlags and segment <RayTMin(), RayTCurrent()> intersects multiple hollow spheres.
bool RaySpheresIntersectionTest(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr)
{
    const int N = 4;
    float3 centers[N] =
    {
        float3(-0.3, -0.3, -0.3),
        float3(0.1, 0.1, 0.4),
        float3(0.35,0.35, 0.0),
        float3(0.5,0.5, 1)

    };
    float  radii[N] = { 0.6, 0.2, 0.15, 0.5 };
    bool hitFound = false;

    //
    // Test for intersection against all spheres and take the closest hit.
    //
    thit = RayTCurrent();

    float _thit;
    float _tmax;
    ProceduralPrimitiveAttributes _attr;
   // for (int i = 0; i < N; i++) {
        if (RaySphereIntersectionTest(ray, _thit, _tmax, _attr, centers[0], radii[0]))
        {
            if (_thit < thit)
            {
                thit = _thit;
                attr = _attr;
                hitFound = true;
            }
        }
  //  }
    return hitFound;

}

// Test if a ray segment <RayTMin(), RayTCurrent()> intersects an AABB.
// Limitation: this test does not take RayFlags into consideration and does not calculate a surface normal.
// Ref: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
bool RayAABBIntersectionTest(Ray ray, float3 aabb[2], out float tmin, out float tmax)
{
    float3 tmin3, tmax3;
    int3 sign3 = ray.direction > 0;

    // Handle rays parallel to any x|y|z slabs of the AABB.
    // If a ray is within the parallel slabs, 
    //  the tmin, tmax will get set to -inf and +inf
    //  which will get ignored on tmin/tmax = max/min.
    // If a ray is outside the parallel slabs, -inf/+inf will
    //  make tmax > tmin fail (i.e. no intersection).
    // TODO: handle cases where ray origin is within a slab 
    //  that a ray direction is parallel to. In that case
    //  0 * INF => NaN
    const float FLT_INFINITY = 1.#INF;
    float3 invRayDirection = ray.direction != 0 
                           ? 1 / ray.direction 
                           : (ray.direction > 0) ? FLT_INFINITY : -FLT_INFINITY;

    tmin3.x = (aabb[1 - sign3.x].x - ray.origin.x) * invRayDirection.x;
    tmax3.x = (aabb[sign3.x].x - ray.origin.x) * invRayDirection.x;

    tmin3.y = (aabb[1 - sign3.y].y - ray.origin.y) * invRayDirection.y;
    tmax3.y = (aabb[sign3.y].y - ray.origin.y) * invRayDirection.y;
    
    tmin3.z = (aabb[1 - sign3.z].z - ray.origin.z) * invRayDirection.z;
    tmax3.z = (aabb[sign3.z].z - ray.origin.z) * invRayDirection.z;
    
    tmin = max(max(tmin3.x, tmin3.y), tmin3.z);
    tmax = min(min(tmax3.x, tmax3.y), tmax3.z);
    
    return tmax > tmin && tmax >= RayTMin() && tmin <= RayTCurrent();
}

// Test if a ray with RayFlags and segment <RayTMin(), RayTCurrent()> intersects a hollow AABB.
bool RayAABBIntersectionTest(Ray ray, float3 aabb[2], out float thit, out ProceduralPrimitiveAttributes attr)
{
    float tmin, tmax;
    if (RayAABBIntersectionTest(ray, aabb, tmin, tmax))
    {
        // Only consider intersections crossing the surface from the outside.
        if (tmin < RayTMin() || tmin > RayTCurrent())
            return false;

        thit = tmin;

        // Set a normal to the normal of a face the hit point lays on.
        float3 hitPosition = ray.origin + thit * ray.direction;
        float3 distanceToBounds[2] = {
            abs(aabb[0] - hitPosition),
            abs(aabb[1] - hitPosition)
        };
        const float eps = 0.0001;
        if (distanceToBounds[0].x < eps) attr.normal = float3(-1, 0, 0);
        else if (distanceToBounds[0].y < eps) attr.normal = float3(0, -1, 0);
        else if (distanceToBounds[0].z < eps) attr.normal = float3(0, 0, -1);
        else if (distanceToBounds[1].x < eps) attr.normal = float3(1, 0, 0);
        else if (distanceToBounds[1].y < eps) attr.normal = float3(0, 1, 0);
        else if (distanceToBounds[1].z < eps) attr.normal = float3(0, 0, 1);

        return IsAValidHit(ray, thit, attr.normal);
    }
    return false;
}

#endif // ANALYTICPRIMITIVES_H