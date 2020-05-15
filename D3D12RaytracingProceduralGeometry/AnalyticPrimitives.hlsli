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

float3 CalculateNormalForARayQuadricHit(in Ray r, in float thit, in float4x4 Q)
{
    float n_x, n_y, n_z;
    float3 intersectionPoint;
    float3 dir = normalize(r.direction);

    intersectionPoint = r.origin + thit * r.direction;

    float4 Q_X = mul(Q, float4(intersectionPoint, 1));
    n_x = dot(float4(2, 0, 0, 0), Q_X);
    n_y = dot(float4(0, 2, 0, 0), Q_X);
    n_z = dot(float4(0, 0, 2, 0), Q_X);
    float3 norm = normalize(float3(n_x, n_y, n_z));

    if (dot(norm, dir) > 0) {
        norm = -norm;
    }

    return norm;
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




bool SolveRayQuadricInteresection(in Ray ray, out float tmin, out float tmax, in float4x4 Q) {

    float4 AD = mul(Q, float4(ray.direction, 0));
    float4 AC = mul(Q, float4(ray.origin, 1));

    float a = dot(float4(ray.direction, 0), AD);
    float b = dot(float4(ray.origin, 1), AD) + dot(float4(ray.direction, 0), AC);
    float c = dot(float4(ray.origin, 1), AC);
    if (abs(a) == 0) {
        float t = -c / b;
        tmin = t;
        tmax = t;
        return true;
    }

    return SolveQuadraticEqn(a, b, c, tmin, tmax);
}

bool QuadricRayIntersectionTest(in Ray r, inout float tmin, inout float tmax, out float thit, out ProceduralPrimitiveAttributes attr, in AnalyticPrimitive::Enum analyticPrimitive = AnalyticPrimitive::Enum::Hyperboloid, in float4x4 Q = float4x4(-1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, -1.0f)) {


    float n_x, n_y, n_z;

    if (!SolveRayQuadricInteresection(r, tmin, tmax, Q)) {
        return false;
    }

    if (tmin < RayTMin()) {
        if (tmax < RayTMin()) {
            return false;
        }
        thit = tmax;
    }
    else {
        thit = tmin;
    }

    float3 intersectionPoint = r.origin + thit * r.direction;

    if (abs(intersectionPoint.x) > 2) {
        thit = tmax;
        tmin = tmax;
    }

    if (analyticPrimitive == AnalyticPrimitive::Paraboloid || analyticPrimitive == AnalyticPrimitive::Cone) {
        if ((abs(intersectionPoint.y) > 2) || (abs(intersectionPoint.z) > 2)) {
            thit = tmax;
        }
    }

    intersectionPoint = r.origin + thit * r.direction;

    attr.normal = CalculateNormalForARayQuadricHit(r, thit, Q);
    //bound along x-axis
    if (abs(intersectionPoint.x) > 2) {
        return false;
    }
   
    //todo -- add caps - intersect planes
    
    /*if(analyticPrimitive == AnalyticPrimitive::Cylinder) {
        if (intersectionPoint.x == 2) {
            attr.normal = float3(1, 0, 0);

        }
        else if (intersectionPoint.x == -2) {
            attr.normal = float3(-1, 0, 0);
        }
    }*/

    //bound paraboloid in all axes
    if (analyticPrimitive == AnalyticPrimitive::Cone) {
        if ((abs(intersectionPoint.y) > 2) || (abs(intersectionPoint.z) > 2) || intersectionPoint.y > 0) {
            return false;
        }
    }    if (analyticPrimitive == AnalyticPrimitive::Paraboloid) {
        if ((abs(intersectionPoint.y) > 2) || (abs(intersectionPoint.z) > 2)) {
            return false;
        }
    }
    tmin = thit;
    return true;
}

bool RayQuadric(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr, in AnalyticPrimitive::Enum type) {
 
    //thit = RayTCurrent();

    float4x4 Q;
    bool hitFound = false;
    switch (type) {
    case AnalyticPrimitive::Hyperboloid: Q = float4x4( -1.0f ,0.0f, 0.0f, 0.0f,
                                                    0.0f, 1.0f, 0.0f,  0.0f,
                                                     0.0f, 0.0f, 1.0f , 0.0f,
                                                       0.0f, 0.0f, 0.0f, -0.09f );
        break;
    case AnalyticPrimitive::Ellipsoid: Q = float4x4( 1.0f / 1.5f, 0.0f, 0.0f,0.0f,
                                             0.0f, 1.0f, 0.0f, 0.0f,
                                              0.0f, 0.0f, 1.0f / 2.0f , 0.0f,
                                              0.0f, 0.0f, 0.0f, -1.0f );
        break;
    case AnalyticPrimitive::Paraboloid: Q = float4x4 ( 1.0f / 2, 0.0f, 0.0f, 0.0f,
                                                0.0f, 0.0f, 0.0f,  -0.1f,
                                                 0.0f, 0.0f, 1.0f / 1.5f, 0.0f,
                                                   0.0f, -0.1f, 0.0f, 0.0f );
        break;
    case AnalyticPrimitive::Cylinder: Q = float4x4( 0.0f, 0.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 0.0f, -0.5f );
        break;
    case AnalyticPrimitive::Cone: Q = float4x4 (-1.0f, 0.0f, 0.0f, 0.0f,
                                                0.0f, 1.0f, 0.0f, 0.0f,
                                                0.0f, 0.0f, -1.0f, 0.0f,
                                                0.0f, 0.0f, 0.0f, 0.0f);
        break;
     default: return false;

    }

    float _tmin;
    float _thit;
    float _tmax;
    ProceduralPrimitiveAttributes _attr;

    if (QuadricRayIntersectionTest(ray, _tmin, _tmax, _thit, _attr, type, Q))
    {
       
            thit = _thit;
            attr = _attr;
            hitFound = true;
        
    }
 
    return hitFound;

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


bool Intersection(float4 intersections, in float thit, in ProceduralPrimitiveAttributes first_attr, in ProceduralPrimitiveAttributes second_attr, inout ProceduralPrimitiveAttributes attr ) {
    if ((intersections.x < intersections.z) && (intersections.y > intersections.z))  {
        thit = intersections.z;
        attr = second_attr;
    }
    else if ((intersections.z < intersections.x) && (intersections.w > intersections.x)) {
        thit = intersections.x;
        attr = first_attr;
    }
    else {
        return false;
    }
    return true;
}
void insertionSort(inout float4 intersections) {

    int j;
    float key;
    for (int i = 1; i < 4; i++) {
        key = intersections[i];
        j = i - 1;
        while (j >= 0 && intersections[j] > key) {
            intersections[j + 1] = intersections[j];
            j--;
        }
        intersections[j + 1] = key;
    }
}
bool Union(float s_min, float b_min, out float thit, out ProceduralPrimitiveAttributes attr) {
    thit = min(s_min, b_min);
  
    return true;
}
bool ConstructiveSolidGeometry(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr) {
    //get intersections for both AABB, and sphere.
    float s_tmin, s_tmax;
    float b_tmin, b_tmax;
    float t_hit;
    ProceduralPrimitiveAttributes s_attr;
    ProceduralPrimitiveAttributes b_attr;
  /*float4x4 Q = float4x4(-1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, -0.9f);
        */
    float4x4 Q = float4x4 (-1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f);

   bool sphere_hit = RaySphereIntersectionTest(ray, s_tmin, s_tmax, s_attr);
   bool hyp_hit = QuadricRayIntersectionTest(ray, b_tmin, b_tmax, t_hit, b_attr, AnalyticPrimitive::Enum::Cone, Q);
    if (sphere_hit || hyp_hit) {
        if (sphere_hit && hyp_hit) {
            float2 diffs;

            diffs.x = s_tmin;
            diffs.y = t_hit;
            bool hit = Union(s_tmin, t_hit, thit, attr);
            float3 intersectionPoint = ray.origin + thit * ray.direction;
            // thit = t_hit;
            // attr.normal = CalculateNormalForARayQuadricHit(ray, t_hit, Q);
            if (thit == s_tmin) {
                attr = s_attr;
            }
            else {
                attr = b_attr;
            }
        }
        else if (sphere_hit && !hyp_hit) {
            thit = s_tmin;
            attr = s_attr;
        }
        else {
            thit = b_tmin;
            attr = b_attr;
        }
        return true;
    
    }

    return false;
    
    //turn true;
}

bool ConstructiveSolidGeometry_I(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr) {
    //get intersections for both AABB, and sphere.
    float s_tmin, s_tmax;
    float b_tmin, b_tmax;
    float t_hit;    
    float4 intersections;


    ProceduralPrimitiveAttributes s_attr;
    ProceduralPrimitiveAttributes b_attr;
   
    float4x4 Q = float4x4 (-1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f);

    bool sphere_hit = RaySphereIntersectionTest(ray, s_tmin, s_tmax, s_attr);
    bool hyp_hit = QuadricRayIntersectionTest(ray, b_tmin, b_tmax, t_hit, b_attr, AnalyticPrimitive::Enum::Cone, Q);

    if (sphere_hit && hyp_hit) {
        intersections = float4(s_tmin, s_tmax, b_tmin, b_tmax);
        float e = max(intersections.x, intersections.z);
        float f = min(intersections.y, intersections.w);
        if (e <= f) {
            if (e == intersections.x) {
                attr = s_attr;
            }
            else {
                attr = b_attr;
            }
            if (RayTMin() < e) {
                thit = e;
            }
            else {
                thit = f;
            }
            return true;
            //attr = s_attr;
        }
        //return Intersection(intersections, thit, s_attr, b_attr, attr);
    }

    return false;


    //turn true;
}


bool ConstructiveSolidGeometry_D(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr) {
    //get intersections for both AABB, and sphere.
    float s_tmin, s_tmax;
    float b_tmin, b_tmax;
    float t_hit;
    float4 intersections;


    ProceduralPrimitiveAttributes s_attr;
    ProceduralPrimitiveAttributes b_attr;
    float4x4 Q =  float4x4 (-1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f);

    bool sphere_hit = RaySphereIntersectionTest(ray, s_tmin, s_tmax, s_attr);
    bool hyp_hit = QuadricRayIntersectionTest(ray, b_tmin, b_tmax, t_hit, b_attr, AnalyticPrimitive::Enum::Cone, Q);

    if (sphere_hit && hyp_hit) {
        intersections = float4(b_tmin, b_tmax, s_tmin, s_tmax);
        if(intersections.x < intersections.z){
            thit = intersections.x;
            attr = b_attr;
        }
        else if (intersections.w < intersections.y) {
            thit = intersections.w;
            attr.normal = -s_attr.normal;
        }
        else {
            return false;
        }

        return true;
        //return Intersection(intersections, thit, s_attr, b_attr, attr);
    }

    if (hyp_hit) {
        thit = t_hit;
        attr = b_attr;
        return true;
    }
    return false;


    //turn true;
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
    float  radii[N] = { 1, 0.2, 0.15, 0.5 };
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
   //float3 invRayDirection = 1/ray.direction;
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
    
    return tmax > tmin;
}

bool RayAABBTest(Ray ray, float3 aabb[2], inout float tmin, inout  float tmax) {
    return true;
}

// Test if a ray with RayFlags and segment <RayTMin(), RayTCurrent()> intersects a hollow AABB.
bool RayAABBIntersectionTest(Ray ray, float3 aabb[2], out float thit, out ProceduralPrimitiveAttributes attr)
{
    float tmin, tmax;
    if (RayAABBIntersectionTest(ray, aabb, tmin, tmax))
    {
        // Only consider intersections crossing the surface from the outside.
        if (tmin < RayTMin() || tmin > RayTCurrent() )
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

        if (dot(ray.direction, attr.normal) > 0) {
            attr.normal = -attr.normal;
        }
        return true;
       //return IsAValidHit(ray, thit, attr.normal);
    }
    return false;
}

#endif // ANALYTICPRIMITIVES_H