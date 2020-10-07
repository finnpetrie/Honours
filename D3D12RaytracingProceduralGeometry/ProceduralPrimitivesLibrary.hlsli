
//**********************************************************************************************
//
// ProceduralPrimitivesLibrary.hlsli
//
// An interface to call per geometry intersection tests based on primitive type.
//
//**********************************************************************************************

#ifndef PROCEDURALPRIMITIVESLIBRARY_H
#define PROCEDURALPRIMITIVESLIBRARY_H

#include "RaytracingShaderHelper.hlsli"

#include "AnalyticPrimitives.hlsli"
#include "VolumetricPrimitives.hlsli"
#include "SignedDistancePrimitives.hlsli"
#include "SignedDistanceFractals.hlsli"

// Analytic geometry intersection test.
// AABB local space dimensions: <-1,1>.
bool RayAnalyticGeometryIntersectionTest(in Ray ray, in AnalyticPrimitive::Enum analyticPrimitive, out float thit, out ProceduralPrimitiveAttributes attr)
{
    float3 aabb[2] = {
     float3(-1,-1,-1),
     float3(1,1,1)
    };
    float t_min;
    float t_max;
    
    //cubeIntersection(ray, thit, attr);
                   
    switch (analyticPrimitive)
    {
    case AnalyticPrimitive::AABB: return RayAABBIntersectionTest(ray, aabb, thit, attr);//cubeIntersection(ray, thit, attr);
    //case AnalyticPrimitive::Spheres: return RaySpheresIntersectionTest(ray, thit, attr);
    case AnalyticPrimitive::Spheres: return RaySpheresIntersectionTest(ray, thit, attr);
    case AnalyticPrimitive::Sphere: return RaySphereIntersectionTest(ray, thit, t_max, attr);//RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::PointLightSphere: return RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::CSG_Difference:  return ConstructiveSolidGeometry_D(ray, thit, attr);
    case AnalyticPrimitive::CSG_Union: return ConstructiveSolidGeometry(ray, thit, attr);
    case AnalyticPrimitive::CSG_Intersection: return ConstructiveSolidGeometry_I(ray, thit, attr);
    case AnalyticPrimitive::Hyperboloid: return  RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::Ellipsoid: return  RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::Paraboloid: return RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::Cylinder: return RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::Cone: return  RayQuadric(ray, thit, attr, analyticPrimitive);
    case AnalyticPrimitive::Plane: return rayPlane(ray, thit, attr, float3(1, 0, 0), 20, float3(0, 0, 0));

    //case AnalyticPrimitive::CornellBack: return rayPlane(ray, t_min, t_max, thit, attr, float3(0, 0, -1), 0, float3(0, 0, 0));
    default: return false;
    }
}

bool OtherRayCSGGeometryIntervals(in Ray ray, in float minimum, in AnalyticPrimitive::Enum analyticPrimitive, in float elapsedTime, out float thit, out float3 normal) {
    if (analyticPrimitive != 0) {
        if (analyticPrimitive == 9) {
            ProceduralPrimitiveAttributes attr;

            bool hit = RaySignedDistancePrimitiveTestCSG(ray, minimum, thit, attr, 1.0f);
            if (hit) {
                normal = attr.normal;
            }
            return hit;
        }
        else if (analyticPrimitive == 20) {
            ProceduralPrimitiveAttributes attr;

            bool hit = RayMetaballsIntersectionTestCSG(ray, minimum, thit, attr, elapsedTime);
            if (hit) {
                normal = attr.normal;
            }
            return hit;
        }
        else {
            return OtherCSGRayTest(ray, minimum, thit, normal, analyticPrimitive);
        }
    }
    else {
        float3 aabb[2] = {
         float3(-1,-1,-1),
         float3(1,1,1)  
        };

       /* float3 aabb[2] = {
            float3(-1, -1, -1),
            float3(1.5,1.5,1)
        };*/
        ProceduralPrimitiveAttributes attr;
         bool hit = RayAABBIntersectionTestCSG(ray, aabb, minimum, thit, attr);//cubeIntersection(ray, thit, attr);
         if (hit) {
             normal = attr.normal;
         }
         return hit;

    }
}

bool RayCSGGeometryIntervals(in Ray ray, in AnalyticPrimitive::Enum analyticPrimitive, out float tMin, out float tMax, inout float4 intervals, out float3 normal, inout uint count) {
    return CSGRayTest(ray, tMin, tMax, intervals, normal, analyticPrimitive);
}

// Analytic geometry intersection test.
// AABB local space dimensions: <-1,1>.
bool RayVolumetricGeometryIntersectionTest(in Ray ray, in VolumetricPrimitive::Enum volumetricPrimitive, out float thit, out ProceduralPrimitiveAttributes attr, in float elapsedTime)
{
    switch (volumetricPrimitive)
    {
    case VolumetricPrimitive::Metaballs: return RayMetaballsIntersectionTest(ray, thit, attr, elapsedTime);
    default: return false;
    }
}


bool RaySignedDistanceTest(in Ray ray, in SignedDistancePrimitive::Enum signedDistancePrimitive, out float thit, out ProceduralPrimitiveAttributes attr, in float elapsedTime, in float stepScale)
{
    switch (signedDistancePrimitive)
    {
    case SignedDistancePrimitive::MetaBalls: return RayMetaballsIntersectionTest(ray, thit, attr, elapsedTime);
    
    default: return RaySignedDistancePrimitiveTest(ray, signedDistancePrimitive, thit, attr, stepScale, elapsedTime);
    }
}
// Signed distance functions use a shared ray signed distance test.
// The test, instead, calls into this function to retrieve a distance for a primitive.
// AABB local space dimensions: <-1,1>.
// Ref: http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
float GetDistanceFromSignedDistancePrimitive(in float3 position, in SignedDistancePrimitive::Enum signedDistancePrimitive)
{
    switch (signedDistancePrimitive)
    {
    case SignedDistancePrimitive::MiniSpheres:
        return opI(sdSphere(opRep(position + 1, (float3) 2/4), 0.65 / 4), sdBox(position, (float3)1));

    case SignedDistancePrimitive::IntersectedRoundCube:
        return opS(opS(udRoundBox(position, (float3) 0.75, 0.2), sdSphere(position, 1.20)), -sdSphere(position, 1.32));

    case SignedDistancePrimitive::SquareTorus: 
        return sdTorus82(position, float2(0.75, 0.15));
    
    case SignedDistancePrimitive::TwistedTorus: 
        return sdTorus(opTwist(position), float2(0.6, 0.2));
        
    case SignedDistancePrimitive::Cog:
        return opS( sdTorus82(position, float2(0.60, 0.3)),
                    sdCylinder(opRep(float3(atan2(position.z, position.x) / 6.2831, 
                                            1, 
                       0.015 + 0.25 * length(position)) + 1,
                                     float3(0.05, 1, 0.075)),
                               float2(0.02, 0.8)));
    
    case SignedDistancePrimitive::Cylinder: 
        return opI(sdCylinder(opRep(position + float3(1, 1, 1), float3(1, 2, 1)), float2(0.3, 2)),
                   sdBox(position + float3(1, 1, 1), float3(2, 2, 2)));
    
    case SignedDistancePrimitive::QuaternionJulia: 
        float3 trap;
        //return opS(opS(udRoundBox(position, (float3) 0.75, 0.2), sdSphere(position, 1.20)), -sdSphere(position, 1.32));
               // float4 c1 = float4(-0.45, 0.34, -0.09, -0.56)
       // return  sdTorus(position, float2 t)

     return juliaMap(position.xyz, trap,  float4(0.6, 0.6, 0.6, 0));
     //  return sdTorus82(position, float2(0.75, 0.15));
        //return sdQuaternionJuliaSet(position + float3(0, 0, 0), float4(0.894, 0.447, 2.0, 0.0), 2.0f);
      // return sdGyroid(position);

    default: return 0;
    }
}

#endif // PROCEDURALPRIMITIVESLIBRARY_H