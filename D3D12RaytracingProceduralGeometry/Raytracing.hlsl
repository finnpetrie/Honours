
#ifndef RAYTRACING_HLSL
#define RAYTRACING_HLSL

#define HLSL
#include "RaytracingHlslCompat.h"
#include "ProceduralPrimitivesLibrary.hlsli"
#include "RaytracingShaderHelper.hlsli"

//***************************************************************************
//*****------ Shader resources bound via root signatures -------*************
//***************************************************************************

// Scene wide resources.
//  g_* - bound via a global root signature.
//  l_* - bound via a local root signature.
RaytracingAccelerationStructure g_scene : register(t0, space0);
RWTexture2D<float4> g_renderTarget : register(u0);
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);
//ConstantBuffer<CSGNode> g_CSGBuffer : register(b1);
RWTexture2D<float4> screenSpacePhotonMap [2]: register(u1);
//RWBuffer<float4> g_buffer : register(u7);

// Triangle resources
ByteAddressBuffer g_indices : register(t1, space0);
StructuredBuffer<Vertex> g_vertices : register(t2, space0);

// Procedural geometry resources
StructuredBuffer<PrimitiveInstancePerFrameBuffer> g_AABBPrimitiveAttributes : register(t3, space0);
StructuredBuffer<CSGNode> csgTree : register(t4, space0);

ConstantBuffer<PrimitiveConstantBuffer> l_materialCB : register(b1);
ConstantBuffer<PrimitiveInstanceConstantBuffer> l_aabbCB: register(b2);


float rand(in float2 uv) {
    float2 noise = (frac(sin(dot(uv, float2(12.9898, 78.233) * 2.0)) * 43758.5453));
    return abs(noise.x + noise.y) * 0.5;
}

float3 randomDirection(float2 ra) {
    float theta_0 = 2 * M_PI * rand(ra);
    float theta_1 = acos(1 - 2 * rand(ra));
    float3 dir = float3(sin(theta_0) * sin(theta_1), sin(theta_0)*cos(theta_1), sin(theta_1));
    return dir;
}
float FresnelAmount(float n1, float n2, float3 normal, float3 incident) {
    float r0 = (n1 - n2) / (n1 + n2);
    r0 *= r0;
    float cosx = -dot(normal, incident);
    if (n1 > n2) {
        float n = n1 / n2;
        float sint = n * n*(1.0f - cosx * cosx);

        if (sint > 1.0) {
            return 1.0;
        }
        cosx = sqrt(1.0 - sint);
    }
    float x = 1.0 - cosx;
    float ret = r0 + (1.0f - r0) * x * x * x * x * x;
    ret = (l_materialCB.reflectanceCoef + (1.0 - l_materialCB.reflectanceCoef) * ret);
    return ret;
}
float PhongLighting(float3 normal, bool shadowHit) {
    float3 position = HitWorldPosition();
    float3 lightDir = normalize(g_sceneCB.lightPosition.xyz - position);
    float3 viewDir = normalize(-WorldRayDirection());
    float3 refl = normalize(reflect(normal, lightDir));
    float illum;
    if (l_materialCB.refractiveCoef == 0) {
         illum = l_materialCB.diffuseCoef * saturate(dot(lightDir, normal));

        illum += l_materialCB.specularCoef * pow(saturate(dot(refl,viewDir)), l_materialCB.specularPower);
    }
    else {
        illum = l_materialCB.specularCoef * pow(saturate(dot(refl, viewDir)), l_materialCB.specularPower);

    }
    if (!shadowHit) {

    }
    else {
       //llum = 0;

    }
    return illum;
}

bool refractTest(float3 v, float3 normal, float index, inout float3 refracted) {
    float dt = dot(v, normal);
    float discriminant = 1.0 - index * index * (1 - dt * dt);
    if (discriminant > 0) {
        refracted = index * (v - normal * dt) - normal * sqrt(discriminant);
        return true;
    }
    else {
        return false;
    }
}
// Trace a radiance ray into the scene and returns a shaded color.
RayPayload TraceRadianceRay(in Ray ray, in RayPayload payload)
{
    if (payload.recursionDepth >= MAX_RAY_RECURSION_DEPTH)
    {
       payload.color = float4(1,1,1, 1);
       return payload;
    }

    // Set the ray's extents.
    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    // Set TMin to a zero value to avoid aliasing artifacts along contact areas.
    // Note: make sure to enable face culling so as to avoid surface face fighting.
    rayDesc.TMin = 0.001;
    rayDesc.TMax = 10000;
  //  RayPayload rayPayload = { float4(0, 0, 0, 0), payload.recursionDepth + 1, payload.seed};
    //float3 pos = WorldRayOrigin() + RayTCurrent() * WorldRayDirection();
   /* payload.intersections[payload.recursionDepth] = float4(ray.origin, 1);
    switch (payload.recursionDepth) {
   case 0:
       payload.depth_0 = float4(ray.origin, 1);
       break;
   case 1:
       payload.depth_1 = float4(ray.origin, 1);
       payload.intersections[1] = float4(ray.origin, 1);
       break;
   case 2:
       payload.depth_2 = payload.depth_1;
       payload.intersections[2] = float4(ray.origin, 1);

       break;
   case 3:
       payload.depth_3 = float4(ray.origin, 1);
       break;
   case 4:
       payload.depth_4 = float4(ray.origin, 1);
       break;
   case 5:
       payload.depth_5 = float4(ray.origin, 1);
       break;
   }*/
    payload.recursionDepth += 1;

    TraceRay(g_scene, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH,
        TraceRayParameters::InstanceMask,
        TraceRayParameters::HitGroup::Offset[RayType::Radiance],
        TraceRayParameters::HitGroup::GeometryStride,
        TraceRayParameters::MissShader::Offset[RayType::Radiance],
        rayDesc, payload);

    return payload;
}



bool ShadowRay(in Ray ray, in UINT currentRayRecursionDepth) {

    if (currentRayRecursionDepth >= MAX_RAY_RECURSION_DEPTH)
    {
        return false;
    }


    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    rayDesc.TMin = 1;
    rayDesc.TMax = 10000;

    ShadowRayPayload shadow = { true };

    TraceRay(g_scene,
        RAY_FLAG_CULL_BACK_FACING_TRIANGLES
                   // ~skip any hit shaders
        | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER, // ~skip closest hit shaders,  
        TraceRayParameters::InstanceMask,
        TraceRayParameters::HitGroup::Offset[RayType::Shadow],
        TraceRayParameters::HitGroup::GeometryStride,
        TraceRayParameters::MissShader::Offset[RayType::Shadow],
        rayDesc, shadow);

    return shadow.hit;
}


//photon mapping ray-gen

[shader("raygeneration")]
void Photon_Ray_Gen() {


    //for each light source

    //send random ray light source

}

[shader("closesthit")]
void ClosestHit_Photon_Triangle(inout RayPayload payload, in BuiltInTriangleIntersectionAttributes attr) {


}

//don't need to define custom interesctions, since we will use the same as backward ray-tracing
[shader("closesthit")]
void ClosestHit_Photon_Procedural(inout RayPayload payload, in ProceduralPrimitiveAttributes attr) {


}

//***************************************************************************
//********************------ Ray gen shader.. -------************************
//***************************************************************************
[shader("raygeneration")]
void MyRaygenShader()
{
    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.cameraPosition.xyz, g_sceneCB.projectionToWorld);
 
    // Cast a ray into the scene and retrieve a shaded color.
    UINT currentRecursionDepth = 0;

    //zero out our intersection buffer
   /* for (uint i = 0; i < MAX_RAY_RECURSION_DEPTH; i++) {
      
    intersectionBuffer[i][DispatchRaysIndex().xy] = float4(0,0,0,0);
    }*/
    RayPayload payload = { float4(0,0,0,0), 
        0,
        0 };
    RayPayload traced = TraceRadianceRay(ray, payload);

    // Write the raytraced color to the output texture.
    g_renderTarget[DispatchRaysIndex().xy] = traced.color;
    uint offset = 0;

    //if drawing rays,   need to write the intersections for this ray into a 3D tensor.

}




//***************************************************************************
//******************------ Closest hit shaders -------***********************
//***************************************************************************

[shader("closesthit")]
void MyClosestHitShader_Triangle(inout RayPayload rayPayload, in BuiltInTriangleIntersectionAttributes attr)
{
    uint indexSizeInBytes = 4;
    uint indicesPerTriangle = 3;
    uint triangleIndexStride = indicesPerTriangle * indexSizeInBytes;
    uint baseIndex = PrimitiveIndex() * triangleIndexStride;

    // Load up three 16 bit indices for the triangle.
    const uint3 indices = Load3x16BitIndices(baseIndex, g_indices);

    // Retrieve corresponding vertex normals for the triangle vertices.
    float3 triangleNormals[3] = { g_vertices[indices[0]].normal,
                                   g_vertices[indices[1]].normal,
                                    g_vertices[indices[2]].normal
    };

    float3 triangleNormal = HitAttribute(triangleNormals, attr.barycentrics);

    float4 ambient = float4(0.1, 0.1, 0.1, 1);
    float3 hitPos = HitWorldPosition();
    float3 pos = HitWorldPosition();
    float3 dir = normalize(g_sceneCB.lightPosition.xyz - pos);
    float3 r_dir = randomDirection(HitWorldPosition().xy);
 
    float3 pos_n =normalize(HitWorldPosition());

    uint depth = rayPayload.recursionDepth;
    //intersectionBuffer[depth][DispatchRaysIndex().xy] = normalize(float4(pos, 1));


   float4 pathColour = float4(0, 0, 0, 0);
   /*f (true) {
        for (int i = 0; i < 1; i++) {
            float3 r_dir = randomDirection(HitWorldPosition().xy);

            Ray path = { HitWorldPosition(), r_dir };
            pathColour += TraceRadianceRay(path, rayPayload.recursionDepth);
        }
    }*/

    Ray shadowRay;
    shadowRay.origin = HitWorldPosition();
    shadowRay.direction = dir;
    bool shadowHit = ShadowRay(shadowRay, rayPayload.recursionDepth);
    
    float4 reflectionColour = float4(0, 0, 0, 0);
    if (!shadowHit) {
        //float distance = length(g_sceneCB.lightPosition - HitWorldPosition());
        ambient += PhongLighting(float4(triangleNormal, 0), shadowHit);
    }
  

      if(l_materialCB.reflectanceCoef > 0){
        Ray r = { HitWorldPosition(), reflect(WorldRayDirection(), triangleNormal) };
        reflectionColour = TraceRadianceRay(r, rayPayload).color;

    }

    float4 color = ambient + 0.15*reflectionColour;
    float t = RayTCurrent();
  //color = lerp(color, BackgroundColor, 1.0 - exp(-0.000002 * t * t * t));

    //rayPayload.color = color;
    rayPayload.color = lerp(color, BackgroundColor, 1.0 - exp(-0.000002 * t * t * t));

   // rayPayload.intersections[rayPayload.recursionDepth] = color;


}


[shader("closesthit")]
void MyClosestHitShader_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{   

    float3 pos = HitWorldPosition();
    float3 pos_n = normalize(HitWorldPosition());

    //intersectionBuffer[rayPayload.recursionDepth][DispatchRaysIndex().xy] = normalize(float4(pos, 1));
   
    //g_buffer[1080*y + x] = float4(pos, 1);
    float3 l_dir = normalize(g_sceneCB.lightPosition.xyz - pos);
    float4 reflectionColour = float4(0,0,0,1);
    Ray shadowRay = { pos, l_dir };
    float3 currentDir = RayTCurrent() * WorldRayDirection();
    currentDir += WorldRayOrigin();

    float4 ambient = l_materialCB.albedo;
    
    bool shadowHit = ShadowRay(shadowRay, rayPayload.recursionDepth);


    if (!shadowHit) {
        ambient += PhongLighting(float4(attr.normal, 0), shadowHit);
    }
    float4 refractionColour = float4(0, 0, 0, 1);
    float3 dir = normalize(WorldRayDirection());
   
    if (l_materialCB.refractiveCoef > 0) {
        //assume refractive glass
        float n1 = 1;
        float n2 = l_materialCB.refractiveCoef;
        float3 outwardNormal;
        float index;
        float3 refracted;
        if (dot(dir, attr.normal) > 0) {
            outwardNormal = -attr.normal;
            index = n2;
        }
        else {
            outwardNormal = attr.normal;
            index = n1 / n2;
        }
        if (refractTest(dir, outwardNormal, index, refracted)) {
            Ray r = { pos, refracted };
            refractionColour = TraceRadianceRay(r, rayPayload).color;
        }
        else {
            Ray r = { pos, reflect(dir, attr.normal) };
            // refractionColour = TraceRadianceRay(r, rayPayload.recursionDepth);
        }
        float reflectMulti = FresnelAmount(n1, n2, attr.normal, dir);
    }
    if (l_materialCB.reflectanceCoef > 0.1f) {
        Ray r = { pos, reflect(dir, attr.normal) };
        reflectionColour = TraceRadianceRay(r, rayPayload).color;
    }


//0.1f is a good coefficient for reflectioncolour.
 // rayPayload.color += refractionColour + 0.1*reflectionColour;
//  rayPayload.color = refractionColour;
float4 color = ambient + refractionColour +  0.1*reflectionColour;

float t = RayTCurrent();
rayPayload.color = lerp(color, BackgroundColor, 1.0 - exp(-0.000002 * t * t * t));

}

//***************************************************************************
//**********************------ Miss shaders -------**************************
//***************************************************************************

[shader("miss")]
void MyMissShader(inout RayPayload rayPayload)
{
   // float transition = pow(smoothstep(0.02, .5, d.y), 0.4);
//
   // float3 sky = lerp(float3(0.52, 0.77, 1), float3(0.12, 0.43, 1), BackgroundColor);
   // float4 backgroundColor = float4(BackgroundColor);
    uint depth = rayPayload.recursionDepth;
    rayPayload.color = float4(BackgroundColor);
   // intersectionBuffer[depth][DispatchRaysIndex().xy] = float4(0,0,0,0);

}

[shader("miss")]
void MyMissShader_ShadowRay(inout ShadowRayPayload rayPayload)
{


    rayPayload.hit = false;
}


//***************************************************************************
//**********************------ Any Hit Shaders -------**************************
//***************************************************************************
[shader("anyhit")]
void AnyHit_AnalyticPrimitive(inout RayPayload payload, in ProceduralPrimitiveAttributes attr) {
    //IgnoreHit();
    float3 pos = HitWorldPosition();
    if (l_materialCB.refractiveCoef > 0) {
        //  if(ShadowRay(payload)){}
         // payload.color = float4(1, 1, 0, 1);
         //IgnoreHit();
    }
    // payload.color = float4(1, 1, 0, 1);
}

[shader("anyhit")]
void AnyHit_Triangle(inout RayPayload payload, in BuiltInTriangleIntersectionAttributes attr) {
    // payload.color = float4(0, 0, 0, 1);
    //IgnoreHit();
}
//***************************************************************************
//*****************------ Intersection shaders-------************************
//***************************************************************************


[shader("closesthit")]
void ClosestHit_CSG( inout RayPayload payload, in ProceduralPrimitiveAttributes attr){

}
// Get ray in AABB's local space.
Ray GetRayInAABBPrimitiveLocalSpace()
{
    PrimitiveInstancePerFrameBuffer attr = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];

    // Retrieve a ray origin position and direction in bottom level AS space 
    // and transform them into the AABB primitive's local space.
    Ray ray;
    ray.origin = mul(float4(ObjectRayOrigin(), 1), attr.bottomLevelASToLocalSpace).xyz;
    ray.direction = mul(ObjectRayDirection(), (float3x3) attr.bottomLevelASToLocalSpace);
    return ray;
}

[shader("intersection")]
void MyIntersectionShader_AnalyticPrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    AnalyticPrimitive::Enum primitiveType = (AnalyticPrimitive::Enum) l_aabbCB.primitiveType;
   // AnalyticPrimitive::Enum primitiveType = AnalyticPrimitive::Enum::Plane;
    float thit;
    ProceduralPrimitiveAttributes attr;
    if (RayAnalyticGeometryIntersectionTest(localRay, primitiveType, thit, attr))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3) aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3) ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
}

[shader("intersection")]
void MyIntersectionShader_VolumetricPrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    VolumetricPrimitive::Enum primitiveType = (VolumetricPrimitive::Enum) l_aabbCB.primitiveType;
    
    float thit;
    ProceduralPrimitiveAttributes attr;
  
}

[shader("intersection")]
void MyIntersectionShader_SignedDistancePrimitive()
{
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
    SignedDistancePrimitive::Enum primitiveType = (SignedDistancePrimitive::Enum) l_aabbCB.primitiveType;

    float thit;
    ProceduralPrimitiveAttributes attr;

    if (RaySignedDistancePrimitiveTest(localRay, primitiveType, thit, attr, l_materialCB.stepScale))
    {
        PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3) aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3) ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, /*hitKind*/ 0, attr);
    }
 
}



bool isEmpty(CSGNode csgStack[15], int nodePointer) {
    if (nodePointer <= -1) {
        return true;
    }
    return false;
}


void Push(inout CSGNode csgStack[15], inout int nodePointer, CSGNode node) {
    nodePointer += 1;
    csgStack[nodePointer] = node;
}

CSGNode Pop(inout CSGNode csgStack[15], inout int nodePointer) {
    CSGNode popped = csgStack[nodePointer];
    nodePointer -= 1;
    return popped;
}




void CSGCombine(in int operation, in intersectionInterval left, in intersectionInterval right, inout float tmin, inout float tmax, inout float3 normal, inout bool hit) {
  
    if (operation == 0) {
        if (left.hit && right.hit) {
            //union
            //get smallest and largest values over all intersections -- will need to record whether there were intersections for the subtrees
            tmin = min(left.tmin, right.tmin);
            tmax = max(left.tmax, right.tmax);
            if (tmin == left.tmin) {
                normal = left.normal;
            }
            else {
                normal = right.normal;
            }
            hit = true;
        }
        //else we don't intersect one of the subtrees
        else {
            if (right.hit) {
                tmin = right.tmin;
                tmax = right.tmax;
                normal = right.normal;
                hit = true;
            }
            else if(left.hit){
                tmin = left.tmin;
                tmax = left.tmax;
                normal = left.normal;
                hit = true;
            }
            else {
                tmin = -1;
                tmax = -1;
                normal = float3(0, 0, 0);
                hit = false;
            }
        }
    }




    else if (operation == 1) {
        //intersection
        if (left.hit && right.hit) {
            float e = max(left.tmin, right.tmin);
            float f = min(left.tmax, right.tmax);
            hit = true;
            tmin = min(e, f);
            tmax = max(e, f);

            if (tmin == e) {
                if (e == left.tmin) {
                    normal = left.normal;
                }
                else {
                    normal = right.normal;
                }
            }
        }
        else {
            tmin = -1;
            tmax = -1;
            normal = float3(0, 0, 0);
            hit = false;
        }
    }





    else {
        /*
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
*/

    //difference
        if (right.hit && left.hit) {
            if (right.tmin < left.tmin) {
                tmin = right.tmin;
                tmax = left.tmin;
                normal = right.normal;
                hit = true;

            }
            else if (left.tmax < right.tmax) {
                tmin = left.tmax;
                tmax = right.tmax;
                normal = -left.normal;
                hit = true;

            }
            else {
                tmin = -1;
                tmax = -1;
                normal = float3(0, 0, 0);
                hit = false;
                
            }
        }
        else {
            if (right.hit) {
                tmin = right.tmin;
                tmax = right.tmax;
                normal = right.normal;
                hit = true;
            }
            else {
                tmin = -1;
                tmax = -1;
                normal = float3(0, 0, 0);
                hit = false;
            }
        }
    }

    
}
bool alternativeCSG(in Ray ray, out float thit, out ProceduralPrimitiveAttributes attr) {
    //tree is in post-order, just need to iterate the tree.

    intersectionInterval intersections[10];
    CSGNode current = csgTree[0];
    int i = 0;
    int depth = 1;
    while(i < 7) {
        //need to record depth
        if (current.boolValue == -1) {

            //trace a ray and find interesections - store
            float  tmin, tmax;
            float3 normal;
            Ray r;
            r.origin = ray.origin + current.translation;
            r.direction = ray.direction + current.translation;
            //r.direction = ray.direction + current.translation;
            bool hit = RayCSGGeometryIntervals(r, (AnalyticPrimitive::Enum)current.geometry, tmin, tmax, normal);
            intersectionInterval inter = { tmin, tmax, hit, normal };
            //store this interval in the intersections array at   our node's index.
            intersections[i] = inter;
        }
        else {
            //retrieve intersections, and store
            intersectionInterval left = intersections[i - 2*depth];
            intersectionInterval right = intersections[i - 1];
            float tmin, tmax;
            float3 normal;
            bool hit;
            //combine the nodes
            CSGCombine(current.boolValue, left, right, tmin, tmax, normal, hit);
            intersectionInterval inter = { tmin, tmax, hit, normal };
            //store this interval in the intersections array at our node's index.
            intersections[i] = inter;

        }
        i += 1;
        current = csgTree[i];
        if (i % 6 == 0) {
            depth += 1;
        }
    }
 

    intersectionInterval final = intersections[2];

    if (final.hit) {
        if (final.tmin > RayTMin()) {
            thit = final.tmin;
            attr.normal = final.normal;
            return true;
        }
        else if (final.tmax > RayTMin()) {
            thit = final.tmax;
            attr.normal = final.normal;
            return true;

        }
    }
    return false;
}

bool RayCSGIntersectionTest(in Ray ray, in CSGPrimitive::Enum csgPrimitive, out float thit, out ProceduralPrimitiveAttributes attr) {
    float minL = 0;
    float minR = 0;
    bool visitedNodes[7] = { 0, 0,0,0,0,0, 0 };
    intersectionInterval intersections[15];
    CSGNode csgStack[15];
    int nodePointer = -1;
    bool entry = true;
   // csgStack[0] = csgTree[0];
    
    CSGNode root = csgTree[0];
    while (!isEmpty(csgStack, nodePointer) || entry) {
        entry = false;
        //works
        while (root.leftNodeIndex != -1 && !visitedNodes[root.myIndex]) {

            Push(csgStack, nodePointer, csgTree[root.rightNodeIndex]);
            Push(csgStack, nodePointer, root);

            root = csgTree[root.leftNodeIndex];
        }


        if (root.leftNodeIndex == -1 && !visitedNodes[root.myIndex]) {
            if (root.rightNodeIndex != -1) {
                Push(csgStack, nodePointer, csgTree[root.rightNodeIndex]);
            }
            Push(csgStack, nodePointer, root);
        }
        //works
        root = Pop(csgStack, nodePointer);
        //
        if ((root.rightNodeIndex != -1) && (csgStack[nodePointer].parentIndex == root.myIndex)) {
            CSGNode right = Pop(csgStack, nodePointer);
            Push(csgStack, nodePointer, root);
            root = right;
            //return fal
        }else {
            //evaluate node
            //intersect root.
          
            int index = root.myIndex;
            //works - or at least runs.
            if (root.boolValue == -1) {
                //primitive node

                float  tmin, tmax;
                float3 normal;
                bool hit = RayCSGGeometryIntervals(ray, (AnalyticPrimitive::Enum)root.geometry, tmin, tmax, normal);
                //  float3 normal = attr.normal;
                      //intersect geometry
                intersectionInterval i = { tmin, tmax, hit, normal };
                //store this interval in the intersections array at our node's index.
                intersections[index] = i;
                visitedNodes[index] = true;

            }
            else {
              
                 //internal node
                 //get left node and right node intervals -- we know these have been stored since this is a postorder traversal
              
                //we certainly reach this.
                intersectionInterval left = intersections[root.leftNodeIndex];
                intersectionInterval right = intersections[root.rightNodeIndex];
                float tmin, tmax;
                float3 normal;
                bool hit;
                //combine the nodes
                CSGCombine(root.boolValue, left, right, tmin, tmax, normal, hit);

                //store the new tmin tmax value and normal at the node's index
                intersectionInterval interval = { tmin, tmax, hit, normal };
                intersections[index] = interval;
                visitedNodes[index] = true;

            }
        }
    }

    root = csgTree[0];
    intersectionInterval left = intersections[root.leftNodeIndex];
    intersectionInterval right = intersections[root.rightNodeIndex];
    float tmin, tmax;
    float3 normal;
    bool hit;

    CSGCombine(root.boolValue, left, right, tmin, tmax, normal, hit);
   
    /**
    intersectionInterval i = intersections[root.myIndex];
    float tmin = i.tmin;
    float tmax = i.tmax;
    float3 normal = i.normal;
    bool hit = i.hit*/
    
    if (hit) {
       if (tmin > RayTMin()) {
            thit = tmin;
            attr.normal = intersections[root.myIndex].normal;
            return true;

        }
        else if (tmax > RayTMin()) {
            thit = tmax;
            attr.normal = intersections[root.rightNodeIndex].normal;
            return true;

       }
        else {
           return false;
       }
    }
    return false;
}



[shader("intersection")]
void CSG_Intersection() {
    Ray localRay = GetRayInAABBPrimitiveLocalSpace();
   // AnalyticPrimitive::Enum primitiveType = (AnalyticPrimitive::Enum) l_aabbCB.primitiveType;

  CSGPrimitive::Enum primitiveType = (CSGPrimitive::Enum) l_aabbCB.primitiveType;
  // AnalyticPrimitive::Enum primitiveType = AnalyticPrimitive::Enum::Sphere;

    float thit;
    ProceduralPrimitiveAttributes attr;
   // if (RayCSGIntersectionTest(localRay, primitiveType, thit, attr)) {
    if (alternativeCSG(localRay, thit, attr)) {
    PrimitiveInstancePerFrameBuffer aabbAttribute = g_AABBPrimitiveAttributes[l_aabbCB.instanceIndex];
        attr.normal = mul(attr.normal, (float3x3) aabbAttribute.localSpaceToBottomLevelAS);
        attr.normal = normalize(mul((float3x3) ObjectToWorld3x4(), attr.normal));

        ReportHit(thit, 0, attr);
    }
    //do a test based on the CSG tree.
    //look up ways to decompose the recursive structure of the CSG tree.
}
#endif // RAYTRACING_HLSL