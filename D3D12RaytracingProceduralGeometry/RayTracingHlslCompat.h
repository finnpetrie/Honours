
#ifndef RAYTRACINGHLSLCOMPAT_H
#define RAYTRACINGHLSLCOMPAT_H

//**********************************************************************************************
//
// RaytracingHLSLCompat.h
//
// A header with shared definitions for C++ and HLSL source files. 
//
//**********************************************************************************************

#ifdef HLSL
#include "util\HlslCompat.h"
#else
using namespace DirectX;

// Shader will use byte encoding to access vertex indices.
typedef UINT32 Vertex_Index;
#endif

// Number of metaballs to use within an AABB.
#define N_METABALLS 3    // = {3, 5}

// Limitting calculations only to metaballs a ray intersects can speed up raytracing
// dramatically particularly when there is a higher number of metaballs used. 
// Use of dynamic loops can have detrimental effects to performance for low iteration counts
// and outweighing any potential gains from avoiding redundant calculations.
// Requires: USE_DYNAMIC_LOOPS set to 1 to take effect.
#if N_METABALLS >= 5
#define USE_DYNAMIC_LOOPS 1
#define LIMIT_TO_ACTIVE_METABALLS 1
#else 
#define USE_DYNAMIC_LOOPS 0
#define LIMIT_TO_ACTIVE_METABALLS 0
#endif

#define N_FRACTAL_ITERATIONS 4      // = <1,...>

// PERFORMANCE TIP: Set max recursion depth as low as needed
// as drivers may apply optimization strategies for low recursion depths.
#define MAX_RAY_RECURSION_DEPTH 6    // ~ primary rays + reflections + shadow rays from reflected geometry.
#define PHOTON_COUNT 10000

struct ProceduralPrimitiveAttributes
{
    XMFLOAT3 normal;
};

struct RayPayload
{
    XMFLOAT4 color;
    UINT   recursionDepth;

    UINT seed;
};

struct PathTracingPayload {
    XMFLOAT4 colour;
    XMFLOAT3 energy;
    UINT recursionDepth;
    UINT randomSeed;
};


struct PhotonPayload {
    XMFLOAT4 position;
    XMFLOAT4 colour;
    float intensity;
    float probability;
    UINT recursionDepth;
};
struct ShadowRayPayload
{
    bool hit;

};
struct RandomResult
{
    XMFLOAT4 state;
    float value;
};
struct Photon {
    XMFLOAT4 position;
    XMFLOAT4 direction;
    XMFLOAT4 colour;
    XMFLOAT4 normal;
};

struct SceneConstantBuffer
{
    XMMATRIX projectionToWorld;
    XMMATRIX projection;
    XMMATRIX view;
    XMMATRIX viewInverse;
    XMMATRIX projectionInverse;
    XMVECTOR cameraPosition;
    XMVECTOR lightPosition;
    XMVECTOR lightAmbientColor;
    XMVECTOR lightDiffuseColor;
    UINT accumulatedFrames;
    UINT spp;
    UINT frameNumber;
    float lightPower;
    float rand1;
    float rand2;
    float rand3;
    float rand4;
    float    reflectance;
    float    elapsedTime;
    // Elapsed application time.
};

struct ComputeConstantBuffer {
    //tiling needs to know the position and direction of the camera.
    XMVECTOR cameraPos;
    XMVECTOR cameraDirection;
    XMMATRIX projectionToWorld;
};
// Attributes per primitive type.
struct PrimitiveConstantBuffer
{
    XMFLOAT4 albedo;
    float reflectanceCoef;
    float refractiveCoef;
    float diffuseCoef;
    float specularCoef;
    float specularPower;
    float stepScale;                      // Step scale for ray marching of signed distance primitives. 
                                          // - Some object transformations don't preserve the distances and 
                                          //   thus require shorter steps.
    XMFLOAT3 padding;
};

// Attributes per primitive instance.
struct PrimitiveInstanceConstantBuffer
{
    UINT instanceIndex;  
    UINT primitiveType; // Procedural primitive type
};

// Dynamic attributes per primitive instance.
struct PrimitiveInstancePerFrameBuffer
{
    XMMATRIX localSpaceToBottomLevelAS;   // Matrix from local primitive space to bottom-level object space.
    XMMATRIX bottomLevelASToLocalSpace;   // Matrix from bottom-level object space to local primitive space.
};

struct CSGNode {
    //0 = Union, 1 = Difference, 2 = Intersection.
    int boolValue;
    //pertains to the geometry described by the AABB encodings
    int geometry;
    int parentIndex;
    int leftNodeIndex;
    int rightNodeIndex;
    UINT myIndex;
    XMFLOAT3 translation;
    //to guarantee 16 bit byte alignment
    XMFLOAT3 padding2;

};

struct AltNode {
    int boolValue;
    int geometry;
    XMFLOAT3 padding;
};

struct intersectionInterval {
    float tmin;
    float tmax;
    bool hit;
    XMFLOAT3 normal;
};
struct CSGHit {
    UINT Index;
    float t;
};

struct Vertex
{
    XMFLOAT3 position;
    XMFLOAT3 normal;
};


// Ray types traced in this sample.
namespace RayType {
    enum Enum {
        Radiance = 0,   // ~ Primary, reflected camera/view rays calculating color for each hit.
        Shadow,         // ~ Shadow/visibility rays, only testing for occlusion
        Count
    };
}

namespace TraceRayParameters
{
    static const UINT InstanceMask = ~0;   // Everything is visible.
    namespace HitGroup {
        static const UINT Offset[RayType::Count] =
        {
            0, // Radiance ray
            1  // Shadow ray
        };
        static const UINT GeometryStride = RayType::Count;
    }
    namespace MissShader {
        static const UINT Offset[RayType::Count] =
        {
            0, // Radiance ray
            1  // Shadow ray
        };
    }
}

// From: http://blog.selfshadow.com/publications/s2015-shading-course/hoffman/s2015_pbs_physics_math_slides.pdf
static const XMFLOAT4 ChromiumReflectance = XMFLOAT4(0.549f, 0.556f, 0.554f, 1.0f);

static const XMFLOAT4 BackgroundColor = XMFLOAT4(0.8f, 0.9f, 1.0f, 1.0f);
static const XMFLOAT4 SkyColour = XMFLOAT4(0.4f, 0.4f, 1.0f, 1.0f);
//static const XMFLOAT4 BackgroundColor = XMFLOAT4(0.9, 1.0, 1.0, 1.0f);
//static const XMFLOAT4 BackgroundColor = XMFLOAT4(0.0, 0.0, 0.0, 1.0f);
static const float InShadowRadiance = 0.35f;
namespace CSGState {
    enum Enum {
        Classify = 0,
        SaveLeft = 1,
        GoToLeft = 2,
        GoToRight = 3,
        LoadLeft = 4,
        LoadRight = 5
    };
}
namespace AnalyticPrimitive {
    enum Enum {
        AABB  = 0,
        Plane,
        Spheres,
        Sphere,
        Cone,
        Ellipsoid,
        Hyperboloid,
        Cylinder,
        Paraboloid,
        CornellBack,
        CornellTop,
        CornellBottom,
        CornellLeft,
        CornellRight,
        CSG_Difference,
        CSG_Intersection,
        CSG_Union,
        Count
    };
}

namespace VolumetricPrimitive {
    enum Enum {
        Metaballs = 0,
        Count
    };
}

namespace SignedDistancePrimitive {
    enum Enum {
        MiniSpheres = 0,
        IntersectedRoundCube,
        SquareTorus,
        TwistedTorus,
        Cog,
        QuaternionJulia,
        Cylinder,
        Count
    };
}

namespace CSGPrimitive {
    enum Enum {
        CSG = 0,
        Count
    };
}

#endif // RAYTRACINGHLSLCOMPAT_H