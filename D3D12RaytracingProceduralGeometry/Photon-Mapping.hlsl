//first pass, trace rays from light source

//each bounce is rendered into a photon texture - intiail frame is the positions of the photons on the light source, and their random directions

//photons that bounce it space are terminated

//use a uniform grid for storing the photons.
/*One way to index the photons by grid cell is to sort them by
cell and then find the index of the first photon in each cell
using binary search.- bitonic merge sort*/
#ifndef PHOTON_HLSL
#define PHOTON_HLSL

#define HLSL
#include "RaytracingHlslCompat.h"
#include "ProceduralPrimitivesLibrary.hlsli"
#include "RaytracingShaderHelper.hlsli"

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
void ClosetHit_Photon_Procedural(inout RayPayload payload, in ProceduralPrimitiveAttributes attr) {


}

#endif // RAYTRACING_HLSL
