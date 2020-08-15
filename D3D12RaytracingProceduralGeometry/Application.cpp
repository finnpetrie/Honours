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

#include "stdafx.h"
#include "Application.h"
#include "CompiledShaders\Raytracing.hlsl.h"
#include "DirectXTex.h"
#include <iostream>
#include <cstdlib>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
using namespace std;
using namespace DX;

// Shader entry points.
const wchar_t* Application::c_raygenShaderName = L"MyRaygenShader";
const wchar_t* Application::c_intersectionShaderNames[] =
{
    L"MyIntersectionShader_AnalyticPrimitive",
    L"MyIntersectionShader_VolumetricPrimitive",
    L"MyIntersectionShader_SignedDistancePrimitive",
       L"CSG_Intersection"
};
const wchar_t* Application::c_closestHitShaderNames[] =
{
    L"MyClosestHitShader_Triangle",
    L"MyClosestHitShader_AABB",
};

const wchar_t* Application::c_anyHitShaderNames[] =
{
        L"AnyHit_Triangle",
    L"AnyHit_AnalyticPrimitive",
};
const wchar_t* Application::c_missShaderNames[] =
{
    L"MyMissShader", L"MyMissShader_ShadowRay"
};

// Hit groups.
const wchar_t* Application::c_hitGroupNames_TriangleGeometry[] = 
{ 
    L"MyHitGroup_Triangle", L"MyHitGroup_Triangle_ShadowRay" 
};
const wchar_t* Application::c_hitGroupNames_AABBGeometry[][RayType::Count] = 
{
    { L"MyHitGroup_AABB_AnalyticPrimitive", L"MyHitGroup_AABB_AnalyticPrimitive_ShadowRay" },
    { L"MyHitGroup_AABB_VolumetricPrimitive", L"MyHitGroup_AABB_VolumetricPrimitive_ShadowRay" },
    { L"MyHitGroup_AABB_SignedDistancePrimitive", L"MyHitGroup_AABB_SignedDistancePrimitive_ShadowRay" },
    { L"MyHitGroup_AABB_CSG", L"MyHitGroup_AABB_CSG_ShadowRay" },

};



const wchar_t* Application::c_photon_rayGen = L"Photon_Ray_Gen";

const wchar_t* Application::c_photon_closestHit[] =
{
    L"ClosestHit_Photon_Triangle",
    L"ClosestHit_Photon_Procedural",
};

const wchar_t* Application::c_photonMiss[] =
{
    L"Photon_Miss", L"Photon_Shadow_Miss"
};


Application::Application(UINT width, UINT height, std::wstring name) :
    DXSample(width, height, name),
    m_raytracingOutputResourceUAVDescriptorHeapIndex(UINT_MAX),
    m_animateGeometryTime(0.0f),
    m_animateGeometry(true),
    m_animateLight(false),
    m_descriptorsAllocated(0),
    m_descriptorSize(0),
    m_missShaderTableStrideInBytes(UINT_MAX),
    m_hitGroupShaderTableStrideInBytes(UINT_MAX)
{
    UpdateForSizeChange(width, height);
}





void Application::OnInit()
{
    m_deviceResources = std::make_unique<DeviceResources>(
        DXGI_FORMAT_R8G8B8A8_UNORM,
        DXGI_FORMAT_UNKNOWN,
        FrameCount,
        D3D_FEATURE_LEVEL_11_0,
        // Sample shows handling of use cases with tearing support, which is OS dependent and has been supported since TH2.
        // Since the sample requires build 1809 (RS5) or higher, we don't need to handle non-tearing cases.
        DeviceResources::c_RequireTearingSupport,
        m_adapterIDoverride
        );
    m_deviceResources->RegisterDeviceNotify(this);
    m_deviceResources->SetWindow(Win32Application::GetHwnd(), m_width, m_height);
    m_deviceResources->InitializeDXGIAdapter();

    ThrowIfFalse(IsDirectXRaytracingSupported(m_deviceResources->GetAdapter()),
        L"ERROR: DirectX Raytracing is not supported by your OS, GPU and/or driver.\n\n");

    m_deviceResources->CreateDeviceResources();
    m_deviceResources->CreateWindowSizeDependentResources();

    //InitializeScene();
    scene = new Scene(m_deviceResources);
    scene->Init(m_aspectRatio);
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

}




// Create resources that depend on the device.
void Application::CreateDeviceDependentResources()
{
    CreateAuxilaryDeviceResources();

    // Initialize raytracing pipeline.

    // Create raytracing interfaces: raytracing device and commandlist.
    CreateRaytracingInterfaces();
    //pipeline = new Pipeline(m_deviceResources, m_dxrDevice, m_dxrStateObject);
    // Create root signatures for the shaders.
    CreatePhotonMappingRootSignatures();
    CreateRootSignatures();
    CreateComputePhotonTilingRootSignature();

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used during raytracing.
    CreateRaytracingPipelineStateObject();
    CreatePhotonTilingComptuePassStateObject();
    CreatePhotonMappingFirstPassStateObject();
    CreateRasterRootSignatures();
   // CreateRasterisationPipeline();
     //createRayTracingPipeline_Two();

    // Create a heap for descriptors.
    CreateDescriptorHeap();

   // CreateIntersectionVertexBuffer();
    CreateRasterisationBuffers();
    // Build geometry to be used in the sample.
    BuildGeometry();


    // Build raytracing acceleration structures from the generated geometry.
    acclerationStruct = new AccelerationStructure(m_deviceResources, scene, m_dxrDevice, m_dxrCommandList);
    // Create constant buffers for the geometry and the scene.
    //CreateConstantBuffers();

    // Create AABB primitive attribute buffers.
    scene->CreateAABBPrimitiveAttributesBuffers(m_deviceResources);
    scene->CreateCSGTree(m_deviceResources);
    scene->convertCSGToArray(10, m_deviceResources);
    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();
    BuildPhotonShaderTable();

    // Create an output 2D texture to store the raytracing result to.
    CreateRaytracingOutputResource();
}

void Application::SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig)
{
    auto device = m_deviceResources->GetD3DDevice();
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error), error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}


void Application::CreatePhotonMappingRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[4]; // Perfomance TIP: Order from most frequent to least frequent.
        //1 output texture, 1 read back buffer, MAX_RAY_DEPTH_OUTPUT
        //render view
        //1 output, 6 screen space maps, 3 photon g-buffers, 1 photon count buffer
        //1 output, 6 screen space maps, 1 photon structured buffer, 1 photon count
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 1);
        ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 2);
      //  ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 3);
        //ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1 + MAX_RAY_RECURSION_DEPTH + 2, 0);  // 1 output texture
        ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);  // 2 static index and vertex buffers.

        CD3DX12_ROOT_PARAMETER rootParameters[PhotonGlobalRoot::Slot::Count];
        rootParameters[PhotonGlobalRoot::Slot::OutputView].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[PhotonGlobalRoot::Slot::PhotonBuffer].InitAsDescriptorTable(1, &ranges[1]);
        rootParameters[PhotonGlobalRoot::Slot::PhotonCounter].InitAsDescriptorTable(1, &ranges[2]);
        rootParameters[PhotonGlobalRoot::Slot::AccelerationStructure].InitAsShaderResourceView(0);
        rootParameters[PhotonGlobalRoot::Slot::SceneConstant].InitAsConstantBufferView(0);
        rootParameters[PhotonGlobalRoot::Slot::AABBattributeBuffer].InitAsShaderResourceView(3);
        rootParameters[PhotonGlobalRoot::Slot::CSGTree].InitAsShaderResourceView(4); //yes will need this for CSG
        rootParameters[PhotonGlobalRoot::Slot::VertexBuffers].InitAsDescriptorTable(1, &ranges[3]);
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_photonGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables - this will be needed for the photon mapping process
    {
        // Triangle geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::Triangle::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(SizeOfInUint32(PrimitiveConstantBuffer), 1);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_photonLocalRootSignature[LocalRootSignature::Type::Triangle]);
        }

        // AABB geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::AABB::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(SizeOfInUint32(PrimitiveConstantBuffer), 1);
            rootParameters[RootSignatureSlots::GeometryIndex].InitAsConstants(SizeOfInUint32(PrimitiveInstanceConstantBuffer), 2);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_photonLocalRootSignature[LocalRootSignature::Type::AABB]);
        }
    }
}

void Application::CreateComputePhotonTilingRootSignature() {
    auto device = m_deviceResources->GetD3DDevice();
    CD3DX12_DESCRIPTOR_RANGE1 ranges[3];
    
    //6 photon buffers, 1 bucket, 1 counter, and 1 photon buffer
    //1 bucket, 1 buffer
    //ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);// 6 + 3, 0); //number of photon buffers + buckets for tiled rendering
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 1);
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 2);
    CD3DX12_ROOT_PARAMETER1 rootParameters[4];
    rootParameters[ComputeRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[ComputeRootSignatureParams::PhotonBuffer].InitAsDescriptorTable(1, &ranges[1]);

    rootParameters[ComputeRootSignatureParams::TiledPhotonMap].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[ComputeRootSignatureParams::ParamConstantBuffer].InitAsConstantBufferView(0);
    //
    //rootParameters[ComputeRootSignatureParams::ParamConstantBuffer].InitAsConstantBufferView(0);

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC computeRootSignatureDesc;
    computeRootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters, 0, nullptr);

    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3DX12SerializeVersionedRootSignature(&computeRootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error));
    ThrowIfFailed(device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&m_computeRootSignature)));

}
void Application::CreateRasterRootSignatures() {

    auto device = m_deviceResources->GetD3DDevice();


    CD3DX12_DESCRIPTOR_RANGE ranges[2];
    CD3DX12_ROOT_PARAMETER rootParameters[2];

    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0, 0, D3D12_DESCRIPTOR_RANGE_FLAG_DATA_STATIC);
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
    rootParameters[0].InitAsDescriptorTable(1, &ranges[0], D3D12_SHADER_VISIBILITY_VERTEX);
    rootParameters[1].InitAsUnorderedAccessView(0);
    D3D12_ROOT_SIGNATURE_FLAGS rootSignatureFlags =
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_PIXEL_SHADER_ROOT_ACCESS;

    CD3DX12_ROOT_SIGNATURE_DESC rasterRoot(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, rootSignatureFlags);

    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;
    ThrowIfFailed(D3D12SerializeRootSignature(&rasterRoot, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error));
    ThrowIfFailed(device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&m_rasterRootSignature)));

}


void Application::CreateRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[2]; // Perfomance TIP: Order from most frequent to least frequent.
        //1 output texture, 1 read back buffer, MAX_RAY_DEPTH_OUTPUT
        UINT uavSize;
        if (recordIntersections) {
            //usually 2
            uavSize = 2 + MAX_RAY_RECURSION_DEPTH;
        }
        else {
            uavSize = 1;
        }
        //output, photon-gbuffers, and three 1D photon buffers
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);
       // ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 1);// 1 output texture
        //ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 2);
        //ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 3);
      //  ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);  // 2 static index and vertex buffers.

        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignature::Slot::Count];
        rootParameters[GlobalRootSignature::Slot::OutputView].InitAsDescriptorTable(1, &ranges[0]);
       // rootParameters[GlobalRootSignature::Slot::PhotonCount].InitAsDescriptorTable(1, &ranges[1]);
        //rootParameters[GlobalRootSignature::Slot::PhotonBuffer].InitAsDescriptorTable(1, &ranges[2]);
        //rootParameters[GlobalRootSignature::Slot::PhotonCountBuffer].InitAsDescriptorTable(1, &ranges[3]);
        rootParameters[GlobalRootSignature::Slot::AccelerationStructure].InitAsShaderResourceView(0);
        rootParameters[GlobalRootSignature::Slot::SceneConstant].InitAsConstantBufferView(0);
        rootParameters[GlobalRootSignature::Slot::AABBattributeBuffer].InitAsShaderResourceView(3);
        rootParameters[GlobalRootSignature::Slot::CSGTree].InitAsShaderResourceView(4);
        rootParameters[GlobalRootSignature::Slot::VertexBuffers].InitAsDescriptorTable(1, &ranges[1]);
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_raytracingGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    {
        // Triangle geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::Triangle::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(SizeOfInUint32(PrimitiveConstantBuffer), 1);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::Triangle]);
        }

        // AABB geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::AABB::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::MaterialConstant].InitAsConstants(SizeOfInUint32(PrimitiveConstantBuffer), 1);
            rootParameters[RootSignatureSlots::GeometryIndex].InitAsConstants(SizeOfInUint32(PrimitiveInstanceConstantBuffer), 2);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::AABB]);
        }
    }
}

// Create raytracing device and command list.
void Application::CreateRaytracingInterfaces()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    ThrowIfFailed(device->QueryInterface(IID_PPV_ARGS(&m_dxrDevice)), L"Couldn't get DirectX Raytracing interface for the device.\n");
    ThrowIfFailed(commandList->QueryInterface(IID_PPV_ARGS(&m_dxrCommandList)), L"Couldn't get DirectX Raytracing interface for the command list.\n");
}

// DXIL library
// This contains the shaders and their entrypoints for the state object.
// Since shaders are not considered a subobject, they need to be passed in via DXIL library subobjects.
void Application::CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void *)g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);
    {
        lib->DefineExport(c_raygenShaderName);

        lib->DefineExports(c_closestHitShaderNames);
        lib->DefineExports(c_intersectionShaderNames);
        lib->DefineExports(c_missShaderNames);
        lib->DefineExports(c_anyHitShaderNames);

    }
    // Use default shader exports for a DXIL library/collection subobject ~ surface all shaders.
}

// Hit groups
// A hit group specifies closest hit, any hit and intersection shaders 
// to be executed when a ray intersects the geometry.
void Application::CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Triangle geometry hit groups
    {
        for (UINT rayType = 0; rayType < RayType::Count; rayType++)
        {
            auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
            if (rayType == RayType::Radiance)
            {
                hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::Triangle]);
                hitGroup->SetAnyHitShaderImport(c_anyHitShaderNames[GeometryType::Triangle]);
            }
            hitGroup->SetHitGroupExport(c_hitGroupNames_TriangleGeometry[rayType]);
            hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);
        }
    }

    // AABB geometry hit groups
    {
        // Create hit groups for each intersection shader.
        for (UINT t = 0; t < IntersectionShaderType::Count; t++)
            for (UINT rayType = 0; rayType < RayType::Count; rayType++)
            {
                auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
                hitGroup->SetIntersectionShaderImport(c_intersectionShaderNames[t]);
                if (rayType == RayType::Radiance)
                {
                    hitGroup->SetAnyHitShaderImport(c_anyHitShaderNames[GeometryType::AABB]);
                    hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::AABB]);
                }
                hitGroup->SetHitGroupExport(c_hitGroupNames_AABBGeometry[t][rayType]);
                hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
            }
    }
}


void Application::CreateHitGroupSubobjectsPhotonPass(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Triangle geometry hit groups
    {
        for (UINT rayType = 0; rayType < RayType::Count; rayType++)
        {
            auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
            if (rayType == RayType::Radiance)
            {
                hitGroup->SetClosestHitShaderImport(c_photon_closestHit[GeometryType::Triangle]);
               // hitGroup->SetAnyHitShaderImport(c_anyHitShaderNames[GeometryType::Triangle]);
            }
            hitGroup->SetHitGroupExport(c_hitGroupNames_TriangleGeometry[rayType]);
            hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);
        }
    }

    // AABB geometry hit groups
    {
        // Create hit groups for each intersection shader.
        for (UINT t = 0; t < IntersectionShaderType::Count; t++)
            for (UINT rayType = 0; rayType < RayType::Count; rayType++)
            {
                auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
                hitGroup->SetIntersectionShaderImport(c_intersectionShaderNames[t]);
                if (rayType == RayType::Radiance)
                {
                   // hitGroup->SetAnyHitShaderImport(c_anyHitShaderNames[GeometryType::AABB]);
                    hitGroup->SetClosestHitShaderImport(c_photon_closestHit[GeometryType::AABB]);
                }
                hitGroup->SetHitGroupExport(c_hitGroupNames_AABBGeometry[t][rayType]);
                hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
            }
    }
}


// Local root signature and shader association
// This is a root signature that enables a shader to have unique arguments that come from shader tables.
void Application::CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline,  ComPtr<ID3D12RootSignature> *rootSig)
{
    // Ray gen and miss shaders in this sample are not using a local root signature and thus one is not associated with them.

    // Hit groups
    // Triangle geometry
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(rootSig[LocalRootSignature::Type::Triangle].Get());
        // Shader association
        auto rootSignatureAssociation = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        rootSignatureAssociation->AddExports(c_hitGroupNames_TriangleGeometry);
    }

    // AABB geometry
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(rootSig[LocalRootSignature::Type::AABB].Get());
        // Shader association
        auto rootSignatureAssociation = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        for (auto& hitGroupsForIntersectionShaderType : c_hitGroupNames_AABBGeometry)
        {
            rootSignatureAssociation->AddExports(hitGroupsForIntersectionShaderType);
        }
    }
}

void Application::CreateRasterisationPipeline() {


    auto device = m_deviceResources->GetD3DDevice();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();
    auto commandList = m_deviceResources->GetCommandList();


    CComPtr<IDxcLibrary> library;
    HRESULT hr = DxcCreateInstance(CLSID_DxcLibrary, IID_PPV_ARGS(&library));
    ThrowIfFailed(hr);



    CComPtr<IDxcCompiler> compiler;
    hr = DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&compiler));
    ThrowIfFailed(hr);
    LPCWSTR shaderPath = L"/Users/endev/Documents/Honours/DirectX-Graphics-Samples-master/DirectX-Graphics-Samples-master/Samples/Desktop/D3D12Raytracing/src/D3D12RaytracingProceduralGeometry/shaders.hlsl";
  /*  LPCWSTR compatPath = L"/Users/endev/Documents/Honours/DirectX-Graphics-Samples-master/DirectX-Graphics-Samples-master/Samples/Desktop/D3D12Raytracing/src/D3D12RaytracingProceduralGeometry/RaytracingHlslCompat.h";

    CComPtr<IDxcIncludeHandler> includeHandler;
    ThrowIfFailed(library->CreateIncludeHandler(&includeHandler));
    CComPtr<IDxcBlob> includeSource;
    includeHandler->LoadSource(compatPath, &includeSource);
    */
    uint32_t codePage = CP_UTF8;
    CComPtr<IDxcBlobEncoding> sourceBlob;
    hr = library->CreateBlobFromFile(shaderPath, &codePage, &sourceBlob);

    CComPtr<IDxcOperationResult> result;
    hr = compiler->Compile(sourceBlob, shaderPath, L"VSMain", L"vs_6_0", NULL, 0, NULL, 0, NULL, &result);



    if (SUCCEEDED(hr)) {
        result->GetStatus(&hr);
    }

    if (FAILED(hr)) {
        if (result) {
            CComPtr<IDxcBlobEncoding> errorsBlob;
            hr = result->GetErrorBuffer(&errorsBlob);
            if (SUCCEEDED(hr) && errorsBlob) {
                const char* error = (const char*)errorsBlob->GetBufferPointer();
                wchar_t wtext[10000];
                std::mbstowcs(wtext, error, strlen(error) + 1);
                LPCWSTR ptr = wtext;
                OutputDebugString(wtext);
                ThrowIfFalse(false);
            }
        }
    }


    CComPtr<IDxcBlob> code;
    result->GetResult(&code);

    hr = compiler->Compile(sourceBlob, shaderPath, L"PSMain", L"ps_6_0", NULL, 0, NULL, 0, NULL, &result);
    if (SUCCEEDED(hr)) {
        result->GetStatus(&hr);
    }

    if (FAILED(hr)) {
        if (result) {
            CComPtr<IDxcBlobEncoding> errorsBlob;
            hr = result->GetErrorBuffer(&errorsBlob);
            if (SUCCEEDED(hr) && errorsBlob) {
                const char* error = (const char*)errorsBlob->GetBufferPointer();
                wchar_t wtext[10000];
                std::mbstowcs(wtext, error, strlen(error) + 1);
                LPCWSTR ptr = wtext;
                OutputDebugString(wtext);
                ThrowIfFalse(false);
            }
        }
    }

    CComPtr<IDxcBlob> psCode;
    result->GetResult(&psCode);


  /*  auto device = m_deviceResources->GetD3DDevice();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();
    auto commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;

    UINT compileFlags = D3DCOMPILE_DEBUG;
    std::wstring shaderPath = L"/Users/endev/Documents/Honours/DirectX-Graphics-Samples-master/DirectX-Graphics-Samples-master/Samples/Desktop/D3D12Raytracing/src/D3D12RaytracingProceduralGeometry/";
    ThrowIfFailed(D3DCompileFromFile((shaderPath + L"shaders.hlsl").c_str(), nullptr, nullptr, "VSMain", "vs_5_0", compileFlags, 0, &vertexShader, nullptr));
    ThrowIfFailed(D3DCompileFromFile((shaderPath + L"shaders.hlsl").c_str(), nullptr, nullptr, "PSMain", "ps_5_0", compileFlags, 0, &pixelShader, nullptr));
    */


    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };


    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = { inputElementDescs, _countof(inputElementDescs) };
    psoDesc.pRootSignature = m_rasterRootSignature.Get();
    psoDesc.VS.BytecodeLength = code->GetBufferSize();
    psoDesc.VS.pShaderBytecode = code->GetBufferPointer();
    //= CD3DX12_SHADER_BYTECODE(code.Get());
    psoDesc.PS.BytecodeLength = psCode->GetBufferSize();
    psoDesc.PS.pShaderBytecode = psCode->GetBufferPointer();
   // psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState.DepthEnable = FALSE;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.SampleDesc.Count = 1;

    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_rasterState)));
    ThrowIfFailed(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocator, m_rasterState.Get(), IID_PPV_ARGS(&commandList)));

    // Command lists are created in the recording state, but there is nothing
    // to record yet. The main loop expects it to be closed, so close it now.
    ThrowIfFailed(commandList->Close());
}


void Application::CreatePhotonTilingComptuePassStateObject() {
    auto device = m_deviceResources->GetD3DDevice();
    
    CComPtr<IDxcLibrary> library;
    HRESULT hr = DxcCreateInstance(CLSID_DxcLibrary, IID_PPV_ARGS(&library));
    ThrowIfFailed(hr);


  
    CComPtr<IDxcCompiler> compiler;
    hr = DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&compiler));
    ThrowIfFailed(hr);
    LPCWSTR shaderPath = L"/Users/endev/Documents/Honours/DirectX-Graphics-Samples-master/DirectX-Graphics-Samples-master/Samples/Desktop/D3D12Raytracing/src/D3D12RaytracingProceduralGeometry/PhotonTiling.hlsl";
    LPCWSTR compatPath = L"/Users/endev/Documents/Honours/DirectX-Graphics-Samples-master/DirectX-Graphics-Samples-master/Samples/Desktop/D3D12Raytracing/src/D3D12RaytracingProceduralGeometry/RaytracingHlslCompat.h";

    CComPtr<IDxcIncludeHandler> includeHandler;
    ThrowIfFailed(library->CreateIncludeHandler(&includeHandler));
    CComPtr<IDxcBlob> includeSource;
    includeHandler->LoadSource(compatPath, &includeSource);

    uint32_t codePage = CP_UTF8;
    CComPtr<IDxcBlobEncoding> sourceBlob;
    hr = library->CreateBlobFromFile(shaderPath, &codePage, &sourceBlob);

    CComPtr<IDxcOperationResult> result;
    hr = compiler->Compile(sourceBlob, shaderPath, L"main", L"cs_6_3", NULL, 0, NULL, 0, NULL, &result);

    if (SUCCEEDED(hr)) {
        result->GetStatus(&hr);
    }

    if (FAILED(hr)) {
        if (result) {
            CComPtr<IDxcBlobEncoding> errorsBlob;
            hr = result->GetErrorBuffer(&errorsBlob);
            if (SUCCEEDED(hr) && errorsBlob) {
                const char* error = (const char*)errorsBlob->GetBufferPointer();
                wchar_t wtext[10000];
                std::mbstowcs(wtext, error, strlen(error) + 1);
                LPCWSTR ptr = wtext;
               OutputDebugString(wtext);
               ThrowIfFalse(false);
            }
        }
    }

    
    CComPtr<IDxcBlob> code;
    result->GetResult(&code);
   /**
    ID3DBlob* shaderBlob = nullptr;
    ID3DBlob* errorBlob = nullptr;

    ComPtr<ID3DBlob> computeShader;
    ComPtr<ID3DBlob> error;
    UINT compileFlags = D3DCOMPILE_DEBUG;
    ThrowIfFailed(D3DCompileFromFile((shaderPath + L"PhotonTiling.hlsl").c_str(), nullptr, D3D_COMPILE_STANDARD_FILE_INCLUDE, "main", "cs_5_0", compileFlags, 0, &computeShader, &error));
    */
    D3D12_COMPUTE_PIPELINE_STATE_DESC tilingPhotonPipe = {};

    tilingPhotonPipe.pRootSignature = m_computeRootSignature.Get();
    tilingPhotonPipe.CS.BytecodeLength = code->GetBufferSize();
    tilingPhotonPipe.CS.pShaderBytecode = code->GetBufferPointer();
    // tilingPhotonPipe.CS = CD3DX12_SHADER_BYTECODE(computeShader.Get());

    ThrowIfFailed(device->CreateComputePipelineState(&tilingPhotonPipe, IID_PPV_ARGS(&m_computeStateObject)));

}
void Application::CreatePhotonMappingFirstPassStateObject() {
    // 1 - Pipeline config
    CD3DX12_STATE_OBJECT_DESC photonMapping{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };

    // DXIL library
    auto lib = photonMapping.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void*)g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);
   
    {
        lib->DefineExport(c_photon_rayGen);
       
        lib->DefineExports(c_photon_closestHit);
        lib->DefineExports(c_intersectionShaderNames);
        lib->DefineExports(c_photonMiss);
    }
    // Hit groups
    CreateHitGroupSubobjectsPhotonPass(&photonMapping);

    // Shader config
    // Defines the maximum sizes in bytes for the ray rayPayload and attribute structure.
    auto shaderConfig = photonMapping.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize = max(sizeof(PhotonPayload), sizeof(ShadowRayPayload));
    UINT attributeSize = sizeof(struct ProceduralPrimitiveAttributes);
    shaderConfig->Config(payloadSize, attributeSize);
    CreateLocalRootSignatureSubobjects(&photonMapping, m_photonLocalRootSignature);

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = photonMapping.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_photonGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = photonMapping.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    UINT maxRecursionDepth = MAX_RAY_RECURSION_DEPTH;
    pipelineConfig->Config(maxRecursionDepth);

    PrintStateObjectDesc(photonMapping);

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(photonMapping, IID_PPV_ARGS(&m_photonMapStateObject)), L"Couldn't create DirectX Raytracing state object.\n");

}

// Create a raytracing pipeline state object (RTPSO).
// An RTPSO represents a full set of shaders reachable by a DispatchRays() call,
// with all configuration options resolved, such as local signatures and other state.
void Application::CreateRaytracingPipelineStateObject()
{
    // Create 18 subobjects that combine into a RTPSO:
    // Subobjects need to be associated with DXIL exports (i.e. shaders) either by way of default or explicit associations.
    // Default association applies to every exported shader entrypoint that doesn't have any of the same type of subobject associated with it.
    // This simple sample utilizes default shader association except for local root signature subobject
    // which has an explicit association specified purely for demonstration purposes.
    // 1 - DXIL library
    // 8 - Hit group types - 4 geometries (1 triangle, 3 aabb) x 2 ray types (ray, shadowRay)
    // 1 - Shader config
    // 6 - 3 x Local root signature and association
    // 1 - Global root signature
    // 1 - Pipeline config
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };

    // DXIL library
    CreateDxilLibrarySubobject(&raytracingPipeline);

    // Hit groups
    CreateHitGroupSubobjects(&raytracingPipeline);

    // Shader config
    // Defines the maximum sizes in bytes for the ray rayPayload and attribute structure.
    auto shaderConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize = max(sizeof(RayPayload), sizeof(ShadowRayPayload));
    UINT attributeSize = sizeof(struct ProceduralPrimitiveAttributes);
    shaderConfig->Config(payloadSize, attributeSize);

    // Local root signature and shader association
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    CreateLocalRootSignatureSubobjects(&raytracingPipeline, m_raytracingLocalRootSignature);

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_raytracingGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    UINT maxRecursionDepth = MAX_RAY_RECURSION_DEPTH;
    pipelineConfig->Config(maxRecursionDepth);

    PrintStateObjectDesc(raytracingPipeline);

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)), L"Couldn't create DirectX Raytracing state object.\n");
}

void Application::CreateTiledPhotonMap() {

    auto device = m_deviceResources->GetD3DDevice();
    auto backBufferFormat = m_deviceResources->GetBackBufferFormat();
    {
        UINT photonsPerTile = 10000;
        UINT tiles = 20;
        auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(backBufferFormat, photonsPerTile, 20, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(device->CreateCommittedResource(&defaultHeap, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&tiledPhotonMap)));
        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle; 
        tiledPhotonMapDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, tiledPhotonMapDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC unorderedViewDescription = {};
        unorderedViewDescription.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        device->CreateUnorderedAccessView(tiledPhotonMap.Get(), nullptr, &unorderedViewDescription, uavDescriptorHandle);
        tiledPhotonMapGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), tiledPhotonMapDescriptorHeapIndex, m_descriptorSize);
    }
}
void Application::CreatePhotonCountTest() {
    auto device = m_deviceResources->GetD3DDevice();
    {
        auto uavDesc = CD3DX12_RESOURCE_DESC::Buffer(1 * sizeof(UINT), D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProps = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProps, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonCountBuffer)));

        CD3DX12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonCountUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonCountUavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};
        uavPhotonDesc.Buffer.NumElements = 1;
        uavPhotonDesc.Buffer.FirstElement = 0;
        uavPhotonDesc.Buffer.StructureByteStride = sizeof(UINT);
        uavPhotonDesc.Format = DXGI_FORMAT_UNKNOWN;
        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;

        device->CreateUnorderedAccessView(photonCountBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonCountUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonCountUavDescriptorHeapIndex, m_descriptorSize);
    }
}
/**
void Application::CreatePhotonCountBuffer()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    // Create the output resource. The dimensions and format should match the swap-chain.
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex1D(backbufferFormat, 1, 1, 1, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(
        &defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonCountBuffer)));
    NAME_D3D12_OBJECT(photonCountBuffer);


    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    photonCountUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonCountUavDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
    device->CreateUnorderedAccessView(photonCountBuffer.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    photonCountUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonCountUavDescriptorHeapIndex, m_descriptorSize);
}
*/
void Application::CreateRaytracingOutputResource()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    // Create the output resource. The dimensions and format should match the swap-chain.
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(backbufferFormat, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(
        &defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_raytracingOutput)));
    NAME_D3D12_OBJECT(m_raytracingOutput);


    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    m_raytracingOutputResourceUAVDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, m_raytracingOutputResourceUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_raytracingOutput.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    m_raytracingOutputResourceUAVGpuDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), m_raytracingOutputResourceUAVDescriptorHeapIndex, m_descriptorSize);
}


void Application::CreateCountBuffer() {
    auto device = m_deviceResources->GetD3DDevice();

    UINT size = 4;
    D3D12_RESOURCE_DESC desc = {};
    desc.Alignment = 0;
    desc.DepthOrArraySize = 1;
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.Height = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Width = (UINT64)size;

    auto heapProps = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);

    ThrowIfFailed(device->CreateCommittedResource(&heapProps, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonCountBuffer)));

    photonCountBuffer->SetName(L"CountingPhotonsBuffer");

    D3D12_CPU_DESCRIPTOR_HANDLE uavDesciptorHand;
    photonCounterDescriptorHeapIndex = AllocateDescriptor(&uavDesciptorHand, photonCounterDescriptorHeapIndex);

    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
    uavDesc.Format = DXGI_FORMAT_R32_TYPELESS;
    uavDesc.Buffer.NumElements = 1;
    uavDesc.Buffer.Flags = D3D12_BUFFER_UAV_FLAG_RAW;

    device->CreateUnorderedAccessView(photonCountBuffer.Get(), nullptr, &uavDesc, uavDesciptorHand);
    photonCounterGpuDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonCountUavDescriptorHeapIndex, m_descriptorSize);

}

void Application::CreatePhotonStructuredBuffer() {
    
    auto device = m_deviceResources->GetD3DDevice();
    auto backBufferFormat = m_deviceResources->GetBackBufferFormat();
    {
        //create counter




    }
    {
        //numelements *elementsize
        UINT64 size = sizeof(Photon);
        UINT64 numElements = 100;
        UINT64 bufferSize = numElements * size;
        auto uavDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);

        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonStructBuffer)));
        NAME_D3D12_OBJECT(photonStructBuffer);

        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonStructGpuHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonStructGpuHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};
        uavPhotonDesc.Buffer.NumElements = numElements;
        uavPhotonDesc.Buffer.FirstElement = 0;
        uavPhotonDesc.Buffer.StructureByteStride = size;
        uavPhotonDesc.Buffer.CounterOffsetInBytes = 0;
        uavPhotonDesc.Format = DXGI_FORMAT_UNKNOWN;
        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
        device->CreateUnorderedAccessView(photonStructBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonStructGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonStructGpuHeapIndex, m_descriptorSize);

       
    }
}
/**

void Application::CreatePhotonBuffer_2() {
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();
    {
        UINT size = 10001;
        auto uavDesc = CD3DX12_RESOURCE_DESC::Tex1D(backbufferFormat, size, 1, 1, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonBuffer)));
        NAME_D3D12_OBJECT(photonBuffer);
        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonUavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};

        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
        device->CreateUnorderedAccessView(photonBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonUavDescriptorHeapIndex, m_descriptorSize);
    }
}
void Application::CreatePhotonBuffer() {
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();
    {   //position buffer
    

        //want to have dimension = MAX_RAY_RECURSION
        //format DXGI_FORMAT_R32G32B32A32_FLOAT
        auto uavDesc = CD3DX12_RESOURCE_DESC::Tex1D(DXGI_FORMAT_R8G8B8A8_UNORM, 16000, 1, 1, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);


        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonBuffer)));
        NAME_D3D12_OBJECT(photonBuffer);
        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonUavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};
        
        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
        device->CreateUnorderedAccessView(photonBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonUavDescriptorHeapIndex, m_descriptorSize);


    }

    //colour buffer
    {
    //want to have dimension = MAX_RAY_RECURSION
    //format DXGI_FORMAT_R32G32B32A32_FLOAT
        auto uavDesc = CD3DX12_RESOURCE_DESC::Tex1D(DXGI_FORMAT_R8G8B8A8_UNORM, 16000, 1, 1, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);


        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonColourBuffer)));
        NAME_D3D12_OBJECT(photonColourBuffer);
        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonColourUavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};
        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
        device->CreateUnorderedAccessView(photonColourBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonColourUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonColourUavDescriptorHeapIndex, m_descriptorSize);
    }

    {
        auto uavDesc = CD3DX12_RESOURCE_DESC::Tex1D(DXGI_FORMAT_R8G8B8A8_UNORM, 16000, 1, 1, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);


        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&photonDirectionBuffer)));
        NAME_D3D12_OBJECT(photonDirectionBuffer);
        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        photonUavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, photonDirectionUavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavPhotonDesc = {};
        uavPhotonDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE1D;
        device->CreateUnorderedAccessView(photonDirectionBuffer.Get(), nullptr, &uavPhotonDesc, uavDescriptorHandle);
        photonDirectionUavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), photonDirectionUavDescriptorHeapIndex, m_descriptorSize);

    }
}

void Application::CreateIntersectionBuffers() {
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    //want to have dimension = MAX_RAY_RECURSION
    //format DXGI_FORMAT_R32G32B32A32_FLOAT
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_R8G8B8A8_UNORM, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    intersectionBuffers.clear();

    for (int i = 0; i < MAX_RAY_RECURSION_DEPTH; i++) {
        IBuffer intersectionBuffer = {};

        
        ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&intersectionBuffer.textureResource)));
        NAME_D3D12_OBJECT(intersectionBuffer.textureResource);
        intersectionBuffer.uavDescriptorHeapIndex = UINT_MAX;

        D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
        intersectionBuffer.uavDescriptorHeapIndex = AllocateDescriptor(&uavDescriptorHandle, intersectionBuffer.uavDescriptorHeapIndex);
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
        uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
        uavDesc.Texture2DArray.ArraySize = 1;
        device->CreateUnorderedAccessView(intersectionBuffer.textureResource.Get(), nullptr, &uavDesc, uavDescriptorHandle);
        intersectionBuffer.uavGPUDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), intersectionBuffer.uavDescriptorHeapIndex, m_descriptorSize);

        intersectionBuffers.push_back(intersectionBuffer);

    }

}*/
// Create a 2D output texture for raytracing.


void Application::CreateAuxilaryDeviceResources()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandQueue = m_deviceResources->GetCommandQueue();

    for (auto& gpuTimer : m_gpuTimers)
    {
        gpuTimer.RestoreDevice(device, commandQueue, FrameCount);
    }
}




void Application::CreateDescriptorHeap()
{
    auto device = m_deviceResources->GetD3DDevice();

    D3D12_DESCRIPTOR_HEAP_DESC descriptorHeapDesc = {};
    // Allocate a heap for (3 + RAY_DEPTH) descriptors:
    // 2 - vertex and index  buffer SRVs
    // 1 - raytracing output texture SRV
    // n - Interesection Buffers
    // 1 flat gbuffer
   // descriptorHeapDesc.NumDescriptors = 15 + MAX_RAY_RECURSION_DEPTH;
    //2 index/vertex buffers, 1 photon structured buffer, 1 photon count buffer, 1 output buffer
    //1 tiledPhotonMap
    //2 v, i buffers, 1 output, 1, 
    descriptorHeapDesc.NumDescriptors = 6;
    descriptorHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    descriptorHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    descriptorHeapDesc.NodeMask = 0;
    device->CreateDescriptorHeap(&descriptorHeapDesc, IID_PPV_ARGS(&m_descriptorHeap));
    NAME_D3D12_OBJECT(m_descriptorHeap);

    m_descriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
}

 
void Application::CreateRasterisationBuffers() {
    auto device = m_deviceResources->GetD3DDevice();
    
    D3D12_DESCRIPTOR_HEAP_DESC rasterCBVDesc = {};
    rasterCBVDesc.NumDescriptors = 1;
    rasterCBVDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    rasterCBVDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

    ThrowIfFailed(device->CreateDescriptorHeap(&rasterCBVDesc, IID_PPV_ARGS(&m_rasterHeap)));
    Vertex triangleVertices[] =
    {
        { { 0.0f, 0.25f * m_aspectRatio, 0.0f }, { 1.0f, 0.0f, 0.0f } },
        { { 0.25f, -0.25f * m_aspectRatio, 0.0f }, { 0.0f, 1.0f, 0.0f } },
        { { -0.25f, -0.25f * m_aspectRatio, 0.0f }, { 0.0f, 0.0f, 1.0f } }
    };

    const UINT bufferSize = sizeof(triangleVertices);

    ThrowIfFailed(device->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(bufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&rasterVertexBuffer)));


    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(rasterVertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, triangleVertices, sizeof(triangleVertices));
    rasterVertexBuffer->Unmap(0, nullptr);  

    rasterVertexView.BufferLocation = rasterVertexBuffer->GetGPUVirtualAddress();
    rasterVertexView.StrideInBytes = sizeof(Vertex);
    rasterVertexView.SizeInBytes = bufferSize;

    ThrowIfFailed(device->CreateCommittedResource(&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(1024 * 64),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&rasterConstant)));

    D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
    cbvDesc.BufferLocation = rasterConstant->GetGPUVirtualAddress();
    cbvDesc.SizeInBytes = (sizeof(RasterSceneCB) + 255) & ~255;
    device->CreateConstantBufferView(&cbvDesc, m_rasterHeap->GetCPUDescriptorHandleForHeapStart());

    rasterConstantBuffer.mvp = scene->GetMVP();
    ThrowIfFailed(rasterConstant->Map(0, &readRange, reinterpret_cast<void**>(&m_pCbvDataBegin)));
    memcpy(m_pCbvDataBegin, &rasterConstantBuffer, sizeof(rasterConstantBuffer));

    m_deviceResources->WaitForGpu();

}

// Build geometry used in the sample.
void Application::BuildGeometry()
{

    scene->BuildProceduralGeometryAABBs(m_deviceResources);
    scene->BuildMeshes(m_deviceResources);

    UINT descriptorIndexIB = CreateBufferSRV(&scene->m_indexBuffer, scene->totalIndices.size(), 0);
    UINT descriptorIndexVB = CreateBufferSRV(&scene->m_vertexBuffer, scene->totalVertices.size(), sizeof(scene->totalVertices[0]));
    ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1, L"Vertex Buffer descriptor index must follow that of Vertex_Index Buffer descriptor index");

}

// Build geometry descs for bottom-level AS.


void Application::BuildPhotonShaderTable() {
    auto device = m_deviceResources->GetD3DDevice();

    void* rayGenShaderID;
    void* missShaderIDs[RayType::Count];
    void* hitGroupShaderIDs_Triangle[RayType::Count];
    void* hitGroupShaderIDs_AABBGeometry[IntersectionShaderType::Count][RayType::Count];

    unordered_map<void*, wstring> shaderIdToStringMap;


    auto GetShaderIDs = [&](auto* stateObjectProperties)
    {
        rayGenShaderID = stateObjectProperties->GetShaderIdentifier(c_photon_rayGen);
        shaderIdToStringMap[rayGenShaderID] = c_photon_rayGen;

        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderIDs[i] = stateObjectProperties->GetShaderIdentifier(c_photonMiss[i]);
            shaderIdToStringMap[missShaderIDs[i]] = c_photonMiss[i];
        }
        for (UINT i = 0; i < RayType::Count; i++)
        {
            hitGroupShaderIDs_Triangle[i] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_TriangleGeometry[i]);
            shaderIdToStringMap[hitGroupShaderIDs_Triangle[i]] = c_hitGroupNames_TriangleGeometry[i];
        }
        for (UINT r = 0; r < IntersectionShaderType::Count; r++)
            for (UINT c = 0; c < RayType::Count; c++)
            {
                hitGroupShaderIDs_AABBGeometry[r][c] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_AABBGeometry[r][c]);
                shaderIdToStringMap[hitGroupShaderIDs_AABBGeometry[r][c]] = c_hitGroupNames_AABBGeometry[r][c];
            }
    };

    UINT shaderIDSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_photonMapStateObject.As(&stateObjectProperties));
        GetShaderIDs(stateObjectProperties.Get());
        shaderIDSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIDSize; // No root arguments

        ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"PhotonRayGenTable");
        rayGenShaderTable.push_back(ShaderRecord(rayGenShaderID, shaderRecordSize, nullptr, 0));
        rayGenShaderTable.DebugPrint(shaderIdToStringMap);
        m_photonRayGenTable = rayGenShaderTable.GetResource();
    }

    // Miss shader table.
    {
        UINT numShaderRecords = RayType::Count;
        UINT shaderRecordSize = shaderIDSize; // No root arguments

        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissPhotonTable");
        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderTable.push_back(ShaderRecord(missShaderIDs[i], shaderIDSize, nullptr, 0));
        }
        missShaderTable.DebugPrint(shaderIdToStringMap);
        m_missPhotonTableStrideInBytes = missShaderTable.GetShaderRecordSize();
        m_missPhotonTable = missShaderTable.GetResource();
    }

    // Hit group shader table.
    {
        UINT numShaderRecords = RayType::Count + IntersectionShaderType::TotalPrimitiveCount * RayType::Count;
        UINT shaderRecordSize = shaderIDSize + LocalRootSignature::MaxRootArgumentsSize();
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"PhotonHitGroupTable");

        // Triangle geometry hit groups.
        {
            LocalRootSignature::Triangle::RootArguments rootArgs;
            rootArgs.materialCb = scene->m_planeMaterialCB;

            for (auto& hitGroupShaderID : hitGroupShaderIDs_Triangle)
            {
                hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
            }
        }

        // AABB geometry hit groups.
        {
            LocalRootSignature::AABB::RootArguments rootArgs;
            UINT instanceIndex = 0;

            // Create a shader record for each primitive.
            for (UINT iShader = 0, instanceIndex = 0; iShader < IntersectionShaderType::Count; iShader++)
            {
                UINT numPrimitiveTypes = IntersectionShaderType::PerPrimitiveTypeCount(static_cast<IntersectionShaderType::Enum>(iShader));

                // Primitives for each intersection shader.
                for (UINT primitiveIndex = 0; primitiveIndex < numPrimitiveTypes; primitiveIndex++, instanceIndex++)
                {
                    //    rootArgs.materialCb = m_aabbMaterialCB[instanceIndex];
                    rootArgs.materialCb = scene->m_aabbMaterialCB[instanceIndex];
                    rootArgs.aabbCB.instanceIndex = instanceIndex;
                    rootArgs.aabbCB.primitiveType = primitiveIndex;

                    // Ray types.
                    for (UINT r = 0; r < RayType::Count; r++)
                    {
                        auto& hitGroupShaderID = hitGroupShaderIDs_AABBGeometry[iShader][r];
                        hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
                    }
                }
            }
        }
        hitGroupShaderTable.DebugPrint(shaderIdToStringMap);
        m_hitgroupPhotonTableStrideInBytes = hitGroupShaderTable.GetShaderRecordSize();
        m_hitgroupPhotonTable = hitGroupShaderTable.GetResource();
    }

}
// Build shader tables.
// This encapsulates all shader records - shaders and the arguments for their local root signatures.
void Application::BuildShaderTables()
{
    auto device = m_deviceResources->GetD3DDevice();

    void* rayGenShaderID;
    void* missShaderIDs[RayType::Count];
    void* hitGroupShaderIDs_TriangleGeometry[RayType::Count];
    void* hitGroupShaderIDs_AABBGeometry[IntersectionShaderType::Count][RayType::Count];

    // A shader name look-up table for shader table debug print out.
    unordered_map<void*, wstring> shaderIdToStringMap;

    auto GetShaderIDs = [&](auto* stateObjectProperties)
    {
        rayGenShaderID = stateObjectProperties->GetShaderIdentifier(c_raygenShaderName);
        shaderIdToStringMap[rayGenShaderID] = c_raygenShaderName;

        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderIDs[i] = stateObjectProperties->GetShaderIdentifier(c_missShaderNames[i]);
            shaderIdToStringMap[missShaderIDs[i]] = c_missShaderNames[i];
        }
        for (UINT i = 0; i < RayType::Count; i++)
        {
            hitGroupShaderIDs_TriangleGeometry[i] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_TriangleGeometry[i]);
            shaderIdToStringMap[hitGroupShaderIDs_TriangleGeometry[i]] = c_hitGroupNames_TriangleGeometry[i];
        }
        for (UINT r = 0; r < IntersectionShaderType::Count; r++)
            for (UINT c = 0; c < RayType::Count; c++)        
            {
                hitGroupShaderIDs_AABBGeometry[r][c] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_AABBGeometry[r][c]); 
                shaderIdToStringMap[hitGroupShaderIDs_AABBGeometry[r][c]] = c_hitGroupNames_AABBGeometry[r][c];
            }
    };

    // Get shader identifiers.
    UINT shaderIDSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIDs(stateObjectProperties.Get());
        shaderIDSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

     // RayGen shader table.
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIDSize; // No root arguments
        
        ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable" );
        rayGenShaderTable.push_back(ShaderRecord(rayGenShaderID, shaderRecordSize, nullptr, 0));
        rayGenShaderTable.DebugPrint(shaderIdToStringMap);
        m_rayGenShaderTable = rayGenShaderTable.GetResource();
    }
    
    // Miss shader table.
    {
        UINT numShaderRecords = RayType::Count;
        UINT shaderRecordSize = shaderIDSize; // No root arguments

        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissShaderTable");
        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderTable.push_back(ShaderRecord(missShaderIDs[i], shaderIDSize, nullptr, 0));
        }
        missShaderTable.DebugPrint(shaderIdToStringMap);
        m_missShaderTableStrideInBytes = missShaderTable.GetShaderRecordSize();
        m_missShaderTable = missShaderTable.GetResource();
    }

    // Hit group shader table.
    {
        UINT numShaderRecords = RayType::Count + IntersectionShaderType::TotalPrimitiveCount * RayType::Count;
        UINT shaderRecordSize = shaderIDSize + LocalRootSignature::MaxRootArgumentsSize();
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");

        // Triangle geometry hit groups.
        {
            LocalRootSignature::Triangle::RootArguments rootArgs;
            rootArgs.materialCb = scene->m_planeMaterialCB;

            for (auto& hitGroupShaderID : hitGroupShaderIDs_TriangleGeometry)
            {
                hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
            }
        }
      
        // AABB geometry hit groups.
        {
            LocalRootSignature::AABB::RootArguments rootArgs;
            UINT instanceIndex = 0;

            // Create a shader record for each primitive.
            for (UINT iShader = 0, instanceIndex = 0; iShader < IntersectionShaderType::Count; iShader++)
            {
                UINT numPrimitiveTypes = IntersectionShaderType::PerPrimitiveTypeCount(static_cast<IntersectionShaderType::Enum>(iShader));
                
                // Primitives for each intersection shader.
                for (UINT primitiveIndex = 0; primitiveIndex < numPrimitiveTypes; primitiveIndex++, instanceIndex++)
                {
                //    rootArgs.materialCb = m_aabbMaterialCB[instanceIndex];
                    rootArgs.materialCb =  scene->m_aabbMaterialCB[instanceIndex];
                    rootArgs.aabbCB.instanceIndex = instanceIndex;
                    rootArgs.aabbCB.primitiveType = primitiveIndex;
                    
                    // Ray types.
                    for (UINT r = 0; r < RayType::Count; r++)
                    {
                        auto& hitGroupShaderID = hitGroupShaderIDs_AABBGeometry[iShader][r];
                        hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, &rootArgs, sizeof(rootArgs)));
                    }
                }
            }
        }
        hitGroupShaderTable.DebugPrint(shaderIdToStringMap);
        m_hitGroupShaderTableStrideInBytes = hitGroupShaderTable.GetShaderRecordSize();
        m_hitGroupShaderTable = hitGroupShaderTable.GetResource();
    }
}

void Application::OnKeyDown(UINT8 key)
{
    scene->keyPress(key);
    switch(key){
    case 'G':
        m_animateGeometry = !m_animateGeometry;
        break;
    case 'L': 
        m_animateLight = !m_animateLight;
        break;
    case '0':
        intersectionIndex = 0;
        break;
    case '1':
        intersectionIndex = 1;
        break;
    case '2': 
        intersectionIndex = 2;
        break;
    case '3':
        intersectionIndex = 3;
        break;
    case '4': 
        intersectionIndex = 4;
        break;
    case '5':
        intersectionIndex = 5;
        break;
    }
}

void Application::OnMouseMove(float dx, float dy) {
    //pass to scene camera to handle mouse movement
    scene->mouseMove(dx, dy);
}

// Update frame-based values.
void Application::OnUpdate()
{
    m_timer.Tick();
    CalculateFrameStats();
    float elapsedTime = static_cast<float>(m_timer.GetElapsedSeconds());
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();
    auto prevFrameIndex = m_deviceResources->GetPreviousFrameIndex();
   
    scene->sceneUpdates(m_animateGeometryTime, m_deviceResources, m_animateLight, elapsedTime);

    //upload compute constants
    m_computeConstantBuffer->cameraDirection = scene->getCameraDirection();
    m_computeConstantBuffer->cameraPos = scene->getCameraPosition();
    m_computeConstantBuffer->projectionToWorld = XMMatrixInverse(nullptr, scene->GetMVP());
    //memcpy
    
    //scene->U(&m_computeConstantBuffer);
    if (false) {
        rasterConstantBuffer.mvp = scene->GetMVP();
        memcpy(m_pCbvDataBegin, &rasterConstantBuffer, sizeof(rasterConstantBuffer));
    }
    if (false)
    {
        m_animateGeometryTime += elapsedTime;
    }
}


void Application::DoRasterisation() {
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();
    auto viewPort = m_deviceResources->GetScreenViewport();
    auto scissorRect = m_deviceResources->GetScissorRect();
    auto rtv = m_deviceResources->GetRenderTargetView();
    auto renderTarget = m_deviceResources->GetRenderTarget();
    commandList->SetPipelineState(m_rasterState.Get());
    commandList->SetGraphicsRootSignature(m_rasterRootSignature.Get());
    
    ID3D12DescriptorHeap* ppHeaps[] = { m_rasterHeap.Get() };
    commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);

    commandList->SetGraphicsRootDescriptorTable(0, m_rasterHeap->GetGPUDescriptorHandleForHeapStart());
    commandList->RSSetViewports(1, &viewPort);
    commandList->RSSetScissorRects(1, &scissorRect);

   // commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

    //CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    //commandList->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
    m_deviceResources->SetRasterRenderTarget();
  //  const float clearColor[] = { 0.0f, 0.2f, 0.4f, 1.0f };
   // commandList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);
    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList->IASetVertexBuffers(0, 1, &rasterVertexView);
    commandList->DrawInstanced(3, 1, 0, 0);

    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

   // ThrowIfFailed(commandList->Close());
}

void Application::DoScreenSpacePhotonMapping()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    auto DispatchRays = [&](auto* raytracingCommandList, auto* stateObject, auto* dispatchDesc)
    {
        dispatchDesc->HitGroupTable.StartAddress = m_hitgroupPhotonTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = m_hitgroupPhotonTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = m_hitgroupPhotonTableStrideInBytes;
        dispatchDesc->MissShaderTable.StartAddress = m_missPhotonTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = m_missPhotonTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = m_missPhotonTableStrideInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_photonRayGenTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_photonRayGenTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        raytracingCommandList->SetPipelineState1(stateObject);

        m_gpuTimers[GpuTimers::Raytracing].Start(commandList);
        raytracingCommandList->DispatchRays(dispatchDesc);
        m_gpuTimers[GpuTimers::Raytracing].Stop(commandList);
    };

    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList)
    {
        descriptorSetCommandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
        // Set index and successive vertex buffer decriptor tables.
        commandList->SetComputeRootDescriptorTable(PhotonGlobalRoot::Slot::VertexBuffers, scene->m_indexBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(PhotonGlobalRoot::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(PhotonGlobalRoot::Slot::PhotonBuffer, photonUavGPUDescriptor);
        commandList->SetComputeRootDescriptorTable(PhotonGlobalRoot::Slot::PhotonCounter, photonCounterGpuDescriptor);
        commandList->SetComputeRootDescriptorTable(PhotonGlobalRoot::Slot::PhotonBuffer, photonStructGPUDescriptor);
        //  commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
        
       // commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::PhotonCountBuffer, photonCountUavGPUDescriptor);
         //  commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::PhotonBuffer, photonStructGPUDescriptor);
        
    };

    commandList->SetComputeRootSignature(m_photonGlobalRootSignature.Get());

    // Copy dynamic buffers to GPU.
    {
        scene->getSceneBuffer()->CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(PhotonGlobalRoot::Slot::SceneConstant, scene->getSceneBuffer()->GpuVirtualAddress(frameIndex));
        scene->getPrimitiveAttributes()->CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(PhotonGlobalRoot::Slot::AABBattributeBuffer, scene->getPrimitiveAttributes()->GpuVirtualAddress(frameIndex));

        if (scene->CSG) {
            scene->getCSGTree()->CopyStagingToGpu(frameIndex);
            commandList->SetComputeRootShaderResourceView(PhotonGlobalRoot::Slot::CSGTree, scene->getCSGTree()->GpuVirtualAddress(frameIndex));
        }
    }

    // Bind the heaps, acceleration structure and dispatch rays.  
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    SetCommonPipelineState(commandList);
    commandList->SetComputeRootShaderResourceView(PhotonGlobalRoot::Slot::AccelerationStructure, acclerationStruct->getTopLevel()->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_photonMapStateObject.Get(), &dispatchDesc);

    m_deviceResources->ExecuteCommandList();
    m_deviceResources->WaitForGpu();
    commandList->Reset(commandAllocator, nullptr);
}


void Application::DoTiling(UINT tileX, UINT tileY, UINT tileDepth) {
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    commandList->SetPipelineState(m_computeStateObject.Get());
    commandList->SetComputeRootSignature(m_computeRootSignature.Get());

    commandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
    commandList->SetComputeRootDescriptorTable(ComputeRootSignatureParams::OutputViewSlot, m_raytracingOutputResourceUAVGpuDescriptor);
    commandList->SetComputeRootDescriptorTable(ComputeRootSignatureParams::PhotonBuffer, photonStructGPUDescriptor);
    commandList->SetComputeRootDescriptorTable(ComputeRootSignatureParams::TiledPhotonMap, tiledPhotonMapGPUDescriptor);

    
    {
        m_computeConstantBuffer.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(ComputeRootSignatureParams::ParamConstantBuffer, m_computeConstantBuffer.GpuVirtualAddress(frameIndex));
    }

    commandList->Dispatch(tileX, tileY, tileDepth);

}

void Application::DoRaytracing()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    auto DispatchRays = [&](auto* raytracingCommandList, auto* stateObject, auto* dispatchDesc)
    {
        dispatchDesc->HitGroupTable.StartAddress = m_hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = m_hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = m_hitGroupShaderTableStrideInBytes;
        dispatchDesc->MissShaderTable.StartAddress = m_missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = m_missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = m_missShaderTableStrideInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        raytracingCommandList->SetPipelineState1(stateObject);

        m_gpuTimers[GpuTimers::Raytracing].Start(commandList);
        raytracingCommandList->DispatchRays(dispatchDesc);
        m_gpuTimers[GpuTimers::Raytracing].Stop(commandList);
    };

    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList)
    {
        descriptorSetCommandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
        // Set index and successive vertex buffer decriptor tables.
      //  commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::VertexBuffers, m_indexBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::VertexBuffers, scene->m_indexBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
       // commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::PhotonCount, photonCountUavGPUDescriptor);
        
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy dynamic buffers to GPU.
    {
        scene->getSceneBuffer()->CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(GlobalRootSignature::Slot::SceneConstant, scene->getSceneBuffer()->GpuVirtualAddress(frameIndex));
        scene->getPrimitiveAttributes()->CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::AABBattributeBuffer, scene->getPrimitiveAttributes()->GpuVirtualAddress(frameIndex));

        if (scene->CSG) {
            scene->getCSGTree()->CopyStagingToGpu(frameIndex);
            commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::CSGTree, scene->getCSGTree()->GpuVirtualAddress(frameIndex));
        }
     }

    // Bind the heaps, acceleration structure and dispatch rays.  
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    SetCommonPipelineState(commandList);
    commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::AccelerationStructure, acclerationStruct->getTopLevel()->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_dxrStateObject.Get(), &dispatchDesc);

    m_deviceResources->WaitForGpu();
}

// Update the application state with the new resolution.
void Application::UpdateForSizeChange(UINT width, UINT height)
{
    DXSample::UpdateForSizeChange(width, height);
}




void Application::CopyIntersectionBufferToBackBuffer(UINT intersectionIndex) {


    auto commandList = m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();


    
    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST);
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(intersectionBuffers[intersectionIndex].textureResource.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);
   


    commandList->CopyResource(renderTarget, intersectionBuffers[intersectionIndex].textureResource.Get());
 
    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(intersectionBuffers[intersectionIndex].textureResource.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);

    
  
}
// Copy the raytracing output to the backbuffer.
void Application::CopyRaytracingOutputToBackbuffer()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();

    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);

    commandList->CopyResource(renderTarget, m_raytracingOutput.Get());

    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);


}

// Create resources that are dependent on the size of the main window.
void Application::CreateWindowSizeDependentResources()
{
    auto device = m_deviceResources->GetD3DDevice();
    CreateRaytracingOutputResource();
    CreateCountBuffer();
    CreatePhotonStructuredBuffer();

    //for compute and final gathering stage
    CreateTiledPhotonMap();
    // CreatePhotonBuffer_2();
    if (recordIntersections) {
       //. CreateIntersectionBuffers();
    }
    //
    //CreatePhotonBuffer();
   // CreatePhotonCountTest();
    //CreatePhotonCountBuffer();
    //CreatePhotonStructuredBuffer();
    //CreatePhotonBuffer();
    scene->sceneUpdates(0, m_deviceResources);
    m_computeConstantBuffer.Create(device, FrameCount, L"ComputeConstants");
    m_computeConstantBuffer->cameraDirection = scene->getCameraDirection();
    m_computeConstantBuffer->cameraPos = scene->getCameraPosition();
    m_computeConstantBuffer->projectionToWorld = XMMatrixInverse(nullptr, scene->GetMVP());
    //memcpy
  
}

// Release resources that are dependent on the size of the main window.
void Application::ReleaseWindowSizeDependentResources()
{
    m_raytracingOutput.Reset();
    for (auto& I : intersectionBuffers) {
        I.textureResource.Reset();
        I.uavDescriptorHeapIndex = UINT_MAX;
    }
}

// Release all resources that depend on the device.
void Application::ReleaseDeviceDependentResources()
{
    for (auto& gpuTimer : m_gpuTimers)
    {
        gpuTimer.ReleaseDevice();
    }

    m_raytracingGlobalRootSignature.Reset();
    ResetComPtrArray(&m_raytracingLocalRootSignature);
       

    m_dxrDevice.Reset();
    m_dxrCommandList.Reset();
    m_dxrStateObject.Reset();

    m_raytracingGlobalRootSignature.Reset();
    ResetComPtrArray(&m_raytracingLocalRootSignature);

    m_descriptorHeap.Reset();
    m_descriptorsAllocated = 0;
    scene->releaseResources();

    acclerationStruct->Reset();
  

    m_raytracingOutput.Reset();
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;

    for (auto& I : intersectionBuffers) {
        I.textureResource.Reset();
        I.uavDescriptorHeapIndex = UINT_MAX;
    }
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();
}

void Application::RecreateD3D()
{
    // Give GPU a chance to finish its execution in progress.
    try
    {
        m_deviceResources->WaitForGpu();
    }
    catch (HrException&)
    {
        // Do nothing, currently attached adapter is unresponsive.
    }
    m_deviceResources->HandleDeviceLost();
}


void Application::CopyIntersectionToCPU() {
    auto queue = m_deviceResources->GetCommandQueue();
    auto device = m_deviceResources->GetD3DDevice();
    auto renderTarget = m_deviceResources->GetRenderTarget();
   
    
   /* ComPtr<ID3D12CommandAllocator> commandAlloc;
    ComPtr<ID3D12GraphicsCommandList> commandList;
    ComPtr<ID3D12Fence> fence;
    ThrowIfFailed(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&commandAlloc)));
    ThrowIfFailed(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAlloc.Get(), nullptr, IID_PPV_ARGS(&commandList)));
    ThrowIfFailed(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));

    UINT fenceValue = 0;
    queue->Signal(fence.Get(), fenceValue);
  
    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    D3D12_RESOURCE_BARRIER gBufferBarrier{
        CD3DX12_RESOURCE_BARRIER::Transition(intersectionBuffers[1].textureResource.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE)
    };
    commandList->ResourceBarrier(1, &gBufferBarrier);
    
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT bufferFootPrint = {};
    bufferFootPrint.Footprint.Width = m_width;
    bufferFootPrint.Footprint.Height = m_height;
    bufferFootPrint.Footprint.Depth = 1;
    bufferFootPrint.Footprint.RowPitch = sizeof(XMFLOAT4) * m_width;
    bufferFootPrint.Footprint.Format = DXGI_FORMAT_R8G8B8A8_UNORM;

    CD3DX12_TEXTURE_COPY_LOCATION copyDest(outputBuffer.Get(), bufferFootPrint);
    CD3DX12_TEXTURE_COPY_LOCATION copySrc(intersectionBuffers[1].textureResource.Get(), 0);

    commandList->CopyTextureRegion(&copyDest, 0, 0, 0, &copySrc, nullptr);

    D3D12_RESOURCE_BARRIER reTrans{
    CD3DX12_RESOURCE_BARRIER::Transition(intersectionBuffers[1].textureResource.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
    };
    commandList->ResourceBarrier(1, &reTrans);
    ThrowIfFailed(commandList->Close());
    queue->ExecuteCommandLists(1, CommandListCast(commandList.GetAddressOf()));
    m_deviceResources->WaitForGpu();


    XMFLOAT4* readBack;
    outputBuffer->Map(0, nullptr, reinterpret_cast<void**>(&readBack));
    
    std::vector<XMFLOAT4> values;
    for (int i = 0; i < m_width * m_height; i++) {
        values.push_back(readBack[i]);
    }

    D3D12_RANGE empty{ 0, 0 };
    outputBuffer->Unmap(0, &empty);
    
   */std::vector<float> pixelData;
    
     ScratchImage result;
     auto c = CaptureTexture(queue, m_raytracingOutput.Get(), false, result, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
      const DirectX::Image *image = result.GetImages();
    
      //auto pixels = result.GetPixels();

      //ThrowIfFailed(DirectX::SaveToDDSFile(result.GetImages(), result.GetImageCount(), result.GetMetadata(), DDS_FLAGS_NONE, L"File.DDS"));

      
}

// Render the scene.
void Application::OnRender()
{
    if (!m_deviceResources->IsWindowVisible())
    {
        return;
    }

    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    // Begin frame.
    m_deviceResources->Prepare();
   for (auto& gpuTimer : m_gpuTimers)
    {
        gpuTimer.BeginFrame(commandList);
    }

    DoScreenSpacePhotonMapping();
    DoTiling(1024, 720, 1);
    DoRaytracing();
    CopyRaytracingOutputToBackbuffer();  
    if (recordIntersections) {
        //CopyIntersectionToCPU();
     //  CopyIntersectionBufferToBackBuffer(0);
    }
    if (false) {
      //  m_deviceResources->Prepare();

        DoRasterisation();
    }
    // End frame.
    for (auto& gpuTimer : m_gpuTimers)
    {
        gpuTimer.EndFrame(commandList);
    }

    m_deviceResources->Present(D3D12_RESOURCE_STATE_PRESENT);
}

void Application::OnDestroy()
{
    // Let GPU finish before releasing D3D resources.
    m_deviceResources->WaitForGpu();
    OnDeviceLost();
}

// Release all device dependent resouces when a device is lost.
void Application::OnDeviceLost()
{
    ReleaseWindowSizeDependentResources();
    ReleaseDeviceDependentResources();
}

// Create all device dependent resources when a device is restored.
void Application::OnDeviceRestored()
{
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();
}

// Compute the average frames per second and million rays per second.
void Application::CalculateFrameStats()
{
    static int frameCnt = 0;
    static double prevTime = 0.0f;
    double totalTime = m_timer.GetTotalSeconds();

    frameCnt++;
  
    
    //has run for a total of 30 seconds, write the fps to a file, and stop.

    if (totalTime >= 30.0f && testing) {
        std::ofstream output_file("/fps_stats_triangle_10_million_1stw_division.txt");
        for (const auto& e : fpsAverages) output_file << e << "\n";
      //  double fps = frameCnt / totalTime;
        //output_file << fps << "\n";
        exit(0);

    }
    
    // Compute averages over one second period.
    if ((totalTime - prevTime) >= 1.0f)
    {
        double diff = (totalTime - prevTime);
        double fps = (frameCnt) / diff; // Normalize to an exact second.
        fpsAverages.push_back(fps);
        frameCnt = 0;
        prevTime = totalTime;
        float raytracingTime = static_cast<float>(m_gpuTimers[GpuTimers::Raytracing].GetElapsedMS());
        float MRaysPerSecond = NumMRaysPerSecond(m_width, m_height, raytracingTime);
        
        wstringstream windowText;
        windowText << setprecision(2) << fixed
            << L"    fps: " << fps 
            << L"    DispatchRays(): " << raytracingTime << "ms"
            << L"     ~Million Primary Rays/s: " << MRaysPerSecond
            << L"    GPU[" << m_deviceResources->GetAdapterID() << L"]: " << m_deviceResources->GetAdapterDescription();
        SetCustomWindowText(windowText.str().c_str());
    }
}

// Handle OnSizeChanged message event.
void Application::OnSizeChanged(UINT width, UINT height, bool minimized)
{
    if (!m_deviceResources->WindowSizeChanged(width, height, minimized))
    {
        return;
    }

    UpdateForSizeChange(width, height);

    ReleaseWindowSizeDependentResources();
    CreateWindowSizeDependentResources();
}

// Allocate a descriptor and return its index. 
// If the passed descriptorIndexToUse is valid, it will be used instead of allocating a new one.
UINT Application::AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse)
{
    auto descriptorHeapCpuBase = m_descriptorHeap->GetCPUDescriptorHandleForHeapStart();
    if (descriptorIndexToUse >= m_descriptorHeap->GetDesc().NumDescriptors)
    {
        ThrowIfFalse(m_descriptorsAllocated < m_descriptorHeap->GetDesc().NumDescriptors, L"Ran out of descriptors on the heap!" );
        descriptorIndexToUse = m_descriptorsAllocated++;
    }
    *cpuDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(descriptorHeapCpuBase, descriptorIndexToUse, m_descriptorSize);
    return descriptorIndexToUse;
}

// Create a SRV for a buffer.
UINT Application::CreateBufferSRV(D3DBuffer* buffer, uint32_t numElements, UINT elementSize)
{
    auto device = m_deviceResources->GetD3DDevice();

    // SRV
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements = numElements;
    if (elementSize == 0)
    {
        srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    }
    else
    {
        srvDesc.Format = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    UINT descriptorIndex = AllocateDescriptor(&buffer->cpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->cpuDescriptorHandle);
    buffer->gpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_descriptorSize);
    return descriptorIndex;
}
