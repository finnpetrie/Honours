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

#pragma once

#include "DXSample.h"
#include "StepTimer.h"
#include "RaytracingSceneDefines.h"
#include "DirectXRaytracingHelper.h"
#include "PerformanceTimers.h"
#include <dxcapi.h>
#include <fstream>
#include "Primitive.h"

#include "PlyFile.h"
class D3D12RaytracingProceduralGeometry : public DXSample
{
public:
    CComPtr<IDxcBlob> compileShaders(LPCWSTR fileName);
    CComPtr<IDxcBlob> compileShaderTwo(LPCWSTR fileName);
    void createRayTracingPipeline_Two();
    D3D12RaytracingProceduralGeometry(UINT width, UINT height, std::wstring name);

    // IDeviceNotify
    virtual void OnDeviceLost() override;
    virtual void OnDeviceRestored() override;

    // Messages
    virtual void OnInit();
    virtual void OnKeyDown(UINT8 key);
    virtual void OnUpdate();
    virtual void OnRender();
    virtual void OnSizeChanged(UINT width, UINT height, bool minimized);
    virtual void OnDestroy();
    virtual void OnMouseMove(float x, float y);
    virtual IDXGISwapChain* GetSwapchain() { return m_deviceResources->GetSwapChain(); }

private:
    static const UINT FrameCount = 3;

    // Constants.
    UINT NUM_BLAS = 100000;          // Triangle + AABB bottom-level AS.
    const float c_aabbWidth = 2;      // AABB width.
    const float c_aabbDistance = 2;   // Distance between AABBs.
    float speed = 0.2f;
    // DirectX Raytracing (DXR) attributes
    ComPtr<ID3D12Device5> m_dxrDevice;
    ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList;
    ComPtr<ID3D12StateObject> m_dxrStateObject;

    // Root signatures
    ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature[LocalRootSignature::Type::Count];

    // Descriptors
    ComPtr<ID3D12DescriptorHeap> m_descriptorHeap;
    UINT m_descriptorsAllocated;
    UINT m_descriptorSize;

    // Raytracing scene
    ConstantBuffer<SceneConstantBuffer> m_sceneCB;
    StructuredBuffer<PrimitiveInstancePerFrameBuffer> m_aabbPrimitiveAttributeBuffer;
    std::vector<D3D12_RAYTRACING_AABB> m_aabbs;

    // Root constants
    PrimitiveConstantBuffer m_planeMaterialCB;
    PrimitiveConstantBuffer m_aabbMaterialCB[IntersectionShaderType::TotalPrimitiveCount];
    IDxcBlob* m_rayGenLibrary;
    // Geometry
    PlyFile* cooridnates;
    //ObjFile* mesh;
    std::vector<Primitive> sceneObjects;
    D3DBuffer m_indexBuffer;
    D3DBuffer m_vertexBuffer;
    D3DBuffer m_aabbBuffer;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    D3D12_INDEX_BUFFER_VIEW indexBufferView;


    // Acceleration structure
    ComPtr<ID3D12Resource> m_bottomLevelAS[BottomLevelASType::Count];
    ComPtr<ID3D12Resource> m_topLevelAS;

    // Raytracing output
    ComPtr<ID3D12Resource> m_raytracingOutput;
    D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingOutputResourceUAVGpuDescriptor;
    UINT m_raytracingOutputResourceUAVDescriptorHeapIndex;

    // Shader tables
    static const wchar_t* c_hitGroupNames_TriangleGeometry[RayType::Count];
    static const wchar_t* c_hitGroupNames_AABBGeometry[IntersectionShaderType::Count][RayType::Count];
    static const wchar_t* c_raygenShaderName;
    static const wchar_t* c_intersectionShaderNames[IntersectionShaderType::Count];
    static const wchar_t* c_closestHitShaderNames[GeometryType::Count];
    static const wchar_t* c_anyHitShaderNames[GeometryType::Count];

    static const wchar_t* c_missShaderNames[RayType::Count];

    ComPtr<ID3D12Resource> m_missShaderTable;
    UINT m_missShaderTableStrideInBytes;
    ComPtr<ID3D12Resource> m_hitGroupShaderTable;
    UINT m_hitGroupShaderTableStrideInBytes;
    ComPtr<ID3D12Resource> m_rayGenShaderTable;

    // Application state
    DX::GPUTimer m_gpuTimers[GpuTimers::Count];
    StepTimer m_timer;
    float m_animateGeometryTime;
    bool m_animateGeometry;
    bool m_animateCamera;
    bool m_animateLight;
    UINT lastX = 400;
    UINT lastY = 300;
    float pitch = 0.0f;
    float yaw = -90.0f;
    bool firstMouse = true;

    XMVECTOR m_front;
    XMVECTOR m_eye;
    XMVECTOR m_at;
    XMVECTOR m_up;
    XMVECTOR m_right;
    XMVECTOR m_direction;


    void UpdateCameraMatrices();
    void UpdateAABBPrimitiveAttributes(float animationTime);
	void CreateSpheres();
	void CreateGeometry();
    void InitializeScene();
    void RecreateD3D();
    void DoRaytracing();
    void CreateConstantBuffers();
    void CreateAABBPrimitiveAttributesBuffers();
    void CreateDeviceDependentResources();
    void CreateWindowSizeDependentResources();
    void ReleaseDeviceDependentResources();
    void ReleaseWindowSizeDependentResources();
    void CreateRaytracingInterfaces();
    void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);
    void CreateRootSignatures();
    void CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateRaytracingPipelineStateObject();
    void CreateAuxilaryDeviceResources();
    void CreateDescriptorHeap();
    void CreateRaytracingOutputResource();
    void BuildProceduralGeometryAABBs();
    void BuildGeometry();
    void Create_Vertex_Buffer(ID3D12Resource** ppResource);
    void Create_Index_Buffer(ID3D12Resource** ppResource);
    void BuildMeshes();
    void BuildPlaneGeometry();
    void BuildGeometryDescsForBottomLevelAS(std::array<std::vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count>& geometryDescs);
    template <class InstanceDescType, class BLASPtrType>
    void BuildBotomLevelASInstanceDescs(BLASPtrType *bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource);
    AccelerationStructureBuffers BuildBottomLevelAS(const std::vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDesc, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
    AccelerationStructureBuffers BuildTopLevelAS(AccelerationStructureBuffers bottomLevelAS[BottomLevelASType::Count], D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
    void BuildAccelerationStructures();
    void BuildShaderTables();
    void UpdateForSizeChange(UINT clientWidth, UINT clientHeight);
    void CopyRaytracingOutputToBackbuffer();
    void CalculateFrameStats();
    UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
    UINT CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize);
};
