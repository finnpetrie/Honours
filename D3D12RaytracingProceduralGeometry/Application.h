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
#include <d3dcompiler.h>
#include <fstream>
#include "Primitive.h"
#include "Camera.h"
#include "PlyFile.h"
#include "Scene.h"
#include "AccelerationStructure.h"
#include "Pipeline.h"

//constant buffer for rasterisation

//intersection buffers

struct IBuffer
{
    ComPtr<ID3D12Resource> textureResource;
    D3D12_GPU_DESCRIPTOR_HANDLE uavGPUDescriptor;
    UINT uavDescriptorHeapIndex;
};

class Application : public DXSample
{
public:

    
    CComPtr<IDxcBlob> compileShaders(LPCWSTR fileName);
    CComPtr<IDxcBlob> compileShaderTwo(LPCWSTR fileName);
    void createRayTracingPipeline_Two();
    Application(UINT width, UINT height, std::wstring name);

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
    std::vector<float> fpsAverages;
    bool testing = false;
    bool drawRays = false;
    bool recordIntersections = true;
    // Constants.
    UINT photonCount = 1000;
    UINT NUM_BLAS = 100000;          // Triangle + AABB bottom-level AS.
    const float c_aabbWidth = 2;      // AABB width.
    const float c_aabbDistance = 2;   // Distance between AABBs.

    // DirectX Raytracing (DXR) attributes
    ComPtr<ID3D12Device5> m_dxrDevice;
    ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList;
    ComPtr<ID3D12StateObject> m_dxrStateObject;
    ComPtr<ID3D12StateObject>  m_photonMapStateObject;
    ComPtr<ID3D12PipelineState> m_computeStateObject;
    //Raster Pipeline
    ComPtr<ID3D12PipelineState> m_rasterState;
    // Root signatures
    ComPtr<ID3D12RootSignature> m_rasterRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature[LocalRootSignature::Type::Count];
    ComPtr<ID3D12RootSignature> m_computeRootSignature;

    ComPtr<ID3D12RootSignature> m_photonLocalRootSignature[LocalRootSignature::Type::Count];
    ComPtr<ID3D12RootSignature> m_photonGlobalRootSignature;;

    ComPtr<ID3D12Resource> m_missPhotonTable;
    UINT m_missPhotonTableStrideInBytes;
    ComPtr<ID3D12Resource> m_hitgroupPhotonTable;
    UINT m_hitgroupPhotonTableStrideInBytes;
    ComPtr<ID3D12Resource> m_photonRayGenTable;
  
    



    bool screenSpaceMap = false;
    IBuffer stagingResource;
    // Descriptors
    ComPtr<ID3D12DescriptorHeap> m_descriptorHeap;
    UINT m_descriptorsAllocated;
    UINT m_descriptorSize;
    
    IDxcBlob* m_rayGenLibrary;
    
    ComPtr<ID3D12Resource> photonCountBuffer;
    D3D12_GPU_DESCRIPTOR_HANDLE photonCountUavGPUDescriptor;
    UINT photonCountUavDescriptorHeapIndex;

    ConstantBuffer<ComputeConstantBuffer> m_computeConstantBuffer;
    ConstantBuffer<RasterSceneCB> m_rasterConstantBuffer;
    ComPtr<ID3D12Resource> photonStructBuffer;
    D3D12_GPU_DESCRIPTOR_HANDLE photonStructGPUDescriptor;
    UINT photonStructGpuHeapIndex;

    ComPtr<ID3D12Resource> photonBuffer;
    ComPtr<ID3D12Resource> photonCounter;
    D3D12_GPU_DESCRIPTOR_HANDLE photonCounterGpuDescriptor;
    UINT photonCounterDescriptorHeapIndex;

    ComPtr<ID3D12Resource> tiledPhotonMap;
    D3D12_GPU_DESCRIPTOR_HANDLE tiledPhotonMapGPUDescriptor;
    UINT tiledPhotonMapDescriptorHeapIndex;

    std::vector<IBuffer> intersectionBuffers;
    std::vector<IBuffer> geometryBuffers;

    //raster resources
    ComPtr<ID3D12Resource> intersectionBuffer;
  


    ComPtr<ID3D12Resource> rasterVertexBuffer;
    D3D12_VERTEX_BUFFER_VIEW rasterVertexView;
    ComPtr<ID3D12Resource> rasterConstant;
    RasterSceneCB rasterConstantBuffer;
    ComPtr<ID3D12DescriptorHeap> m_rasterHeap;
    UINT8* m_pCbvDataBegin;   
    
    
    Scene* scene;
    AccelerationStructure* acclerationStruct;
    Pipeline* pipeline;
    // Acceleration structure

    // Raytracing output
    ComPtr<ID3D12Resource> m_raytracingOutput;
    D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingOutputResourceUAVGpuDescriptor;
    UINT m_raytracingOutputResourceUAVDescriptorHeapIndex;
    //collection of intersection buffers for writing intersections.
    UINT intersectionIndex = 1;
    // Shader tables
    static const wchar_t* c_hitGroupNames_TriangleGeometry[RayType::Count];
    static const wchar_t* c_hitGroupNames_AABBGeometry[IntersectionShaderType::Count][RayType::Count];
    static const wchar_t* c_raygenShaderName;
    static const wchar_t* c_intersectionShaderNames[IntersectionShaderType::Count];
    static const wchar_t* c_closestHitShaderNames[GeometryType::Count];
    static const wchar_t* c_anyHitShaderNames[GeometryType::Count];
    static const wchar_t* c_missShaderNames[RayType::Count];

    static const wchar_t* c_photonMiss[RayType::Count];
    static const wchar_t* c_photon_rayGen;
    static const wchar_t* c_photon_closestHit[GeometryType::Count];

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
    bool m_animateLight;

	
    void RecreateD3D();
	void CopyIntersectionToCPU();
    void DoScreenSpacePhotonMapping();
    void DoTiling();
    void DoTiling(UINT tileX, UINT tileY, UINT tileDepth);
    void DoRaytracing();
   
    void CreatePhotonBuffer_2();

    void CreatePhotonBuffer();
    void CreateDeviceDependentResources();
    void CreateWindowSizeDependentResources();
	void CreateTiledPhotonMap();
	void CreatePhotonCountTest();
    void CreatePhotonCountBuffer();
    void ReleaseDeviceDependentResources();
    void ReleaseWindowSizeDependentResources();
    void CreateRaytracingInterfaces();
    void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);
    void CreatePhotonMappingRootSignatures();
    void CreateRootSignatures();
    void CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateHitGroupSubobjectsPhotonPass(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
	void CreateComputePhotonTilingRootSignature();
	void CreateRasterRootSignatures();
   
    void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline, ComPtr<ID3D12RootSignature>* rootSig);
    void CreateRasterisationPipeline();
	void CreatePhotonTilingComptuePassStateObject();
	void CreatePhotonMappingFirstPassStateObject();
    void CreateRaytracingPipelineStateObject();
    void CreateIntersectionVertexBuffer();
    void CreateDeferredGBuffer();
    void CreateIntersectionBuffers();
    void CreateAuxilaryDeviceResources();
    void CreateDescriptorHeap();
	void CreateBufferForIntersectionData();
	void CreateRasterisationBuffers();
    void CreateRasterConstantBuffer();
    void CreateRaytracingOutputResource();
    void CreateCountBuffer();
    void CreatePhotonStructuredBuffer();
    void CreateComputeConstantBuffer();
    void BuildGeometry();
    void DoRasterisation();
   
   
    void BuildPhotonShaderTable();

    void BuildShaderTables();
    void UpdateForSizeChange(UINT clientWidth, UINT clientHeight);
	void CreateStagingRenderTargetResource();
	void CopyIntersectionBufferToBackBuffer(UINT intersectionIndex);
    void CopyGBufferToBackBuffer();
	void CopyRaytracingOutputToBackbuffer();
    void CalculateFrameStats();
    UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
    UINT CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize);
};
