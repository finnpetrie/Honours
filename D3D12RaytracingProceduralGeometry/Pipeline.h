#pragma once
#include "RayTracingHlslCompat.h"
#include "RaytracingSceneDefines.h"
#include "DirectXRaytracingHelper.h"
class Pipeline
{
private:

	static const wchar_t* c_hitGroupNames_TriangleGeometry[RayType::Count];
	static const wchar_t* c_hitGroupNames_AABBGeometry[IntersectionShaderType::Count][RayType::Count];
	static const wchar_t* c_raygenShaderName;
	static const wchar_t* c_intersectionShaderNames[IntersectionShaderType::Count];
	static const wchar_t* c_closestHitShaderNames[GeometryType::Count];
	static const wchar_t* c_anyHitShaderNames[GeometryType::Count];

	static const wchar_t* c_missShaderNames[RayType::Count];

	ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
	ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature[LocalRootSignature::Type::Count];

public:




	Pipeline(std::unique_ptr<DX::DeviceResources>& m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12StateObject> m_dxrStateObject);

	void SerializeAndCreateRaytracingRootSignature(std::unique_ptr<DX::DeviceResources>& m_deviceResources, D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);



	void CreateRootSignatures(std::unique_ptr<DX::DeviceResources>& m_deviceResouces);


	void CreateRaytracingPipelineStateObject(ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12StateObject> m_dxrStateObject);

	void CreateDxilLibrarySubobject(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);

	void CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);

	void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);

};

