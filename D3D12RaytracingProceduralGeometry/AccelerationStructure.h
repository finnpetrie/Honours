#pragma once
#include "Scene.h"
#include <vector>
using namespace std;
class AccelerationStructure
{

private:
	Scene* scene;

	ComPtr<ID3D12Resource> m_bottomLevelAS[BottomLevelASType::Count];
	ComPtr<ID3D12Resource> m_topLevelAS;

	void BuildGeometryDescsForBottomLevelAS(array<vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count>& geometryDescs);
	AccelerationStructureBuffers BuildBottomLevelAS(const vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDescs, std::unique_ptr<DX::DeviceResources>& m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
	
	AccelerationStructureBuffers BuildTopLevelAS(AccelerationStructureBuffers bottomLevelAS[BottomLevelASType::Count], std::unique_ptr<DX::DeviceResources>& m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);

	void BuildAccelerationStructures(std::unique_ptr<DX::DeviceResources>& m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, ComPtr<ID3D12StateObject> m_dxrStateObject);

public:




	AccelerationStructure(std::unique_ptr<DX::DeviceResources>& m_deviceResources, Scene* scene, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, ComPtr<ID3D12StateObject> m_dxrStateObject);

	
	ComPtr<ID3D12Resource> getTopLevel();

	void Reset();

	template<class InstanceDescType, class BLASPtrType>
	void BuildBotomLevelASInstanceDescs(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource, std::unique_ptr<DX::DeviceResources>& m_deviceResources);

};

