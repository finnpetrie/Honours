#pragma once
#include "stdafx.h"
#include "Primitive.h"
#include "RaytracingSceneDefines.h"
#include "Camera.h"
#include "DeviceResources.h"
#include "DirectXRaytracingHelper.h"
#include "PlyFile.h"
class Scene
{
private:
		ConstantBuffer<SceneConstantBuffer> m_sceneCB;
		StructuredBuffer<PrimitiveInstancePerFrameBuffer> m_aabbPrimitiveAttributeBuffer;
		std::vector<D3D12_RAYTRACING_AABB> m_aabbs;

		D3DBuffer m_aabbBuffer;

		std::vector<Primitive> sceneObjects;
		Camera* camera;

		const float c_aabbWidth = 2;      // AABB width.
		const float c_aabbDistance = 2;   // Distance between AABBs.

public:
	uint32_t NUM_BLAS = 10;
	PlyFile* coordinates;
	PrimitiveConstantBuffer m_aabbMaterialCB[IntersectionShaderType::TotalPrimitiveCount];
	PrimitiveConstantBuffer m_planeMaterialCB;


	virtual void keyPress(UINT8 key);
	virtual void mouseMove(float dx, float dy);
	Scene(std::unique_ptr<DX::DeviceResources> &m_deviceResources);
	void Init(float m_aspectRatio);
	void UpdateAABBPrimitiveAttributes(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources);
	void Scene::BuildProceduralGeometryAABBs(std::unique_ptr<DX::DeviceResources> &m_deviceResources);

	void sceneUpdates(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources, bool m_animateLights = false, float time = 0);



	void CreateAABBPrimitiveAttributesBuffers(std::unique_ptr<DX::DeviceResources>& m_deviceResources);


	ConstantBuffer<SceneConstantBuffer>* getSceneBuffer();
	D3DBuffer* getAABB();
	StructuredBuffer<PrimitiveInstancePerFrameBuffer>* getPrimitiveAttributes();
	void CreateSpheres();
	void CreateGeometry();
};

