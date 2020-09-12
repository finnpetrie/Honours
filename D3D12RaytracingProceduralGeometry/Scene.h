#pragma once
#include "stdafx.h"
#include "Primitive.h"
#include "RaytracingSceneDefines.h"
#include "Camera.h"
#include "DeviceResources.h"
#include "DirectXRaytracingHelper.h"
#include "PlyFile.h"
#include "Geometry.h"
class Scene
{
private:

		ConstantBuffer<SceneConstantBuffer> m_sceneCB;
		StructuredBuffer<PrimitiveInstancePerFrameBuffer> m_aabbPrimitiveAttributeBuffer;
		std::vector<D3D12_RAYTRACING_AABB> m_aabbs;
		
		StructuredBuffer<CSGNode> csgTree;

		std::vector<Geometry> meshes;
		Camera* camera;

		

public:
	UINT frameCount = 0;

	D3DBuffer m_aabbBuffer;
	D3DBuffer m_vertexBuffer;
	D3DBuffer m_indexBuffer;

	std::vector<Vertex> totalVertices;
	std::vector<uint32_t> totalIndices;


	std::vector<Primitive> analyticalObjects;
	bool instancing = false;
	bool CSG = true;
	bool triangleInstancing = false;
	bool quatJulia = true;
	bool plane = true;
	const float c_aabbWidth = 2;      // AABB width.
	const float c_aabbDistance = 2;   // Distance between AABBs.
	uint32_t NUM_BLAS = 10;
	PlyFile* coordinates;
	PrimitiveConstantBuffer m_aabbMaterialCB[IntersectionShaderType::TotalPrimitiveCount];
	PrimitiveConstantBuffer m_planeMaterialCB;


	virtual void keyPress(UINT8 key);
	virtual void mouseMove(float dx, float dy);
	Scene(std::unique_ptr<DX::DeviceResources> &m_deviceResources);
	XMMATRIX GetMVP();
	void Init(float m_aspectRatio);
	void convertCSGToArray(int numberOfNodes, std::unique_ptr<DX::DeviceResources>& m_deviceResources);
	void UploadCompute(ComputeConstantBuffer& computeBuffer);
	void UpdateAABBPrimitiveAttributes(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources);
	void BuildMeshes(std::unique_ptr<DX::DeviceResources>& m_deviceResources);
	void Scene::BuildProceduralGeometryAABBs(std::unique_ptr<DX::DeviceResources> &m_deviceResources);


	void sceneUpdates(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources, ConstantBuffer<RasterSceneCB> &m_rasterConstantBuffer,  bool m_animateLights = false, float time = 0);
	void CreateCSGTree(std::unique_ptr<DX::DeviceResources>& m_deviceResources);
	void CreateAABBPrimitiveAttributesBuffers(std::unique_ptr<DX::DeviceResources>& m_deviceResources);


	void releaseResources();

	XMVECTOR getCameraDirection();

	XMVECTOR getCameraPosition();

	ConstantBuffer<SceneConstantBuffer>* getSceneBuffer();
	D3DBuffer* getAABB();
	StructuredBuffer<PrimitiveInstancePerFrameBuffer>* getPrimitiveAttributes();
	StructuredBuffer<CSGNode>* getCSGTree();
	void CreateSpheres();
	void CreateGeometry();
	UINT CreateBufferSRV(std::unique_ptr<DX::DeviceResources> m_deviceResources, D3DBuffer* buffer, uint32_t numElements, UINT elementSize);
};

