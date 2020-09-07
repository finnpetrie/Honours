#pragma once
#include "stdafx.h"
#include "RaytracingSceneDefines.h"

struct RasterSceneCB {
	//XMMATRIX mvp;
	XMMATRIX view;
	XMMATRIX projection;
};
class Camera
{
private:

	//camera spatial variables
	XMVECTOR m_pos;
	XMVECTOR m_front;
	XMVECTOR m_up;
	XMVECTOR m_at;
	XMVECTOR m_direction;
	XMVECTOR m_right;
	XMVECTOR m_cameraUp;

	XMMATRIX view;
	//camera input variables
	UINT lastX = 400;
	UINT lastY = 300;
	float pitch = 0.0f;
	float yaw = -90.0f;
	bool firstMouse = true;
	float speed = 0.2f;

	float aspectRatio;


public:
	bool moving = false;


	void Update(ConstantBuffer<SceneConstantBuffer>& scene, ConstantBuffer<RasterSceneCB> &m_rasterConstantBuffer);

	virtual void OnKeyDown(UINT8 key);
	XMMATRIX getMVP();
	virtual void OnMouseMove(float x, float y);

	Camera(float aspectRatio);
	XMVECTOR getPosition();

	XMVECTOR getDirection();
	

};

