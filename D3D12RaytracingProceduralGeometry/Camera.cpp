#include "stdafx.h"
#include "Camera.h"
using namespace DirectX;


Camera::Camera(float aspectRatio) : aspectRatio(aspectRatio) {
    m_pos = { 0.0f, 10.0f, 1.0f, 0.0f };
    //m_at = { 0.0f, 0.0f, 0.0f, 1.0f };
    m_front = { 0.0f, 0.0f, -1.0f, 0.0f };

    m_at = XMVectorAdd(m_pos, m_front);

    m_direction = XMVector3Normalize(m_pos - m_at);
    m_up = { 0.0f, 1.0f, 0.0f, 0.0f };
    m_right = XMVector3Normalize(XMVector3Cross(m_up, m_direction));
    m_cameraUp = XMVector3Cross(m_direction, m_right);
   // m_up = XMVector3Normalize(XMVector3Cross(m_direction, m_right));
}

XMVECTOR Camera::getPosition()
{
    return this->m_pos;
}

XMVECTOR Camera::getDirection() {
    return this->m_direction;
}


void Camera::Update(ConstantBuffer<SceneConstantBuffer> &scene, ConstantBuffer<RasterSceneCB>& m_rasterConstantBuffer)
{
   // m_at = XMVectorAdd(m_eye, m_front);

    m_at = XMVectorAdd(m_front, m_pos);
    m_direction = XMVector3Normalize(m_at - m_pos);
    scene->cameraPosition = m_pos;

    float fovAngleY = 45.0f;
    XMMATRIX view = XMMatrixLookAtRH(m_pos, m_at, m_up);
    XMMATRIX proj = XMMatrixPerspectiveFovRH(XMConvertToRadians(fovAngleY), aspectRatio, 0.01f, 1000.0f);
    XMMATRIX viewProj = view * proj;
    XMVECTOR det;
    XMMATRIX viewInverse = XMMatrixInverse(&det, view);
    XMMATRIX projectionInverse = XMMatrixInverse(&det, proj);
    scene->projectionToWorld = XMMatrixInverse(&det, viewProj);

    m_rasterConstantBuffer->view = view;
    m_rasterConstantBuffer->projection = viewProj;
    // m_rasterConstantBuffer->mvp = viewProj;
    scene->view = view;
    scene->viewInverse = viewInverse;
    scene->projectionInverse = projectionInverse;
    scene->projection = proj;


}

void Camera::OnKeyDown(UINT8 key) {
    XMVECTOR perp_Pos = XMVector3Normalize(XMVector3Cross(m_front, m_cameraUp));

    switch (key) {
    case 'A':
        m_pos = XMVector3Transform(m_pos, XMMatrixTranslationFromVector(-speed * perp_Pos));
        //::cout << m_eye << std::endl;
        break;
    case 'S':
        m_pos = XMVector3Transform(m_pos, XMMatrixTranslationFromVector(-speed * m_front));
        break;
    case 'W':

        m_pos = XMVector3Transform(m_pos, XMMatrixTranslationFromVector(speed * m_front));
        break;
    case 'D':
        m_pos = XMVector3Transform(m_pos, XMMatrixTranslationFromVector(speed * perp_Pos));
        break;
    case 'I':
        speed += 1.0f;
        break;
    case 'O':
        speed -= 1.0f;
        if (speed < 0) {
            speed = 0.1;
        }
        break;
    }
    moving = true;
}

XMMATRIX Camera::getMVP() {
    float fovAngleY = 45.0f;

    XMMATRIX view = XMMatrixLookAtRH(m_pos, m_at, m_up);
    XMMATRIX proj = XMMatrixPerspectiveFovRH(XMConvertToRadians(fovAngleY), aspectRatio, 0.01f, 125.0f);
    XMMATRIX viewProj = view * proj;
    return viewProj;
}
void Camera::OnMouseMove(float dx, float dy) {

    if (firstMouse) // initially set to true
    {
        lastX = dx;
        lastY = dy;
        firstMouse = false;
    }
 
    lastX = dx;
    lastY = dy;
    float xoffset = -(dx);
    float yoffset = -(dy);
    float sensitivity = 0.05f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;


    yaw -= xoffset;
    pitch += yoffset;
   

    if (pitch > 89.0f) {
        pitch = 89.0f;
    }

    if (pitch < -89.0f) {
        pitch = -89.0f;
    }
    float pitchRad = XMConvertToRadians(pitch);
    float yawRad = XMConvertToRadians(yaw);
    XMVECTOR new_Front = { cos(pitchRad) * cos(yawRad), sin(pitchRad), cos(pitchRad) * sin(yawRad) };
    m_front = XMVector3Normalize(new_Front);
    moving = true;
}