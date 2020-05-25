#include "stdafx.h"
#include "Scene.h"

void Scene::keyPress(UINT8 key)
{
    camera->OnKeyDown(key);

  
}

void Scene::mouseMove(float dx, float dy)
{
    camera->OnMouseMove(dx, dy);
}

Scene::Scene(std::unique_ptr<DX::DeviceResources> &m_deviceResources)
{
    //initialise the scene constant buffer
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_sceneCB.Create(device, frameCount, L"Scene Constant Buffer");
    m_sceneCB->accumulatedFrames = 0;
}



void Scene::Init(float m_aspectRatio)
{


    CreateSpheres();
    coordinates = new PlyFile("/Models/Head_AM/AM_Head.ply");
    // cooridnates->translateToOrigin(cooridnates->centroid());
     //because triangle geometry can't be stored in the procedural geometry BLAS, we add +1
    NUM_BLAS = coordinates->size() + 1;
        auto SetAttributes = [&](
            UINT primitiveIndex,
            const XMFLOAT4& albedo,
            float reflectanceCoef = 0.0f,
            float refractiveCoef = 0.0f,
            float diffuseCoef = 0.9f,
            float specularCoef = 0.7f,
            float specularPower = 50.0f,
            float stepScale = 1.0f
            )
        {
            auto& attributes = m_aabbMaterialCB[primitiveIndex];
            attributes.albedo = albedo;
            attributes.reflectanceCoef = reflectanceCoef;
            attributes.refractiveCoef = refractiveCoef;
            attributes.diffuseCoef = diffuseCoef;
            attributes.specularCoef = specularCoef;
            attributes.specularPower = specularPower;
            attributes.stepScale = stepScale;
        };

    

    m_planeMaterialCB = { XMFLOAT4(0.9f, 0.9f, 0.9f, 1.0f), 1, 0,  1, 0.4f, 50, 1 };

    // Albedos
    XMFLOAT4 green = XMFLOAT4(0.1f, 1.0f, 0.5f, 1.0f);
    XMFLOAT4 red = XMFLOAT4(1.0f, 0.5f, 0.5f, 1.0f);
    XMFLOAT4 yellow = XMFLOAT4(1.0f, 1.0f, 0.5f, 1.0f);

    UINT offset = 0;
    // Analytic primitives.
    {
        float X = 1.0f;
        using namespace AnalyticPrimitive;
        for (Primitive& p : sceneObjects) {
            PrimitiveConstantBuffer material = p.getMaterial();
            float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
            float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
            float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
            if (x < 0.1) {
                x = 0;
            }
            else if (y < 0.1) {
                y = 0;
            }
            // XMFLOAT4 alb =  XMFLOAT4(x, y, z, 0);
            // std::cout << p.getType() << std::endl; 
            SetAttributes(offset + p.getType(), material.albedo, material.reflectanceCoef, material.refractiveCoef, material.diffuseCoef, material.specularCoef, material.specularPower, material.stepScale);
            // SetAttributes(offset + Spheres, material.albedo, material.reflectanceCoef, material.refractiveCoef, material.diffuseCoef, material.specularCoef, material.specularPower, material.stepScale);
        }
            /*SetAttributes(offset + AABB, red);
            SetAttributes(offset + Spheres, ChromiumReflectance, 0.1f, 1.5, 0.6f,0.0f, 1);
            SetAttributes(offset + Cone, ChromiumReflectance, 0.1f, 1.5, 0.6, 0.0f, 1);
            SetAttributes(offset + Hyperboloid, red, 0.1f, 1.5, 0.6f, 0.0f, 1);
            SetAttributes(offset + Ellipsoid, ChromiumReflectance, 0.1f, 1.5, 0.6f, 0.0f, 1);
            SetAttributes(offset + Cylinder, ChromiumReflectance, 0.1f, 0, 0.6f, 0.0f, 1);
            */

            // offset += AnalyticPrimitive::Count;
    }


    {
        using namespace SignedDistancePrimitive;
        //SetAttributes(offset + SignedDistancePrimitive::FractalPyramid, ChromiumReflectance, 0.0f, 0.0f, 1.0f, 0.0f, 1);
    }

    // Setup camera.
    {
        camera = new Camera(m_aspectRatio);
    }

// Setup lights.
    {
        // Initialize the lighting parameters.
        XMFLOAT4 lightPosition;
        XMFLOAT4 lightAmbientColor;
        XMFLOAT4 lightDiffuseColor;

        lightPosition = XMFLOAT4(20.0f, 18.0f, -20.0f, 0.0f);
        m_sceneCB->lightPosition = XMLoadFloat4(&lightPosition);

        lightAmbientColor = XMFLOAT4(0.25f, 0.25f, 0.25f, 1.0f);
        m_sceneCB->lightAmbientColor = XMLoadFloat4(&lightAmbientColor);

        float d = 0.6f;
        lightDiffuseColor = XMFLOAT4(d, d, d, 1.0f);
        m_sceneCB->lightDiffuseColor = XMLoadFloat4(&lightDiffuseColor);
    }
}

void Scene::UpdateAABBPrimitiveAttributes(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources)
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    XMMATRIX mIdentity = XMMatrixIdentity();

    XMMATRIX mScale15y = XMMatrixScaling(1, 1.5, 1);
    XMMATRIX mScale15 = XMMatrixScaling(1.5, 1.5, 1.5);
    XMMATRIX mScaleHalf = XMMatrixScaling(0.2, 0.2, 0.2);
    XMMATRIX mScaleTiny = XMMatrixScaling(0.05, 0.05, 0.05);

    XMMATRIX mScale2 = XMMatrixScaling(2, 2, 2);
    XMMATRIX mScale3 = XMMatrixScaling(3, 3, 3);

    XMMATRIX mRotation = XMMatrixRotationY(-2 * animationTime);

    // Apply scale, rotation and translation transforms.
    // The intersection shader tests in this sample work with local space, so here
    // we apply the BLAS object space translation that was passed to geometry descs.
    auto SetTransformForAABB = [&](UINT primitiveIndex, XMMATRIX& mScale, XMMATRIX& mRotation)
    {
        XMVECTOR vTranslation =
            0.5f * (XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MinX))
                + XMLoadFloat3(reinterpret_cast<XMFLOAT3*>(&m_aabbs[primitiveIndex].MaxX)));
        XMMATRIX mTranslation = XMMatrixTranslationFromVector(vTranslation);

        XMMATRIX mTransform = mScale * mRotation * mTranslation;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].localSpaceToBottomLevelAS = mTransform;
        m_aabbPrimitiveAttributeBuffer[primitiveIndex].bottomLevelASToLocalSpace = XMMatrixInverse(nullptr, mTransform);
    };

    UINT offset = 0;
    // Analytic primitives.
    {
        using namespace AnalyticPrimitive;
        for (Primitive& p : sceneObjects) {
            SetTransformForAABB(offset + p.getType(), mScaleTiny, mRotation);
        }
        /*   SetTransformForAABB(offset + AABB, mScale15y, mIdentity);
            SetTransformForAABB(offset + Spheres, mScale15, mRotation);
            SetTransformForAABB(offset + Cone, mScaleHalf, mRotation);
            SetTransformForAABB(offset + Hyperboloid, mScaleHalf, mRotation);
            SetTransformForAABB(offset + Ellipsoid, mScale15, mRotation);
            */

        offset += AnalyticPrimitive::Count;
    }


}
void Scene::BuildProceduralGeometryAABBs(std::unique_ptr<DX::DeviceResources> &m_deviceResources)
{
    auto device = m_deviceResources->GetD3DDevice();

    // Set up AABBs on a grid.
    {
        XMINT3 aabbGrid = XMINT3(1, 1, 1);
        const XMFLOAT3 basePosition =
        {
            -(aabbGrid.x * c_aabbWidth + (aabbGrid.x - 1) * c_aabbDistance) / 2.0f,
            -(aabbGrid.y * c_aabbWidth + (aabbGrid.y - 1) * c_aabbDistance) / 2.0f,
            -(aabbGrid.z * c_aabbWidth + (aabbGrid.z - 1) * c_aabbDistance) / 2.0f,
        };

        XMFLOAT3 stride = XMFLOAT3(c_aabbWidth + c_aabbDistance, c_aabbWidth + c_aabbDistance, c_aabbWidth + c_aabbDistance);
        auto InitializeAABB = [&](auto& offsetIndex, auto& size)
        {
            return D3D12_RAYTRACING_AABB{
                basePosition.x + offsetIndex.x * stride.x,
                basePosition.y + offsetIndex.y * stride.y,
                basePosition.z + offsetIndex.z * stride.z,
                basePosition.x + offsetIndex.x * stride.x + size.x,
                basePosition.y + offsetIndex.y * stride.y + size.y,
                basePosition.z + offsetIndex.z * stride.z + size.z,
            };
        };
        //resize to number of actual geometry in the bottom level acceleration structure
        m_aabbs.resize(20);
        UINT offset = 0;

        // Analytic primitives.
        {
            using namespace AnalyticPrimitive;
            for (Primitive& p : sceneObjects) {
                //m_aabbs[offset + Spheres] = InitializeAABB(XMFLOAT3(rand() % 2, 1, rand() % 2), XMFLOAT3(6, 6, 6));
                //offset++;
                m_aabbs[offset + p.getType()] = InitializeAABB(p.getIndex(), p.getSize());
                offset++;
            }
            /*   m_aabbs[offset + AABB] = InitializeAABB(XMINT3(3, 0, 1), XMFLOAT3(2, 3, 2));
               m_aabbs[offset + Spheres] = InitializeAABB(XMFLOAT3(2.0f, 0, 0.0f), XMFLOAT3(3, 3, 3));
               m_aabbs[offset + Hyperboloid] = InitializeAABB(XMFLOAT3(0, 1, 0), XMFLOAT3(4, 4, 4));
              // m_aabbs[offset + Hyperboloid] = InitializeAABB(XMFLOAT3(0, -1, 0), XMFLOAT3(4, 4, 4));
               m_aabbs[offset + Ellipsoid] = InitializeAABB(XMFLOAT3(0, 0, 0), XMFLOAT3(4, 4, 4));
               */
               // offset += AnalyticPrimitive::Count;
        }
        {
            // using namespace SignedDistancePrimitive;

           //  m_aabbs[offset + FractalPyramid] = InitializeAABB(XMINT3(2, 0, 2), XMFLOAT3(6, 6, 6));

        }


        AllocateUploadBuffer(device, m_aabbs.data(), m_aabbs.size() * sizeof(m_aabbs[0]), &m_aabbBuffer.resource);
    }
}

void Scene::sceneUpdates(float animationTime, std::unique_ptr<DX::DeviceResources>& m_deviceResources, bool m_animateLights, float time)
{
    camera->Update(m_sceneCB);
    UpdateAABBPrimitiveAttributes(animationTime, m_deviceResources);

    if (m_animateLights)
    {
        float secondsToRotateAround = 8.0f;
        float angleToRotateBy = -360.0f * (time / secondsToRotateAround);
        XMMATRIX rotate = XMMatrixRotationY(XMConvertToRadians(angleToRotateBy));
        const XMVECTOR& prevLightPosition = m_sceneCB->lightPosition;
        //
        m_sceneCB->lightPosition = XMVector3Transform(prevLightPosition, rotate);

    }
}


void Scene::CreateAABBPrimitiveAttributesBuffers(std::unique_ptr<DX::DeviceResources>& m_deviceResources)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();
    m_aabbPrimitiveAttributeBuffer.Create(device, IntersectionShaderType::TotalPrimitiveCount, frameCount, L"AABB primitive attributes");
}


ConstantBuffer<SceneConstantBuffer>* Scene::getSceneBuffer()
{
    return &m_sceneCB;
}

D3DBuffer* Scene::getAABB()
{
    return &m_aabbBuffer;
}

StructuredBuffer<PrimitiveInstancePerFrameBuffer>* Scene::getPrimitiveAttributes() {
    return &m_aabbPrimitiveAttributeBuffer;
}


void Scene::CreateSpheres() {
    float X = 1.0f;
    for (int i = 0; i < 3; i++) {
        float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
        float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
        float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
        PrimitiveConstantBuffer sphere_b = { XMFLOAT4(x, y, z, 0), 1.5f, 1.5f, 1, 0.4f, 50, 1 };

        Primitive sphere(AnalyticPrimitive::Enum::Spheres, sphere_b, XMFLOAT3(0, 0, 0), XMFLOAT3(0.5
            , 0.5
            , 0.5
        ));
        sceneObjects.push_back(sphere);
    }
}

void Scene::CreateGeometry() {
    XMMATRIX nullQ = XMMatrixIdentity();

    XMMATRIX Q_Ellipse(1.0f / 1.5f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f / 2.0f, 0.0f,
        0.0f, 0.0f, 0.0f, -1.0f);


    XMMATRIX Q_paraboloid = { 1.0f / 2, 0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,  -0.1f,
                        0.0f, 0.0f, 1.0f / 1.5f, 0.0f,
                        0.0f, -0.1f, 0.0f, 0.0f };

    XMMATRIX Q_cone = { -1.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, -1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 0.0f };

    XMMATRIX Q_hyperboloid = { -1.0f ,0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,  0.0f,
                        0.0f, 0.0f, 1.0f , 0.0f,
                        0.0f, 0.0f, 0.0f, -0.09f };
    XMMATRIX Q_cylinder = { 0.0f,0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,  0.0f,
                    0.0f, 0.0f, 1.0f   , 0.0f,
                    0.0f, 0.0f, 0.0f, -0.5f };


    PrimitiveConstantBuffer sphere_b = { XMFLOAT4(0, 0.1, 0.0, 0), 1, 1.5f, 1, 0.4f, 50, 1 };
    Primitive sphere(AnalyticPrimitive::Enum::Spheres, sphere_b, XMFLOAT3(-1.0f, 0.0f, 0.0f), XMFLOAT3(6, 6, 6));
    PrimitiveConstantBuffer hy_b = { XMFLOAT4(0, 0.1, 0.1, 0), 1, 2.417f, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer ellipse_b = { XMFLOAT4(0.01, 0.01, 0.01, 0), 1, 1.5f, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer AABB_b = { XMFLOAT4(0.0, 0.0, 0.1, 0), 1, 2.417f, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer cylin_b = { XMFLOAT4(0.1, 0.0, 0.1, 0), 1, 1.5f, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer parab_b = { XMFLOAT4(0.0, 0.0, 0.05, 0), 1.0f, 1.333f, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer cone_b = { XMFLOAT4(0.05, 0.0, 0, 0), 1, 0.0, 1, 0.4f, 50, 1 };
    PrimitiveConstantBuffer CSG = { XMFLOAT4(0.0, 0.0, 0.0, 0), 1, 0, 1, 0.4f, 50, 1 };

    PrimitiveConstantBuffer e = { ChromiumReflectance, 0, 0, 1, 0.4f, 50, 1 };

    Primitive hyperboloid(AnalyticPrimitive::Enum::Hyperboloid, hy_b, XMFLOAT3(0.0f, 0.0f, 2.0f), XMFLOAT3(6, 6, 6));
    Primitive ellipsoid(AnalyticPrimitive::Enum::Ellipsoid, ellipse_b, XMFLOAT3(1, 0.0f, 0.0f), XMFLOAT3(6, 6, 6));
    Primitive AABB(AnalyticPrimitive::AABB, AABB_b, XMFLOAT3(3, 0.0f, 0.0f), XMFLOAT3(3, 3, 3));
    Primitive Cone(AnalyticPrimitive::Cone, cone_b, XMFLOAT3(4, 0.0f, 0.0f), XMFLOAT3(6, 6, 6));
    //Primitive Square(AnalyticPrimitive::AABB, c, XMFLOAT3(2.0f, 0.0f, 0.0f), XMFLOAT3(3, 3, 3));
    Primitive Paraboloid(AnalyticPrimitive::Paraboloid, parab_b, XMFLOAT3(3.0f, 0.0, -2.0f), XMFLOAT3(6, 6, 6));
    Primitive Cylinder(AnalyticPrimitive::Cylinder, cylin_b, XMFLOAT3(3.0f, 0.0, 2.0f), XMFLOAT3(6, 6, 6));
    Primitive difference(AnalyticPrimitive::Enum::CSG_Difference, CSG, XMFLOAT3(0.0f, 0.0f, -2.0f), XMFLOAT3(6, 6, 6));
    Primitive csg_union(AnalyticPrimitive::Enum::CSG_Union, CSG, XMFLOAT3(-2.0f, 0.0f, -2.0f), XMFLOAT3(6, 6, 6));
    Primitive intersection(AnalyticPrimitive::Enum::CSG_Intersection, CSG, XMFLOAT3(-1, 0.0f, -2.0f), XMFLOAT3(6, 6, 6));


    sceneObjects = { sphere, hyperboloid, ellipsoid, AABB, Cylinder, Paraboloid, Cone, difference, csg_union, intersection };

}