#include "stdafx.h"

#include "AccelerationStructure.h"


void AccelerationStructure::BuildGeometryDescsForBottomLevelAS(array<vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count>& geometryDescs)
{
    // Mark the geometry as opaque. 
    // PERFORMANCE TIP: mark geometry as opaque whenever applicable as it can enable important ray processing optimizations.
    // Note: When rays encounter opaque geometry an any hit shader will not be executed whether it is present or not.
    //note only apply to plane
    D3D12_RAYTRACING_GEOMETRY_FLAGS geometryFlags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;

    // Triangle geometry desc
    {
        // Triangle bottom-level AS contains a single plane geometry.
        geometryDescs[BottomLevelASType::Triangle].resize(1);

        // Plane geometry
        auto& geometryDesc = geometryDescs[BottomLevelASType::Triangle][0];
        geometryDesc = {};
        geometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
        geometryDesc.Triangles.IndexBuffer = scene->m_indexBuffer.resource->GetGPUVirtualAddress();
        geometryDesc.Triangles.IndexCount = static_cast<UINT>(scene->m_indexBuffer.resource->GetDesc().Width) / sizeof(uint32_t);
        geometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
        geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
        geometryDesc.Triangles.VertexCount = static_cast<UINT>(scene->m_vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
        geometryDesc.Triangles.VertexBuffer.StartAddress = scene->m_vertexBuffer.resource->GetGPUVirtualAddress();
        geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
        geometryDesc.Flags = geometryFlags;

        //Loaded mesh geometry
    }

    // AABB geometry descz
    {
        D3D12_RAYTRACING_GEOMETRY_DESC aabbDescTemplate = {};
        aabbDescTemplate.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
        aabbDescTemplate.AABBs.AABBCount = 1;
        aabbDescTemplate.AABBs.AABBs.StrideInBytes = sizeof(D3D12_RAYTRACING_AABB);
        aabbDescTemplate.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;

        // One AABB primitive per geometry.
        geometryDescs[BottomLevelASType::AABB].resize(IntersectionShaderType::TotalPrimitiveCount, aabbDescTemplate);
        // geometryDescs[BottomLevelASType::AABB].resize(scene->analyticalObjects.size(), aabbDescTemplate);

         // Create AABB geometries. 
         // Having separate geometries allows of separate shader record binding per geometry.
         // In this sample, this lets us specify custom hit groups per AABB geometry.
        // for (UINT i = 0; i < IntersectionShaderType::TotalPrimitiveCount; i++)
        for (UINT i = 0; i < IntersectionShaderType::TotalPrimitiveCount; i++) {
            auto& geometryDesc = geometryDescs[BottomLevelASType::AABB][i];
            geometryDesc.AABBs.AABBs.StartAddress = scene->getAABB()->resource->GetGPUVirtualAddress() + i * sizeof(D3D12_RAYTRACING_AABB);
        }
    }
}

AccelerationStructureBuffers AccelerationStructure::BuildBottomLevelAS(const vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDescs, std::unique_ptr<DX::DeviceResources>& m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> bottomLevelAS;

    // Get the size requirements for the scratch and AS buffers.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bottomLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& bottomLevelInputs = bottomLevelBuildDesc.Inputs;
    bottomLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    bottomLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    bottomLevelInputs.Flags = buildFlags;
    bottomLevelInputs.NumDescs = static_cast<UINT>(geometryDescs.size());
    bottomLevelInputs.pGeometryDescs = geometryDescs.data();

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO bottomLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&bottomLevelInputs, &bottomLevelPrebuildInfo);
    ThrowIfFalse(bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    // Create a scratch buffer.
    AllocateUAVBuffer(device, bottomLevelPrebuildInfo.ScratchDataSizeInBytes, &scratch, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap equivalent). 
    // Default heap is OK since the application doesn’t need CPU read/write access to them. 
    // The resources that will contain acceleration structures must be created in the state D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, 
    // and must have resource flag D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both: 
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        AllocateUAVBuffer(device, bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes, &bottomLevelAS, initialResourceState, L"BottomLevelAccelerationStructure");
    }

    // bottom-level AS desc.
    {
        bottomLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
        bottomLevelBuildDesc.DestAccelerationStructureData = bottomLevelAS->GetGPUVirtualAddress();
    }

    // Build the acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers bottomLevelASBuffers;
    bottomLevelASBuffers.accelerationStructure = bottomLevelAS;
    bottomLevelASBuffers.scratch = scratch;
    bottomLevelASBuffers.ResultDataMaxSizeInBytes = bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return bottomLevelASBuffers;
}

template <class InstanceDescType, class BLASPtrType>
void AccelerationStructure::BuildBotomLevelASInstanceDescs(BLASPtrType* bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource, std::unique_ptr<DX::DeviceResources> &m_deviceResources)
{
    auto device = m_deviceResources->GetD3DDevice();

    vector<InstanceDescType> instanceDescs;
    // instanceDescs.resize(scene->NUM_BLAS + 1);
    instanceDescs.resize(scene->NUM_BLAS);

    // Width of a bottom-level AS geometry.
    // Make the plane a little larger than the actual number of primitives in each dimension.
    const XMUINT3 NUM_AABB = XMUINT3(700, 1, 700);
    const XMFLOAT3 fWidth = XMFLOAT3(
        NUM_AABB.x * scene->c_aabbWidth + (NUM_AABB.x - 1) * scene->c_aabbDistance,
        NUM_AABB.y * scene->c_aabbWidth + (NUM_AABB.y - 1) * scene->c_aabbDistance,
        NUM_AABB.z * scene->c_aabbWidth + (NUM_AABB.z - 1) * scene->c_aabbDistance);
    const XMVECTOR vWidth = XMLoadFloat3(&fWidth);


    // Bottom-level AS with a single plane.
    {

        if (!scene->triangleInstancing && !scene->instancing) {
            auto& instanceDesc = instanceDescs[BottomLevelASType::Triangle];
            instanceDesc = {};
            instanceDesc.InstanceMask = 1;
            instanceDesc.InstanceContributionToHitGroupIndex = 0;
            instanceDesc.AccelerationStructure = bottomLevelASaddresses[BottomLevelASType::Triangle];

            // Calculate transformation matrix.

            XMVECTOR vBasePosition = vWidth * XMLoadFloat3(&XMFLOAT3(-0.35f, 7.35f, -0.35f));

            if (scene->instancing) {
                vBasePosition = vWidth * XMLoadFloat3(&XMFLOAT3(-0.35f, -1.0f, -0.35f));
            }

            XMMATRIX mScale;
            XMMATRIX mTransform;
            XMMATRIX mTranslation = XMMatrixTranslationFromVector(vBasePosition);

            if (scene->plane) {
                mScale = XMMatrixScaling(fWidth.x, fWidth.y, fWidth.z);
                mTransform = mScale + mTranslation;
            }
            else {

                mScale = XMMatrixScaling(10, 10, 10);
                mTransform = mScale;
            }


            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTransform);
        }
        else if(scene->triangleInstancing && scene->instancing) {
            for (int i = 0; i < scene->NUM_BLAS - 1; i++) {
                auto& instanceDesc = instanceDescs[BottomLevelASType::Triangle + i];
                instanceDesc = {};
                instanceDesc.InstanceMask = 1;

                // Set hit group offset to beyond the shader records for the triangle AABB.
                instanceDesc.InstanceContributionToHitGroupIndex = BottomLevelASType::Triangle * RayType::Count;
                instanceDesc.AccelerationStructure = bottomLevelASaddresses[(BottomLevelASType::Triangle)];
                /*float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
                 float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
                 float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));*/
                float x;
                float y;
                float z;

                Vertex_Ply coord = scene->coordinates->getPointAt(i);

                x = 1 * coord.location.x();
                y = 1 * coord.location.y();
                z = 1 * coord.location.z();
                XMMATRIX mScale = XMMatrixScaling(0.1, 0.1, 0.1);

                XMMATRIX mTranslation = XMMatrixTranslationFromVector(XMLoadFloat3(&XMFLOAT3(x, y, z)));
                XMMATRIX mTransform = mScale + mTranslation;

                XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTransform);
            }
        }
    }
        // Scale in XZ dimensions.
      //XMMATRIX mScale = XMMatrixScaling(fWidth.x, fWidth.y, fWidth.z);
      
    
    float X = 10;
    // Create instanced bottom-level AS with procedural geometry AABBs.
    // Instances share all the data, except for a transform.
    {
        //we only iterate over the n-1 bottom level acc. structs, since triangle geomtry is a struct of its own.
        if (!scene->triangleInstancing) {
            for (int i = 0; i < scene->NUM_BLAS - 1; i++) {
                auto& instanceDesc = instanceDescs[BottomLevelASType::AABB + i];
                instanceDesc = {};
                instanceDesc.InstanceMask = 1;

                // Set hit group offset to beyond the shader records for the triangle AABB.
                instanceDesc.InstanceContributionToHitGroupIndex = BottomLevelASType::AABB * RayType::Count;
                instanceDesc.AccelerationStructure = bottomLevelASaddresses[(BottomLevelASType::AABB)];
                /*float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
                 float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));
                 float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / X));*/
                float x;
                float y;
                float z;
                if (scene->instancing) {
                    Vertex_Ply coord = scene->coordinates->getPointAt(i);

                    x = 1 * coord.location.x();
                    y = 1 * coord.location.y();
                    z = 1 * coord.location.z();
                }
                else {
                    x = 0;
                    y = scene->c_aabbWidth / 2;
                    z = 0;
                }
                // Move all AABBS above the ground plane.
                XMMATRIX mTranslation = XMMatrixTranslationFromVector(XMLoadFloat3(&XMFLOAT3(x, y, z)));
                XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), mTranslation);
            }
        }
    }
    UINT64 bufferSize = static_cast<UINT64>(instanceDescs.size() * sizeof(instanceDescs[0]));
    AllocateUploadBuffer(device, instanceDescs.data(), bufferSize, &(*instanceDescsResource), L"InstanceDescs");
};

AccelerationStructureBuffers AccelerationStructure::BuildTopLevelAS(AccelerationStructureBuffers bottomLevelAS[BottomLevelASType::Count], std::unique_ptr<DX::DeviceResources> &m_deviceResources, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList,  D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> topLevelAS;

    // Get required sizes for an acceleration structure.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& topLevelInputs = topLevelBuildDesc.Inputs;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.Flags = buildFlags;
    topLevelInputs.NumDescs = scene->NUM_BLAS;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    AllocateUAVBuffer(device, topLevelPrebuildInfo.ScratchDataSizeInBytes, &scratch, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap equivalent). 
    // Default heap is OK since the application doesn’t need CPU read/write access to them. 
    // The resources that will contain acceleration structures must be created in the state D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, 
    // and must have resource flag D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both: 
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        AllocateUAVBuffer(device, topLevelPrebuildInfo.ResultDataMaxSizeInBytes, &topLevelAS, initialResourceState, L"TopLevelAccelerationStructure");
    }

    // Create instance descs for the bottom-level acceleration structures.
    ComPtr<ID3D12Resource> instanceDescsResource;
    {
        D3D12_RAYTRACING_INSTANCE_DESC instanceDescs[BottomLevelASType::Count] = {};
        D3D12_GPU_VIRTUAL_ADDRESS bottomLevelASaddresses[BottomLevelASType::Count] =
        {
            bottomLevelAS[0].accelerationStructure->GetGPUVirtualAddress(),
            bottomLevelAS[1].accelerationStructure->GetGPUVirtualAddress()
        };
        BuildBotomLevelASInstanceDescs<D3D12_RAYTRACING_INSTANCE_DESC>(bottomLevelASaddresses, &instanceDescsResource, m_deviceResources);
    }

    // Top-level AS desc
    {
        topLevelBuildDesc.DestAccelerationStructureData = topLevelAS->GetGPUVirtualAddress();
        topLevelInputs.InstanceDescs = instanceDescsResource->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
    }

    // Build acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers topLevelASBuffers;
    topLevelASBuffers.accelerationStructure = topLevelAS;
    topLevelASBuffers.instanceDesc = instanceDescsResource;
    topLevelASBuffers.scratch = scratch;
    topLevelASBuffers.ResultDataMaxSizeInBytes = topLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return topLevelASBuffers;
}

// Build acceleration structure needed for raytracing.
void AccelerationStructure::BuildAccelerationStructures(std::unique_ptr<DX::DeviceResources> &m_deviceResources,  ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, ComPtr<ID3D12StateObject> m_dxrStateObject)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    auto commandQueue = m_deviceResources->GetCommandQueue();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    // Build bottom-level AS.
    AccelerationStructureBuffers bottomLevelAS[BottomLevelASType::Count];
    array<vector<D3D12_RAYTRACING_GEOMETRY_DESC>, BottomLevelASType::Count> geometryDescs;
    {
        BuildGeometryDescsForBottomLevelAS(geometryDescs);

        // Build all bottom-level AS.
        for (UINT i = 0; i < BottomLevelASType::Count; i++)
        {
            bottomLevelAS[i] = BuildBottomLevelAS(geometryDescs[i], m_deviceResources, m_dxrDevice, m_dxrCommandList);
        }
    }

    // Batch all resource barriers for bottom-level AS builds.
    D3D12_RESOURCE_BARRIER resourceBarriers[BottomLevelASType::Count];
    for (UINT i = 0; i < BottomLevelASType::Count; i++)
    {
        resourceBarriers[i] = CD3DX12_RESOURCE_BARRIER::UAV(bottomLevelAS[i].accelerationStructure.Get());
    }
    commandList->ResourceBarrier(BottomLevelASType::Count, resourceBarriers);

    // Build top-level AS.
    AccelerationStructureBuffers topLevelAS = BuildTopLevelAS(bottomLevelAS, m_deviceResources, m_dxrDevice, m_dxrCommandList);

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();

    // Store the AS buffers. The rest of the buffers will be released once we exit the function.
    for (UINT i = 0; i < BottomLevelASType::Count; i++)
    {
        m_bottomLevelAS[i] = bottomLevelAS[i].accelerationStructure;
    }
    m_topLevelAS = topLevelAS.accelerationStructure;
}

AccelerationStructure::AccelerationStructure(std::unique_ptr<DX::DeviceResources>& m_deviceResources, Scene* scene, ComPtr<ID3D12Device5> m_dxrDevice, ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList, ComPtr<ID3D12StateObject> m_dxrStateObject)
{
   // m_deviceResources = m_deviceResources;
    //scene = scene;
    this->scene = scene;
    BuildAccelerationStructures(m_deviceResources, m_dxrDevice, m_dxrCommandList, m_dxrStateObject);
}

ComPtr<ID3D12Resource> AccelerationStructure::getTopLevel()
{
    return m_topLevelAS;
}

void AccelerationStructure::Reset() {
    ResetComPtrArray(&m_bottomLevelAS);
    m_topLevelAS.Reset();
}