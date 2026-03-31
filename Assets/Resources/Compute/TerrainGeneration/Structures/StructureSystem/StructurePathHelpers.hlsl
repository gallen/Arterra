#ifndef STRUCTURE_PATH_HELPERS
#define STRUCTURE_PATH_HELPERS
#include "Assets/Resources/Compute/Utility/RotationTables.hlsl"
#include "Assets/Resources/Compute/TerrainGeneration/Structures/StructureSystem/SolveAnchorRotation.hlsl"
#include "Assets/Resources/Compute/TerrainGeneration/Structures/StructureSystem/StructurePathTypes.hlsl"

#ifndef HAS_STRUCT_SETTINGS
#define HAS_STRUCT_SETTINGS
StructuredBuffer<settings> _StructureSettings;
#endif 

StructuredBuffer<SystemStructure> _SystemStructures;
StructuredBuffer<Port> _StructurePorts; //6 ports per system structure
StructuredBuffer<Socket> _StructureSockets;
StructuredBuffer<Transition> _StructureTransitions;

bool IsInsideBatch(int3 coord) {
    int3 extent = int3(batchSize, batchSize, batchSize);
    return !any(coord < 0) && !any(coord >= extent);
}

float3 GetSocketBasePosition(uint3 size, float2 uv, uint baseFace) {
    float align = baseFace < 3u ? 0.0 : 1.0;

    if (baseFace == 0u || baseFace == 3u)
        return float3(align, uv.x, uv.y) * (float3)size;
    if (baseFace == 1u || baseFace == 4u)
        return float3(uv.x, align, uv.y) * (float3)size;
    return float3(uv.x, uv.y, align) * (float3)size;
}

uint GetObjectFaceFromBaseFace(uint baseFace, uint rotMeta)
{
    uint3 rot = GetRot(rotMeta);
    float3x3 rotMatrix = RotationLookupTable[rot.y][rot.x][rot.z];
    return DirToFaceIdx(mul(rotMatrix, FaceIdxToDir(baseFace)));
}

uint GetBaseFaceFromObjectFace(uint objectFace, uint rotMeta)
{
    uint3 rot = GetRot(rotMeta);
    float3x3 rotMatrix = RotationLookupTable[rot.y][rot.x][rot.z];
    return DirToFaceIdx(mul(transpose(rotMatrix), FaceIdxToDir(objectFace)));
}

uint GetRotatedPortMask(uint basePorts, uint rotMeta)
{
    uint rotatedMask = 0u;
    [unroll]
    for (uint baseFace = 0u; baseFace < 6u; baseFace++) {
        if ((basePorts & FaceBit(baseFace)) == 0u)
            continue;
        rotatedMask |= FaceBit(GetObjectFaceFromBaseFace(baseFace, rotMeta));
    }
    return rotatedMask;
}

int3 GetSocketOffset(uint sysStructIndex, uint rotMeta, uint objectFace)
{
    SystemStructure systemStructure = _SystemStructures[sysStructIndex];
    settings structureSettings = _StructureSettings[systemStructure.structureIndex];
    uint baseFace = GetBaseFaceFromObjectFace(objectFace, rotMeta);
    Port port = _StructurePorts[sysStructIndex * 6u + baseFace];

    uint3 rot = GetRot(rotMeta);
    float3x3 rotMatrix = RotationLookupTable[rot.y][rot.x][rot.z];
    float3 length = mul(rotMatrix, (float3)structureSettings.size);
    float3 newOrigin = min(length, 0.0);
    float3 baseSocket = GetSocketBasePosition(structureSettings.size, port.UV, baseFace);

    return int3(mul(rotMatrix, baseSocket) - newOrigin);
}

int3 GetSocketObjectPosition(int3 origin, uint sysStructIndex, uint rotMeta, uint objectFace) {
    return origin + GetSocketOffset(sysStructIndex, rotMeta, objectFace);
}

int3 GetOriginFromSocket(int3 socketObj, uint sysStructIndex, uint rotMeta, uint objectFace) {
    return socketObj - GetSocketOffset(sysStructIndex, rotMeta, objectFace);
}


bool CanConnectStructures(uint currentSysStructIndex, uint currentRotMeta, uint currentObjectFace,
    uint nextSysStructIndex, uint nextRotMeta, uint nextObjectFace)
{
    uint currentBaseFace = GetBaseFaceFromObjectFace(currentObjectFace, currentRotMeta);
    Port port = _StructurePorts[currentSysStructIndex * 6u + currentBaseFace];
    uint nextBaseFace = GetBaseFaceFromObjectFace(nextObjectFace, nextRotMeta);

    [loop][fastopt]
    for (uint socketIndex = port.sockets.x; socketIndex < port.sockets.y; socketIndex++) {
        Socket socket = _StructureSockets[socketIndex];
        for (uint transitionIndex = socket.transitions.x; transitionIndex < socket.transitions.y; transitionIndex++) {
            Transition transition = _StructureTransitions[transitionIndex];
            if (transition.structure != nextSysStructIndex)
                continue;
            if ((transition.oppSocketFace & FaceBit(nextBaseFace)) != 0u)
                return true;
        }
    }

    return false;
}


#endif