#ifndef APPEND_STRUCT
#define APPEND_STRUCT
#include "Assets/Resources/Compute/TerrainGeneration/Structures/StructureSystem/StructurePathTypes.hlsl"

#ifndef HAS_STRUCT_SETTINGS
#define HAS_STRUCT_SETTINGS
StructuredBuffer<settings> _StructureSettings;
#endif 

inline uint PackStctMeta(uint index, uint rotMeta) {
    settings set = _StructureSettings[index];
    return rotMeta
        | ((IsEnhanced(set.config) ? 1u : 0u) << 6)
        | (index << 7);
}

inline void UnpackStctMeta(uint meta, out uint index, out uint rotMeta) {
    index = meta >> 7;
    rotMeta = meta & 0x3Fu;
}

RWStructuredBuffer<int> counters;
RWStructuredBuffer<structureData> genStructures; //final output structures
uint bCOUNT_struct;
uint bSTART_struct;

void AppendStructure(structureData st) {
    uint index; uint rotMeta;
    UnpackStctMeta(st.meta, index, rotMeta);
    uint3 rot = GetRot(rotMeta);
    settings set = _StructureSettings[index];

    float3x3 rotMatrix = RotationLookupTable[rot.y][rot.x][rot.z];
    float3 length = abs(mul(rotMatrix, set.size));
    int3 origin = floor(st.structurePos);

    if(any(origin + length) < 0) 
        return;
    if(any(origin) > numVoxelsPerChunk)
        return;

    int appendInd;
    InterlockedAdd(counters[bCOUNT_struct], 1, appendInd);
    genStructures[bSTART_struct + appendInd] = st;
}

#endif