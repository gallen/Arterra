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
RWStructuredBuffer<intermediateStructureData> genStructures;
uint bCOUNT_struct;
uint bSTART_struct;

void AppendStructureWithInfo(structureData st, uint2 info) {
    int appendInd;
    InterlockedAdd(counters[bCOUNT_struct], 1, appendInd);
    intermediateStructureData entry;
    entry.structure = st;
    entry.info = info;
    genStructures[bSTART_struct + appendInd] = entry;
}

void AppendStructure(structureData st) {
    AppendStructureWithInfo(st, PackStructInfo(0u, false, INVALID_OWNER_ID, 0u));
}

#endif