#ifndef STRUCTURE_PATH_TYPES
#define STRUCTURE_PATH_TYPES

static const uint MAX_BATCH_FRONTIER = 512u;
static const uint MAX_BATCH_STEPS = 16u;
static const int INVALID_VISITED = -1;
static const uint FRONTIER_COORD_MASK = 0x1FFFFFFFu;
static const uint INVALID_PATH = 0xFFFFFFFFu;
int batchSize;

struct pathEnds {
    int3 start;
    int3 end;
};

struct Port {
    float2 UV;
    uint2 sockets;
};

struct Socket {
    float connectChance;
    uint2 transitions;
};

struct Transition {
    uint structure;
    uint oppSocketFace;
};

struct settings {
    uint3 size;
    int minimumLOD;
    uint config;
};

struct checkData {
    float3 position;
    uint bounds;
};

struct SystemStructure {
    uint structureIndex;
    uint basePorts;
};

struct anchor {
    int3 pos;
    int system;
    int stct;
};

struct PathNode {
    uint id;
    int visited;
};

struct structureData {
    float3 structurePos;
    uint meta;
};

inline uint3 GetRot(uint meta) { return uint3((meta >> 2) & 0x3u, meta & 0x3u, (meta >> 4) & 0x3u); }
inline uint PackRot(uint3 rot) { return (rot.y & 0x3u) | ((rot.x & 0x3u) << 2) | ((rot.z & 0x3u) << 4); }
inline uint ConcatStct(uint rot, uint index) { return (rot << 24) | index & 0xFFFFFF; }
inline uint2 UnpackStct(uint stct) { return uint2((stct >> 24) & 0x3Fu, stct & 0xFFFFFFu); }

inline uint PackVisited(uint sysStructIndex, uint rotMeta, uint incomingFace, uint outgoingFace)
{
    return (sysStructIndex & 0xFFFFFu) | ((rotMeta & 0x3Fu) << 20) | ((incomingFace & 0x7u) << 26) | ((outgoingFace & 0x7u) << 29);
}

inline void UnpackVisited(uint pcked, out uint sysStructIndex, out uint rotMeta, out uint incomingFace, out uint outgoingFace)
{
    sysStructIndex = pcked & 0xFFFFFu;
    rotMeta = (pcked >> 20) & 0x3Fu;
    incomingFace = (pcked >> 26) & 0x7u;
    outgoingFace = (pcked >> 29) & 0x7u;
}

inline uint PackFrontierMeta(uint sysStructIndex, uint rotMeta)
{
    return (sysStructIndex & 0xFFFFFFu) | ((rotMeta & 0x3Fu) << 24);
}


inline uint RotMetaFromIndex(uint rotIndex, uint maxY, uint maxX)
{
    uint rotY = rotIndex % maxY;
    uint rotX = (rotIndex / maxY) % maxX;
    uint rotZ = rotIndex / (maxY * maxX);
    return PackRot(uint3(rotX, rotY, rotZ));
}

int3 DecodeBatchCoord(uint flat)
{
    int flatCoord = (int)flat;
    int plane = batchSize * batchSize;
    return int3(flatCoord % batchSize, (flatCoord / batchSize) % batchSize, flatCoord / plane);
}

uint2 MakeFrontierNode(uint sysStructIndex, uint rotMeta, int3 localCoord, uint objectFace)
{
    return uint2(PackFrontierMeta(sysStructIndex, rotMeta),
        (uint(indexFromCoord(localCoord)) & FRONTIER_COORD_MASK) | ((objectFace & 0x7u) << 29));
}

void DecodeFrontierNode(uint2 packed, out uint sysStructIndex, out uint rotMeta, out int3 localCoord, out uint objectFace)
{
    sysStructIndex = packed.x & 0xFFFFFFu;
    rotMeta = (packed.x >> 24) & 0x3Fu;
    objectFace = (packed.y >> 29) & 0x7u;
    localCoord = DecodeBatchCoord(packed.y & FRONTIER_COORD_MASK);
}

uint3 MakeSocketCap(uint sysStructInd, uint rotMeta, int3 socketPos, int socketFace, int pathInd) {
    return uint3(MakeFrontierNode(sysStructInd, rotMeta, socketPos, socketFace), pathInd);
}//

#endif