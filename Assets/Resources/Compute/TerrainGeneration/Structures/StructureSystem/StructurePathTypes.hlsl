#ifndef STRUCTURE_PATH_TYPES
#define STRUCTURE_PATH_TYPES

const static uint MAX_TRANSITIONS_PER_NODE = 24u;
static const uint MAX_BATCH_FRONTIER = 1024u;
static const uint MAX_BATCH_STEPS = 12u;
static const int INVALID_VISITED = -1;
static const uint FRONTIER_COORD_MASK = 0x1FFFFFFFu;
static const uint INVALID_PATH = 0xFFFFFFFFu;
int numVoxelsPerChunk;
int oCellOffset;
int batchSize;

struct pathEnds {
    int3 start;
    int3 end;
};

struct StructurePort {
    int socketSystemId;
    float2 UV;
    int2 sockets;
};

struct PortSocketOption {
    float chance;
    int socketSystemId;
};

struct SocketPortTransitions {
    int2 range;
};

struct TransDeltas {
    int3 deltaPosition;
    int3 originDelta;
    int structMeta;
    int nextPort;
    int inputFace;
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
    uint socketAtlasStart;
};

struct anchor {
    int3 pos;
    int system;
    int stct;
};

struct PathNode {
    int id;
    int visited;
};

struct structureData {
    float3 structurePos;
    uint meta;
};

inline uint3 GetRot(uint meta) { return uint3((meta >> 2) & 0x3u, meta & 0x3u, (meta >> 4) & 0x3u); }
inline uint PackRot(uint3 rot) { return (rot.y & 0x3u) | ((rot.x & 0x3u) << 2) | ((rot.z & 0x3u) << 4); }
inline uint ConcatStct(uint rot, uint index) { return (rot << 24) | index & 0xFFFFFFu; }
inline uint2 UnpackStct(uint stct) { return uint2((stct >> 24) & 0x3Fu, stct & 0xFFFFFFu); }
inline uint RotMetaFromIndex(uint rotIndex, uint maxY, uint maxX)
{
    uint rotY = rotIndex % maxY;
    uint rotX = (rotIndex / maxY) % maxX;
    uint rotZ = rotIndex / (maxY * maxX);
    return PackRot(uint3(rotX, rotY, rotZ));
}

int3 DecodeBatchCoord(uint flat, int sideLength)
{
    return coordFromIndexManual(flat, (uint)sideLength);
}

uint EncodeCoordFace(uint flatCoord, uint objectFace)
{
    return (flatCoord & FRONTIER_COORD_MASK) | ((objectFace & 0x7u) << 29);
}

uint2 MakeFrontierNode(int3 localCoord, uint portIndex, uint objectFace)
{
    return uint2(portIndex, EncodeCoordFace(uint(indexFromCoord(localCoord)), objectFace));
}

void DecodeFrontierNode(uint2 packed, out int3 localCoord, out uint portIndex, out uint objectFace)
{
    portIndex = packed.x;
    objectFace = (packed.y >> 29) & 0x7u;
    localCoord = DecodeBatchCoord(packed.y & FRONTIER_COORD_MASK, batchSize);
}
 
void DecodeSocketCap(uint2 packed, out uint portIndex, out int3 localCoord, out uint objectFace) {
    portIndex = packed.x;
    objectFace = (packed.y >> 29) & 0x7u;
    localCoord = DecodeBatchCoord(packed.y & FRONTIER_COORD_MASK, numVoxelsPerChunk) + oCellOffset;
}

uint3 MakeSocketCap(uint portIndex, int3 socketPos, uint objectFace, int pathInd) {
    socketPos -= oCellOffset;
    return uint3(portIndex,
        EncodeCoordFace(indexFromCoordManual(socketPos, numVoxelsPerChunk), objectFace),
        pathInd
    );
}

#endif