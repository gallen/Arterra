#ifndef STRUCTURE_PATH_TYPES
#define STRUCTURE_PATH_TYPES

const static uint MAX_TRANSITIONS_PER_NODE = 24u;
static const uint MAX_BATCH_FRONTIER = 864u;
static const uint MAX_BATCH_STEPS = 8u;
static const uint INVALID_VISITED = 0u;
static const uint FRONTIER_COORD_MASK = 0x1FFFFFFFu;
static const uint INVALID_PATH = 0xFFFFFFFFu;
static const uint VISITED_DIR_BIT = 0x80000000u;
static const uint VISITED_TRANSITION_MASK = 0x1FFFFFFu;
static const uint VISITED_STEP_SHIFT = 25u;
static const uint VISITED_STEP_MASK = 0x3Fu << VISITED_STEP_SHIFT;
static const uint VISITED_SEED_TRANSITION = VISITED_TRANSITION_MASK;
static const int3 NO_ENDS_COORD = int3(-32768, -32768, -32768);
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
    uint visited;
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

inline bool VisitedFromEnd(uint visited)
{
    return (visited & VISITED_DIR_BIT) != 0u;
}

inline uint VisitedTransitionIndex(uint visited)
{
    return visited & VISITED_TRANSITION_MASK;
}

inline uint EncodeVisited(uint transitionIndex, bool fromEnd, uint step)
{
    uint stepEnc = (63u - min(step, 63u)) & 0x3Fu;
    uint value = (transitionIndex & VISITED_TRANSITION_MASK) | (stepEnc << VISITED_STEP_SHIFT);
    if (fromEnd) value |= VISITED_DIR_BIT;
    return value;
}

int3 DecodeBatchCoord(uint flat, int sideLength)
{
    return coordFromIndexManual(flat, (uint)sideLength);
}

uint EncodeCoordFace(uint flatCoord, uint objectFace)
{
    return (flatCoord & FRONTIER_COORD_MASK) | ((objectFace & 0x7u) << 29);
}

uint MakeFrontierNode(int3 localCoord)
{
    return uint(indexFromCoord(localCoord));
}

int3 DecodeFrontierNode(uint packed)
{
    return DecodeBatchCoord(packed, batchSize);
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