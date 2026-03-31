#ifndef STRUCT_SOCKET_CONNECTION
#define STRUCT_SOCKET_CONNECTION
#include "Assets/Resources/Compute/Utility/Encodings.hlsl"
static const uint SOCKET_NEG_X = 0;
static const uint SOCKET_POS_X = 3;
static const uint SOCKET_NEG_Y = 1;
static const uint SOCKET_POS_Y = 4;
static const uint SOCKET_NEG_Z = 2;
static const uint SOCKET_POS_Z = 5;

static const uint SOCKET_MASK_NEG_X = 1u << SOCKET_NEG_X;
static const uint SOCKET_MASK_POS_X = 1u << SOCKET_POS_X;
static const uint SOCKET_MASK_NEG_Y = 1u << SOCKET_NEG_Y;
static const uint SOCKET_MASK_POS_Y = 1u << SOCKET_POS_Y;
static const uint SOCKET_MASK_NEG_Z = 1u << SOCKET_NEG_Z;
static const uint SOCKET_MASK_POS_Z = 1u << SOCKET_POS_Z;

static const uint SOCKET_USED_MASK = 0x3Fu;

RWStructuredBuffer<uint> socketUsage;
uint bSTART_sockets;

inline uint2 GetFaceDirs(uint data) { return uint2(data & 0x7u, (data >> 3u) & 0x7u); }
inline uint PackFaceDirs(uint2 dirs) { return (dirs.y << 3u) | (dirs.x & 0x7u); }

uint EncodePriority(uint distEnc, uint pref) {
    return (((pref + 1u) & 0x3u) << 29) | ((0x1FFFFFFFu - min(distEnc, 0x1FFFFFFFu)) & 0x1FFFFFFFu);
}

uint SocketUsageIndex(uint anchorIndex, uint face){
    return bSTART_sockets + anchorIndex * 6u + face;
}

uint OppositeFace(uint face) {
    return (face + 3u) % 6u;
}

uint FaceBit(uint face) {
    return 1u << face;
}

uint AxisToNegFace(uint axis)
{
    return axis == 0u ? SOCKET_NEG_X : (axis == 1u ? SOCKET_NEG_Y : SOCKET_NEG_Z);
}

uint AxisToPosFace(uint axis)
{
    return axis == 0u ? SOCKET_POS_X : (axis == 1u ? SOCKET_POS_Y : SOCKET_POS_Z);
}

uint FaceFromAxisDelta(int delta, uint axis)
{
    return delta < 0 ? AxisToNegFace(axis) : AxisToPosFace(axis);
}

uint GetSocketUsageMask(uint anchorIndex)
{
    uint mask = 0u;
    [unroll]
    for (uint face = 0u; face < 6u; face++) {
        if (socketUsage[SocketUsageIndex(anchorIndex, face)] != 0u)
            mask |= FaceBit(face);
    }
    return mask;
}

void SortAxesByMagnitude(int3 delta, out uint3 axes)
{
    int3 absDelta = abs(delta);
    axes = uint3(0u, 1u, 2u);

    if (absDelta[axes.y] > absDelta[axes.x]) {
        uint tmp = axes.x;
        axes.x = axes.y;
        axes.y = tmp;
    }
    if (absDelta[axes.z] > absDelta[axes.y]) {
        uint tmp = axes.y;
        axes.y = axes.z;
        axes.z = tmp;
    }
    if (absDelta[axes.y] > absDelta[axes.x]) {
        uint tmp = axes.x;
        axes.x = axes.y;
        axes.y = tmp;
    }
}

void TrySetSocketConnection(uint a1, uint a2, int3 start, int3 end)
{
    if (a1 == a2)
        return;

    int3 diff1 = end - start;
    int3 diff2 = -diff1;

    uint dist1 = DistanceEncode(diff1);
    uint dist2 = DistanceEncode(diff2);
    uint3 axes;
    SortAxesByMagnitude(diff1, axes);

    uint pref = 2u;
    [unroll]
    for (uint order = 0u; order < 3u; order++) {
        uint axis = axes[order];
        if (diff1[axis] == 0)
            continue;

        uint face1 = FaceFromAxisDelta(diff1[axis], axis);
        uint face2 = FaceFromAxisDelta(diff2[axis], axis);
        uint encoded1 = EncodePriority(dist1, pref);
        uint encoded2 = EncodePriority(dist2, pref);
        uint prev;
        InterlockedMax(socketUsage[SocketUsageIndex(a1, face1)], encoded1, prev);
        InterlockedMax(socketUsage[SocketUsageIndex(a2, face2)], encoded2, prev);

        if (pref > 0u)
            pref--;
    }
}

uint TryGetSocketConnection(uint a1, uint a2, int3 start, int3 end) {
    if (a1 == a2)
        return 0u;

    int3 diff1 = end - start;
    int3 diff2 = -diff1;

    uint dist1 = DistanceEncode(diff1);
    uint dist2 = DistanceEncode(diff2);
    uint3 axes;
    SortAxesByMagnitude(diff1, axes);

    uint pref = 2u;
    [unroll]
    for (uint order = 0u; order < 3u; order++) {
        uint axis = axes[order];
        if (diff1[axis] == 0)
            continue;

        uint face1 = FaceFromAxisDelta(diff1[axis], axis);
        uint face2 = FaceFromAxisDelta(diff2[axis], axis);
        uint encoded1 = EncodePriority(dist1, pref);
        uint encoded2 = EncodePriority(dist2, pref);
        if (socketUsage[SocketUsageIndex(a1, face1)] == encoded1
            && socketUsage[SocketUsageIndex(a2, face2)] == encoded2)
            return PackFaceDirs(uint2(face1, face2));

        if (pref > 0u)
            pref--;
    }

    return 0u;
}
#endif