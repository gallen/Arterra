#ifndef STRUCT_SOCKET_CONNECTION
#define STRUCT_SOCKET_CONNECTION
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
static const uint SOCKET_LOCK_BIT  = 0x80000000u;

//high bit -> writeLock, lowest 6 bits -> has '-x,+x,-y,+y,-z,+z' connections
RWStructuredBuffer<uint> socketUsage;
uint bSTART_sockets;

void LockSocketUsage(uint index) {
    uint oldValue;
    uint newValue;

    [allow_uav_condition]
    while (true) {
        oldValue = socketUsage[index + bSTART_sockets];
        if ((oldValue & SOCKET_LOCK_BIT) != 0u)
            continue;

        newValue = oldValue | SOCKET_LOCK_BIT;

        uint prev;
        InterlockedCompareExchange(socketUsage[index + bSTART_sockets], newValue, oldValue, prev);
        if (prev == oldValue)
            break;
    }
}

void UnlockSocketUsage(uint index)
{
    uint dummy;
    InterlockedAnd(socketUsage[index + bSTART_sockets], ~SOCKET_LOCK_BIT, dummy);
}

uint OppositeFace(uint face) {
    return (face + 3u) % 6u;
}

uint FaceBit(uint face) {
    return 1u << face;
}

uint BuildAllowedMask(int3 diff)
{
    uint mask = 0u;

    if (diff.x >= 0) mask |= SOCKET_MASK_POS_X;
    if (diff.x <= 0) mask |= SOCKET_MASK_NEG_X;

    if (diff.y >= 0) mask |= SOCKET_MASK_POS_Y;
    if (diff.y <= 0) mask |= SOCKET_MASK_NEG_Y;

    if (diff.z >= 0) mask |= SOCKET_MASK_POS_Z;
    if (diff.z <= 0) mask |= SOCKET_MASK_NEG_Z;

    return mask;
}

inline uint2 GetFaceDirs(uint data) { return uint2(data & 0x3, (data >> 2) & 0x3); }
inline uint PackFaceDirs(uint2 dirs) { return (dirs.y << 3u) | dirs.x; }


uint TryGetSocketConnection(uint a1, uint a2, int3 diff)
{
    // Never connect an anchor to itself.
    if (a1 == a2) return 0u;

    // Lock in stable order to avoid deadlock.
    uint lo = min(a1, a2);
    uint hi = max(a1, a2);

    LockSocketUsage(lo);
    LockSocketUsage(hi);

    uint usage1 = socketUsage[a1 + bSTART_sockets];
    uint usage2 = socketUsage[a2 + bSTART_sockets];

    uint used1 = usage1 & SOCKET_USED_MASK;
    uint used2 = usage2 & SOCKET_USED_MASK;

    uint allowed1 = BuildAllowedMask(diff) & ~used1;
    uint allowed2 = BuildAllowedMask(-diff) & ~used2;

    if (allowed1 == 0u || allowed2 == 0u) {
        UnlockSocketUsage(hi);
        UnlockSocketUsage(lo);
        return 0u;
    }

    uint chosenA1 = 0u;
    uint chosenA2 = 0u;
    bool found = false;

    // Prefer opposite-facing pairs along the strongest available direction.
    [unroll]
    for (uint f1 = 0u; f1 < 6u; f1++) {
        uint bit1 = FaceBit(f1);
        if ((allowed1 & bit1) == 0u) continue;

        uint f2 = OppositeFace(f1);
        uint bit2 = FaceBit(f2);
        if ((allowed2 & bit2) == 0u) continue;

        chosenA1 = f1;
        chosenA2 = f2;
        found = true;
        break;
    }

    if (!found) {
        UnlockSocketUsage(hi);
        UnlockSocketUsage(lo);
        return 0u;
    }

    // Mark both sockets as used. Keep lock bit intact.
    socketUsage[a1 + bSTART_sockets] = usage1 | FaceBit(chosenA1);
    socketUsage[a2 + bSTART_sockets] = usage2 | FaceBit(chosenA2);

    UnlockSocketUsage(hi);
    UnlockSocketUsage(lo);

    return PackFaceDirs(uint2(chosenA1, chosenA2)); 
}
#endif