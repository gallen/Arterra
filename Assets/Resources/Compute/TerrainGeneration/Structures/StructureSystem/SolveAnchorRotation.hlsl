#include "Assets/Resources/Compute/Utility/RotationTables.hlsl"
#include "Assets/Resources/Compute/TerrainGeneration/Structures/StructureSystem/SocketConnector.hlsl"

//Rotation around y, x, and z axis respectively
inline bool HadRandYRot(uint config) { return (config & 0x1) != 0; }
inline bool HadRandXRot(uint config) { return (config & 0x2) != 0; }
inline bool HadRandZRot(uint config) { return (config & 0x4) != 0; }
inline bool IsEnhanced(uint config) { return (config & 0x8) != 0; }

// Decode a rotated unit axis direction (one nonzero component of ±1) to a face index.
// axis + sign*3: axis in {0,1,2} for X/Y/Z, sign in {0=neg,1=pos}
uint DirToFaceIdx(float3 d) {
    float3 a = abs(d);
    uint axis = (a.y > a.x) ? ((a.z > a.y) ? 2u : 1u) : ((a.z > a.x) ? 2u : 0u);
    uint sgn = (d[axis] > 0.0) ? 1u : 0u;
    return axis + sgn * 3u;
}

float3 FaceIdxToDir(uint face) {
    uint axis = face % 3u;
    float sgn = (face / 3u) == 0u ? -1.0 : 1.0;
    return float3(axis == 0u, axis == 1u, axis == 2u) * sgn;
}


//-1 if no solution, otherwise any such (rotY & 0x3) | ((rotX & 0x3) << 2) | ((rotZ & 0x3) << 4)
int SolveForRotation(uint index, uint config, uint basePorts) {
    uint expected = socketUsage[index] & SOCKET_USED_MASK;

    uint maxY = HadRandYRot(config) ? 4u : 1u;
    uint maxX = HadRandXRot(config) ? 4u : 1u;
    uint maxZ = HadRandZRot(config) ? 4u : 1u;

    uint total = maxY * maxX * maxZ;
    for (uint idx = 0u; idx < total; idx++) {
        uint rotY = idx % maxY;
        uint rotX = (idx / maxY) % maxX;
        uint rotZ = (idx / (maxY * maxX)) % maxZ;

        float3x3 rotMat = RotationLookupTable[rotY][rotX][rotZ];

        // Rotate the 3 positive axis directions — negatives are just their negatives
        float3 rx = mul(rotMat, float3(1,0,0));
        float3 ry = mul(rotMat, float3(0,1,0));
        float3 rz = mul(rotMat, float3(0,0,1));

        uint rotatedPorts = 0u;
        if (basePorts & SOCKET_MASK_POS_X) rotatedPorts |= 1u << DirToFaceIdx( rx);
        if (basePorts & SOCKET_MASK_NEG_X) rotatedPorts |= 1u << DirToFaceIdx(-rx);
        if (basePorts & SOCKET_MASK_POS_Y) rotatedPorts |= 1u << DirToFaceIdx( ry);
        if (basePorts & SOCKET_MASK_NEG_Y) rotatedPorts |= 1u << DirToFaceIdx(-ry);
        if (basePorts & SOCKET_MASK_POS_Z) rotatedPorts |= 1u << DirToFaceIdx( rz);
        if (basePorts & SOCKET_MASK_NEG_Z) rotatedPorts |= 1u << DirToFaceIdx(-rz);

        if (rotatedPorts == expected)
            return int((rotY & 0x3) | ((rotX & 0x3) << 2) | ((rotZ & 0x3) << 4));
    }
    return -1;
}