#ifndef BLEND_HELPER
#define BLEND_HELPER
/*
* z
* ^     6--------7
* |    /|       /|
* |   / |      / |    y
* |  5--+-----4  |   /\
* |  |  |     |  |   /
* |  |  0-----+--1  /
* |  | /      | /  /
* |  3________2/  /
* +---------> x  /
*/

struct Corner{
    float influence;
    uint3 mapCoord;
};

struct Influences{
    Corner corner[8];
};

Influences GetBlendInfo(float3 positionMS){
    Influences influences = (Influences)0;

    //Ceil & Floor will casue error if positionMS is grid-aligned. With this, caller must guarantee positionMS is not max value
    influences.corner[0].mapCoord = uint3(floor(positionMS.x)    , floor(positionMS.y) + 1, floor(positionMS.z));
    influences.corner[1].mapCoord = uint3(floor(positionMS.x) + 1, floor(positionMS.y) + 1, floor(positionMS.z));
    influences.corner[2].mapCoord = uint3(floor(positionMS.x) + 1, floor(positionMS.y)    , floor(positionMS.z));
    influences.corner[3].mapCoord = uint3(floor(positionMS.x)    , floor(positionMS.y)    , floor(positionMS.z));
    influences.corner[4].mapCoord = uint3(floor(positionMS.x) + 1, floor(positionMS.y)    , floor(positionMS.z) + 1);
    influences.corner[5].mapCoord = uint3(floor(positionMS.x)    , floor(positionMS.y)    , floor(positionMS.z) + 1);
    influences.corner[6].mapCoord = uint3(floor(positionMS.x)    , floor(positionMS.y) + 1, floor(positionMS.z) + 1);
    influences.corner[7].mapCoord = uint3(floor(positionMS.x) + 1, floor(positionMS.y) + 1, floor(positionMS.z) + 1);

    [unroll]for(uint i = 0; i < 8; i++){
        uint3 oppositeCorner = influences.corner[(i+4u)%8u].mapCoord;
        influences.corner[i].influence = abs(positionMS.x - oppositeCorner.x) * abs(positionMS.y - oppositeCorner.y) * abs(positionMS.z - oppositeCorner.z);
    }

    return influences;
}

struct Corner2D{
    float influence;
    uint2 mapCoord;
};

struct Influences2D{
    Corner2D corner[4];
};

Influences2D GetBlendInfo(float2 positionMS){
    Influences2D influences = (Influences2D)0;

    //Ceil & Floor will casue error if positionMS is grid-aligned. With this, caller must guarantee positionMS is not max value
    influences.corner[0].mapCoord = uint2(floor(positionMS.x)    , floor(positionMS.y) + 1);
    influences.corner[1].mapCoord = uint2(floor(positionMS.x) + 1, floor(positionMS.y) + 1);
    influences.corner[2].mapCoord = uint2(floor(positionMS.x) + 1, floor(positionMS.y)    );
    influences.corner[3].mapCoord = uint2(floor(positionMS.x)    , floor(positionMS.y)    );

    [unroll]for(uint i = 0; i < 4; i++){
        uint2 oppositeCorner = influences.corner[(i+2u)%4u].mapCoord;
        influences.corner[i].influence = abs(positionMS.x - oppositeCorner.x) * abs(positionMS.y - oppositeCorner.y);
    }

    return influences;
}
#endif