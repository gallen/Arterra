struct BMaterial{
    int materialIndex;
    float genNoiseSize;
    float genNoiseShape;
    float frequency;
    float height;
};

float GetNoiseCentered(float val, float center)
{
    val = saturate(val);

    float base;
    if (val > center) base = 1 - smoothstep(center, 1, val);
    else base = smoothstep(0, center, val);

    //Counterscale the middle
    float ratio = abs(center * 2 - 1);
    float exponent = lerp(4.0, 1.0, ratio);
    return pow(abs(base), exponent);
}

float GetMaterialWeight(BMaterial material, float coarse, float fine, float height){
    float coarsePref = material.genNoiseSize;
    float noiseCenter = material.genNoiseShape;

    float coarseCentered = GetNoiseCentered(coarse, noiseCenter);
    float fineCentered = GetNoiseCentered(fine, noiseCenter);

    float baseWeight = coarsePref * coarseCentered + (1.0f-coarsePref) * fineCentered;
    //freq^((1-v)/freq) * v gives more saturated results with higher frequencies
    baseWeight = pow(abs(material.frequency), (1 - baseWeight)/material.frequency) * baseWeight;
    if(height <= 1.0f) baseWeight *= 1 - abs(height - material.height);
    return baseWeight;
}


int GetMaterial(StructuredBuffer<BMaterial> materials, int2 biomeBounds, float coarse, float fine, float height, out float maxWeight){
    int bestMat = materials[biomeBounds.x].materialIndex;
    maxWeight = -1.0f;
    
    for(int matInd = biomeBounds.x; matInd < biomeBounds.y; matInd++){
        BMaterial material = materials[matInd];
        float weight = GetMaterialWeight(material, coarse, fine, height);

        if(weight > maxWeight){
            maxWeight = weight;
            bestMat = material.materialIndex;
        }
    }

    return bestMat;
}