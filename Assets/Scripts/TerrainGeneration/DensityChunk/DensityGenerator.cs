using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using static UtilityBuffers;

public static class DensityGenerator
{
    [Header("Terrain Generation Shaders")]
    static ComputeShader baseGenCompute;//
    static ComputeShader meshGenerator;//
    static ComputeShader densitySimplification;//

    public const int VERTEX_STRIDE_WORD = 3 * 2 + 2;
    public const int TRI_STRIDE_WORD = 3;

    public static int dictStart_U;
    public static int vertexStart_U;
    public static int baseTriStart_U;
    public static int waterTriStart_U;
    public static int3 countInd = new int3(0, 1, 2);
    

    static DensityGenerator(){ //That's a lot of Compute Shaders XD
        baseGenCompute = Resources.Load<ComputeShader>("TerrainGeneration/BaseGeneration/ChunkDataGen");
        meshGenerator = Resources.Load<ComputeShader>("TerrainGeneration/BaseGeneration/MarchingCubes");
        densitySimplification = Resources.Load<ComputeShader>("TerrainGeneration/BaseGeneration/DensitySimplificator");

        indirectCountToArgs = Resources.Load<ComputeShader>("Utility/CountToArgs");
    }

    public static void PresetData(MeshCreatorSettings meshSettings, int maxChunkSize){
        baseGenCompute.SetInt("coarseCaveSampler", meshSettings.CoarseTerrainNoise);
        baseGenCompute.SetInt("fineCaveSampler", meshSettings.FineTerrainNoise);
        baseGenCompute.SetInt("coarseMatSampler", meshSettings.CoarseMaterialNoise);
        baseGenCompute.SetInt("fineMatSampler", meshSettings.FineMaterialNoise);

        baseGenCompute.SetFloat("waterHeight", meshSettings.waterHeight);
        baseGenCompute.SetInt("waterMat", meshSettings.waterMat);

        baseGenCompute.SetBuffer(0, "baseMap", UtilityBuffers.GenerationBuffer);

        //Set Marching Cubes Data
        int numPointsAxes = maxChunkSize + 1;
        int numOfPoints = numPointsAxes * numPointsAxes * numPointsAxes;
        int numOfTris = (numPointsAxes - 1) * (numPointsAxes - 1) * (numPointsAxes - 1) * 5;

        dictStart_U = 3;
        int dictEnd_W = dictStart_U + numOfPoints * TRI_STRIDE_WORD;

        vertexStart_U = Mathf.CeilToInt((float)dictEnd_W / VERTEX_STRIDE_WORD);
        int vertexEnd_W = vertexStart_U * VERTEX_STRIDE_WORD + (numOfPoints * 3) * VERTEX_STRIDE_WORD;

        baseTriStart_U = Mathf.CeilToInt((float)vertexEnd_W / TRI_STRIDE_WORD);
        int baseTriEnd_W = baseTriStart_U * TRI_STRIDE_WORD + numOfTris * TRI_STRIDE_WORD;

        waterTriStart_U = Mathf.CeilToInt((float)baseTriEnd_W / TRI_STRIDE_WORD);
        int waterTriEnd_W = waterTriStart_U * TRI_STRIDE_WORD + numOfTris * TRI_STRIDE_WORD;

        //They're all the same buffer lol
        meshGenerator.SetBuffer(0, "vertexes", UtilityBuffers.GenerationBuffer);
        meshGenerator.SetBuffer(0, "triangles", UtilityBuffers.GenerationBuffer);
        meshGenerator.SetBuffer(0, "triangleDict", UtilityBuffers.GenerationBuffer);
        meshGenerator.SetBuffer(0, "counter", UtilityBuffers.GenerationBuffer);
        meshGenerator.SetInts("counterInd", new int[3]{countInd.x, countInd.y, countInd.z});

        meshGenerator.SetInt("bSTART_dict", dictStart_U);
        meshGenerator.SetInt("bSTART_verts", vertexStart_U);    
        meshGenerator.SetInt("bSTART_baseT", baseTriStart_U);
        meshGenerator.SetInt("bSTART_waterT", waterTriStart_U);
    }

    public static void SimplifyMap(int chunkSize, int meshSkipInc, TerrainChunk.MapData[] chunkData, ref Queue<ComputeBuffer> bufferHandle)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int totalPointsAxes = chunkSize + 1;
        int totalPoints = totalPointsAxes * totalPointsAxes * totalPointsAxes;
        
        ComputeBuffer fullMap = new ComputeBuffer(totalPoints, sizeof(float) * 2 + sizeof(int), ComputeBufferType.Structured, ComputeBufferMode.Immutable);
        fullMap.SetData(chunkData);
            
        bufferHandle.Enqueue(fullMap);

        densitySimplification.DisableKeyword("USE_INT");
        densitySimplification.SetInt("meshSkipInc", meshSkipInc);
        densitySimplification.SetInt("totalPointsPerAxis", totalPointsAxes);
        densitySimplification.SetInt("pointsPerAxis", numPointsAxes);
        densitySimplification.SetBuffer(0, "points_full", fullMap);
        densitySimplification.SetBuffer(0, "points", UtilityBuffers.GenerationBuffer);

        densitySimplification.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        densitySimplification.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
    }

       
    public static void GenerateBaseData( SurfaceChunk.SurfData surfaceData, float IsoLevel, int chunkSize, int meshSkipInc, Vector3 offset)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;

        baseGenCompute.SetFloat("IsoLevel", IsoLevel);
        baseGenCompute.SetBuffer(0, "_SurfMemoryBuffer", surfaceData.Memory);
        baseGenCompute.SetBuffer(0, "_SurfAddressDict", surfaceData.Addresses);
        baseGenCompute.SetInt("surfAddress", (int)surfaceData.addressIndex);
        baseGenCompute.SetInt("numPointsPerAxis", numPointsAxes);

        baseGenCompute.SetFloat("meshSkipInc", meshSkipInc);
        baseGenCompute.SetFloat("chunkSize", chunkSize);
        baseGenCompute.SetFloat("offsetY", offset.y);
        
        SetSampleData(baseGenCompute, offset, chunkSize, meshSkipInc);

        baseGenCompute.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);
        baseGenCompute.Dispatch(0, numThreadsAxis, numThreadsAxis, numThreadsAxis);
    }

    public static void GenerateMesh(GPUDensityManager densityManager, Vector3 CCoord, int chunkSize, int meshSkipInc, float IsoLevel)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        UtilityBuffers.ClearCounters(UtilityBuffers.GenerationBuffer, 3);

        meshGenerator.SetBuffer(0, "_MemoryBuffer", densityManager.AccessStorage());
        meshGenerator.SetBuffer(0, "_AddressDict", densityManager.AccessAddresses());
        meshGenerator.SetInts("CCoord", new int[] { (int)CCoord.x, (int)CCoord.y, (int)CCoord.z });
        densityManager.SetCCoordHash(meshGenerator);

        meshGenerator.SetFloat("IsoLevel", IsoLevel);
        meshGenerator.SetInt("numPointsPerAxis", numPointsAxes);
        meshGenerator.SetFloat("meshSkipInc", meshSkipInc);

        meshGenerator.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        meshGenerator.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
    }

    /*
    public static void SimplifyMaterials(int chunkSize, int meshSkipInc, int[] materials, ComputeBuffer pointBuffer, ref Queue<ComputeBuffer> bufferHandle)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int totalPointsAxes = chunkSize + 1;
        int totalPoints = totalPointsAxes * totalPointsAxes * totalPointsAxes;
        ComputeBuffer completeMaterial = new ComputeBuffer(totalPoints, sizeof(int), ComputeBufferType.Structured, ComputeBufferMode.Immutable);
        completeMaterial.SetData(materials);
        bufferHandle.Enqueue(completeMaterial);

        densitySimplification.EnableKeyword("USE_INT");
        densitySimplification.SetInt("meshSkipInc", meshSkipInc);
        densitySimplification.SetInt("totalPointsPerAxis", totalPointsAxes);
        densitySimplification.SetInt("pointsPerAxis", numPointsAxes);
        densitySimplification.SetBuffer(0, "points_full", completeMaterial);
        densitySimplification.SetBuffer(0, "points", pointBuffer);

        densitySimplification.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        densitySimplification.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
    }*/

    /*
    public static ComputeBuffer GenerateTerrain(int chunkSize, int meshSkipInc, SurfaceChunk.SurfData surfaceData, int coarseCave, int fineCave, Vector3 offset, float IsoValue, ref Queue<ComputeBuffer> bufferHandle)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int numPoints = numPointsAxes * numPointsAxes * numPointsAxes;
        
        ComputeBuffer densityMap = new ComputeBuffer(numPoints, sizeof(float), ComputeBufferType.Structured);
        bufferHandle.Enqueue(densityMap);

        terrainNoiseCompute.SetBuffer(0, "points", densityMap);
        terrainNoiseCompute.SetBuffer(0, "_SurfMemoryBuffer", surfaceData.Memory);
        terrainNoiseCompute.SetBuffer(0, "_SurfAddressDict", surfaceData.Addresses);
        terrainNoiseCompute.SetInt("surfAddress", (int)surfaceData.addressIndex);

        terrainNoiseCompute.SetInt("coarseSampler", coarseCave);
        terrainNoiseCompute.SetInt("fineSampler", fineCave);

        terrainNoiseCompute.SetInt("numPointsPerAxis", numPointsAxes);
        terrainNoiseCompute.SetFloat("meshSkipInc", meshSkipInc);
        terrainNoiseCompute.SetFloat("chunkSize", chunkSize);
        terrainNoiseCompute.SetFloat("offsetY", offset.y);
        terrainNoiseCompute.SetFloat("IsoLevel", IsoValue);
        SetSampleData(terrainNoiseCompute, offset, chunkSize, meshSkipInc);

        terrainNoiseCompute.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        terrainNoiseCompute.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);

        return densityMap;
    }

    public static ComputeBuffer GenerateNoiseMap(ComputeShader shader, Vector3 offset, int chunkSize, int meshSkipInc, ref Queue<ComputeBuffer> bufferHandle){
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int numPoints = numPointsAxes * numPointsAxes * numPointsAxes;

        ComputeBuffer density = new ComputeBuffer(numPoints, sizeof(float), ComputeBufferType.Structured);
        bufferHandle.Enqueue(density);

        shader.SetBuffer(0, "points", density);
        shader.SetInt("numPointsPerAxis", numPointsAxes);
        SetSampleData(shader, offset, chunkSize, meshSkipInc);

        shader.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);
        shader.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
        return density;
    }

    public static ComputeBuffer GenerateNoiseMap(int chunkSize, int meshSkipInc, NoiseData noiseData, Vector3 offset, ref Queue<ComputeBuffer> bufferHandle)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int numPoints = numPointsAxes * numPointsAxes * numPointsAxes;

        ComputeBuffer density = new ComputeBuffer(numPoints, sizeof(float), ComputeBufferType.Structured);
        bufferHandle.Enqueue(density);

        rawNoiseSampler.SetBuffer(0, "points", density);
        rawNoiseSampler.SetInt("numPointsPerAxis", numPointsAxes);
        SetNoiseData(rawNoiseSampler, chunkSize, meshSkipInc, noiseData, offset);

        rawNoiseSampler.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        rawNoiseSampler.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
        return density;
    }*/

    /*
    public static ComputeBuffer GenerateCaveNoise(SurfaceChunk.SurfData surfaceData, Vector3 offset, int coarseSampler, int fineSampler, int chunkSize, int meshSkipInc, ref Queue<ComputeBuffer> bufferHandle){
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        int numPoints = numPointsAxes * numPointsAxes * numPointsAxes;

        ComputeBuffer caveDensity = new ComputeBuffer(numPoints, sizeof(float), ComputeBufferType.Structured);
        bufferHandle.Enqueue(caveDensity);
        
        baseCaveGenerator.SetBuffer(0, "_SurfMemoryBuffer", surfaceData.Memory);
        baseCaveGenerator.SetBuffer(0, "_SurfAddressDict", surfaceData.Addresses);
        baseCaveGenerator.SetInt("surfAddress", (int)surfaceData.addressIndex);

        baseCaveGenerator.SetInt("coarseSampler", coarseSampler);
        baseCaveGenerator.SetInt("fineSampler", fineSampler);
        baseCaveGenerator.SetInt("numPointsPerAxis", numPointsAxes);
        SetSampleData(baseCaveGenerator, offset, chunkSize, meshSkipInc);

        baseCaveGenerator.SetBuffer(0, "densityMap", caveDensity);

        baseCaveGenerator.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);
        baseCaveGenerator.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, numThreadsPerAxis);
        
        return caveDensity;
    }*/

    /*
    public ComputeBuffer GetAdjacentDensity(GPUDensityManager densityManager, Vector3 CCoord, int chunkSize, int meshSkipInc, ref Queue<ComputeBuffer> bufferHandle)
    {
        int numPointsAxes = chunkSize / meshSkipInc + 1;
        ComputeBuffer neighborDensity = new ComputeBuffer(numPointsAxes * numPointsAxes * 6, sizeof(float), ComputeBufferType.Structured);
        bufferHandle.Enqueue(neighborDensity);

        neighborDensitySampler.SetBuffer(0, "_MemoryBuffer", densityManager.AccessStorage());
        neighborDensitySampler.SetBuffer(0, "_AddressDict", densityManager.AccessAddresses());
        densityManager.SetCCoordHash(neighborDensitySampler);

        neighborDensitySampler.SetInts("CCoord", new int[] { (int)CCoord.x, (int)CCoord.y, (int)CCoord.z });
        neighborDensitySampler.SetInt("numPointsPerAxis", numPointsAxes);
        neighborDensitySampler.SetInt("meshSkipInc", meshSkipInc);
        neighborDensitySampler.SetBuffer(0, "nDensity", neighborDensity);

        neighborDensitySampler.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsPerAxis = Mathf.CeilToInt(numPointsAxes / (float)threadGroupSize);

        neighborDensitySampler.Dispatch(0, numThreadsPerAxis, numThreadsPerAxis, 1);
        return neighborDensity;
    }*/
}