using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static EndlessTerrain;
using static UtilityBuffers;

[CreateAssetMenu(menuName = "Settings/DensityManager")]
public class GPUDensityManager : ScriptableObject
{
    private ComputeShader dictReplaceKey;
    private ComputeShader transcribeMapInfo;
    private ComputeBuffer _ChunkAddressDict;
    private SegmentBinManager memorySpace;
    private int numChunksAxis;
    private int mapChunkSize;
    private float lerpScale;
    private const int pointStride4Byte = 3; //Only density for now
    public Queue<ComputeBuffer> tempBuffers = new Queue<ComputeBuffer>();
    [HideInInspector]
    public bool initialized = false;

    public void InitializeManage(LODInfo[] detaiLevels, int mapChunkSize, float lerpScale)
    {
        OnDisable();
        
        dictReplaceKey = Resources.Load<ComputeShader>("MapData/ReplaceDictChunk");
        transcribeMapInfo = Resources.Load<ComputeShader>("MapData/TranscribeMapInfo");

        this.lerpScale = lerpScale;

        this.mapChunkSize = mapChunkSize;
        this.numChunksAxis = 2 * (Mathf.RoundToInt(detaiLevels[detaiLevels.Length-1].distanceThresh / mapChunkSize) + 1);
        int numChunks = numChunksAxis * numChunksAxis * numChunksAxis;

        this._ChunkAddressDict = new ComputeBuffer(numChunks, sizeof(uint) * 2, ComputeBufferType.Structured);
        this._ChunkAddressDict.SetData(Enumerable.Repeat(0u, numChunks * 2).ToArray());
        this.memorySpace = new SegmentBinManager(mapChunkSize, detaiLevels, pointStride4Byte);

        initialized = true;
    }

    public int HashCoord(Vector3 CCoord){
        float xHash = CCoord.x < 0 ? numChunksAxis - (Mathf.Abs(CCoord.x) % numChunksAxis) : Mathf.Abs(CCoord.x) % numChunksAxis;
        float yHash = CCoord.y < 0 ? numChunksAxis - (Mathf.Abs(CCoord.y) % numChunksAxis) : Mathf.Abs(CCoord.y) % numChunksAxis;
        float zHash = CCoord.z < 0 ? numChunksAxis - (Mathf.Abs(CCoord.z) % numChunksAxis) : Mathf.Abs(CCoord.z) % numChunksAxis;

        int hash = ((int)xHash * numChunksAxis * numChunksAxis) + ((int)yHash * numChunksAxis) + (int)zHash;
        return hash;
    }

    public ComputeBuffer AccessStorage(){
        return this.memorySpace.AccessStorage();
    }

    public ComputeBuffer AccessAddresses(){
        return this._ChunkAddressDict;
    }

    private void OnDisable()
    {
        _ChunkAddressDict?.Release();
        this.memorySpace?.Release();
        initialized = false;
    }

    public void SubscribeChunk(Vector3 CCoord, int LOD)
    {
        int meshSkipInc = meshSkipTable[LOD];
        int numPointsAxis = mapChunkSize / meshSkipInc + 1;
        int numPoints = numPointsAxis * numPointsAxis * numPointsAxis;

        ComputeBuffer address = this.memorySpace.AllocateChunk(LOD);
        TranscribeData(this.memorySpace.AccessStorage(), address, numPoints);

        ReplaceAddress(address, CCoord, meshSkipInc);
        this.memorySpace.ReleaseChunk(address);
    }

    void TranscribeData(ComputeBuffer memory, ComputeBuffer address, int numPoints)
    {
        this.transcribeMapInfo.SetBuffer(0, "_MemoryBuffer", memory);
        this.transcribeMapInfo.SetBuffer(0, "chunkData", UtilityBuffers.GenerationBuffer);
        this.transcribeMapInfo.SetBuffer(0, "startAddress", address); 
        this.transcribeMapInfo.SetInt("numPoints", numPoints);

        this.transcribeMapInfo.GetKernelThreadGroupSizes(0, out uint threadGroupSize, out _, out _);
        int numThreadsAxis = Mathf.CeilToInt(numPoints / (float)threadGroupSize);
        this.transcribeMapInfo.Dispatch(0, numThreadsAxis, 1, 1);
    }

    void ReplaceAddress(ComputeBuffer memoryAddress, Vector3 CCoord, int meshSkipInc)
    {
        this.dictReplaceKey.SetBuffer(0, "chunkAddressDict", _ChunkAddressDict);
        this.dictReplaceKey.SetBuffer(0, "nAddress", memoryAddress);
        this.dictReplaceKey.SetInts("CCoord", new int[3] { (int)CCoord.x, (int)CCoord.y, (int)CCoord.z });
        this.dictReplaceKey.SetInt("meshSkipInc", meshSkipInc);

        SetCCoordHash(this.dictReplaceKey);

        this.dictReplaceKey.Dispatch(0, 1, 1, 1);
    }


    public void SetDensitySampleData(ComputeShader shader) {
        if(!initialized)
            return;

        this.SetCCoordHash(shader);
        this.SetWSCCoordHelper(shader);

        shader.SetBuffer(0, "_ChunkAddressDict", this._ChunkAddressDict);
        shader.SetBuffer(0, "_ChunkInfoBuffer", this.memorySpace.AccessStorage());
    }

    public void SetDensitySampleData(Material material){
        if(!initialized)
            return;
            
        this.SetCCoordHash(material);
        this.SetWSCCoordHelper(material);

        material.SetBuffer("_ChunkAddressDict", this._ChunkAddressDict);
        material.SetBuffer("_ChunkInfoBuffer", this.memorySpace.AccessStorage());
    }

    void SetWSCCoordHelper(ComputeShader shader) { 
        shader.SetFloat("lerpScale", lerpScale);
        shader.SetInt("mapChunkSize", mapChunkSize);
    }

    void SetWSCCoordHelper(Material material) { 
        material.SetFloat("lerpScale", lerpScale);
        material.SetInt("mapChunkSize", mapChunkSize);
    }
    public void SetCCoordHash(ComputeShader shader) { shader.SetInt("numChunksAxis", numChunksAxis); }
    public void SetCCoordHash(Material material) { material.SetInt("numChunksAxis", numChunksAxis); }
    


}