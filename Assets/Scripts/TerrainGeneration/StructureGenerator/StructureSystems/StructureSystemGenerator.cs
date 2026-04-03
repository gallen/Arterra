using Arterra.Configuration;
using Arterra.Data.Structure.Jigsaw;
using Arterra.Utils;
using Arterra.Editor;
using Unity.Mathematics;
using UnityEngine;
using Newtonsoft.Json;
using FMOD.Studio;
using System;
using Arterra.Data.Biome;

namespace Arterra.Data.Structure.Jigsaw {
    [Serializable]
    public class StructureSystem {
        //Warning don't make this too small or memory overflow
        public int CellSizeFactor = 4;
        public int MaxConnectionDist = 32;
        public int MaxBatchSize = 128;

        [RegistryReference("Noise")]
        public string coarseSSystemNoise;
        [RegistryReference("Noise")]
        public string fineSSystemNoise;
        [JsonIgnore]
        public int CoarseSSystemNoise => Config.CURRENT.Generation.Noise.RetrieveIndex(coarseSSystemNoise);
        [JsonIgnore]
        public int FineSSystemNoise => Config.CURRENT.Generation.Noise.RetrieveIndex(fineSSystemNoise);
        [JsonIgnore]
        public int CellSize => 1 << CellSizeFactor;
        public int MaxSystemLoD = 2;
    }
}

namespace Arterra.Engine.Terrain.Structure.Jigsaw {
/*
Plan:
- For each grid cell, determine structure ID it belongs to
    - Based on sampled biome at center of cell, and 2-3 other noise maps
- Poisson sample some grid cells(int pos) to be anchor points
- Each anchor point adopts the unqiue StructureSystemID of its parent cell
- Bin anchor points into StructureSystems by their Structure System ID
- Determine which anchor cells are connected to which other anchor cells
    - Look in fixed radius in the positive direction from every cell
    - Each anchor cell will own connections to all those cells it finds
- Assign jigsaw pieces to the anchor cell based on its connections and worldpos.
    - Test whether there are any jigsaw pieces that fit these connections and all its checks

ComputeRegion size determined by amount of memory for LUTs(100mb -> ~64^3 voxels)
Amount of ComputeRegions determined by chunk size / compute region size

Construct ComputeRegion LUT:
Divide grid into (overlapping) ComputeRegions:
- For each path, compute the actual physical paths given the pieces
    - Discard any that have been pruned
- Determine which Compute Region fully contains it
- Add line to linked list of that compute region

ForEach Compute Region:
    Section Path Space:
        - For each grid voxel, go through all lines in compute region:
            - Determine closest line, set "id" to that path
        - Clear visited uint (pop. with structure id and rot)

    Pathfind:
        - Groupshared memory for frontier
        - Update expand loop until reach end
        - Only explore if voxel id belongs to path
        - Only explore if socket is closer to endpoint to avoid self-intersecting cycles
            (as we don't mark the entire structure region as visited, only socket points)
    Backtrack, add sockets:
        - Add structures to completed structure list
        - Add any open sockets to to-cap list
    Cap Open Sockets:
        - For all open socket, try to add random structure
          with 2 or 1 connection ports
            - Only add if completely within voxel id
        - Fail if not found, otherwise add to list
*/

public static class Generator {
    private const int ANCHOR_STRIDE_WORD = 3 + 1 + 1;
    private const int ANCHOR_PATH_WORD = 3;
#if UNITY_EDITOR || DEVELOPMENT_BUILD
    private const int SANITY_SAMPLE_COUNT = 16;
    private static readonly int[] _counterReadback = new int[1];
#endif

    public static SSystemOffsets offsets;
    private static ComputeShader AnchorSampler;
    private static ComputeShader GraphConnector;
    private static ComputeShader SanitateBatches;
    private static ComputeShader StructurePathfinder;
    private static ComputeShader PathSetupRetriever;
    private static StructureSystem jigsaw => Config.CURRENT.Generation.Structures.value.StructureSystemSettings;

    static Generator() {
        AnchorSampler = Resources.Load<ComputeShader>("Compute/TerrainGeneration/Structures/StructureSystem/SampleAnchors");
        GraphConnector = Resources.Load<ComputeShader>("Compute/TerrainGeneration/Structures/StructureSystem/GraphConnector");
        SanitateBatches = Resources.Load<ComputeShader>("Compute/TerrainGeneration/Structures/StructureSystem/SanitatePathBatches");
        StructurePathfinder = Resources.Load<ComputeShader>("Compute/TerrainGeneration/Structures/StructureSystem/PopulatePaths");
        PathSetupRetriever = Resources.Load<ComputeShader>("Compute/TerrainGeneration/Structures/StructureSystem/ComputeBatchSetup");
    }

    private static int CalculateBatchAxisCount(int totalAxisSize, int batchSize, int overlapSize) {
        if (totalAxisSize <= batchSize)
            return 1;

        int batchStride = batchSize - overlapSize;
        if (batchStride <= 0)
            throw new InvalidOperationException($"Invalid structure-system batch stride: batchSize={batchSize}, overlapSize={overlapSize}");

        return Mathf.CeilToInt((totalAxisSize - batchSize) / (float)batchStride) + 1;
    }

    public static void Initialize() {
        offsets = new SSystemOffsets();
        Configuration.Quality.Terrain rSettings = Config.CURRENT.Quality.Terrain.value;
        Data.Structure.Generation structures = Config.CURRENT.Generation.Structures.value;
        int batchStep = Mathf.CeilToInt((float)jigsaw.MaxBatchSize / jigsaw.CellSize);
        batchStep = batchStep * batchStep * batchStep * 6;
        int originOffset = -(jigsaw.MaxConnectionDist * 2);
        int cellsPerChunk = rSettings.mapChunkSize / jigsaw.CellSize;

        int maxChunkSize = rSettings.mapChunkSize * (1 << structures.maxLoD);
        offsets = new SSystemOffsets(maxChunkSize, jigsaw.MaxConnectionDist, jigsaw.MaxBatchSize, jigsaw.CellSize, 0);

        int kernel = AnchorSampler.FindKernel("SamplePoints");
        AnchorSampler.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        AnchorSampler.SetInt("coarseSSystemNoise", jigsaw.CoarseSSystemNoise);
        AnchorSampler.SetInt("fineSSystemNoise", jigsaw.FineSSystemNoise);
        AnchorSampler.SetInt("cellSize", jigsaw.CellSize);
        AnchorSampler.SetInt("cellsPerChunk", cellsPerChunk);
        AnchorSampler.SetInt("bSTART_anchors", offsets.anchorsStart);
        AnchorSampler.SetInt("oCellOffset", originOffset);
        Structure.Generator.SetStructIDSettings(AnchorSampler);

        kernel = AnchorSampler.FindKernel("PoissonPrune");
        AnchorSampler.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        AnchorSampler.SetBuffer(kernel, "anchorDict", UtilityBuffers.GenerationBuffer);
        AnchorSampler.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        AnchorSampler.SetInt("bCOUNT_dict", offsets.anchorDictCounter);
        AnchorSampler.SetInt("bSTART_dict", offsets.anchorDictStart);

        kernel = GraphConnector.FindKernel("ClearSockets");
        GraphConnector.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchorDict", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "socketUsage", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetInt("oCellOffset", originOffset);
        GraphConnector.SetInt("cellsPerChunk", cellsPerChunk);
        GraphConnector.SetInt("bCOUNT_dict", offsets.anchorDictCounter);
        GraphConnector.SetInt("bSTART_dict", offsets.anchorDictStart);
        GraphConnector.SetInt("bSTART_sockets", offsets.socketUsageStart);

        kernel = GraphConnector.FindKernel("SetSocketConnections");
        GraphConnector.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchorDict", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "socketUsage", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetInt("cellSize", jigsaw.CellSize);
        GraphConnector.SetInt("connectRadius", jigsaw.MaxConnectionDist);
        GraphConnector.SetInt("bSTART_anchors", offsets.anchorsStart);

        kernel = GraphConnector.FindKernel("ConnectGraph");
        GraphConnector.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchorDict", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "socketUsage", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetBuffer(kernel, "anchorPaths", UtilityBuffers.GenerationBuffer);
        GraphConnector.SetInt("cellSize", jigsaw.CellSize);
        GraphConnector.SetInt("connectRadius", jigsaw.MaxConnectionDist);
        GraphConnector.SetInt("bCOUNT_paths", offsets.anchorPathCounter);
        GraphConnector.SetInt("bSTART_anchors", offsets.anchorsStart);
        GraphConnector.SetInt("bSTART_paths", offsets.anchorPathStart);

        kernel = SanitateBatches.FindKernel("SelectAnchorPieces");
        SanitateBatches.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "anchorDict", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "socketUsage", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "genStructures", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetInt("bCOUNT_dict", offsets.anchorDictCounter);
        SanitateBatches.SetInt("bSTART_dict", offsets.anchorDictStart);
        SanitateBatches.SetInt("bSTART_anchors", offsets.anchorsStart);
        SanitateBatches.SetInt("bSTART_sockets", offsets.socketUsageStart);
        SanitateBatches.SetInt("bCOUNT_paths", offsets.anchorPathCounter);
        SanitateBatches.SetInt("bSTART_paths", offsets.anchorPathStart);
        SanitateBatches.SetInt("bSTART_endpts", offsets.pathEndsStart);
        SanitateBatches.SetInt("bCOUNT_struct", offsets.finalStructsCounter);
        SanitateBatches.SetInt("bSTART_struct", offsets.finalStructsStart);
        SanitateBatches.SetInt("connectRadius", jigsaw.MaxConnectionDist);
        Structure.Generator.SetStructIDSettings(SanitateBatches);

        kernel = SanitateBatches.FindKernel("GetRealEndpoints");
        SanitateBatches.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "anchorPaths", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);

        kernel = SanitateBatches.FindKernel("FilterPathBatches");
        SanitateBatches.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "anchorPaths", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "overlapLines", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "batchLines", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetBuffer(kernel, "SocketCaps", UtilityBuffers.GenerationBuffer);
        SanitateBatches.SetInt("oCellOffset", originOffset);
        SanitateBatches.SetInt("bSTEP_batch", batchStep);
        SanitateBatches.SetInt("bSTART_batchCounts", offsets.batchPathCounter);
        SanitateBatches.SetInt("bSTART_capCounts", offsets.batchSocketCapCounter);
        SanitateBatches.SetInt("bSTART_batch", offsets.batchPathStart);
        SanitateBatches.SetInt("bSTART_caps", offsets.batchSocketCapStart);

        kernel = StructurePathfinder.FindKernel("BatchPathfind");
        StructurePathfinder.SetBuffer(kernel, "overlapLines", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "batchLines", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "anchorPaths", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetInt("bSTEP_batch", batchStep);
        StructurePathfinder.SetInt("bSTART_batchCounts", offsets.batchPathCounter);
        StructurePathfinder.SetInt("bSTART_batch", offsets.batchPathStart);
        StructurePathfinder.SetInt("bSTART_endpts", offsets.pathEndsStart);
        StructurePathfinder.SetInt("bSTART_paths", offsets.anchorPathStart);
        StructurePathfinder.SetInt("bSTART_anchors", offsets.anchorsStart);
        StructurePathfinder.SetInt("bSTART_visited", offsets.batchVistStart);
        Structure.Generator.SetStructIDSettings(StructurePathfinder);
        
        StructurePathfinder.SetBuffer(kernel, "genStructures", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetBuffer(kernel, "counters1", UtilityBuffers.GenerationBuffer);
        StructurePathfinder.SetInt("bCOUNT_struct", offsets.finalStructsCounter);
        StructurePathfinder.SetInt("bSTART_struct", offsets.finalStructsStart);

        kernel = PathSetupRetriever.FindKernel("SectionGrid");
        PathSetupRetriever.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "overlapLines", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchLines", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetInt("bSTEP_batch", batchStep);
        PathSetupRetriever.SetInt("bSTART_batch", offsets.batchPathStart);
        PathSetupRetriever.SetInt("bSTART_batchCounts", offsets.batchPathCounter);
        PathSetupRetriever.SetInt("bSTART_visited", offsets.batchVistStart);
        PathSetupRetriever.SetInt("bSTART_endpts", offsets.pathEndsStart);

        kernel = PathSetupRetriever.FindKernel("BacktrackGridPath");
        PathSetupRetriever.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchLines", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "SocketCaps", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "genStructures", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "anchorPaths", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "anchors", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetInt("bSTART_capCounts", offsets.batchSocketCapCounter);
        PathSetupRetriever.SetInt("bCOUNT_struct", offsets.finalStructsCounter);
        PathSetupRetriever.SetInt("bSTART_struct", offsets.finalStructsStart);
        PathSetupRetriever.SetInt("bSTART_caps", offsets.batchSocketCapStart);
        PathSetupRetriever.SetInt("bSTART_paths", offsets.anchorPathStart);
        PathSetupRetriever.SetInt("bSTART_anchors", offsets.anchorsStart);
        PathSetupRetriever.SetInt("oCellOffset", originOffset);

        kernel = PathSetupRetriever.FindKernel("CapDanglingSockets");
        PathSetupRetriever.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "SocketCaps", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "genStructures", UtilityBuffers.GenerationBuffer);
        Structure.Generator.SetStructIDSettings(PathSetupRetriever);
        Shader.SetGlobalInt("_StructTestDot", Config.CURRENT.Generation.Structures.value.   StructureDictionary.RetrieveIndex("Dot"));
        Shader.SetGlobalInt("_StructTestDot1", Config.CURRENT.Generation.Structures.value.StructureDictionary.RetrieveIndex("Dot1"));
    }

    public static bool PlanStructureSystems(int chunkSize, int depth, int3 CCoord) {
        if (depth > jigsaw.MaxSystemLoD) return false;

        int counterStart = offsets.countersRange.x;
        int counterEnd = offsets.countersRange.y;
        UtilityBuffers.ClearRange(
            UtilityBuffers.GenerationBuffer,
            counterEnd - counterStart,
            counterStart
        );
        SampleSystemAnchors(chunkSize, depth, CCoord);
        ConnectGraphAnchors(chunkSize, depth, CCoord);
        SanitateComputeBatches(chunkSize, depth, CCoord);
        PopulatePathsWithStructures(chunkSize, depth, CCoord);

        int count = ReadCounterValue(Generator.offsets.finalStructsCounter);
        UtilityBuffers.CopyBufferRegion(
            Generator.offsets.finalStructsCounter,
            Generator.offsets.finalStructsStart,
            Structure.Generator.offsets.structureCounter,
            Structure.Generator.offsets.structureStart,
            Creator.STRUCTURE_STRIDE_WORD
        );
#if UNITY_EDITOR || DEVELOPMENT_BUILD
        //LogCopiedStructureRegion();
#endif
        return true;
    }

#if UNITY_EDITOR || DEVELOPMENT_BUILD
    private const int GEN_STRUCT_WORD = 4;

    private static int ReadCounterValue(int counterOffset) {
        UtilityBuffers.CopyCount(UtilityBuffers.GenerationBuffer, UtilityBuffers.TransferBuffer, counterOffset, 0);
        UtilityBuffers.TransferBuffer.GetData(_counterReadback, 0, 0, 1);
        return _counterReadback[0];
    }

    private static void ReadBufferRegion(int start, int length, int[] destination) {
        UtilityBuffers.GenerationBuffer.GetData(destination, 0, start, length);
    }

    private static int3 DecodeRotMeta(uint rotMeta) {
        return new int3(
            (int)((rotMeta >> 2) & 0x3u),
            (int)(rotMeta & 0x3u),
            (int)((rotMeta >> 4) & 0x3u)
        );
    }

    private static int CalculateBatchEntryCapacity(int batchSize) {
        int cellsPerAxis = Mathf.CeilToInt((float)batchSize / jigsaw.CellSize);
        return cellsPerAxis * cellsPerAxis * cellsPerAxis * 6;
    }

    private static void LogBatchCounterSaturation(string label, int counterStart, int batchesPerAxis, int batchCapacity, int3 chunkCoord, int depth) {
        int totalBatches = batchesPerAxis * batchesPerAxis * batchesPerAxis;
        int[] counts = new int[totalBatches];
        ReadBufferRegion(counterStart, totalBatches, counts);

        int saturatedCount = 0;
        int maxCount = 0;
        int maxIndex = -1;
        for (int index = 0; index < totalBatches; index++) {
            int count = counts[index];
            if (count >= batchCapacity)
                saturatedCount++;
            if (count > maxCount) {
                maxCount = count;
                maxIndex = index;
            }
        }

        if (saturatedCount <= 0)
            return;

        int batchArea = batchesPerAxis * batchesPerAxis;
        int3 maxCoord = new int3(
            maxIndex / batchArea,
            (maxIndex / batchesPerAxis) % batchesPerAxis,
            maxIndex % batchesPerAxis
        );
        Debug.LogWarning(
            $"Jigsaw {label} saturation: chunk={chunkCoord}, depth={depth}, saturatedBatches={saturatedCount}/{totalBatches}, capacity={batchCapacity}, maxCount={maxCount} at batch=({maxCoord.x}, {maxCoord.y}, {maxCoord.z})"
        );
    }

    private static void LogCopiedStructureRegion() {
        int sourceCount = ReadCounterValue(offsets.finalStructsCounter);
        int destCount = ReadCounterValue(Structure.Generator.offsets.structureCounter);
        int totalStructureCount = Config.CURRENT.Generation.Structures.value.StructureDictionary.Reg.Count;

        Debug.Log($"Jigsaw copy readback: sourceCount={sourceCount}, destCount={destCount}, sourceStart={offsets.finalStructsStart}, destStart={Structure.Generator.offsets.structureStart}");

        if (destCount <= 0) {
            Debug.Log("Jigsaw copy readback: destination structure region is empty after CopyBufferRegion.");
            return;
        }

        int[] structData = new int[destCount * GEN_STRUCT_WORD];
        ReadBufferRegion(Structure.Generator.offsets.structureStart * GEN_STRUCT_WORD, structData.Length, structData);

        for (int i = 0; i < destCount; i++) {
            int baseWord = i * GEN_STRUCT_WORD;
            float posX = BitConverter.Int32BitsToSingle(structData[baseWord + 0]);
            float posY = BitConverter.Int32BitsToSingle(structData[baseWord + 1]);
            float posZ = BitConverter.Int32BitsToSingle(structData[baseWord + 2]);
            uint meta = (uint)structData[baseWord + 3];
            uint rotMeta = meta & 0x3Fu;
            uint structureIndex = meta >> 7;
            bool isFinite = !float.IsNaN(posX) && !float.IsInfinity(posX)
                && !float.IsNaN(posY) && !float.IsInfinity(posY)
                && !float.IsNaN(posZ) && !float.IsInfinity(posZ);
            bool validIndex = structureIndex < (uint)totalStructureCount;
            int3 rot = DecodeRotMeta(rotMeta);

            Debug.Log(
                $"Jigsaw copied structure[{i}]: pos=({posX}, {posY}, {posZ}), meta=0x{meta:X8}, structureIndex={structureIndex}, rot=({rot.x}, {rot.y}, {rot.z}), finite={isFinite}, validIndex={validIndex}"
            );
        }
    }
#endif


    public static void SampleSystemAnchors(int chunkSize, int depth, int3 CCoord) {
        int worldChunkSize = chunkSize * (1 << depth);
        int paddedChunkSize = worldChunkSize + jigsaw.MaxConnectionDist * 4;
        int cellsPerChunk = worldChunkSize / jigsaw.CellSize;
        int numCellsPerAxis = Mathf.CeilToInt((float)paddedChunkSize / jigsaw.CellSize);
        AnchorSampler.SetInts("oCCoord", new int[] {CCoord.x, CCoord.y, CCoord.z});
        AnchorSampler.SetInt("cellsPerChunk", cellsPerChunk);
        AnchorSampler.SetInt("numPointsPerAxis", numCellsPerAxis);
        UtilityBuffers.SetSampleData(AnchorSampler, (float3)(CCoord * worldChunkSize), 1);

        int kernel = AnchorSampler.FindKernel("SamplePoints");
        AnchorSampler.GetKernelThreadGroupSizes(kernel, out uint threadGroupSize, out uint _, out _);
        int numGroupsPerAxis = Mathf.CeilToInt(numCellsPerAxis / (float)threadGroupSize);

        AnchorSampler.Dispatch(kernel, numGroupsPerAxis, numGroupsPerAxis, numGroupsPerAxis);
        kernel = AnchorSampler.FindKernel("PoissonPrune");
        AnchorSampler.Dispatch(kernel, numGroupsPerAxis, numGroupsPerAxis, numGroupsPerAxis);
    }

    public static void ConnectGraphAnchors(int chunkSize, int depth, int3 CCoord) {
        int worldChunkSize = chunkSize * (1 << depth);
        int paddedChunkSize = worldChunkSize + jigsaw.MaxConnectionDist * 4;
        int cellsPerChunk = worldChunkSize / jigsaw.CellSize;
        int numCellsPerAxis = Mathf.CeilToInt((float)paddedChunkSize / jigsaw.CellSize);
        GraphConnector.SetInts("oCCoord", new int[] {CCoord.x, CCoord.y, CCoord.z});
        GraphConnector.SetInt("cellsPerChunk", cellsPerChunk);
        GraphConnector.SetInt("numPointsPerAxis", numCellsPerAxis);

        int kernel = GraphConnector.FindKernel("ClearSockets");
        ComputeBuffer args = UtilityBuffers.CountToArgs(GraphConnector, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        GraphConnector.DispatchIndirect(kernel, args);

        kernel = GraphConnector.FindKernel("SetSocketConnections");
        args = UtilityBuffers.CountToArgs(GraphConnector, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        GraphConnector.DispatchIndirect(kernel, args);
        
        kernel = GraphConnector.FindKernel("ConnectGraph");
        args = UtilityBuffers.CountToArgs(GraphConnector, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        GraphConnector.DispatchIndirect(kernel, args);

    #if UNITY_EDITOR || DEVELOPMENT_BUILD
        /*
        int anchorCount = ReadCounterValue(offsets.anchorDictCounter);
        int pathCount = ReadCounterValue(offsets.anchorPathCounter);
        Debug.Log($"Jigsaw graph readback: chunk={CCoord}, depth={depth}, anchors={anchorCount}, paths={pathCount}");*/
    #endif
    }

    public static void SanitateComputeBatches(int chunkSize, int depth, int3 CCoord) {
        chunkSize *= 1 << depth;
        chunkSize += jigsaw.MaxConnectionDist * 4;

        int batchSize = math.min(chunkSize, jigsaw.MaxBatchSize);
        int batchStride = batchSize - jigsaw.MaxConnectionDist;
        int batchesPerAxis = CalculateBatchAxisCount(chunkSize, batchSize, jigsaw.MaxConnectionDist);
        int batchCapacity = CalculateBatchEntryCapacity(batchSize);

        int kernel = SanitateBatches.FindKernel("SelectAnchorPieces");
        ComputeBuffer args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);

        kernel = SanitateBatches.FindKernel("GetRealEndpoints");
        args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorPathCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);

        SanitateBatches.SetInt("batchSize", batchSize);
        SanitateBatches.SetInt("numPointsPerAxis", batchesPerAxis);
        SanitateBatches.SetInt("numVoxelsPerChunk", chunkSize);

        kernel = SanitateBatches.FindKernel("FilterPathBatches");
        args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorPathCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);

    #if UNITY_EDITOR || DEVELOPMENT_BUILD
        //LogBatchCounterSaturation("path", offsets.batchPathCounter, batchesPerAxis, batchCapacity, CCoord, depth);
    #endif
    }

    public static void PopulatePathsWithStructures(int chunkSize, int depth, int3 CCoord) {
        int worldChunkSize = chunkSize * (1 << depth);
        chunkSize = worldChunkSize + jigsaw.MaxConnectionDist * 4;

        int batchSize = math.min(chunkSize, jigsaw.MaxBatchSize);
        int batchStride = batchSize - jigsaw.MaxConnectionDist;
        int batchesPerAxis = CalculateBatchAxisCount(chunkSize, batchSize, jigsaw.MaxConnectionDist);
        int batchCapacity = CalculateBatchEntryCapacity(batchSize);
        int originOffset = -(jigsaw.MaxConnectionDist * 2);
        UtilityBuffers.SetSampleData(PathSetupRetriever, (float3)(CCoord * worldChunkSize), 1);
        UtilityBuffers.SetSampleData(StructurePathfinder, (float3)(CCoord * worldChunkSize), 1);
        PathSetupRetriever.SetInt("numPointsPerAxis", batchSize);
        StructurePathfinder.SetInt("numPointsPerAxis", batchSize);
        PathSetupRetriever.SetInt("numVoxelsPerChunk", chunkSize);
        StructurePathfinder.SetInt("numVoxelsPerChunk", chunkSize);
        PathSetupRetriever.SetInt("batchSize", batchSize);
        StructurePathfinder.SetInt("batchSize", batchSize);
        ComputeBuffer args;
        //int totalPaths = ReadCounterValue(offsets.anchorPathCounter);
        //int totalAnchors = ReadCounterValue(offsets.anchorDictCounter);
        //int batchPaths = 0;

        for (int x = 0; x < batchesPerAxis; x++) {
        for (int y = 0; y < batchesPerAxis; y++) {
        for (int z = 0; z < batchesPerAxis; z++) {
            int index = CustomUtility.indexFromCoord(x,y,z, batchesPerAxis);
            int3 batchOffset = new int3(x,y,z) * batchStride + originOffset;
            
            PathSetupRetriever.SetInt("batchIndex", index);
            PathSetupRetriever.SetInts("batchOffset", new int[] {batchOffset.x, batchOffset.y, batchOffset.z});
            int kernel = PathSetupRetriever.FindKernel("SectionGrid");
            PathSetupRetriever.GetKernelThreadGroupSizes(kernel, out uint sectionGridThreads, out uint _, out _);
            int numGroupsPerAxis = Mathf.CeilToInt(batchSize / (float)sectionGridThreads);
            PathSetupRetriever.Dispatch(kernel, numGroupsPerAxis, numGroupsPerAxis, numGroupsPerAxis);
            
            //batchPaths += ReadCounterValue(offsets.batchPathCounter + index);
            kernel = StructurePathfinder.FindKernel("BatchPathfind");
            StructurePathfinder.SetInt("batchIndex", index);
            StructurePathfinder.SetInts("batchOffset", new int[] {batchOffset.x, batchOffset.y, batchOffset.z});
            // BatchPathfind consumes one whole thread group per path and uses groupId.x as the path slot.
            args = UtilityBuffers.CountToArgs(1, UtilityBuffers.GenerationBuffer, offsets.batchPathCounter + index);
            StructurePathfinder.DispatchIndirect(kernel, args);
            
            kernel = PathSetupRetriever.FindKernel("BacktrackGridPath");
            args = UtilityBuffers.CountToArgs(PathSetupRetriever, UtilityBuffers.GenerationBuffer, offsets.batchPathCounter + index, kernel);
            PathSetupRetriever.DispatchIndirect(kernel, args);

            kernel = PathSetupRetriever.FindKernel("CapDanglingSockets");
            args = UtilityBuffers.CountToArgs(PathSetupRetriever, UtilityBuffers.GenerationBuffer, offsets.batchSocketCapCounter + index, kernel);
            PathSetupRetriever.DispatchIndirect(kernel, args);
        }}}

#if UNITY_EDITOR || DEVELOPMENT_BUILD
        //LogBatchCounterSaturation("cap", offsets.batchSocketCapCounter, batchesPerAxis, batchCapacity, CCoord, depth);
#endif

        //Debug.Log($"TotalPaths {totalPaths}, Batch Paths {batchPaths}, Total Anchors {totalAnchors}");
    }

    public struct SSystemOffsets : BufferOffsets {
        public int anchorDictCounter;
        public int anchorPathCounter;
        public int finalStructsCounter;
        public int batchPathCounter;
        public int batchSocketCapCounter;
        public int2 countersRange;

        public int anchorsStart;
        public int anchorDictStart;
        public int socketUsageStart;
        public int anchorPathStart;
        public int pathEndsStart;
        public int batchPathStart;
        public int batchSocketCapStart;
        public int batchVistStart;
        public int finalStructsStart;

        private int offsetStart; private int offsetEnd;
        /// <summary> The start of the buffer region that is used by the ssystem generator. 
        /// See <see cref="offsetss.bufferStart"/> for more info. </summary>
        public int bufferStart{get{return offsetStart;}} 
        /// <summary> The end of the buffer region that is used by the ssystem generator. 
        /// See <see cref="offsetss.bufferEnd"/> for more info. </summary>
        public int bufferEnd{get{return offsetEnd;}}

        const int ANCHOR_STRIDE_WORD = 3 + 1 + 1;
        const int ANCHOR_DICT_WORD = 1;
        const int SOCKET_USAGE_WORD = 6;
        const int ANCHOR_PATH_WORD = 3;
        const int PATH_ENDS_WORD = 6;
        const int PATH_INDEX_WORD = 1;
        const int STRUCT_SOCKET_WORD = 3;
        const int VISITED_NODE_WORD = 2;

        const int GEN_STRUCT_WORD = 4;
        const int AVG_STRUCTS_PER_PATH = 3;

        //... this buffer is sectioned way too much lol
        public SSystemOffsets(int maxChunkAxis, int maxPathLength, int batchSize, int cellSize, int bufferStart) {
            maxChunkAxis += maxPathLength * 4;
            int maxBatchAxis = CalculateBatchAxisCount(maxChunkAxis, batchSize, maxPathLength);
            int maxCellsPerChunkAxis = Mathf.CeilToInt((float)maxChunkAxis / cellSize);
            int maxCellsPerBatchAxis = Mathf.CeilToInt((float)batchSize / cellSize);

            int maxPointsPerBatch = batchSize * batchSize * batchSize;
            int maxBatchesPerChunk = maxBatchAxis * maxBatchAxis * maxBatchAxis;
            int maxCellsPerChunk = maxCellsPerChunkAxis * maxCellsPerChunkAxis * maxCellsPerChunkAxis;
            int maxCellsPerBatch = maxCellsPerBatchAxis * maxCellsPerBatchAxis * maxCellsPerBatchAxis;
            int maxPathsPerBatch = maxCellsPerBatch * 6;
            int maxPathsPerChunk = maxCellsPerChunk * 6;

            this.offsetStart = bufferStart;
            anchorDictCounter = 0;
            anchorPathCounter = 1;
            finalStructsCounter = 2;
            batchPathCounter = finalStructsCounter + maxBatchesPerChunk;
            batchSocketCapCounter = batchPathCounter + maxBatchesPerChunk;
            countersRange = new (anchorDictCounter, batchSocketCapCounter + maxBatchesPerChunk);

            anchorsStart = Mathf.CeilToInt((float)countersRange.y / ANCHOR_STRIDE_WORD);
            int AnchorEndInd_W = (anchorsStart + maxCellsPerChunk) * ANCHOR_STRIDE_WORD;

            anchorDictStart = Mathf.CeilToInt((float)AnchorEndInd_W / ANCHOR_DICT_WORD);
            int AnchorDictEndInd_W = (anchorDictStart + maxCellsPerChunk) * ANCHOR_DICT_WORD;

            socketUsageStart = Mathf.CeilToInt((float)AnchorDictEndInd_W / SOCKET_USAGE_WORD);
            int SocketUsageEndInd_W = (socketUsageStart + maxCellsPerChunk) * SOCKET_USAGE_WORD;

            anchorPathStart = Mathf.CeilToInt((float)SocketUsageEndInd_W / ANCHOR_PATH_WORD);
            int AnchorPathEndInd_W = (anchorPathStart + maxPathsPerChunk) * ANCHOR_PATH_WORD;

            pathEndsStart = Mathf.CeilToInt((float)AnchorPathEndInd_W / PATH_ENDS_WORD);
            int PathEndsEndInd_W = (pathEndsStart + maxPathsPerChunk) * PATH_ENDS_WORD;

            batchPathStart = Mathf.CeilToInt((float)PathEndsEndInd_W / PATH_INDEX_WORD);
            int BatchPathsEndInd_W = (batchPathStart + maxBatchesPerChunk * maxPathsPerBatch) * PATH_INDEX_WORD;

            //This one is not mathematically accurate upper bound but an estimate
            batchSocketCapStart = Mathf.CeilToInt((float)BatchPathsEndInd_W / STRUCT_SOCKET_WORD); 
            int BatchSocketEndInd_W = (batchSocketCapStart + maxBatchesPerChunk * maxPathsPerBatch) * STRUCT_SOCKET_WORD;

            batchVistStart = Mathf.CeilToInt((float)BatchSocketEndInd_W / VISITED_NODE_WORD); 
            int BatchVisitedEndInd_W = (batchVistStart + maxPointsPerBatch) * VISITED_NODE_WORD;

            finalStructsStart = Mathf.CeilToInt((float)BatchVisitedEndInd_W / GEN_STRUCT_WORD);
            offsetEnd = (finalStructsStart + maxPathsPerChunk * AVG_STRUCTS_PER_PATH) * GEN_STRUCT_WORD;
        }
    }
}
}