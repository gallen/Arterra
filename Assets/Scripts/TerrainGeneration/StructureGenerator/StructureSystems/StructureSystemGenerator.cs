using Arterra.Configuration;
using Arterra.Data.Structure.Jigsaw;
using Arterra.Utils;
using Arterra.Editor;
using Unity.Mathematics;
using UnityEngine;
using Newtonsoft.Json;
using FMOD.Studio;
using System;

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
        AnchorSampler.SetInt("oCellOffset", originOffset);
        AnchorSampler.SetInt("cellsPerChunk", cellsPerChunk);
        AnchorSampler.SetInt("bSTART_anchors", offsets.anchorsStart);
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
        SanitateBatches.SetInt("bCOUNT_dict", offsets.anchorDictCounter);
        SanitateBatches.SetInt("bSTART_dict", offsets.anchorDictStart);
        SanitateBatches.SetInt("bSTART_anchors", offsets.anchorsStart);
        SanitateBatches.SetInt("bSTART_sockets", offsets.socketUsageStart);
        SanitateBatches.SetInt("bCOUNT_paths", offsets.anchorPathCounter);
        SanitateBatches.SetInt("bSTART_paths", offsets.anchorPathStart);
        SanitateBatches.SetInt("bSTART_endpts", offsets.pathEndsStart);
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
        SanitateBatches.SetInt("bSTART_olpCounts", offsets.batchOverlapCounter);
        SanitateBatches.SetInt("bSTART_batchCounts", offsets.batchPathCounter);
        SanitateBatches.SetInt("bSTART_capCounts", offsets.batchSocketCapCounter);
        SanitateBatches.SetInt("bSTART_overlap", offsets.batchOverlapStart);
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

        kernel = PathSetupRetriever.FindKernel("SectionGrid");
        PathSetupRetriever.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "endPoints", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "overlapLines", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchLines", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetInt("bSTEP_batch", batchStep);
        PathSetupRetriever.SetInt("bSTART_batch", offsets.batchPathStart);
        PathSetupRetriever.SetInt("bSTART_overlap", offsets.batchOverlapStart);
        PathSetupRetriever.SetInt("bSTART_olpCounts", offsets.batchOverlapCounter);
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

        kernel = PathSetupRetriever.FindKernel("CapDanglingSockets");
        PathSetupRetriever.SetBuffer(kernel, "counters", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "batchVisit", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "SocketCaps", UtilityBuffers.GenerationBuffer);
        PathSetupRetriever.SetBuffer(kernel, "genStructures", UtilityBuffers.GenerationBuffer);
        Structure.Generator.SetStructIDSettings(PathSetupRetriever);
    }

    public static bool PlanStructureSystems(int chunkSize, int depth, int3 CCoord) {
        if (depth > jigsaw.MaxSystemLoD) return false;
        int expandedChunkSize = (chunkSize * (1 << depth)) + (jigsaw.MaxConnectionDist * 4);
        int maxCellsPerAxis = Mathf.CeilToInt((float)expandedChunkSize / jigsaw.CellSize);
        int maxCells = maxCellsPerAxis * maxCellsPerAxis * maxCellsPerAxis;

        int counterStart = offsets.countersRange.x;
        int counterEnd = offsets.countersRange.y;
        UtilityBuffers.ClearRange(
            UtilityBuffers.GenerationBuffer,
            counterEnd - counterStart,
            counterStart
        );
        //ValidateCountersCleared(counterStart, counterEnd);

        SampleSystemAnchors(chunkSize, depth, CCoord);
        //ValidateSampleAnchorsState(maxCells);

        ConnectGraphAnchors(chunkSize, depth, CCoord);
        //ValidateConnectedGraphState(maxCells);

        SanitateComputeBatches(chunkSize, depth, CCoord);
        int batchSize = math.min(expandedChunkSize, jigsaw.MaxBatchSize);
        int batchStride = batchSize - jigsaw.MaxConnectionDist;
        int batchesPerAxis = (expandedChunkSize - batchSize) / batchStride + 1;
        int maxBatches = batchesPerAxis * batchesPerAxis * batchesPerAxis;
        //ValidateSanitatedBatchesState(maxBatches, maxCells);
        PopulatePathsWithStructures(chunkSize, depth, CCoord);
        ValidatePopulatePathsState(maxBatches, maxCells);
        return true;
    }

#if UNITY_EDITOR || DEVELOPMENT_BUILD
    private static int ReadCounterValue(int counterOffset) {
        UtilityBuffers.CopyCount(UtilityBuffers.GenerationBuffer, UtilityBuffers.TransferBuffer, counterOffset, 0);
        UtilityBuffers.TransferBuffer.GetData(_counterReadback, 0, 0, 1);
        return _counterReadback[0];
    }

    private static void Require(bool condition, string message) {
        if (condition) return;
        throw new InvalidOperationException($"StructureSystem sanity check failed: {message}");
    }

    private static void ReadBufferRegion(int start, int length, int[] destination) {
        UtilityBuffers.GenerationBuffer.GetData(destination, 0, start, length);
    }

    private static void ValidateCountersCleared(int counterStart, int counterEnd) {
        int length = math.max(0, counterEnd - counterStart);
        if (length == 0) return;

        int[] counters = new int[length];
        ReadBufferRegion(counterStart, length, counters);
        for (int i = 0; i < counters.Length; i++) {
            Require(counters[i] == 0, $"counter[{counterStart + i}] expected 0 after clear, got {counters[i]}");
        }
    }

    private static void ValidateSampleAnchorsState(int maxCells) {
        int dictCount = ReadCounterValue(offsets.anchorDictCounter);
        Require(dictCount >= 0 && dictCount <= maxCells,
            $"anchorDictCounter out of bounds ({dictCount}, expected [0, {maxCells}])");
        
        if (dictCount == 0) return;

        int sampleCount = math.min(dictCount, SANITY_SAMPLE_COUNT);
        int[] anchorIndices = new int[sampleCount];
        ReadBufferRegion(offsets.anchorDictStart, sampleCount, anchorIndices);

        int[] anchorData = new int[ANCHOR_STRIDE_WORD];
        // offsets.anchorsStart is in anchor-element coordinates (GPU struct indexing)
        // Convert to word offset: GPU element M -> word M * ANCHOR_STRIDE_WORD
        int anchorsStartWordOffset = offsets.anchorsStart * ANCHOR_STRIDE_WORD;
        
        for (int i = 0; i < sampleCount; i++) {
            int anchorIndex = anchorIndices[i];
            Require(anchorIndex >= 0 && anchorIndex < maxCells,
                $"anchorDict[{i}] has invalid anchor index {anchorIndex}, expected [0, {maxCells})");

            // Each anchor is ANCHOR_STRIDE_WORD ints; compute absolute word position in buffer
            int anchorWordStart = anchorsStartWordOffset + anchorIndex * ANCHOR_STRIDE_WORD;
            ReadBufferRegion(anchorWordStart, ANCHOR_STRIDE_WORD, anchorData);
            int systemId = anchorData[3];
            Require(systemId >= 0,
                $"anchor[{anchorIndex}] referenced by anchorDict has invalid system id {systemId}");
        }
    }

    private static void ValidateConnectedGraphState(int maxCells) {
        int dictCount = ReadCounterValue(offsets.anchorDictCounter);
        int pathCount = ReadCounterValue(offsets.anchorPathCounter);
        int maxPaths = maxCells * 6;

        Require(pathCount >= 0 && pathCount <= maxPaths,
            $"anchorPathCounter out of bounds ({pathCount}, expected [0, {maxPaths}])");

        if (pathCount == 0) return;

        int sampleCount = math.min(pathCount, SANITY_SAMPLE_COUNT);
        int[] pathData = new int[sampleCount * ANCHOR_PATH_WORD];
        
        // offsets.anchorPathStart is in path-element coordinates; convert to word offset
        int anchorPathStartWordOffset = offsets.anchorPathStart * ANCHOR_PATH_WORD;
        ReadBufferRegion(anchorPathStartWordOffset, pathData.Length, pathData);
        
        for (int i = 0; i < sampleCount; i++) {
            int baseOffset = i * ANCHOR_PATH_WORD;
            int startIndex = pathData[baseOffset + 0];
            int endIndex = pathData[baseOffset + 1];
            int socketMask = pathData[baseOffset + 2];

            Require(startIndex >= 0 && startIndex < maxCells,
                $"anchorPaths[{i}].x invalid ({startIndex}, expected [0, {maxCells}))");
            Require(endIndex >= 0 && endIndex < maxCells,
                $"anchorPaths[{i}].y invalid ({endIndex}, expected [0, {maxCells}))");
            Require(socketMask != 0,
                $"anchorPaths[{i}].z expected non-zero socket mask");
        }

        int[] socketSample = new int[math.min(dictCount, SANITY_SAMPLE_COUNT)];
        if (socketSample.Length > 0) {
            // offsets.socketUsageStart is in socket-element coordinates  
            int socketUsageStartWordOffset = offsets.socketUsageStart * 1;  // SOCKET_USAGE_WORD = 1
            ReadBufferRegion(socketUsageStartWordOffset, socketSample.Length, socketSample);
            bool anyUsed = false;
            for (int i = 0; i < socketSample.Length; i++) {
                if (socketSample[i] != 0) {
                    anyUsed = true;
                    break;
                }
            }
            Require(anyUsed || pathCount == 0,
                "socket usage region did not receive any updates after graph connection");
        }
    }

    private static void ValidateSanitatedBatchesState(int maxBatches, int maxCells) {
        int pathCount = ReadCounterValue(offsets.anchorPathCounter);
        Require(pathCount >= 0 && pathCount <= maxCells * 6,
            $"anchorPathCounter invalid before batch sanitation ({pathCount})");

        int[] overlapCounts = new int[maxBatches];
        int[] batchCounts = new int[maxBatches];
        int[] capCounts = new int[maxBatches];

        ReadBufferRegion(offsets.batchOverlapCounter, maxBatches, overlapCounts);
        ReadBufferRegion(offsets.batchPathCounter, maxBatches, batchCounts);
        ReadBufferRegion(offsets.batchSocketCapCounter, maxBatches, capCounts);
        int maxBatchEntries = Mathf.CeilToInt((float)jigsaw.MaxBatchSize / jigsaw.CellSize);
        maxBatchEntries = maxBatchEntries * maxBatchEntries * maxBatchEntries * 6;

        int overlapTotal = 0;
        int batchTotal = 0;
        int capTotal = 0;

        for (int i = 0; i < maxBatches; i++) {
            Require(overlapCounts[i] >= 0, $"batch overlap count at {i} is negative ({overlapCounts[i]})");
            Require(batchCounts[i] >= 0, $"batch path count at {i} is negative ({batchCounts[i]})");
            Require(capCounts[i] >= 0, $"batch cap count at {i} is negative ({capCounts[i]})");
            Require(overlapCounts[i] <= maxBatchEntries,
                $"batch overlap count at {i} exceeds capacity ({overlapCounts[i]} > {maxBatchEntries})");
            Require(batchCounts[i] <= maxBatchEntries,
                $"batch path count at {i} exceeds capacity ({batchCounts[i]} > {maxBatchEntries})");
            Require(capCounts[i] <= maxBatchEntries,
                $"batch cap count at {i} exceeds capacity ({capCounts[i]} > {maxBatchEntries})");

            overlapTotal += overlapCounts[i];
            batchTotal += batchCounts[i];
            capTotal += capCounts[i];
        }

        if (pathCount > 0) {
            Require(overlapTotal > 0 || batchTotal > 0 || capTotal > 0,
                $"batch sanitation produced no overlap/path/cap assignments despite non-zero({pathCount}) paths");
        }
    }

    private static void ValidatePopulatePathsState(int maxBatches, int maxCells) {
        int finalStructCount = ReadCounterValue(offsets.finalStructsCounter);
        int maxPaths = maxCells * 6;
        int maxBatchEntries = Mathf.CeilToInt((float)jigsaw.MaxBatchSize / jigsaw.CellSize);
        maxBatchEntries = maxBatchEntries * maxBatchEntries * maxBatchEntries * 6;
        const int GEN_STRUCT_WORD = 4;

        Debug.Log(finalStructCount);
        Require(finalStructCount >= 0,
            $"finalStructsCounter is negative ({finalStructCount})");
        Require(finalStructCount <= maxPaths * 6,
            $"finalStructsCounter exceeds plausible bound ({finalStructCount} > {maxPaths * 6})");

        int[] batchPathCounts = new int[maxBatches];
        int[] batchCapCounts = new int[maxBatches];
        ReadBufferRegion(offsets.batchPathCounter, maxBatches, batchPathCounts);
        ReadBufferRegion(offsets.batchSocketCapCounter, maxBatches, batchCapCounts);

        for (int i = 0; i < maxBatches; i++) {
            Require(batchPathCounts[i] >= 0,
                $"batchPathCounter[{i}] is negative ({batchPathCounts[i]}) after pathfind");
            Require(batchPathCounts[i] <= maxBatchEntries,
                $"batchPathCounter[{i}] exceeds capacity ({batchPathCounts[i]} > {maxBatchEntries}) after pathfind");
            Require(batchCapCounts[i] >= 0,
                $"batchSocketCapCounter[{i}] is negative ({batchCapCounts[i]}) after pathfind");
            Require(batchCapCounts[i] <= maxBatchEntries,
                $"batchSocketCapCounter[{i}] exceeds capacity ({batchCapCounts[i]} > {maxBatchEntries}) after pathfind");
        }

        if (finalStructCount == 0) return;

        // Count total jigsaw system structures for structure-index bounds checking
        // meta >> 7 is an index into the flat list of all JigsawSystem.Structures entries
        int totalSysStructCount = Config.CURRENT.Generation.Structures.value.StructureDictionary.Reg.Count;

        // Read ALL generated structs: layout is float3 structurePos (words 0-2) + uint meta (word 3)
        int[] structData = new int[finalStructCount * GEN_STRUCT_WORD];
        ReadBufferRegion(offsets.finalStructsStart * GEN_STRUCT_WORD, structData.Length, structData);

        for (int i = 0; i < finalStructCount; i++) {
            int baseWord = i * GEN_STRUCT_WORD;
            float posX = BitConverter.Int32BitsToSingle(structData[baseWord + 0]);
            float posY = BitConverter.Int32BitsToSingle(structData[baseWord + 1]);
            float posZ = BitConverter.Int32BitsToSingle(structData[baseWord + 2]);
            uint meta  = (uint)structData[baseWord + 3];
            uint structIndex = meta >> 7;

            Require(!float.IsNaN(posX) && !float.IsInfinity(posX),
                $"genStructures[{i}].structurePos.x is non-finite ({posX})");
            Require(!float.IsNaN(posY) && !float.IsInfinity(posY),
                $"genStructures[{i}].structurePos.y is non-finite ({posY})");
            Require(!float.IsNaN(posZ) && !float.IsInfinity(posZ),
                $"genStructures[{i}].structurePos.z is non-finite ({posZ})");
            Require(totalSysStructCount == 0 || structIndex < (uint)totalSysStructCount,
                $"genStructures[{i}].meta structureIndex {structIndex} out of range [0, {totalSysStructCount})");
        }
    }
#else
    private static void ValidateCountersCleared(int counterStart, int counterEnd) {}
    private static void ValidateSampleAnchorsState(int maxCells) {}
    private static void ValidateConnectedGraphState(int maxCells) {}
    private static void ValidateSanitatedBatchesState(int maxBatches, int maxCells, int maxBatchEntries) {}
    private static void ValidatePopulatePathsState(int maxBatches, int maxCells) {}
#endif

    public static void SampleSystemAnchors(int chunkSize, int depth, int3 CCoord) {
        chunkSize *= 1 << depth;
        chunkSize += jigsaw.MaxConnectionDist * 4;
        int numCellsPerAxis = Mathf.CeilToInt((float)chunkSize / jigsaw.CellSize);
        AnchorSampler.SetInts("oCCoord", new int[] {CCoord.x, CCoord.y, CCoord.z});
        AnchorSampler.SetInt("numPointsPerAxis", numCellsPerAxis);
        UtilityBuffers.SetSampleData(AnchorSampler, (float3)(CCoord * chunkSize), 1);

        int kernel = AnchorSampler.FindKernel("SamplePoints");
        AnchorSampler.GetKernelThreadGroupSizes(kernel, out uint threadGroupSize, out uint _, out _);
        int numGroupsPerAxis = Mathf.CeilToInt(numCellsPerAxis / (float)threadGroupSize);

        AnchorSampler.Dispatch(kernel, numGroupsPerAxis, numGroupsPerAxis, numGroupsPerAxis);
        kernel = AnchorSampler.FindKernel("PoissonPrune");
        AnchorSampler.Dispatch(kernel, numGroupsPerAxis, numGroupsPerAxis, numGroupsPerAxis);
    }

    public static void ConnectGraphAnchors(int chunkSize, int depth, int3 CCoord) {
        chunkSize *= 1 << depth;
        chunkSize += jigsaw.MaxConnectionDist * 4;
        int numCellsPerAxis = Mathf.CeilToInt((float)chunkSize / jigsaw.CellSize);
        GraphConnector.SetInts("oCCoord", new int[] {CCoord.x, CCoord.y, CCoord.z});
        GraphConnector.SetInt("numPointsPerAxis", numCellsPerAxis);

        int kernel = GraphConnector.FindKernel("ClearSockets");
        ComputeBuffer args = UtilityBuffers.CountToArgs(GraphConnector, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        GraphConnector.DispatchIndirect(kernel, args);

        kernel = GraphConnector.FindKernel("ConnectGraph");
        args = UtilityBuffers.CountToArgs(GraphConnector, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        GraphConnector.DispatchIndirect(kernel, args);
    }

    public static void SanitateComputeBatches(int chunkSize, int depth, int3 CCoord) {
        chunkSize *= 1 << depth;
        chunkSize += jigsaw.MaxConnectionDist * 4;

        int batchSize = math.min(chunkSize, jigsaw.MaxBatchSize);
        int batchStride = batchSize - jigsaw.MaxConnectionDist;
        int batchesPerAxis = (chunkSize - batchSize) / batchStride + 1;

        int kernel = SanitateBatches.FindKernel("SelectAnchorPieces");
        ComputeBuffer args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorDictCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);

        kernel = SanitateBatches.FindKernel("GetRealEndpoints");
        args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorPathCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);

        SanitateBatches.SetInt("batchSize", batchSize);
        SanitateBatches.SetInt("numPointsPerAxis", batchesPerAxis);

        kernel = SanitateBatches.FindKernel("FilterPathBatches");
        args = UtilityBuffers.CountToArgs(SanitateBatches, UtilityBuffers.GenerationBuffer, offsets.anchorPathCounter, kernel);
        SanitateBatches.DispatchIndirect(kernel, args);
    }

    public static void PopulatePathsWithStructures(int chunkSize, int depth, int3 CCoord) {
        chunkSize *= 1 << depth;
        chunkSize += jigsaw.MaxConnectionDist * 4;

        int batchSize = math.min(chunkSize, jigsaw.MaxBatchSize);
        int batchStride = batchSize - jigsaw.MaxConnectionDist;
        int batchesPerAxis = (chunkSize - batchSize) / batchStride + 1;
        int originOffset = -(jigsaw.MaxConnectionDist * 2);
        UtilityBuffers.SetSampleData(PathSetupRetriever, (float3)(CCoord * chunkSize), 1);
        UtilityBuffers.SetSampleData(StructurePathfinder, (float3)(CCoord * chunkSize), 1);
        PathSetupRetriever.SetInt("numPointsPerAxis", batchSize);
        StructurePathfinder.SetInt("numPointsPerAxis", batchSize);
        PathSetupRetriever.SetInt("batchSize", batchSize);
        StructurePathfinder.SetInt("batchSize", batchSize);
        ComputeBuffer args;

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

            kernel = StructurePathfinder.FindKernel("BatchPathfind");
            StructurePathfinder.SetInt("batchIndex", index);
            StructurePathfinder.SetInts("batchOffset", new int[] {batchOffset.x, batchOffset.y, batchOffset.z});
            args = UtilityBuffers.CountToArgs(StructurePathfinder, UtilityBuffers.GenerationBuffer, offsets.batchPathCounter + index, kernel);
            StructurePathfinder.DispatchIndirect(kernel, args);

            kernel = PathSetupRetriever.FindKernel("BacktrackGridPath");
            args = UtilityBuffers.CountToArgs(PathSetupRetriever, UtilityBuffers.GenerationBuffer, offsets.batchPathCounter + index, kernel);
            PathSetupRetriever.DispatchIndirect(kernel, args);

            kernel = PathSetupRetriever.FindKernel("CapDanglingSockets");
            args = UtilityBuffers.CountToArgs(PathSetupRetriever, UtilityBuffers.GenerationBuffer, offsets.batchSocketCapCounter + index, kernel);
            PathSetupRetriever.DispatchIndirect(kernel, args);
        }}}
    }

    public struct SSystemOffsets : BufferOffsets {
        public int anchorDictCounter;
        public int anchorPathCounter;
        public int finalStructsCounter;
        public int batchOverlapCounter;
        public int batchPathCounter;
        public int batchSocketCapCounter;
        public int2 countersRange;

        public int anchorsStart;
        public int anchorDictStart;
        public int socketUsageStart;
        public int anchorPathStart;
        public int pathEndsStart;
        public int batchOverlapStart;
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
        const int SOCKET_USAGE_WORD = 1;
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
            int maxBatchAxis = (maxChunkAxis - batchSize) / (batchSize - maxPathLength) + 1;
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
            batchOverlapCounter = 3;
            batchPathCounter = batchOverlapCounter + maxBatchesPerChunk;
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

            batchOverlapStart = Mathf.CeilToInt((float)PathEndsEndInd_W / PATH_INDEX_WORD);
            int BatchOverlapEndInd_W = (batchOverlapStart + maxBatchesPerChunk * maxPathsPerBatch) * PATH_INDEX_WORD;

            batchPathStart = Mathf.CeilToInt((float)BatchOverlapEndInd_W / PATH_INDEX_WORD);
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