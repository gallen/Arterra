using System.Collections;
using System.Collections.Generic;
using Newtonsoft.Json;
using UnityEngine;

[CreateAssetMenu(menuName = "ShaderData/SnowShader/Generator")]
public class ProceduralSnowRenderer : SpecialShader
{
    public SnowSettings snowSettings = default;

    [System.Serializable]
    public struct SnowSettings
    {
        [Tooltip("How much to tesselate base mesh")]
        public uint tesselationFactor;//3

        [Tooltip("The tesselation compute shader")][JsonIgnore][UIgnore]
        public Option<ComputeShader> tesselComputeShader;
        [JsonIgnore][UIgnore]
        public Option<Material> material;
    }
    // Start is called before the first frame update
    public override Material GetMaterial()
    {
        return snowSettings.material.value;
    }
    public override void ProcessGeoShader(Transform transform, GenerationPreset.MemoryHandle memoryHandle, int vertAddress, int triAddress, 
                        int baseGeoStart, int baseGeoCount, int geoCounter, int geoStart, int geoInd)
    {
        ComputeShader tesselCompute = snowSettings.tesselComputeShader.value;

        int idGrassKernel = tesselCompute.FindKernel("Main");
        ComputeBuffer memory = memoryHandle.AccessStorage();
        ComputeBuffer addresses = memoryHandle.AccessAddresses();

        ComputeBuffer args = UtilityBuffers.PrefixCountToArgs(tesselCompute, UtilityBuffers.GenerationBuffer, baseGeoCount);

        tesselCompute.SetBuffer(idGrassKernel, "SourceVertices", memory);
        tesselCompute.SetBuffer(idGrassKernel, "SourceTriangles", memory);
        tesselCompute.SetBuffer(idGrassKernel, "_AddressDict", addresses); 
        tesselCompute.SetInt("vertAddress", vertAddress);
        tesselCompute.SetInt("triAddress", triAddress);
        tesselCompute.SetInt("geoInd", geoInd);

        tesselCompute.SetBuffer(idGrassKernel, "counters", UtilityBuffers.GenerationBuffer);
        tesselCompute.SetBuffer(idGrassKernel, "BaseTriangles", UtilityBuffers.GenerationBuffer);
        tesselCompute.SetBuffer(idGrassKernel, "DrawTriangles", UtilityBuffers.GenerationBuffer);
        tesselCompute.SetInt("bSTART_base", baseGeoStart);
        tesselCompute.SetInt("bCOUNT_base", baseGeoCount);
        tesselCompute.SetInt("bSTART_oGeo", geoStart);
        tesselCompute.SetInt("bCOUNT_oGeo", geoCounter);

        tesselCompute.SetInt("tesselFactor", (int)snowSettings.tesselationFactor);
        tesselCompute.SetMatrix("_LocalToWorld", transform.localToWorldMatrix);

        tesselCompute.DispatchIndirect(idGrassKernel, args);
    }
}