using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Utils;

[CreateAssetMenu(menuName = "ShaderData/GrassShader/Generator")]
public class ProceduralGrassRenderer : SpecialShader
{
    public GrassSettings grassSettings = default;

    private Queue<ComputeBuffer> tempBuffers = new Queue<ComputeBuffer>();

    private int idGrassKernel;
    private int idIndirectArgsKernel;

    public override Material GetMaterial()
    {
        return grassSettings.material;
    }

    public override void ProcessGeoShader(Transform transform, MemoryBufferSettings memoryHandle, int vertAddress, int triAddress, 
                        int baseGeoStart, int baseGeoCount, int geoCounter, int geoStart)
    {
        idGrassKernel = grassSettings.grassComputeShader.FindKernel("Main");
        idIndirectArgsKernel = grassSettings.indirectArgsShader.FindKernel("Main");
        ComputeBuffer memory = memoryHandle.AccessStorage();
        ComputeBuffer addresses = memoryHandle.AccessAddresses();

        ComputeBuffer args = UtilityBuffers.PrefixCountToArgs(grassSettings.grassComputeShader, UtilityBuffers.GenerationBuffer, baseGeoCount);

        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "SourceVertices", memory);
        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "SourceTriangles", memory);
        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "_AddressDict", addresses); 
        grassSettings.grassComputeShader.SetInt("vertAddress", vertAddress);
        grassSettings.grassComputeShader.SetInt("triAddress", triAddress);

        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "counters", UtilityBuffers.GenerationBuffer);
        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "BaseTriangles", UtilityBuffers.GenerationBuffer);
        grassSettings.grassComputeShader.SetBuffer(idGrassKernel, "DrawTriangles", UtilityBuffers.GenerationBuffer);
        grassSettings.grassComputeShader.SetInt("bSTART_base", baseGeoStart);
        grassSettings.grassComputeShader.SetInt("bCOUNT_base", baseGeoCount);
        grassSettings.grassComputeShader.SetInt("bSTART_oGeo", geoStart);
        grassSettings.grassComputeShader.SetInt("bCOUNT_oGeo", geoCounter);

        grassSettings.grassComputeShader.SetFloat("_TotalHeight", grassSettings.grassHeight);
        grassSettings.grassComputeShader.SetFloat("_WorldPositionToUVScale", grassSettings.worldPositionUVScale);
        grassSettings.grassComputeShader.SetInt("_MaxLayers", grassSettings.maxLayers);
        grassSettings.grassComputeShader.SetMatrix("_LocalToWorld", transform.localToWorldMatrix);

        grassSettings.grassComputeShader.DispatchIndirect(idGrassKernel, args);
    }

    public override void ReleaseTempBuffers()
    {
        while (tempBuffers.Count > 0)
        {
            tempBuffers.Dequeue().Release();
        }
    }


    public Bounds TransformBounds(Bounds boundsOS, Transform transform)
    {
        var center = transform.TransformPoint(boundsOS.center);

        var size = boundsOS.size;
        var axisX = transform.TransformVector(size.x, 0, 0);
        var axisY = transform.TransformVector(0, size.y, 0);
        var axisZ = transform.TransformVector(0, 0, size.z);

        size.x = Mathf.Abs(axisX.x) + Mathf.Abs(axisY.x) + Mathf.Abs(axisZ.x);
        size.y = Mathf.Abs(axisX.y) + Mathf.Abs(axisY.y) + Mathf.Abs(axisZ.y);
        size.z = Mathf.Abs(axisX.z) + Mathf.Abs(axisY.z) + Mathf.Abs(axisZ.z);

        return new Bounds(center, size);
    }
}