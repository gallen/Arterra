using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Mathematics;
using System.Linq;
using UnityEngine.Rendering;
using static CPUDensityManager;
using static PlayerHandler;
using Newtonsoft.Json;


[Serializable]
public class TerraformSettings : ICloneable{
    public int terraformRadius = 5;
    public float terraformSpeed = 4;
    public float maxTerraformDistance = 60;

    public float CursorSize = 2;
    public Vec4 CursorColor = new (0, 1, 0, 0.5f);
    public bool ShowCursor = true;

    public object Clone(){
        return new TerraformSettings{
            terraformRadius = this.terraformRadius,
            terraformSpeed = this.terraformSpeed,
            maxTerraformDistance = this.maxTerraformDistance,
        };
    }
}

public class TerraformController
{
    private TerraformSettings settings;
    private bool shiftPressed = false;
    private bool hasHit;
    float3 hitPoint;

    int IsoLevel;

    Material OverlayMaterial;
    Mesh SphereMesh;
    Transform cam;



    // Start is called before the first frame update
    public TerraformController()
    {
        settings = WorldStorageHandler.WORLD_OPTIONS.GamePlay.Terraforming.value;
        cam = Camera.main.transform;

        IsoLevel = Mathf.RoundToInt(WorldStorageHandler.WORLD_OPTIONS.Quality.Rendering.value.IsoLevel * 255);
        SetUpOverlay();

        InputPoller.AddBinding(new InputPoller.Binding("Place Liquid", "GamePlay", InputPoller.BindPoll.Down, (_) => shiftPressed = true));
        InputPoller.AddBinding(new InputPoller.Binding("Place Liquid", "GamePlay", InputPoller.BindPoll.Up, (_) => shiftPressed = false));
        InputPoller.AddBinding(new InputPoller.Binding("Place Terrain", "GamePlay", InputPoller.BindPoll.Hold, PlaceTerrain));
        InputPoller.AddBinding(new InputPoller.Binding("Remove Terrain", "GamePlay", InputPoller.BindPoll.Hold, RemoveTerrain));
    }


    public void Update()
    {
        RayTest();
        DrawOverlays();
    }

    void SetUpOverlay(){
        Shader shader = Shader.Find("Hidden/Internal-Colored");
        OverlayMaterial = new Material(shader);
        OverlayMaterial.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
        OverlayMaterial.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
        OverlayMaterial.SetInt("_Cull", (int)CullMode.Back);
        OverlayMaterial.SetInt("_ZTest", (int)CompareFunction.Less);
        OverlayMaterial.SetInt("_ZWrite", 1);

        float3[] positions = GenerateIcosphere();


        Vector3[] vertices = new Vector3[positions.Length];
        int[] triangles = new int[positions.Length];
        
        for (int i = 0; i < positions.Length; i++){
            vertices[i] = math.normalize(positions[i]);
            triangles[i] = i;
        }

        SphereMesh = new Mesh{
            vertices = vertices,
            triangles = triangles
        };
    }

    float3[] GenerateIcosphere(){
        float phi = (1.0f + math.sqrt(5.0f)) / 2.0f;
        float a = 1.0f;
        float b = 1.0f / phi;

        float3 v1  = new (0, b, -a);
        float3 v2  = new(b, a, 0);
        float3 v3  = new(-b, a, 0);
        float3 v4  = new(0, b, a);
        float3 v5  = new(0, -b, a);
        float3 v6  = new(-a, 0, b);
        float3 v7  = new(0, -b, -a);
        float3 v8  = new(a, 0, -b);
        float3 v9  = new(a, 0, b);
        float3 v10 = new(-a, 0, -b);
        float3 v11 = new(b, -a, 0);
        float3 v12 = new(-b, -a, 0);

        float3[] mesh = new float3[]{
            v3, v2, v1,
            v2, v3, v4,
            v6, v5, v4,
            v5, v9, v4,
            v8, v7, v1,
            v7, v10, v1,
            v12, v11, v5,
            v11, v12, v7,
            v10, v6, v3,
            v6, v10, v12, 
            v9, v8, v2,
            v8, v9, v11,
            v3, v6, v4,
            v9, v2, v4,
            v10, v3, v1,
            v2, v8, v1,
            v12, v10, v7,
            v8, v11, v7,
            v6, v12, v5,
            v11, v9, v5
        };

        for(int i = 0; i < 2; i++){//
            List<float3> newMesh = new List<float3>();
            for(int j = 0; j < mesh.Length; j += 3){
                v1 = mesh[j];
                v2 = mesh[j + 1];
                v3 = mesh[j + 2];

                v4 = (v1 + v2) / 2;
                v5 = (v2 + v3) / 2;
                v6 = (v3 + v1) / 2;

                newMesh.Add(v1);
                newMesh.Add(v4);
                newMesh.Add(v6);

                newMesh.Add(v4);
                newMesh.Add(v2);
                newMesh.Add(v5);

                newMesh.Add(v6);
                newMesh.Add(v5);
                newMesh.Add(v3);

                newMesh.Add(v4);
                newMesh.Add(v5);
                newMesh.Add(v6);
            }
            mesh = newMesh.ToArray();
        }
        return mesh;
    }

    void DrawOverlays(){
        if(!hasHit) return;
        if(!settings.ShowCursor) return;
        RenderParams rp = new (OverlayMaterial){
            worldBounds = new Bounds(GSToWS(hitPoint), 2 * settings.terraformRadius * Vector3.one),
            matProps = new MaterialPropertyBlock(),
            renderingLayerMask = 1,
        }; 
        rp.matProps.SetColor("_Color", settings.CursorColor.GetColor());
        Matrix4x4 transform = math.mul(Matrix4x4.Translate(GSToWS(hitPoint)), Matrix4x4.Scale(settings.CursorSize * Vector3.one));
        Graphics.RenderMesh(rp, SphereMesh, 0, transform);
    }

    void RayTest(){
        uint RayTestSolid(int3 coord){ 
            MapData pointInfo = SampleMap(coord);
            return (uint)pointInfo.viscosity; 
        }
        uint RayTestLiquid(int3 coord){ 
            MapData pointInfo = SampleMap(coord);
            return (uint)Mathf.Max(pointInfo.viscosity, pointInfo.density - pointInfo.viscosity);
        }

        float3 camPosGC = WSToGS(cam.position);
        if(shiftPressed) hasHit = RayCastTerrain(camPosGC, cam.forward, settings.maxTerraformDistance, RayTestLiquid, out hitPoint);
        else hasHit = RayCastTerrain(camPosGC, cam.forward, settings.maxTerraformDistance, RayTestSolid, out hitPoint);
    }

    private void PlaceTerrain(float _){
        if (!hasHit) return;
        if(InventoryController.Selected.IsItem) return;

        if(InventoryController.Selected.IsSolid)
            CPUDensityManager.Terraform((int3)hitPoint, settings.terraformRadius, HandleAddSolid);
        else
            CPUDensityManager.Terraform((int3)hitPoint, settings.terraformRadius, HandleAddLiquid);
    }

    private void RemoveTerrain(float _){
        if (!hasHit) return;
        if(shiftPressed)
            CPUDensityManager.Terraform((int3)hitPoint, settings.terraformRadius, HandleRemoveLiquid);
        else 
            CPUDensityManager.Terraform((int3)hitPoint, settings.terraformRadius, HandleRemoveSolid);
    }

    int GetStaggeredDelta(int baseDensity, float deltaDensity){
        int staggeredDelta = Mathf.FloorToInt(deltaDensity);
        staggeredDelta += (deltaDensity % 1) == 0 ? 0 : (Time.frameCount % Mathf.CeilToInt(1 / (deltaDensity % 1))) == 0 ? 1 : 0;
        staggeredDelta = Mathf.Abs(Mathf.Clamp(baseDensity + staggeredDelta, 0, 255) - baseDensity);

        return staggeredDelta;
    }

    MapData HandleAddSolid(MapData pointInfo, float brushStrength){
        brushStrength *= settings.terraformSpeed * Time.deltaTime;
        if(brushStrength == 0) return pointInfo;

        int selected = (int)InventoryController.Selected.Index;
        int solidDensity = pointInfo.SolidDensity;
        if(solidDensity < IsoLevel || pointInfo.material == selected){
            //If adding solid density, override water
            int deltaDensity = GetStaggeredDelta(solidDensity, brushStrength);
            deltaDensity = InventoryController.RemoveMaterial(deltaDensity);

            solidDensity += deltaDensity;
            pointInfo.density = math.min(pointInfo.density + deltaDensity, 255);
            pointInfo.viscosity = math.min(pointInfo.viscosity + deltaDensity, 255);
            if(solidDensity >= IsoLevel) pointInfo.material = selected;
        }
        return pointInfo;
    }

    MapData HandleAddLiquid(MapData pointInfo, float brushStrength){
        brushStrength *= settings.terraformSpeed * Time.deltaTime;
        if(brushStrength == 0) return pointInfo;

        int selected = (int)InventoryController.Selected.Index;
        int liquidDensity = pointInfo.LiquidDensity;
        if(liquidDensity < IsoLevel || pointInfo.material == selected){
            //If adding liquid density, only change if not solid
            int deltaDensity = GetStaggeredDelta(pointInfo.density, brushStrength);
            deltaDensity = InventoryController.RemoveMaterial(deltaDensity);

            pointInfo.density += deltaDensity;
            liquidDensity += deltaDensity;
            if(liquidDensity >= IsoLevel) pointInfo.material = selected;
        }
        return pointInfo;
    }



    MapData HandleRemoveSolid(MapData pointInfo, float brushStrength){
        brushStrength *= settings.terraformSpeed * Time.deltaTime;
        if(brushStrength == 0) return pointInfo;

        int solidDensity = pointInfo.SolidDensity;
        if(solidDensity >= IsoLevel){
            int deltaDensity = GetStaggeredDelta(solidDensity, -brushStrength);
            deltaDensity = InventoryController.AddMaterial(
            new InventoryController.Inventory.Slot{
                IsItem = false,
                IsSolid = true,
                Index = pointInfo.material,
                AmountRaw = deltaDensity
            });

            pointInfo.viscosity -= deltaDensity;
            pointInfo.density -= deltaDensity;
        }
        return pointInfo;
    }

    MapData HandleRemoveLiquid(CPUDensityManager.MapData pointInfo, float brushStrength){
        brushStrength *= settings.terraformSpeed * Time.deltaTime;
        if(brushStrength == 0) return pointInfo;

        int liquidDensity = pointInfo.LiquidDensity;
        if (liquidDensity >= IsoLevel){
            int deltaDensity = GetStaggeredDelta(liquidDensity, -brushStrength);
            deltaDensity = InventoryController.AddMaterial(
            new InventoryController.Inventory.Slot{
                IsItem = false,
                IsSolid = false,
                Index = pointInfo.material,
                AmountRaw = deltaDensity
            });

            pointInfo.density -= deltaDensity;
        }
        return pointInfo;
    }
}
