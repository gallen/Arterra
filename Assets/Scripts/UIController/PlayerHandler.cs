using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityStandardAssets.Characters.FirstPerson;
using System.IO;

public class PlayerHandler : MonoBehaviour
{
    public TerraformController terrController;
    private RigidbodyFirstPersonController PlayerController;
    private PlayerData info;
    private bool active = false;
    // Start is called before the first frame update

    public void Activate(){
        active = true;
        PlayerController.ActivateCharacter();
    }
    void OnEnable(){
        PlayerController = this.GetComponent<RigidbodyFirstPersonController>();

        info = LoadPlayerData();
        transform.SetPositionAndRotation(info.position.GetVector(), info.rotation.GetQuaternion());
        this.terrController.MainInventory = info.inventory;
        this.terrController.Activate();
    }

    // Update is called once per frame
    void Update() { 
        if(EndlessTerrain.timeRequestQueue.Count == 0 && !active) Activate();
        if(!active) return;
        
        terrController.Update(); 
    }

    void OnDisable(){
        info.position = new Vec3(transform.position);
        info.rotation = new Vec4(transform.rotation);
        info.inventory = terrController.MainInventory;
        Task.Run(() => SavePlayerData(info));
        active = false;
    }

    async Task SavePlayerData(PlayerData playerInfo){
        string path = WorldStorageHandler.WORLD_OPTIONS.Path + "/PlayerData.json";
        using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None)){
            using StreamWriter writer = new StreamWriter(fs);
            string data = Newtonsoft.Json.JsonConvert.SerializeObject(playerInfo);
            await writer.WriteAsync(data);
            await writer.FlushAsync();
        };
    }

    void OnDrawGizmos(){
        terrController.OnDrawGizmos();
    }

     PlayerData LoadPlayerData(){
        string path = WorldStorageHandler.WORLD_OPTIONS.Path + "/PlayerData.json";
        if(!File.Exists(path)) {
            return new PlayerData{
                position = new Vec3((new Vector3(0, 0, 0) + Vector3.up * (CPUNoiseSampler.SampleTerrainHeight(new (0, 0, 0)) + 5)) * EndlessTerrain.lerpScale),
                rotation = new Vec4(Quaternion.LookRotation(Vector3.forward, Vector3.up)),
                inventory = new MaterialInventory(TerraformController.materialCapacity)
            };
        }

        string data = System.IO.File.ReadAllText(path);
        return  Newtonsoft.Json.JsonConvert.DeserializeObject<PlayerData>(data);
    }

    struct PlayerData{
        public Vec3 position;
        public Vec4 rotation;
        public MaterialInventory inventory;
    }
}