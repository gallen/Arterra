using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using System.IO;
using System.Linq;
using TerrainGeneration;
using WorldConfig;


public class PlayerHandler : UpdateTask
{
    public static GameObject player;
    public static GameObject UIHandle;
    public static TerraformController terrController;
    private static RigidFPController PlayerController;
    private static PlayerData info;
    public static void Initialize(){
        UIHandle = GameObject.Find("MainUI");
        player = GameObject.Instantiate(Resources.Load<GameObject>("Prefabs/GameUI/PlayerController"));
        PlayerController = player.GetComponent<RigidFPController>();

        info = LoadPlayerData();
        player.transform.SetPositionAndRotation(info.position, info.rotation);
        PlayerController.Initialize();
        
        InventoryController.Primary = info.PrimaryI;
        InventoryController.Secondary = info.SecondaryI;
        terrController = new TerraformController();
        DayNightContoller.currentTime = info.currentTime;

        OctreeTerrain.MainLoopUpdateTasks.Enqueue(new PlayerHandler{active = true});
        OctreeTerrain.viewer = player.transform; //set octree's viewer to current player

        LoadingHandler.Initialize();
        PauseHandler.Initialize();
        InventoryController.Initialize();
        DayNightContoller.Initialize();
    }

    // Update is called once per frame
    public override void Update(MonoBehaviour mono) { 
        if(!active) return;
        if(OctreeTerrain.RequestQueue.IsEmpty) {
            PlayerController.ActivateCharacter();
            active = false;
        }
    }

    public static void Release(){
        info.position = player.transform.position;
        info.rotation = player.transform.rotation;

        info.PrimaryI = InventoryController.Primary;
        info.SecondaryI = InventoryController.Secondary;
        info.currentTime = DayNightContoller.currentTime;

        InputPoller.SetCursorLock(false); //Release Cursor Lock
        Task.Run(() => SavePlayerData(info));

        InventoryController.Release();
        GameObject.Destroy(player);
    }
    

    static async Task SavePlayerData(PlayerData playerInfo){
        playerInfo.Serialize();
        string path = WorldStorageHandler.WORLD_SELECTION.First.Value.Path + "/PlayerData.json";
        using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None)){
            using StreamWriter writer = new StreamWriter(fs);
            string data = Newtonsoft.Json.JsonConvert.SerializeObject(playerInfo);
            await writer.WriteAsync(data);
            await writer.FlushAsync();
        };
    }

    static PlayerData LoadPlayerData(){
        string path = WorldStorageHandler.WORLD_SELECTION.First.Value.Path + "/PlayerData.json";
        if(!File.Exists(path)) { return PlayerData.GetDefault(); }

        string data = System.IO.File.ReadAllText(path);
        PlayerData playerInfo = Newtonsoft.Json.JsonConvert.DeserializeObject<PlayerData>(data);
        playerInfo.Deserialize();
        return playerInfo;
    }

    struct PlayerData{
        public Vector3 position;
        public Quaternion rotation;
        public List<string> SerializedNames;
        public InventoryController.Inventory PrimaryI;
        public InventoryController.Inventory SecondaryI;
        public DateTime currentTime;

        public void Serialize(){
            //Marks updated slots dirty so they are rendered properlly when deserialized
            // (Register Name, Index) -> Name Index
            Dictionary<string, int> lookup = new Dictionary<string, int>();
            int OnSerialize(string name){
                lookup.TryAdd(name, lookup.Count);
                return lookup[name];
            }
            
            for(int i = 0; i < PrimaryI.Info.Length; i++){
                if(PrimaryI.Info[i] == null) continue;
                PrimaryI.Info[i].Serialize(OnSerialize);
            }
            for(int i = 0; i < SecondaryI.Info.Length; i++){
                if(SecondaryI.Info[i] == null) continue;
                SecondaryI.Info[i].Serialize(OnSerialize);
            }

            SerializedNames = lookup.Keys.ToList();
        }

        public void Deserialize(){
            List<string> names = SerializedNames;
            string OnDeserialize(int name){
                if(name < 0 || name >= names.Count) Debug.Log(name);
                if(name < 0 || name >= names.Count) return "";
                return names[name];
            }
            
            for(int i = 0; i < PrimaryI.Info.Length; i++){
                if(PrimaryI.Info[i] == null) continue;
                PrimaryI.Info[i].Deserialize(OnDeserialize);
                PrimaryI.Info[i].IsDirty = true;
            }
            for(int i = 0; i < SecondaryI.Info.Length; i++){
                if(SecondaryI.Info[i] == null) continue;
                SecondaryI.Info[i].Deserialize(OnDeserialize);
                SecondaryI.Info[i].IsDirty = true;
            }
        }

        public static PlayerData GetDefault(){
            return new PlayerData{
                position = new Vector3(0, 0, 0) + (CPUNoiseSampler.SampleTerrainHeight(new (0, 0, 0)) + 5) * Config.CURRENT.Quality.Terrain.value.lerpScale * Vector3.up,
                rotation = Quaternion.LookRotation(Vector3.forward, Vector3.up),
                PrimaryI = new InventoryController.Inventory(Config.CURRENT.GamePlay.Inventory.value.PrimarySlotCount),
                SecondaryI = new InventoryController.Inventory(Config.CURRENT.GamePlay.Inventory.value.SecondarySlotCount),
                currentTime = DateTime.Now.Date + TimeSpan.FromHours(Config.CURRENT.GamePlay.DayNightCycle.value.startHour)
            };
        }
    }
}
