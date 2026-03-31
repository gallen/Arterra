using System;
using System.Collections.Generic;
using System.Linq;
using Arterra.Configuration;
using Arterra.Editor;
using Unity.Mathematics;
using UnityEngine;

namespace Arterra.Data.Structure.Jigsaw {
    [CreateAssetMenu(menuName = "Generation/Structure/Jigsaw/System")]
    public class JigsawSystem : Category<JigsawSystem>{
        /// <summary>The names of all structures within the external <see cref="Config.GenerationSettings.Structures"/> registry that
        /// are used in the system. All entries that require references to a structure may indicate the index within
        /// this list of the name of the material in the external registry. </summary>
        public Option<List<string>> Names;
        public Option<List<JigsawStructure>> Structures;
        public float edgeFrequency;

        [Serializable]
        public struct JigsawStructure {
            [RegistryReference("Structures")]
            public int Structure;
            public Option<List<JigsawSocket>> Sockets;

            public void CollectSocketNames(int index, ref Dictionary<string, Dictionary<int, uint>> TransitionMap) {
                //socket name -> structure -> allowed faces
                List<JigsawSocket> sockets = Sockets.value;
                for(int i = 0; i < sockets.Count; i++) {
                    TransitionMap.TryAdd(sockets[i].Name, new Dictionary<int, uint>());
                    Dictionary<int, uint> StructAllowedTrans = TransitionMap[sockets[i].Name];
                    if(!StructAllowedTrans.TryAdd(index, 1u << (int)sockets[i].Connection))
                        StructAllowedTrans[index] |= 1u << (int)sockets[i].Connection;
                }
            }

            public void SerializeStructure(JigsawSystem system, Dictionary<string, uint2> TransitionRange, out SystemStructure st, ref List<Port> ports, ref List<Socket> sockets) {
                Port[] prt = new Port[6];
                List<JigsawSocket> socketList = Sockets.value;

                uint basePorts = 0;
                for(int i = 0; i < socketList.Count; i++) {
                    basePorts |= 1u << (int)socketList[i].Connection;
                    int start = sockets.Count();
                    foreach(JigsawConnection connection in socketList[i].Transitions.value) {
                        sockets.Add(new Socket{
                            chance = connection.chance,
                            transitions = TransitionRange[connection.TargetName],
                        });
                    }

                    int portIndex = (int)socketList[i].Connection;
                    prt[portIndex].UV = socketList[i].UV;
                    prt[portIndex].sockets = new uint2((uint)start, (uint)sockets.Count());
                }

                ports.AddRange(prt);
                string name = system.Names.value[Structure];
                st = new SystemStructure {
                    structureIndex = (uint)Config.CURRENT.Generation.Structures.value.StructureDictionary.RetrieveIndex(name),
                    basePorts = basePorts
                };
            }
        }

        [Serializable]
        public struct JigsawSocket {
            public string Name;
            public Facing Connection;
            public float2 UV;
            public Option<List<JigsawConnection>> Transitions;
            public enum Facing : uint {
                Left = 0, Right = 3,
                Bottom = 1, Top = 4,
                Back = 2, Forward = 5,
            }
        };

        [Serializable]
        public struct JigsawConnection {
            public string TargetName;
            public float chance;
        };

        public struct SystemStructure {
            public static int size => 2 * sizeof(uint);
            public uint structureIndex;
            public uint basePorts;
        };

        public struct Port {
            public static int size => 2 * sizeof(float) + 2 * sizeof(uint);
            public float2 UV;
            public uint2 sockets;
        };

        public struct Socket {
            public static int size => 2 * sizeof(uint) + sizeof(float);
            public float chance;
            public uint2 transitions;
        }

        public struct Transition {
            public static int size => sizeof(int) + sizeof(uint);
            public int Structure;
            //The faces that this face can connect to
            public uint AllowedOppositeFaces;
        }
    }
}