using UnityEngine;
using Arterra.Configuration;
using Arterra.Editor;

namespace Arterra.Data.Intrinsic {
    /// <summary>
    /// Settings controlling the apperance and behavior of the resource inventory. The resource 
    /// inventory allows the player to obtain any item publicly supported in the game.
    /// </summary>
    [CreateAssetMenu(menuName = "Settings/ResourceInventory")]
    public class ResourceInventory : ScriptableObject {
        /// <summary> The name of the texture within the texture registry of 
        /// the icon displayed on the <see cref="PanelNavbarManager">Navbar</see>
        /// referring to the Resource Inventory.  </summary>
        [RegistryReference("Textures")]
        public string DisplayIcon;
        /// <summary>The maximum number of items that will be displayed when they are searched in the UI panel </summary>
        public int MaxResourceSearchDisplay;
    }
}