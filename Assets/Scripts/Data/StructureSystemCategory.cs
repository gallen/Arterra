
using System;
using System.Collections.Generic;
using UnityEngine;
using Arterra.Configuration;

namespace Arterra.Data.Structure.Jigsaw{

    [Serializable]
    [CreateAssetMenu(menuName = "Generation/Structure/Jigsaw/Category")]
    public class JigsawSystemCategory : Category<JigsawSystem>
    {
        public Option<List<Option<Category<JigsawSystem>>>> Children;
        protected override Option<List<Option<Category<JigsawSystem>>>>? GetChildren() => Children;
        protected override void SetChildren(Option<List<Option<Category<JigsawSystem>>>> value) => Children = value;
}

}
