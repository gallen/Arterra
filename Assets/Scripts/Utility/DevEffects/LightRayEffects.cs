using UnityEngine;
using Unity.Mathematics;
using Arterra.Core.Storage;

namespace DevEffects
{
    public class LightRayEffects : MonoBehaviour
    {
        public int rayCount = 100;
        public float maxAngle = 45f;
        public float rayLength = 10f;
        public float cylinderRadius = 0.01f;
        public Material cylinderMaterial;
        public float animationDuration = 1.5f;

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.P))
            {
                SpawnRays();
            }
        }

        void SpawnRays()
        {
            if (Arterra.GamePlay.PlayerHandler.data == null) return;
            float3 playerPos = Arterra.GamePlay.PlayerHandler.data.head;
            float3 forward = math.normalize(Arterra.GamePlay.PlayerHandler.data.Forward);
            Vector3 playerPosV3 = new Vector3(playerPos.x, playerPos.y, playerPos.z);

            for (int i = 0; i < rayCount; i++)
            {
                Vector3 dir = RandomDirectionInCone(forward, maxAngle);
                Vector3 start = playerPosV3 + dir * rayLength;
                Vector3 end = playerPosV3;
                SpawnCylinder(start, end);
            }
        }

        Vector3 RandomDirectionInCone(float3 forward, float maxAngleDeg)
        {
            float maxAngleRad = maxAngleDeg * Mathf.Deg2Rad;
            float z = Mathf.Cos(UnityEngine.Random.Range(0, maxAngleRad));
            float theta = UnityEngine.Random.Range(0, 2 * Mathf.PI);
            float r = Mathf.Sqrt(1 - z * z);
            float x = r * Mathf.Cos(theta);
            float y = r * Mathf.Sin(theta);
            Vector3 localDir = new Vector3(x, y, z);
            return Quaternion.FromToRotation(Vector3.forward, new Vector3(forward.x, forward.y, forward.z)) * localDir;
        }

        void SpawnCylinder(Vector3 start, Vector3 end)
        {
            GameObject cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cyl.name = "LightRayCylinder";
            var renderer = cyl.GetComponent<Renderer>();
            if (cylinderMaterial != null)
                renderer.material = cylinderMaterial;
            renderer.material.color = Color.yellow;
            cyl.AddComponent<AnimatedCylinder>().Init(start, end, cylinderRadius, animationDuration);
        }

        private class AnimatedCylinder : MonoBehaviour
        {
            Vector3 start, end;
            float baseRadius, duration;
            float elapsed = 0f;
            public void Init(Vector3 s, Vector3 e, float r, float d)
            {
                start = s;
                end = e;
                baseRadius = r;
                duration = d;
                UpdateTransform(0f);
            }
            void Update()
            {
                elapsed += Time.deltaTime;
                float t = Mathf.Clamp01(elapsed / duration);
                UpdateTransform(t);
                if (t >= 1f) Destroy(gameObject);
            }
            void UpdateTransform(float t)
            {
                Vector3 dir = end - start;
                Vector3 pos = Vector3.Lerp(start, end, t);
                float length = math.min(1, (end - pos).magnitude/2);
                float radius = Mathf.Lerp(0.05f, baseRadius, 1 - t);
                transform.position = CPUMapManager.GSToWS(pos);
                transform.up = dir.normalized;
                transform.localScale = new Vector3(radius * 2, length, radius * 2);
            }
        }
    }
}
