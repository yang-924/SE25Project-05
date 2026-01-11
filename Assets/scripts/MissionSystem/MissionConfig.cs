using UnityEngine;

namespace MissionSystem
{
    /// <summary>
    /// 任务配置（ScriptableObject）
    /// 用于在 Unity Editor 中预先定义任务参数
    /// 支持多个场景共用同一个任务类型
    /// </summary>
    [CreateAssetMenu(fileName = "New Mission Config", menuName = "MissionSystem/Mission Config")]
    public class MissionConfig : ScriptableObject
    {
        [Header("Basic Info")]
        public string missionName;
        [TextArea] public string description;
        public Sprite icon;

        [Header("Scene")]
        public string targetSceneName; // 任务对应的场景名称（例如 "CityMap"）

        [Header("Mission Type")]
        public MissionType missionType; // 枚举：Waypoint, Cargo, Patrol 等

        [Header("Waypoint Mission Data")]
        [HideInInspector] public Vector3[] waypointPositions; // 航点坐标
        [HideInInspector] public float reachThreshold = 5.0f;

        [Header("Cargo Mission Data")]
        [HideInInspector] public Vector3 pickupLocation;
        [HideInInspector] public Vector3 deliveryLocation;
        [HideInInspector] public GameObject cargoMarkerPrefab;
        [HideInInspector] public string cargoChildName = "box"; // 货物子物体名称
        [HideInInspector] public bool skipPickup = false; // 跳过拾取阶段（货物已在无人机上）

        [HideInInspector] public GameObject deliveryLockerPrefab; // 快递柜prefab
        [HideInInspector] public Vector3[] deliveryLockerPositions; // 可选的快递柜生成位置
        [HideInInspector] public bool useRandomDeliveryPosition = true; // 是否随机选择投放位置

        [Header("Patrol Mission Data")]
        [HideInInspector] public Vector3[] patrolPoints;
        [HideInInspector] public int patrolLoops = 3;

        [Header("Disaster Detection Mission Data")]
        [HideInInspector] public GameObject firePrefab; // 火焰prefab
        [HideInInspector] public Vector3[] fireSpawnPositions; // 所有可能的火焰位置
        [HideInInspector] public int numberOfFires = 3; // 生成火焰数量
        [HideInInspector] public bool randomizeFirePositions = true; // 是否随机选择位置
        [HideInInspector] public float detectionRadius = 20f; // 检测半径（米）
        [HideInInspector] public float photoDistance = 20f; // 拍照有效距离（米）
        [HideInInspector] public bool requirePhotoConfirmation = true; // 是否需要拍照确认

        // 在Inspector中根据任务类型动态显示字段
        private void OnValidate()
        {
            // 这个方法在Inspector值改变时调用，用于自动更新显示
        }
    }

    public enum MissionType
    {
        Waypoint,          // 航点任务
        Cargo,             // 货物运输
        Patrol,            // 巡逻任务
        Recon,             // 侦察任务
        DisasterDetection  // 灾害检测（火灾定位）
    }
}
