using UnityEngine;
using UnityEngine.SceneManagement;
using MissionSystem;

/// <summary>
/// 任务系统桥接器
/// 连接现有 UI 系统（TaskSelectController + SelectController）和新任务系统
/// 负责场景加载、无人机生成、任务初始化、相机关联
/// </summary>
public class MissionBridge : MonoBehaviour
{
    public static MissionBridge Instance;

    [Header("Drone Prefabs")]
    [Tooltip("所有可用的无人机 Prefab")]
    public GameObject[] dronePrefabs;
    [Tooltip("对应的类型名称（与 SelectController 中的 droneNames 保持一致）")]
    public string[] droneTypeNames;

    [Header("Spawn Settings")]
    [Tooltip("无人机生成位置（如果场景没有指定生成点）")]
    public Vector3 defaultSpawnPosition = new Vector3(0, 10, 0);
    public Quaternion defaultSpawnRotation = Quaternion.identity;

    [Header("Camera Settings")]
    [Tooltip("FPV 相机 Prefab（如果场景中没有）")]
    public GameObject fpvCameraPrefab;
    [Tooltip("TPV 相机 Prefab（如果场景中没有）")]
    public GameObject tpvCameraPrefab;
    [Tooltip("相机配置资源（根据无人机类型调整偏移）")]
    public DroneCameraConfig cameraConfig;

    private string pendingDroneType;
    private MissionConfig pendingMissionConfig; // 缓存配置，等待场景加载后传递

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void OnEnable()
    {
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    void OnDisable()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }

    void OnDestroy()
    {
        // 确保事件完全清理
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }

    /// <summary>
    /// 从 SelectController 调用：加载场景 + 任务 + 无人机
    /// </summary>
    public void LoadMissionScene(string sceneName, MissionConfig missionConfig, string droneType)
    {
        Debug.Log($"<color=cyan>[MissionBridge]</color> ===== LoadMissionScene 被调用 =====");
        Debug.Log($"<color=cyan>[MissionBridge]</color> 场景: {sceneName}");
        Debug.Log($"<color=cyan>[MissionBridge]</color> 任务: {missionConfig?.missionName ?? "NULL"}");
        Debug.Log($"<color=cyan>[MissionBridge]</color> 无人机: {droneType}");

        // 缓存配置，等待场景加载后再传递给MissionManager
        pendingDroneType = droneType;
        pendingMissionConfig = missionConfig;

        Debug.Log($"[MissionBridge] Loading scene: {sceneName}, mission: {missionConfig?.missionName}, drone: {droneType}");
        SceneManager.LoadScene(sceneName);
    }

    void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        Debug.Log($"<color=cyan>[MissionBridge]</color> OnSceneLoaded: {scene.name}");

        // 场景加载完成后，将缓存的任务配置传递给MissionManager
        if (pendingMissionConfig != null)
        {
            if (MissionManager.Instance != null)
            {
                Debug.Log($"<color=green>[MissionBridge]</color> 找到 MissionManager，设置配置: {pendingMissionConfig.missionName}");
                MissionManager.Instance.selectedMissionConfig = pendingMissionConfig;
                Debug.Log($"<color=green>[MissionBridge]</color> 验证设置: {MissionManager.Instance.selectedMissionConfig?.missionName ?? "NULL"}");
            }
            else
            {
                Debug.LogError("<color=red>[MissionBridge]</color> OnSceneLoaded: MissionManager.Instance 仍为 null!");
            }
            pendingMissionConfig = null; // 清空缓存
        }

        // 生成无人机
        if (!string.IsNullOrEmpty(pendingDroneType))
        {
            SpawnDrone(pendingDroneType);
            pendingDroneType = null; // 清空标志
        }
    }

    void SpawnDrone(string droneType)
    {
        // 1. 查找并删除场景中的默认无人机（如果有）
        GameObject existingDrone = GameObject.FindGameObjectWithTag("Player");
        if (existingDrone != null)
        {
            Debug.Log($"[MissionBridge] Found existing drone '{existingDrone.name}', destroying it.");
            Destroy(existingDrone);
        }

        // 2. 根据类型查找 Prefab
        GameObject prefab = GetDronePrefab(droneType);
        if (prefab == null)
        {
            Debug.LogError($"[MissionBridge] Drone prefab not found for type: {droneType}. Check dronePrefabs array in MissionBridge.");
            return;
        }

        // 3. 查找场景中的生成点（可选）
        Vector3 spawnPos = defaultSpawnPosition;
        Quaternion spawnRot = defaultSpawnRotation;
        GameObject spawnPoint = GameObject.Find("DroneSpawnPoint");
        if (spawnPoint != null)
        {
            spawnPos = spawnPoint.transform.position;
            spawnRot = spawnPoint.transform.rotation;
        }

        // 4. 生成新无人机
        GameObject drone = Instantiate(prefab, spawnPos, spawnRot);
        drone.tag = "Player"; // 确保任务系统能找到
        drone.name = $"Drone_{droneType}";
        Debug.Log($"[MissionBridge] Spawned drone: {droneType} at {spawnPos}");

        // 5. 将飞手位置设置为无人机生成点
        MinimapController minimapController = FindObjectOfType<MinimapController>();
        if (minimapController != null)
        {
            minimapController.SetPilotPosition(spawnPos);
            Debug.Log($"[MissionBridge] Set pilot position to drone spawn point: {spawnPos}");
        }
        else
        {
            Debug.LogWarning("[MissionBridge] MinimapController not found, pilot position not updated");
        }

        // 6. 关联相机并应用配置
        AssignCameraToDrone(drone, droneType);

        // 7. 关联数据记录器（如果有）
        AssignRecorderToDrone(drone);
    }

    GameObject GetDronePrefab(string typeName)
    {
        for (int i = 0; i < droneTypeNames.Length; i++)
        {
            if (droneTypeNames[i] == typeName && i < dronePrefabs.Length)
            {
                return dronePrefabs[i];
            }
        }
        return null;
    }

    void AssignCameraToDrone(GameObject drone, string droneType)
    {
        // 获取该无人机类型的相机配置
        DroneCameraConfig.CameraOffsetData config = null;
        if (cameraConfig != null)
        {
            config = cameraConfig.GetConfigForDrone(droneType);
        }

        // 查找场景中的相机并更新目标
        // 1. TPV 相机（CameraController）
        CameraController tpvCam = FindObjectOfType<CameraController>();
        if (tpvCam != null)
        {
            tpvCam.obj = drone;
            if (config != null)
            {
                tpvCam.offsetPosition = config.tpvOffset;
                Debug.Log($"[MissionBridge] Assigned TPV camera with offset: {config.tpvOffset}");
            }
            else
            {
                Debug.Log("[MissionBridge] Assigned TPV camera to drone (no config).");
            }
        }
        else
        {
            Debug.LogWarning("[MissionBridge] TPV Camera (CameraController) not found in scene.");
        }

        // 2. FPV 相机（GimbalCamera）
        GimbalCamera fpvCam = FindObjectOfType<GimbalCamera>();
        if (fpvCam != null)
        {
            fpvCam.target = drone.transform;
            if (config != null)
            {
                fpvCam.positionOffset = config.fpvOffset;
                fpvCam.gimbalRotation = config.fpvGimbalRotation;
                Debug.Log($"[MissionBridge] Assigned FPV camera with offset: {config.fpvOffset}, gimbal: {config.fpvGimbalRotation}");
            }
            else
            {
                Debug.Log("[MissionBridge] Assigned FPV camera to drone (no config).");
            }
        }
        else
        {
            Debug.LogWarning("[MissionBridge] FPV Camera (GimbalCamera) not found in scene.");
        }

        // 3. 信号延迟特效（如果 FPV 相机上有）
        SignalLatencyEffect latencyEffect = FindObjectOfType<SignalLatencyEffect>();
        if (latencyEffect != null)
        {
            latencyEffect.droneTransform = drone.transform;
            Debug.Log("[MissionBridge] Assigned SignalLatencyEffect to drone.");
        }
    }

    void AssignRecorderToDrone(GameObject drone)
    {
        // 如果无人机上有 FlightDataRecorder 组件，确保它引用了相机
        FlightDataRecorder recorder = drone.GetComponent<FlightDataRecorder>();
        if (recorder != null)
        {
            // 查找相机并赋值
            GimbalCamera fpv = FindObjectOfType<GimbalCamera>();
            CameraController tpv = FindObjectOfType<CameraController>();

            if (fpv != null) recorder.fpvCamera = fpv.GetComponent<Camera>();
            if (tpv != null) recorder.tpvCamera = tpv.GetComponent<Camera>();

            Debug.Log("[MissionBridge] Assigned cameras to FlightDataRecorder.");
        }
    }
}
