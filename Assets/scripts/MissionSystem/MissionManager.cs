using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;

namespace MissionSystem
{
    /// <summary>
    /// 任务管理器
    /// 负责调度当前正在进行的任务
    /// 支持跨场景持久化和动态任务生成
    /// </summary>
    public class MissionManager : MonoBehaviour
    {
        public static MissionManager Instance { get; private set; }

        [Header("Active Mission")]
        public MissionBase currentMission;

        [Header("Mission Selection (Set before loading scene)")]
        public MissionConfig selectedMissionConfig; // 玩家选择的任务配置

        [Header("UI / Debug")]
        public bool autoStartAssignedMission = false;

        [Header("Scene Transitions")]
        [Tooltip("任务成功后跳转的场景名称")]
        public string successSceneName = "Success";
        [Tooltip("任务失败后跳转的场景名称")]
        public string failSceneName = "Fail";
        [Tooltip("任务结束后延迟跳转时间（秒）")]
        public float transitionDelay = 2f;

        private void Awake()
        {
            Debug.Log("<color=magenta>[MissionManager]</color> ===== Awake 被调用 =====");

            // 单例 + 跨场景持久化
            if (Instance == null)
            {
                Debug.Log("<color=magenta>[MissionManager]</color> 设置为单例实例，启用 DontDestroyOnLoad");
                Instance = this;
                DontDestroyOnLoad(gameObject); // 关键：防止场景切换时销毁
            }
            else
            {
                Debug.LogWarning("<color=yellow>[MissionManager]</color> 已存在实例，销毁重复对象");
                Destroy(gameObject);
            }
        }

        private void OnEnable()
        {
            Debug.Log("<color=magenta>[MissionManager]</color> OnEnable - 订阅 sceneLoaded 事件");
            // 监听场景加载完成事件
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private void OnDisable()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded;

            // 清理当前任务的事件订阅
            if (currentMission != null)
            {
                currentMission.OnMissionFinished -= OnMissionFinishedHandler;
            }
        }

        private void OnDestroy()
        {
            // 确保事件完全清理
            SceneManager.sceneLoaded -= OnSceneLoaded;

            if (currentMission != null)
            {
                currentMission.OnMissionFinished -= OnMissionFinishedHandler;
            }
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            Debug.Log($"<color=cyan>[MissionManager]</color> ===== 场景已加载 =====");
            Debug.Log($"<color=cyan>[MissionManager]</color> 场景名称: {scene.name}");
            Debug.Log($"<color=cyan>[MissionManager]</color> selectedMissionConfig: {(selectedMissionConfig != null ? selectedMissionConfig.missionName : "NULL")}");

            // 场景加载完成后，如果有选中的任务配置，自动生成任务
            if (selectedMissionConfig != null)
            {
                Debug.Log($"[MissionManager] Scene loaded: {scene.name}, initializing mission: {selectedMissionConfig.missionName}");
                InitializeMissionFromConfig(selectedMissionConfig);
            }
            else
            {
                Debug.LogWarning("<color=red>[MissionManager]</color> selectedMissionConfig 为 null，无法初始化任务！");
            }
        }

        private void Start()
        {
            if (autoStartAssignedMission && currentMission != null)
            {
                StartMission(currentMission);
            }
        }

        private void Update()
        {
            if (currentMission != null && currentMission.Status == MissionStatus.Running)
            {
                currentMission.OnUpdate();
            }
        }

        /// <summary>
        /// 选择任务并加载对应场景（在 UI 场景中调用）
        /// </summary>
        public void SelectMissionAndLoadScene(MissionConfig config)
        {
            selectedMissionConfig = config;
            Debug.Log($"[MissionManager] Loading scene: {config.targetSceneName}");
            SceneManager.LoadScene(config.targetSceneName);
        }

        /// <summary>
        /// 根据配置动态生成任务实例
        /// </summary>
        private void InitializeMissionFromConfig(MissionConfig config)
        {
            Debug.Log($"<color=cyan>[MissionManager]</color> ===== 开始初始化任务 =====");
            Debug.Log($"<color=cyan>[MissionManager]</color> 任务名称: {config.missionName}");
            Debug.Log($"<color=cyan>[MissionManager]</color> 任务类型: {config.missionType}");
            Debug.Log($"<color=cyan>[MissionManager]</color> 目标场景: {config.targetSceneName}");

            // 先清理旧任务
            if (currentMission != null)
            {
                Debug.Log($"<color=yellow>[MissionManager]</color> 销毁旧任务: {currentMission.missionName}");
                Destroy(currentMission.gameObject);
            }

            // 根据任务类型创建对应的任务脚本
            GameObject missionObj = new GameObject($"Mission_{config.missionName}");
            Debug.Log($"<color=cyan>[MissionManager]</color> 创建任务对象: {missionObj.name}");

            MissionBase mission = null;

            switch (config.missionType)
            {
                case MissionType.Waypoint:
                    Debug.Log("<color=green>[MissionManager]</color> 创建航点任务...");
                    mission = missionObj.AddComponent<WaypointMission>();
                    SetupWaypointMission((WaypointMission)mission, config);
                    break;

                case MissionType.Cargo:
                    Debug.Log("<color=green>[MissionManager]</color> 创建货物运输任务...");
                    mission = missionObj.AddComponent<CargoMission>();
                    SetupCargoMission((CargoMission)mission, config);
                    break;

                case MissionType.DisasterDetection:
                    Debug.Log("<color=green>[MissionManager]</color> 创建灾害检测任务...");
                    mission = missionObj.AddComponent<DisasterDetectMission>();
                    SetupDisasterMission((DisasterDetectMission)mission, config);
                    break;

                case MissionType.Patrol:
                    // mission = missionObj.AddComponent<PatrolMission>();
                    Debug.LogWarning("[MissionManager] PatrolMission not implemented yet.");
                    break;
            }

            if (mission != null)
            {
                Debug.Log($"<color=green>[MissionManager]</color> 任务创建成功: {mission.GetType().Name}");
                Debug.Log($"<color=cyan>[MissionManager]</color> 开始启动任务...");
                StartMission(mission);
            }
            else
            {
                Debug.LogError("<color=red>[MissionManager]</color> 任务创建失败！mission 为 null");
            }
        }

        /// <summary>
        /// 配置航点任务
        /// </summary>
        private void SetupWaypointMission(WaypointMission mission, MissionConfig config)
        {
            mission.missionName = config.missionName;
            mission.reachThreshold = config.reachThreshold;
            mission.description = config.description;

            // 根据配置的坐标创建航点物体
            mission.waypoints = new System.Collections.Generic.List<Transform>();
            GameObject waypointsParent = new GameObject("Waypoints");

            foreach (Vector3 pos in config.waypointPositions)
            {
                GameObject wp = new GameObject($"Waypoint_{mission.waypoints.Count}");
                wp.transform.position = pos;
                wp.transform.parent = waypointsParent.transform;
                mission.waypoints.Add(wp.transform);
            }
        }

        /// <summary>
        /// 配置货物运输任务
        /// </summary>
        private void SetupCargoMission(CargoMission mission, MissionConfig config)
        {
            mission.missionName = config.missionName;
            mission.description = config.description;

            // 创建拾取点
            GameObject pickupObj = new GameObject("CargoPickupPoint");
            pickupObj.transform.position = config.pickupLocation;
            mission.pickupPoint = pickupObj.transform;

            // 随机选择投放位置（如果启用）
            Vector3 selectedDeliveryPosition = config.deliveryLocation;
            if (config.useRandomDeliveryPosition && config.deliveryLockerPositions != null && config.deliveryLockerPositions.Length > 0)
            {
                int randomIndex = Random.Range(0, config.deliveryLockerPositions.Length);
                selectedDeliveryPosition = config.deliveryLockerPositions[randomIndex];
                Debug.Log($"<color=cyan>[MissionManager]</color> 随机选择快递柜位置 [{randomIndex}]: {selectedDeliveryPosition}");
            }

            // 创建投放点
            GameObject deliveryObj = new GameObject("CargoDeliveryPoint");
            deliveryObj.transform.position = selectedDeliveryPosition;
            mission.deliveryPoint = deliveryObj.transform;

            Debug.Log($"<color=cyan>[MissionManager]</color> 投放点已创建: {deliveryObj.transform.position}, Transform名称: {deliveryObj.name}");

            // 生成快递柜模型（如果提供了prefab）
            if (config.deliveryLockerPrefab != null)
            {
                GameObject locker = Instantiate(config.deliveryLockerPrefab, selectedDeliveryPosition, Quaternion.identity);
                locker.name = "DeliveryLocker";
                Debug.Log($"<color=green>[MissionManager]</color> 已生成快递柜: {selectedDeliveryPosition}");
            }
            else
            {
                Debug.LogWarning("<color=yellow>[MissionManager]</color> 未设置快递柜prefab，仅生成投放点标记");
            }

            // 查找货物对象（优先查找无人机子物体）
            GameObject cargo = null;
            GameObject drone = GameObject.FindGameObjectWithTag("Player");

            if (drone != null && !string.IsNullOrEmpty(config.cargoChildName))
            {
                // 在无人机子物体中查找
                Transform cargoTransform = drone.transform.Find(config.cargoChildName);
                if (cargoTransform != null)
                {
                    cargo = cargoTransform.gameObject;
                    Debug.Log($"[MissionManager] Found cargo as drone child: {config.cargoChildName}");
                }
                else
                {
                    Debug.LogWarning($"[MissionManager] Cargo child '{config.cargoChildName}' not found in drone. Searching by tag...");
                }
            }

            // 备选：按名称查找独立的货物对象
            if (cargo == null && !string.IsNullOrEmpty(config.cargoChildName))
            {
                cargo = GameObject.Find(config.cargoChildName);
                if (cargo != null)
                {
                    Debug.Log($"[MissionManager] Found cargo by name: {config.cargoChildName}");
                }
            }

            // 再备选：尝试查找任何名称包含 "Cargo" 的对象
            if (cargo == null)
            {
                GameObject[] allObjects = FindObjectsOfType<GameObject>();
                foreach (GameObject obj in allObjects)
                {
                    if (obj.name.ToLower().Contains("cargo"))
                    {
                        cargo = obj;
                        Debug.Log($"[MissionManager] Found cargo by name search: {obj.name}");
                        break;
                    }
                }
            }

            // 最后方案：从prefab生成
            if (cargo == null && config.cargoMarkerPrefab != null)
            {
                cargo = Instantiate(config.cargoMarkerPrefab, config.pickupLocation, Quaternion.identity);
                cargo.name = "CargoObject";
                Debug.Log("[MissionManager] Generated cargo from prefab.");
            }

            mission.cargoObject = cargo;
            mission.skipPickup = config.skipPickup; // 应用配置

            if (cargo == null)
            {
                Debug.LogError("[MissionManager] Failed to find or create cargo object!");
            }

            Debug.Log($"[MissionManager] Cargo mission setup complete. Pickup: {config.pickupLocation}, Delivery: {config.deliveryLocation}");
        }

        public void StartMission(MissionBase mission)
        {
            Debug.Log($"<color=cyan>[MissionManager]</color> StartMission 被调用: {mission?.missionName ?? "null"}");

            if (currentMission != null && currentMission.Status == MissionStatus.Running)
            {
                Debug.LogWarning("[MissionManager] Aborting current mission to start new one.");
                currentMission.FailMission("Aborted by new mission");
            }

            currentMission = mission;
            if (currentMission != null)
            {
                Debug.Log($"<color=green>[MissionManager]</color> 订阅任务事件: {currentMission.missionName}");
                // 订阅事件以便转发或处理
                currentMission.OnMissionFinished += OnMissionFinishedHandler;

                Debug.Log($"<color=green>[MissionManager]</color> 调用 BeginMission()...");
                currentMission.BeginMission();

                Debug.Log($"<color=green>[MissionManager]</color> 任务状态: {currentMission.Status}");
            }
            else
            {
                Debug.LogError("<color=red>[MissionManager]</color> currentMission 为 null！");
            }
        }

        public void AbortCurrentMission()
        {
            if (currentMission != null && currentMission.Status == MissionStatus.Running)
            {
                currentMission.FailMission("Aborted by user");
            }
        }

        private void OnMissionFinishedHandler(MissionBase mission, bool success)
        {
            Debug.Log($"<color=cyan>[MissionManager]</color> ===== OnMissionFinishedHandler CALLED =====");
            Debug.Log($"<color=cyan>[MissionManager]</color> Mission: {mission.missionName}, Success: {success}");

            // 任务结束后的清理工作
            mission.OnMissionFinished -= OnMissionFinishedHandler;
            Debug.Log($"<color=cyan>[MissionManager]</color> Unsubscribed from mission events");

            Debug.Log($"<color=cyan>[MissionManager]</color> 任务{(success ? "成功" : "失败")}: {mission.missionName}");

            // 延迟跳转到结果场景
            Debug.Log($"<color=yellow>[MissionManager]</color> Starting coroutine to transition to result scene");
            StartCoroutine(TransitionToResultScene(success));
        }

        private IEnumerator TransitionToResultScene(bool success)
        {
            Debug.Log($"<color=yellow>[MissionManager]</color> ===== TransitionToResultScene Coroutine Started =====");

            string targetScene = success ? successSceneName : failSceneName;
            Debug.Log($"<color=yellow>[MissionManager]</color> Target Scene: {targetScene}");
            Debug.Log($"<color=yellow>[MissionManager]</color> Success Scene Name: '{successSceneName}'");
            Debug.Log($"<color=yellow>[MissionManager]</color> Fail Scene Name: '{failSceneName}'");
            Debug.Log($"<color=yellow>[MissionManager]</color> Transition Delay: {transitionDelay} seconds");

            if (string.IsNullOrEmpty(targetScene))
            {
                Debug.LogError($"<color=red>[MissionManager]</color> Target scene name is NULL or EMPTY! Cannot load scene.");
                yield break;
            }

            Debug.Log($"<color=yellow>[MissionManager]</color> Waiting {transitionDelay} seconds before scene transition...");

            // 等待一段时间让玩家看到任务结果
            yield return new WaitForSeconds(transitionDelay);

            Debug.Log($"<color=green>[MissionManager]</color> Delay complete. Now loading scene: {targetScene}");

            try
            {
                SceneManager.LoadScene(targetScene);
                Debug.Log($"<color=green>[MissionManager]</color> SceneManager.LoadScene() called successfully");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"<color=red>[MissionManager]</color> Failed to load scene '{targetScene}': {e.Message}");
            }
        }

        /// <summary>
        /// 配置灾害检测任务
        /// </summary>
        private void SetupDisasterMission(DisasterDetectMission mission, MissionConfig config)
        {
            mission.missionName = config.missionName;
            mission.description = config.description;
            mission.detectionRadius = config.photoDistance; // 使用配置的拍照距离
            mission.requirePhotoConfirmation = config.requirePhotoConfirmation;

            Debug.Log($"<color=cyan>[MissionManager]</color> ===== 配置灾害检测任务 =====");
            Debug.Log($"<color=cyan>[MissionManager]</color> 火点总数: {config.fireSpawnPositions?.Length ?? 0}");
            Debug.Log($"<color=cyan>[MissionManager]</color> 生成数量: {config.numberOfFires}");
            Debug.Log($"<color=cyan>[MissionManager]</color> 检测半径: {config.photoDistance}m");

            // 验证配置
            if (config.fireSpawnPositions == null || config.fireSpawnPositions.Length == 0)
            {
                Debug.LogError("<color=red>[MissionManager]</color> fireSpawnPositions 为空！无法生成火点");
                return;
            }

            if (config.firePrefab == null)
            {
                Debug.LogWarning("<color=yellow>[MissionManager]</color> firePrefab 为空，将只创建标记点");
            }

            // 选择火焰生成位置
            List<Vector3> selectedPositions = new List<Vector3>();

            if (config.randomizeFirePositions)
            {
                // 随机选择N个位置
                List<Vector3> availablePositions = new List<Vector3>(config.fireSpawnPositions);
                int count = Mathf.Min(config.numberOfFires, availablePositions.Count);

                for (int i = 0; i < count; i++)
                {
                    int randomIndex = Random.Range(0, availablePositions.Count);
                    selectedPositions.Add(availablePositions[randomIndex]);
                    availablePositions.RemoveAt(randomIndex);
                }

                Debug.Log($"<color=cyan>[MissionManager]</color> 随机选择了 {selectedPositions.Count} 个火点位置");
            }
            else
            {
                // 使用前N个位置
                int count = Mathf.Min(config.numberOfFires, config.fireSpawnPositions.Length);
                for (int i = 0; i < count; i++)
                {
                    selectedPositions.Add(config.fireSpawnPositions[i]);
                }

                Debug.Log($"<color=cyan>[MissionManager]</color> 按顺序选择了前 {selectedPositions.Count} 个位置");
            }

            // 生成火焰对象
            GameObject firesParent = new GameObject("FirePoints");
            mission.firePoints = new List<DisasterDetectMission.FirePoint>();

            for (int i = 0; i < selectedPositions.Count; i++)
            {
                Vector3 position = selectedPositions[i];
                GameObject fireObj = null;

                if (config.firePrefab != null)
                {
                    // 实例化火焰prefab
                    fireObj = Instantiate(config.firePrefab, position, Quaternion.identity, firesParent.transform);
                    fireObj.name = $"Fire_{i + 1}";

                    // 自动添加火焰增长组件（如果火焰有ParticleSystem）
                    ParticleSystem ps = fireObj.GetComponent<ParticleSystem>();
                    if (ps != null && fireObj.GetComponent<FireGrowthEffect>() == null)
                    {
                        FireGrowthEffect growthEffect = fireObj.AddComponent<FireGrowthEffect>();
                        growthEffect.initialSizeMultiplier = 0.3f;
                        growthEffect.maxSizeMultiplier = 2.5f;
                        growthEffect.growthDuration = 60f;
                        growthEffect.randomizeStartProgress = true;
                        growthEffect.randomProgressRange = 0.3f;
                        Debug.Log($"<color=orange>[MissionManager]</color> 为火点 {i + 1} 添加增长效果");
                    }

                    Debug.Log($"<color=green>[MissionManager]</color> 生成火点 {i + 1}: {position}");
                }
                else
                {
                    // 创建简单标记
                    fireObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    fireObj.transform.position = position;
                    fireObj.transform.localScale = Vector3.one * 2f;
                    fireObj.transform.parent = firesParent.transform;
                    fireObj.name = $"FireMarker_{i + 1}";

                    // 设置颜色（红色）
                    Renderer renderer = fireObj.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        Material mat = new Material(Shader.Find("Standard"));
                        mat.color = Color.red;
                        mat.EnableKeyword("_EMISSION");
                        mat.SetColor("_EmissionColor", Color.red * 2f);
                        renderer.material = mat;
                    }

                    Debug.Log($"<color=yellow>[MissionManager]</color> 创建火点标记 {i + 1}: {position}");
                }

                // 添加到任务的火点列表
                DisasterDetectMission.FirePoint firePoint = new DisasterDetectMission.FirePoint(fireObj, position);
                mission.firePoints.Add(firePoint);
            }

            Debug.Log($"<color=green>[MissionManager]</color> 灾害检测任务配置完成，总火点: {mission.firePoints.Count}");
        }
    }
}
