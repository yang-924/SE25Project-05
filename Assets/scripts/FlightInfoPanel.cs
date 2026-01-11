using UnityEngine;
using TMPro;

public class FlightInfoPanel : MonoBehaviour
{
    public TextMeshProUGUI batteryText;
    public TextMeshProUGUI speedText;
    public TextMeshProUGUI distanceText;
    public TextMeshProUGUI positionText;
    public TextMeshProUGUI latencyText;

    [Header("任务信息显示（可选）")]
    public TextMeshProUGUI targetLocationText;  // 目标位置显示（航点/货物目标点）

    [Header("无人机绑定")]
    [SerializeField] private Transform droneTransform;  // 无人机Transform
    [SerializeField] private string droneObjectName = "Drone";  // 无人机对象名称

    [Header("小地图绑定")]
    [SerializeField] private MinimapController minimapController;  // 小地图控制器

    [Header("延迟效果绑定")]
    [SerializeField] private SignalLatencyEffect signalLatencyEffect;  // 信号延迟效果组件

    [Header("速度计算")]
    [SerializeField] private float speedSmoothTime = 0.3f;  // 速度平滑时间
    [SerializeField] private int positionHistorySize = 10;  // 位置历史记录数量

    [Header("坐标显示设置")]
    [SerializeField] private bool showGPSCoordinates = true;  // 是否显示GPS坐标（经纬度）
    [SerializeField] private bool useSimpleGPSFormat = true;  // 使用简洁格式（否则使用度分秒）

    // 私有变量
    private Vector3 lastPosition;
    private float lastUpdateTime;
    private float currentSpeed = 0f;
    private float smoothedSpeed = 0f;
    private float speedVelocity = 0f;

    // 位置历史记录（用于更平滑的速度计算）
    private Vector3[] positionHistory;
    private int positionIndex = 0;
    private int positionCount = 0;

    void Start()
    {
        // 初始化位置历史记录
        positionHistory = new Vector3[Mathf.Max(2, positionHistorySize)];

        // 尝试自动查找无人机
        if (droneTransform == null)
        {
            FindDroneInScene();
        }

        // 尝试自动查找小地图控制器
        if (minimapController == null)
        {
            FindMinimapController();
        }

        // 尝试自动查找信号延迟效果组件
        if (signalLatencyEffect == null)
        {
            signalLatencyEffect = FindObjectOfType<SignalLatencyEffect>();
            if (signalLatencyEffect != null)
            {
                Debug.Log("[FlightInfoPanel] Auto-found SignalLatencyEffect component");
            }
        }

        // 如果找到了无人机，初始化位置
        if (droneTransform != null)
        {
            lastPosition = droneTransform.position;
            lastUpdateTime = Time.time;

            // 初始化位置历史
            for (int i = 0; i < positionHistory.Length; i++)
            {
                positionHistory[i] = droneTransform.position;
            }
            positionCount = positionHistory.Length;
        }

        // 初始更新
        UpdateInfo();

        // 检查 CoordinateConverter 是否存在
        if (showGPSCoordinates)
        {
            // CoordinateConverter初始化检查（日志已移除）
        }
    }

    void Update()
    {
        // 如果无人机Transform为空，尝试重新查找
        if (droneTransform == null)
        {
            FindDroneInScene();
        }

        // 如果小地图控制器为空，尝试重新查找
        if (minimapController == null)
        {
            FindMinimapController();
        }

        // 更新飞行信息
        UpdateInfo();
    }

    /// <summary>
    /// 在场景中查找无人机对象
    /// </summary>
    private void FindDroneInScene()
    {
        // 按名称查找
        GameObject droneObject = GameObject.Find(droneObjectName);

        // 如果找不到，尝试按标签查找（使用Player tag）
        if (droneObject == null)
        {
            droneObject = GameObject.FindGameObjectWithTag("Player");
        }

        // 备选：通过DroneController组件查找
        if (droneObject == null)
        {
            DroneController controller = FindObjectOfType<DroneController>();
            if (controller != null)
            {
                droneObject = controller.gameObject;
            }
        }

        // 最后尝试：查找包含"Drone"关键字的对象
        if (droneObject == null)
        {
            GameObject[] allObjects = GameObject.FindObjectsOfType<GameObject>();
            foreach (GameObject obj in allObjects)
            {
                if (obj.name.Contains("Drone") || obj.name.Contains("drone"))
                {
                    droneObject = obj;
                    break;
                }
            }
        }

        if (droneObject != null)
        {
            droneTransform = droneObject.transform;
            Debug.Log($"FlightInfoPanel: 已绑定无人机 '{droneObject.name}'");
        }
        else
        {
            Debug.LogWarning($"FlightInfoPanel: 场景中未找到无人机对象 '{droneObjectName}'");
        }
    }

    /// <summary>
    /// 在场景中查找小地图控制器
    /// </summary>
    private void FindMinimapController()
    {
        // 查找场景中所有MinimapController组件
        MinimapController[] controllers = GameObject.FindObjectsOfType<MinimapController>();

        if (controllers.Length > 0)
        {
            minimapController = controllers[0];
            Debug.Log($"FlightInfoPanel: 已绑定小地图控制器");
        }
        else
        {
            Debug.LogWarning($"FlightInfoPanel: 场景中未找到小地图控制器");
        }
    }

    /// <summary>
    /// 实时计算无人机速度
    /// </summary>
    private float CalculateSpeed()
    {
        if (droneTransform == null) return 0f;

        float deltaTime = Time.time - lastUpdateTime;

        if (deltaTime > 0.001f)  // 避免除零
        {
            // 方法1：简单计算瞬时速度
            Vector3 positionDelta = droneTransform.position - lastPosition;
            currentSpeed = positionDelta.magnitude / deltaTime;

            // 方法2：使用位置历史计算更平滑的速度
            if (positionHistorySize > 1)
            {
                UpdatePositionHistory();
                currentSpeed = CalculateSpeedFromHistory();
            }

            // 平滑速度值
            smoothedSpeed = Mathf.SmoothDamp(smoothedSpeed, currentSpeed, ref speedVelocity, speedSmoothTime);

            // 更新记录
            lastPosition = droneTransform.position;
            lastUpdateTime = Time.time;

            return smoothedSpeed;
        }

        return currentSpeed;
    }

    /// <summary>
    /// 更新位置历史记录
    /// </summary>
    private void UpdatePositionHistory()
    {
        positionHistory[positionIndex] = droneTransform.position;
        positionIndex = (positionIndex + 1) % positionHistory.Length;
        positionCount = Mathf.Min(positionCount + 1, positionHistory.Length);
    }

    /// <summary>
    /// 从位置历史计算速度
    /// </summary>
    private float CalculateSpeedFromHistory()
    {
        if (positionCount < 2) return 0f;

        // 获取最早和最新的位置
        int oldestIndex = (positionIndex - positionCount + positionHistory.Length) % positionHistory.Length;
        int newestIndex = (positionIndex - 1 + positionHistory.Length) % positionHistory.Length;

        Vector3 oldestPos = positionHistory[oldestIndex];
        Vector3 newestPos = positionHistory[newestIndex];

        // 计算总位移和时间
        float totalDistance = Vector3.Distance(newestPos, oldestPos);
        float totalTime = Time.deltaTime * (positionCount - 1);

        return totalDistance / Mathf.Max(totalTime, 0.001f);
    }

    /// <summary>
    /// 更新速度显示（支持目标速度对比）
    /// </summary>
    private void UpdateSpeedDisplay(float currentSpeed)
    {
        // 检查是否有任务且启用了定速巡航
        var missionManager = MissionSystem.MissionManager.Instance;
        float targetSpeed = 0f;
        bool hasCruiseControl = false;

        if (missionManager != null && missionManager.currentMission != null)
        {
            // 如果是航点任务，检查是否启用定速巡航
            if (missionManager.currentMission is MissionSystem.WaypointMission)
            {
                // 从配置获取定速巡航参数
                if (ConfigManager.Instance?.currentConfig?.mission?.waypointSettings != null)
                {
                    var settings = ConfigManager.Instance.currentConfig.mission.waypointSettings;
                    targetSpeed = settings.cruiseSpeed;
                    hasCruiseControl = settings.enableCruiseControl;
                }
            }
        }

        // 显示速度
        if (hasCruiseControl && targetSpeed > 0)
        {
            // 定速巡航模式：显示"当前速度 / 目标速度"
            string speedColor = GetSpeedColor(currentSpeed, targetSpeed);
            speedText.text = $"Speed: <color={speedColor}>{currentSpeed:F1}</color> / {targetSpeed:F1} m/s";

            // 记录速度偏差到评分系统
            if (MissionSystem.MissionScoreManager.Instance != null)
            {
                MissionSystem.MissionScoreManager.Instance.RecordSpeedDeviation(currentSpeed, targetSpeed);
            }
        }
        else
        {
            // 普通模式：只显示当前速度
            speedText.text = $"Speed: {currentSpeed:F1} m/s";
        }
    }

    /// <summary>
    /// 根据速度偏差返回颜色
    /// </summary>
    private string GetSpeedColor(float currentSpeed, float targetSpeed)
    {
        float deviation = Mathf.Abs(currentSpeed - targetSpeed);
        float deviationPercent = deviation / targetSpeed;

        if (deviationPercent <= 0.1f) return "green";    // 10%以内：绿色
        else if (deviationPercent <= 0.2f) return "yellow"; // 20%以内：黄色
        else return "red";                                // 超过20%：红色
    }

    /// <summary>
    /// 获取与飞手的距离（从小地图控制器）
    /// </summary>
    private float GetPilotDistance()
    {
        if (minimapController != null)
        {
            return minimapController.GetPilotDistance();
        }
        return 0f;
    }

    /// <summary>
    /// 更新所有显示信息
    /// </summary>
    public void UpdateInfo()
    {
        if (droneTransform == null)
        {
            // 显示错误信息
            batteryText.text = "Battery: --%";
            speedText.text = "Speed: -- m/s";
            distanceText.text = "Distance: -- m";
            positionText.text = "Position: (--, --, --)";
            latencyText.text = "Latency: -- ms";
            if (targetLocationText != null)
                targetLocationText.text = "Target: --";
            return;
        }

        // 获取当前位置
        Vector3 position = droneTransform.position;

        // 计算实时速度
        float speed = CalculateSpeed();

        // 获取与飞手的距离
        float distance = GetPilotDistance();

        // 格式化位置显示
        string positionDisplay;
        if (showGPSCoordinates && CoordinateConverter.Instance != null)
        {
            var (lat, lon, alt) = CoordinateConverter.Instance.WorldToGPS(position);
            if (useSimpleGPSFormat)
            {
                positionDisplay = CoordinateConverter.FormatGPSSimple(lat, lon, alt);
            }
            else
            {
                positionDisplay = CoordinateConverter.FormatGPS(lat, lon, alt);
            }
        }
        else
        {
            positionDisplay = $"({position.x:F1}, {position.y:F1}, {position.z:F1})";
        }

        // 获取实际延迟值
        float latency = 0f;
        if (signalLatencyEffect != null)
        {
            latency = signalLatencyEffect.CurrentLatencyMs;
        }

        // 更新UI文本
        batteryText.text = $"Battery: 100%";
        UpdateSpeedDisplay(speed); // 使用新的速度显示方法
        distanceText.text = $"Distance: {distance:F1} m";
        positionText.text = $"Position: {positionDisplay}";
        latencyText.text = $"Latency: {latency:F0} ms";

        // 更新目标位置（如果有任务）
        if (targetLocationText != null)
        {
            UpdateTargetLocation();
        }
    }

    /// <summary>
    /// 更新目标位置显示
    /// </summary>
    private void UpdateTargetLocation()
    {
        // 检查 targetLocationText 是否绑定
        if (targetLocationText == null)
        {
            return;  // 未绑定则直接返回，不报错
        }

        // 尝试从 MissionManager 获取当前任务
        if (MissionSystem.MissionManager.Instance != null &&
            MissionSystem.MissionManager.Instance.currentMission != null)
        {
            var mission = MissionSystem.MissionManager.Instance.currentMission;

            // 如果是货物运输任务
            if (mission is MissionSystem.CargoMission cargoMission)
            {
                // 显示目标位置（确保UI可见）
                if (!targetLocationText.gameObject.activeSelf)
                {
                    targetLocationText.gameObject.SetActive(true);
                }

                if (cargoMission.deliveryPoint == null)
                {
                    targetLocationText.text = "Target: Delivery Point NULL";
                    return;
                }

                Vector3 targetPos = cargoMission.deliveryPoint.position;

                // 只计算水平距离（忽略高度）
                float horizontalDistance = CoordinateConverter.HorizontalDistance(droneTransform.position, targetPos);
                float altitudeDiff = Mathf.Abs(droneTransform.position.y - targetPos.y);

                string targetDisplay;
                if (showGPSCoordinates && CoordinateConverter.Instance != null)
                {
                    var (lat, lon, alt) = CoordinateConverter.Instance.WorldToGPS(targetPos);

                    if (useSimpleGPSFormat)
                    {
                        // 经纬度保留4位小数（2位太少无法区分）
                        targetDisplay = $"{lat:F4}°, {lon:F4}° (H:{horizontalDistance:F0}m, ΔAlt:{altitudeDiff:F0}m)";
                    }
                    else
                    {
                        targetDisplay = CoordinateConverter.FormatGPS(lat, lon, alt) + $" (H:{horizontalDistance:F0}m, ΔAlt:{altitudeDiff:F0}m)";
                    }
                }
                else
                {
                    targetDisplay = $"({targetPos.x:F1}, {targetPos.y:F1}, {targetPos.z:F1}) - H:{horizontalDistance:F0}m, ΔAlt:{altitudeDiff:F0}m";
                }

                targetLocationText.text = $"Target: {targetDisplay}";
            }
            // 如果是灾害检测任务
            else if (mission is MissionSystem.DisasterDetectMission disasterMission)
            {
                // 显示起火点数量而非GPS坐标（确保UI可见）
                if (!targetLocationText.gameObject.activeSelf)
                {
                    targetLocationText.gameObject.SetActive(true);
                }

                string progressText = disasterMission.GetProgressText();
                float nearestDistance = disasterMission.GetNearestUndetectedFireDistance();

                string distanceInfo = nearestDistance < float.MaxValue
                    ? $"Nearest: {nearestDistance:F0}m"
                    : "All Found";

                targetLocationText.text = $"Fire Points: {progressText} | {distanceInfo}";
            }
            // 如果是航点任务
            else if (mission is MissionSystem.WaypointMission waypointMission)
            {
                if (!targetLocationText.gameObject.activeSelf)
                {
                    targetLocationText.gameObject.SetActive(true);
                }

                // 获取当前航点索引和目标航点
                int currentIndex = waypointMission.currentWaypointIndex;
                var waypoints = waypointMission.waypoints;

                if (waypoints != null && waypoints.Count > 0 && currentIndex < waypoints.Count)
                {
                    Transform targetWaypoint = waypoints[currentIndex];
                    if (targetWaypoint != null)
                    {
                        Vector3 targetPos = targetWaypoint.position;
                        float distance = Vector3.Distance(droneTransform.position, targetPos);

                        targetLocationText.text = $"Waypoint: {currentIndex + 1}/{waypoints.Count} | Dist: {distance:F0}m";
                    }
                    else
                    {
                        targetLocationText.text = $"Waypoint: {currentIndex + 1}/{waypoints.Count}";
                    }
                }
                else
                {
                    targetLocationText.text = "Waypoint: Completed";
                }
            }
            // 如果是侦察任务
            else if (mission is MissionSystem.ReconMission reconMission)
            {
                if (!targetLocationText.gameObject.activeSelf)
                {
                    targetLocationText.gameObject.SetActive(true);
                }

                // 显示当前侦察点进度
                targetLocationText.text = reconMission.GetProgressText();
            }
            else
            {
                // 其他任务类型，隐藏或显示默认文本
                targetLocationText.gameObject.SetActive(false);
                // targetLocationText.text = "Target: None";
            }
        }
        else
        {
            targetLocationText.gameObject.SetActive(false);
            // targetLocationText.text = "Target: No Mission";
        }
    }

    /// <summary>
    /// 手动绑定无人机Transform（可选）
    /// </summary>
    public void BindDrone(Transform drone)
    {
        droneTransform = drone;
        lastPosition = drone.position;
        lastUpdateTime = Time.time;
        Debug.Log($"FlightInfoPanel: 手动绑定无人机 '{drone.name}'");
    }

    /// <summary>
    /// 手动绑定小地图控制器（可选）
    /// </summary>
    public void BindMinimapController(MinimapController minimap)
    {
        minimapController = minimap;
        Debug.Log($"FlightInfoPanel: 手动绑定小地图控制器");
    }

    /// <summary>
    /// 设置要查找的无人机对象名称
    /// </summary>
    public void SetDroneObjectName(string name)
    {
        droneObjectName = name;

        // 如果已经设置了Transform但名称不匹配，清空以便重新查找
        if (droneTransform != null && droneTransform.name != name)
        {
            droneTransform = null;
        }
    }

    /// <summary>
    /// 获取当前无人机速度
    /// </summary>
    public float GetCurrentSpeed()
    {
        return smoothedSpeed;
    }

    /// <summary>
    /// 获取当前无人机位置
    /// </summary>
    public Vector3 GetCurrentPosition()
    {
        if (droneTransform != null)
            return droneTransform.position;
        return Vector3.zero;
    }

    /// <summary>
    /// 检查是否已成功绑定无人机
    /// </summary>
    public bool IsDroneBound()
    {
        return droneTransform != null;
    }

    /// <summary>
    /// 检查是否已成功绑定小地图控制器
    /// </summary>
    public bool IsMinimapControllerBound()
    {
        return minimapController != null;
    }
}