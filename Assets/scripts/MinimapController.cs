using UnityEngine;
using UnityEngine.UI;
using System;

public class MinimapController : MonoBehaviour
{
    [Header("无人机设置")]
    [SerializeField] private string droneObjectName = "Drone";  // 无人机对象名称
    [SerializeField] private Transform droneTransform;          // 缓存无人机Transform

    [Header("UI组件")]
    [SerializeField] private RectTransform droneArrow;  // 无人机箭头
    [SerializeField] private RectTransform pilotDot;    // 飞手点
    [SerializeField] private RectTransform targetDot;   // 目标点

    [Header("地图参数")]
    [SerializeField] private float mapRadius = 50f;     // 世界坐标显示半径
    [SerializeField] private float uiRadius = 80f;      // UI像素半径

    [Header("位置设置（可在Inspector中编辑）")]
    [SerializeField] private Vector3 _pilotPosition = new Vector3(10, 0, 10);
    [SerializeField] private Vector3 _targetPosition = new Vector3(-10, 0, -10);

    [Header("箭头转向设置")]
    [SerializeField] private string movementFieldName = "moveAt"; // 运动方向字段名称
    [SerializeField] private bool useMovementDirection = true;    // 是否使用运动方向
    [SerializeField] private float directionSmoothTime = 0.1f;    // 方向平滑时间

    // 使用属性包装，支持运行时动态修改
    private Vector3 pilotPosition;
    private Vector3 targetPosition;
    private Vector3 currentArrowDirection = Vector3.forward;
    private Vector3 directionVelocity = Vector3.zero;

    [Header("外观设置")]
    [SerializeField] private Color pilotColor = Color.green;
    [SerializeField] private Color targetColor = Color.red;
    [SerializeField] private float dotSize = 10f;

    [Header("调试信息")]
    [SerializeField] private bool showDebugInfo = false;
    [SerializeField] private bool autoFindDrone = true;  // 是否自动查找无人机
    [SerializeField] private float pilotDistance;
    [SerializeField] private float targetDistance;

    // 事件，当位置更新时触发
    public event Action<Vector3> OnPilotPositionChanged;
    public event Action<Vector3> OnTargetPositionChanged;

    // 反射获取运动方向字段
    private System.Reflection.FieldInfo movementField;

    private void Awake()
    {
        // 初始化位置
        pilotPosition = _pilotPosition;
        targetPosition = _targetPosition;
    }

    private void Start()
    {
        InitializeUI();

        // 尝试自动查找无人机
        if (autoFindDrone)
        {
            FindDroneInScene();
        }

        // 初始化运动方向字段
        if (useMovementDirection)
        {
            InitializeMovementField();
        }
    }

    private void Update()
    {
        // 如果无人机Transform为空，尝试重新查找
        if (droneTransform == null)
        {
            if (autoFindDrone)
            {
                FindDroneInScene();
            }

            if (droneTransform == null)
            {
                if (showDebugInfo)
                    Debug.LogWarning($"MinimapController: 未找到无人机对象 '{droneObjectName}'");
                return;
            }
        }

        UpdateDroneArrow();
        UpdatePilotIndicator();
        UpdateTargetIndicator();

        if (showDebugInfo)
        {
            UpdateDebugInfo();
        }
    }

    /// <summary>
    /// 初始化运动方向字段的反射
    /// </summary>
    private void InitializeMovementField()
    {
        if (droneTransform == null) return;

        // 获取无人机上所有MonoBehaviour组件
        MonoBehaviour[] components = droneTransform.GetComponents<MonoBehaviour>();

        foreach (MonoBehaviour comp in components)
        {
            if (comp == null) continue;

            // 查找指定的字段
            System.Type type = comp.GetType();
            movementField = type.GetField(movementFieldName,
                System.Reflection.BindingFlags.Public |
                System.Reflection.BindingFlags.Instance);

            if (movementField != null && movementField.FieldType == typeof(Vector3))
            {
                if (showDebugInfo)
                    Debug.Log($"找到运动方向字段: {type.Name}.{movementFieldName}");
                return;
            }

            // 如果没有找到指定名称的字段，尝试查找类似的字段
            System.Reflection.FieldInfo[] allFields = type.GetFields(
                System.Reflection.BindingFlags.Public |
                System.Reflection.BindingFlags.Instance);

            foreach (System.Reflection.FieldInfo field in allFields)
            {
                if (field.FieldType == typeof(Vector3))
                {
                    // 尝试匹配常见的运动方向字段名称
                    string fieldName = field.Name.ToLower();
                    if (fieldName.Contains("move") ||
                        fieldName.Contains("velocity") ||
                        fieldName.Contains("direction") ||
                        fieldName.Contains("movement"))
                    {
                        movementField = field;
                        movementFieldName = field.Name;

                        if (showDebugInfo)
                            Debug.Log($"找到替代运动方向字段: {type.Name}.{field.Name}");
                        return;
                    }
                }
            }
        }

        if (showDebugInfo)
            Debug.LogWarning($"未找到运动方向字段 '{movementFieldName}'，将使用无人机朝向");
    }

    /// <summary>
    /// 获取无人机的运动方向
    /// </summary>
    private Vector3 GetDroneMovementDirection()
    {
        if (droneTransform == null)
            return Vector3.forward;

        // 如果使用运动方向并且找到了相应的字段
        if (useMovementDirection && movementField != null)
        {
            try
            {
                // 尝试从字段获取值
                Vector3 moveAt = (Vector3)movementField.GetValue(droneTransform.GetComponent<MonoBehaviour>());

                // 归一化运动方向
                if (moveAt.magnitude > 0.01f)
                {
                    moveAt.y = 0; // 忽略垂直分量
                    return moveAt.normalized;
                }
            }
            catch (System.Exception e)
            {
                if (showDebugInfo)
                    Debug.LogWarning($"获取运动方向失败: {e.Message}");
            }
        }

        // 如果无法获取运动方向，使用无人机的前方朝向
        Vector3 forward = droneTransform.forward;
        forward.y = 0;
        return forward.normalized;
    }

    /// <summary>
    /// 更新无人机箭头方向（基于运动方向）
    /// </summary>
    private void UpdateDroneArrow()
    {
        if (droneArrow == null) return;

        // 获取运动方向
        Vector3 targetDirection = GetDroneMovementDirection();

        // 平滑过渡方向（可选）
        if (directionSmoothTime > 0)
        {
            currentArrowDirection = Vector3.SmoothDamp(
                currentArrowDirection,
                targetDirection,
                ref directionVelocity,
                directionSmoothTime);
        }
        else
        {
            currentArrowDirection = targetDirection;
        }

        // 计算箭头旋转角度
        if (currentArrowDirection.magnitude > 0.01f)
        {
            float angle = Mathf.Atan2(currentArrowDirection.x, currentArrowDirection.z) * Mathf.Rad2Deg;
            droneArrow.localEulerAngles = new Vector3(0, 0, -angle);
        }
    }

    /// <summary>
    /// 在场景中查找无人机对象
    /// </summary>
    private void FindDroneInScene()
    {
        // 方法1：按名称查找
        GameObject droneObject = GameObject.Find(droneObjectName);

        // 方法2：如果找不到，尝试按标签查找（使用Player tag）
        if (droneObject == null)
        {
            droneObject = GameObject.FindGameObjectWithTag("Player");
        }

        // 方法3：通过DroneController组件查找
        if (droneObject == null)
        {
            DroneController controller = FindObjectOfType<DroneController>();
            if (controller != null)
            {
                droneObject = controller.gameObject;
            }
        }

        // 方法4：如果还是找不到，尝试查找包含"Drone"关键字的对象
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

            // 重新初始化运动方向字段
            if (useMovementDirection)
            {
                InitializeMovementField();
            }

            if (showDebugInfo)
                Debug.Log($"MinimapController: 已找到并绑定无人机 '{droneObject.name}'");
        }
        else if (showDebugInfo)
        {
            Debug.LogWarning($"MinimapController: 场景中未找到无人机对象");
        }
    }

    /// <summary>
    /// 初始化UI外观
    /// </summary>
    private void InitializeUI()
    {
        // 设置点的大小
        if (pilotDot != null)
        {
            pilotDot.sizeDelta = new Vector2(dotSize, dotSize);
            if (pilotDot.GetComponent<Image>() != null)
                pilotDot.GetComponent<Image>().color = pilotColor;
        }

        if (targetDot != null)
        {
            targetDot.sizeDelta = new Vector2(dotSize, dotSize);
            if (targetDot.GetComponent<Image>() != null)
                targetDot.GetComponent<Image>().color = targetColor;
        }
    }

    /// <summary>
    /// 更新飞手指示器
    /// </summary>
    private void UpdatePilotIndicator()
    {
        if (pilotDot == null) return;

        Vector3 offset = pilotPosition - droneTransform.position;
        Vector2 offset2D = new Vector2(offset.x, offset.z);
        float distance = offset2D.magnitude;

        pilotDistance = distance; // 记录距离用于调试

        UpdateDotPosition(pilotDot, offset2D, distance);
    }

    /// <summary>
    /// 更新目标指示器
    /// </summary>
    private void UpdateTargetIndicator()
    {
        if (targetDot == null) return;

        Vector3 offset = targetPosition - droneTransform.position;
        Vector2 offset2D = new Vector2(offset.x, offset.z);
        float distance = offset2D.magnitude;

        targetDistance = distance; // 记录距离用于调试

        UpdateDotPosition(targetDot, offset2D, distance);
    }

    /// <summary>
    /// 更新点的位置（通用方法）
    /// </summary>
    private void UpdateDotPosition(RectTransform dot, Vector2 offset2D, float distance)
    {
        if (distance <= mapRadius)
        {
            // 在地图范围内
            float ratio = distance / mapRadius;
            Vector2 direction = offset2D.normalized;
            Vector2 uiPos = direction * (ratio * uiRadius);

            dot.localPosition = uiPos;
            dot.gameObject.SetActive(true);
        }
        else
        {
            // 超出地图范围
            dot.gameObject.SetActive(false);
        }
    }

    /// <summary>
    /// 更新调试信息
    /// </summary>
    private void UpdateDebugInfo()
    {
        if (showDebugInfo)
        {
            Vector3 movementDir = GetDroneMovementDirection();
            float movementAngle = Mathf.Atan2(movementDir.x, movementDir.z) * Mathf.Rad2Deg;

            Debug.Log($"飞手距离: {pilotDistance:F1}m, " +
                     $"目标距离: {targetDistance:F1}m, " +
                     $"运动方向角度: {movementAngle:F1}°, " +
                     $"无人机位置: {droneTransform.position}");
        }
    }

    #region 公开API - 供外部脚本调用

    /// <summary>
    /// 手动设置无人机Transform（可选，如果不设置会自动查找）
    /// </summary>
    public void SetDroneTransform(Transform drone)
    {
        droneTransform = drone;

        // 重新初始化运动方向字段
        if (useMovementDirection)
        {
            InitializeMovementField();
        }

        if (showDebugInfo)
            Debug.Log($"MinimapController: 手动绑定无人机 '{drone.name}'");
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
    /// 设置运动方向字段名称
    /// </summary>
    public void SetMovementFieldName(string fieldName)
    {
        movementFieldName = fieldName;

        // 重新初始化运动方向字段
        if (droneTransform != null && useMovementDirection)
        {
            InitializeMovementField();
        }
    }

    /// <summary>
    /// 是否使用运动方向控制箭头
    /// </summary>
    public void SetUseMovementDirection(bool useMovement)
    {
        useMovementDirection = useMovement;

        if (useMovementDirection && droneTransform != null)
        {
            InitializeMovementField();
        }
    }

    /// <summary>
    /// 设置方向平滑时间
    /// </summary>
    public void SetDirectionSmoothTime(float smoothTime)
    {
        directionSmoothTime = Mathf.Max(0, smoothTime);
    }

    /// <summary>
    /// 手动触发查找无人机
    /// </summary>
    public void FindAndBindDrone()
    {
        FindDroneInScene();
    }

    /// <summary>
    /// 设置飞手位置（运行时动态修改）
    /// </summary>
    public void SetPilotPosition(Vector3 position)
    {
        pilotPosition = position;
        _pilotPosition = position; // 同步到Inspector显示
        OnPilotPositionChanged?.Invoke(position);

        if (showDebugInfo)
            Debug.Log($"飞手位置更新: {position}");
    }

    /// <summary>
    /// 设置目标位置（运行时动态修改）
    /// </summary>
    public void SetTargetPosition(Vector3 position)
    {
        targetPosition = position;
        _targetPosition = position; // 同步到Inspector显示
        OnTargetPositionChanged?.Invoke(position);

        if (showDebugInfo)
            Debug.Log($"目标位置更新: {position}");
    }

    /// <summary>
    /// 获取当前飞手位置
    /// </summary>
    public Vector3 GetPilotPosition()
    {
        return pilotPosition;
    }

    /// <summary>
    /// 获取当前飞手位置
    /// </summary>
    public float GetPilotDistance()
    {
        return pilotDistance;
    }

    /// <summary>
    /// 获取当前目标位置
    /// </summary>
    public Vector3 GetTargetPosition()
    {
        return targetPosition;
    }

    /// <summary>
    /// 获取当前绑定的无人机Transform
    /// </summary>
    public Transform GetDroneTransform()
    {
        return droneTransform;
    }

    /// <summary>
    /// 获取当前箭头方向
    /// </summary>
    public Vector3 GetCurrentArrowDirection()
    {
        return currentArrowDirection;
    }

    /// <summary>
    /// 获取当前箭头角度（度）
    /// </summary>
    public float GetCurrentArrowAngle()
    {
        if (currentArrowDirection.magnitude > 0.01f)
        {
            return Mathf.Atan2(currentArrowDirection.x, currentArrowDirection.z) * Mathf.Rad2Deg;
        }
        return 0f;
    }

    #endregion

    #region Editor相关（不影响运行时）

    // 在Inspector中修改值时自动更新
    private void OnValidate()
    {
        // 确保值在合理范围内
        mapRadius = Mathf.Max(0.1f, mapRadius);
        uiRadius = Mathf.Max(10f, uiRadius);
        dotSize = Mathf.Max(2f, dotSize);
        directionSmoothTime = Mathf.Max(0, directionSmoothTime);

        // 立即应用Inspector中的位置更改（仅在编辑器中）
        if (!Application.isPlaying)
        {
            pilotPosition = _pilotPosition;
            targetPosition = _targetPosition;
        }
    }

    #endregion
}