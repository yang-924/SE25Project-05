using UnityEngine;

/// <summary>
/// 模拟机械云台效果的相机控制器 (Gimbal)
/// 适用场景：FPV相机防抖、侦察相机定向
/// 功能：
/// 1. 位置始终跟随目标（无人机）
/// 2. 旋转稳定：过滤掉目标的倾斜（Pitch/Roll），只跟随偏航（Yaw）或锁定方向
/// </summary>
public class GimbalCamera : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("要跟随的目标（无人机）")]
    public Transform target;

    [Tooltip("相机相对于目标的偏移位置（如果为0则在Start时自动计算）")]
    public Vector3 positionOffset;

    [Header("Gimbal Control")]
    [Tooltip("是否跟随目标的偏航角（机头朝向）。\nTrue: 相机随无人机转向，但保持水平。\nFalse: 相机锁定世界方向（如指南针）。")]
    public bool followYaw = true;

    [Tooltip("云台的目标角度 (Pitch, Yaw, Roll)。\nPitch (X): 俯仰角 (如 0=平视, 45=俯视)\nYaw (Y): 偏航角 (followYaw=true时为相对偏移，false时为世界角度)\nRoll (Z): 翻滚角 (通常为0保持水平)")]
    public Vector3 gimbalRotation = Vector3.zero;

    [Header("Smoothing")]
    [Tooltip("是否启用平滑插值")]
    public bool enableSmoothing = true;
    [Tooltip("平滑速度")]
    public float smoothSpeed = 15f;

    void Start()
    {
        // 如果未手动赋值 target，自动查找无人机
        if (target == null)
        {
            // 方法1：通过 Tag 查找
            GameObject drone = GameObject.FindGameObjectWithTag("Player");
            if (drone != null)
            {
                target = drone.transform;
                Debug.Log("[GimbalCamera] Auto-found drone by tag: Player");
            }
            else
            {
                // 方法2：通过 DroneController 组件查找
                DroneController controller = FindObjectOfType<DroneController>();
                if (controller != null)
                {
                    target = controller.transform;
                    Debug.Log("[GimbalCamera] Auto-found drone by DroneController component");
                }
                else if (transform.parent != null)
                {
                    // 方法3：尝试获取父物体
                    target = transform.parent;
                    Debug.Log("[GimbalCamera] Using parent as target");
                }
            }
        }

        if (target == null)
        {
            Debug.LogWarning("[GimbalCamera] No target found! Camera will not follow anything. Make sure drone has 'Player' tag or DroneController component.");
            // 不禁用脚本，允许后续动态赋值
            return;
        }

        // 如果未设置偏移，智能计算初始偏移
        if (positionOffset == Vector3.zero)
        {
            // 检查相机是否是无人机的子物体
            if (transform.parent == target)
            {
                // 如果是子物体，直接使用局部坐标
                positionOffset = transform.localPosition;
                Debug.Log($"[GimbalCamera] Using localPosition as offset: {positionOffset}");
            }
            else
            {
                // 如果不是子物体，计算世界坐标差值
                Vector3 calculatedOffset = target.InverseTransformPoint(transform.position);
                float distance = calculatedOffset.magnitude;

                // 如果距离过大（>50），说明相机可能是场景默认位置，使用合理默认值
                if (distance > 50f)
                {
                    positionOffset = new Vector3(0f, 2f, -5f); // 默认：无人机后上方
                    Debug.LogWarning($"[GimbalCamera] Camera too far from drone ({distance:F1}m), using default offset: {positionOffset}. Consider setting offset manually or making camera a child of drone.");
                }
                else
                {
                    positionOffset = calculatedOffset;
                    Debug.Log($"[GimbalCamera] Calculated offset from world position: {positionOffset}");
                }
            }
        }
        else
        {
            Debug.Log($"[GimbalCamera] Using pre-configured offset: {positionOffset}");
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        // --- 1. 位置更新 ---
        // 无论旋转如何，相机必须物理上跟随无人机
        transform.position = target.TransformPoint(positionOffset);

        // --- 2. 旋转更新 ---
        float targetYaw;

        if (followYaw)
        {
            // 稳健的 Yaw 计算：将目标的前方向量投影到水平面
            // 这样即使无人机大角度俯仰，也能获得正确的水平朝向
            Vector3 forwardFlat = Vector3.ProjectOnPlane(target.forward, Vector3.up);

            // 只有当投影向量长度足够时才更新 Yaw（避免万向节死锁时的抖动）
            if (forwardFlat.sqrMagnitude > 0.001f)
            {
                targetYaw = Quaternion.LookRotation(forwardFlat).eulerAngles.y;
            }
            else
            {
                // 如果无人机垂直朝上/朝下，保持当前的 Yaw 或使用目标的原始 Yaw
                targetYaw = transform.eulerAngles.y;
            }

            // 加上用户设定的相对偏移
            targetYaw += gimbalRotation.y;
        }
        else
        {
            // 锁定世界方向
            targetYaw = gimbalRotation.y;
        }

        // 构造目标旋转：
        // Pitch (X): 用户设定
        // Yaw (Y): 计算结果
        // Roll (Z): 用户设定 (通常为0)
        Quaternion desiredRot = Quaternion.Euler(gimbalRotation.x, targetYaw, gimbalRotation.z);

        // 应用旋转
        if (enableSmoothing)
        {
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, Time.deltaTime * smoothSpeed);
        }
        else
        {
            transform.rotation = desiredRot;
        }
    }

    /// <summary>
    /// 外部接口：设置云台角度 (例如通过 UI 或按键控制)
    /// </summary>
    /// <param name="pitch">俯仰角</param>
    /// <param name="yawOffset">偏航偏移</param>
    public void SetGimbalAngle(float pitch, float yawOffset)
    {
        gimbalRotation.x = pitch;
        gimbalRotation.y = yawOffset;
    }

    /// <summary>
    /// 外部接口：设置观察方向向量
    /// </summary>
    public void SetLookDirection(Vector3 direction)
    {
        if (direction == Vector3.zero) return;
        Quaternion rot = Quaternion.LookRotation(direction);
        // 将方向转换为 gimbalRotation 格式
        // 注意：这会覆盖 followYaw 的逻辑，通常用于锁定模式
        gimbalRotation = rot.eulerAngles;
        followYaw = false; // 强制切换为锁定模式
    }
}
