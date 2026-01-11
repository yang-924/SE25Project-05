using UnityEngine;

/// <summary>
/// 无人机相机配置（ScriptableObject）
/// 为不同类型无人机定义专属的相机偏移和参数
/// </summary>
[CreateAssetMenu(fileName = "DroneCameraConfig", menuName = "Drone/Camera Config")]
public class DroneCameraConfig : ScriptableObject
{
    [System.Serializable]
    public class CameraOffsetData
    {
        [Header("基础配置")]
        public string droneTypeName; // 无人机类型名称（如 "airc", "drone"）

        [Header("FPV 相机配置")]
        public Vector3 fpvOffset = new Vector3(0f, 0.5f, 1f);  // FPV相机偏移
        public Vector3 fpvGimbalRotation = new Vector3(0f, 0f, 0f); // FPV云台角度
        
        [Header("TPV 相机配置")]
        public Vector3 tpvOffset = new Vector3(0f, 3f, -8f);  // TPV相机偏移
        public float tpvDistance = 8f;  // TPV距离
        public float tpvHeight = 3f;    // TPV高度
    }

    [Header("相机配置列表")]
    public CameraOffsetData[] cameraConfigs;

    /// <summary>
    /// 根据无人机类型获取配置
    /// </summary>
    public CameraOffsetData GetConfigForDrone(string droneType)
    {
        foreach (var config in cameraConfigs)
        {
            if (config.droneTypeName == droneType)
            {
                return config;
            }
        }

        // 返回默认配置
        Debug.LogWarning($"[DroneCameraConfig] No config found for '{droneType}', using first config as default.");
        return cameraConfigs.Length > 0 ? cameraConfigs[0] : null;
    }
}
