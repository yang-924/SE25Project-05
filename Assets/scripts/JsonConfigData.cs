using System;
using UnityEngine;

/// <summary>
/// JSON配置文件数据结构
/// 包含任务配置和天气配置
/// </summary>
[System.Serializable]
public class JsonConfigData
{
    [Header("Config Info")]
    public string configName = "Default Config";
    public string description = "";
    public string version = "1.0";

    [Header("Mission Settings")]
    public MissionData mission = new MissionData();

    [Header("Weather Settings")]
    public WeatherData weather = new WeatherData();

    /// <summary>
    /// 任务配置数据
    /// </summary>
    [System.Serializable]
    public class MissionData
    {
        public string type = "DisasterDetection"; // Waypoint, Cargo, DisasterDetection, Patrol
        public string sceneName = "City";
        public string missionName = "Disaster Detection";
        public string description = "";

        // Disaster Detection 专用
        public DisasterSettings disasterSettings = new DisasterSettings();

        // Cargo 专用
        public CargoSettings cargoSettings = new CargoSettings();

        // Waypoint 专用
        public WaypointSettings waypointSettings = new WaypointSettings();

        // Recon 专用
        public ReconSettings reconSettings = new ReconSettings();
    }

    [System.Serializable]
    public class DisasterSettings
    {
        public int numberOfFires = 3;
        public Vector3Data[] firePositions = new Vector3Data[0];
        public float photoDistance = 20f;
        public bool requirePhoto = true;
        public bool randomizePositions = true;
    }

    [System.Serializable]
    public class CargoSettings
    {
        public Vector3Data pickupLocation = new Vector3Data();
        public Vector3Data deliveryLocation = new Vector3Data();
        public Vector3Data[] deliveryLockerPositions = new Vector3Data[0];
        public bool useRandomDeliveryPosition = true;
        public bool skipPickup = false;
    }

    [System.Serializable]
    public class WaypointSettings
    {
        public Vector3Data[] waypointPositions = new Vector3Data[0];
        public float reachThreshold = 5f;

        // 定速巡航参数
        [Tooltip("目标巡航速度（m/s），0表示不限制")]
        public float cruiseSpeed = 0f;
        [Tooltip("是否启用定速巡航模式")]
        public bool enableCruiseControl = false;
        [Tooltip("速度容差（m/s），实际速度在此范围内视为达到目标速度")]
        public float speedTolerance = 1f;
        [Tooltip("是否循环航点")]
        public bool loopWaypoints = false;
    }

    [System.Serializable]
    public class ReconSettings
    {
        public Vector3Data[] reconPoints = new Vector3Data[0];
        public float reachThreshold = 10f;
        public float hoverTime = 3f;
        public float hoverStabilityThreshold = 2f;
    }

    /// <summary>
    /// 天气配置数据
    /// </summary>
    [System.Serializable]
    public class WeatherData
    {
        public float rainIntensity = 0f;       // 0-1
        public float baseWindSpeed = 5f;       // m/s
        public float windDirection = 0f;       // 0-360 degrees
        public float fogDensity = 0f;          // 0-1
        public float windShearExponent = 0.3f;
        public float gustIntensity = 1f;
    }

    /// <summary>
    /// Vector3序列化辅助类（JSON不直接支持Unity的Vector3）
    /// </summary>
    [System.Serializable]
    public class Vector3Data
    {
        public float x;
        public float y;
        public float z;

        public Vector3Data() { }

        public Vector3Data(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3Data(Vector3 v)
        {
            x = v.x;
            y = v.y;
            z = v.z;
        }

        public Vector3 ToVector3()
        {
            return new Vector3(x, y, z);
        }

        public static implicit operator Vector3(Vector3Data data)
        {
            return new Vector3(data.x, data.y, data.z);
        }

        public static implicit operator Vector3Data(Vector3 v)
        {
            return new Vector3Data(v.x, v.y, v.z);
        }
    }
}
