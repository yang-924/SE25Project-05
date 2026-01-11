using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using MissionSystem;

/// <summary>
/// 配置文件管理器
/// 负责加载、解析和应用JSON配置文件
/// 路径策略：项目根目录/Configs（类似FlightData）
/// </summary>
public class ConfigManager : MonoBehaviour
{
    public static ConfigManager Instance { get; private set; }

    [Header("Config Path")]
    [Tooltip("配置文件夹路径（相对项目根目录）")]
    public string configFolderName = "Configs";

    [Header("Current Config")]
    public JsonConfigData currentConfig;
    public string currentConfigFileName = "";

    private string configFolderPath;
    private List<string> availableConfigFiles = new List<string>();

    void Awake()
    {
        // 单例模式
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            Debug.Log("<color=green>[ConfigManager]</color> Initialized");
        }
        else
        {
            Destroy(gameObject);
            return;
        }

        InitializeConfigPath();
    }

    /// <summary>
    /// 初始化配置文件路径
    /// </summary>
    void InitializeConfigPath()
    {
        // 使用与FlightDataRecorder相同的策略：项目根目录
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        configFolderPath = Path.Combine(projectRoot, configFolderName);

        // 创建配置文件夹（如果不存在）
        if (!Directory.Exists(configFolderPath))
        {
            Directory.CreateDirectory(configFolderPath);
            Debug.Log($"<color=yellow>[ConfigManager]</color> Created config folder: {configFolderPath}");
        }

        Debug.Log($"<color=cyan>[ConfigManager]</color> Config path: {configFolderPath}");
    }

    /// <summary>
    /// 扫描配置文件夹，获取所有JSON文件
    /// </summary>
    public List<string> ScanConfigFiles()
    {
        availableConfigFiles.Clear();

        if (!Directory.Exists(configFolderPath))
        {
            Debug.LogWarning($"<color=yellow>[ConfigManager]</color> Config folder not found: {configFolderPath}");
            return availableConfigFiles;
        }

        try
        {
            string[] jsonFiles = Directory.GetFiles(configFolderPath, "*.json");

            foreach (string filePath in jsonFiles)
            {
                string fileName = Path.GetFileName(filePath);
                availableConfigFiles.Add(fileName);
            }

            Debug.Log($"<color=cyan>[ConfigManager]</color> Found {availableConfigFiles.Count} config files");

            foreach (string file in availableConfigFiles)
            {
                Debug.Log($"  - {file}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"<color=red>[ConfigManager]</color> Error scanning config files: {e.Message}");
        }

        return availableConfigFiles;
    }

    /// <summary>
    /// 扫描配置文件夹，根据任务类型筛选配置文件
    /// </summary>
    /// <param name="missionType">任务类型（DisasterDetection, Cargo, Waypoint, Patrol）</param>
    public List<string> ScanConfigFilesByMissionType(MissionSystem.MissionType missionType)
    {
        List<string> filteredFiles = new List<string>();

        if (!Directory.Exists(configFolderPath))
        {
            Debug.LogWarning($"<color=yellow>[ConfigManager]</color> Config folder not found: {configFolderPath}");
            return filteredFiles;
        }

        try
        {
            string[] jsonFiles = Directory.GetFiles(configFolderPath, "*.json");
            string targetType = missionType.ToString();

            foreach (string filePath in jsonFiles)
            {
                // 读取配置文件判断任务类型
                string configType = GetMissionTypeFromConfig(filePath);

                if (configType == targetType)
                {
                    string fileName = Path.GetFileName(filePath);
                    filteredFiles.Add(fileName);
                }
            }

            Debug.Log($"<color=cyan>[ConfigManager]</color> Found {filteredFiles.Count} config files for mission type: {targetType}");

            foreach (string file in filteredFiles)
            {
                Debug.Log($"  - {file}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"<color=red>[ConfigManager]</color> Error scanning config files: {e.Message}");
        }

        availableConfigFiles = filteredFiles;
        return filteredFiles;
    }

    /// <summary>
    /// 从配置文件中读取任务类型（不完全加载配置）
    /// </summary>
    private string GetMissionTypeFromConfig(string filePath)
    {
        try
        {
            string json = File.ReadAllText(filePath);
            JsonConfigData config = JsonUtility.FromJson<JsonConfigData>(json);
            return config.mission.type;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"<color=yellow>[ConfigManager]</color> Error reading mission type from {Path.GetFileName(filePath)}: {e.Message}");
            return string.Empty;
        }
    }

    /// <summary>
    /// 加载指定的配置文件
    /// </summary>
    public bool LoadConfig(string fileName)
    {
        if (string.IsNullOrEmpty(fileName))
        {
            Debug.LogWarning("<color=yellow>[ConfigManager]</color> File name is empty");
            return false;
        }

        string filePath = Path.Combine(configFolderPath, fileName);

        if (!File.Exists(filePath))
        {
            Debug.LogError($"<color=red>[ConfigManager]</color> Config file not found: {filePath}");
            return false;
        }

        try
        {
            string json = File.ReadAllText(filePath);
            currentConfig = JsonUtility.FromJson<JsonConfigData>(json);
            currentConfigFileName = fileName;

            Debug.Log($"<color=green>[ConfigManager]</color> Loaded config: {fileName}");
            Debug.Log($"  Config Name: {currentConfig.configName}");
            Debug.Log($"  Mission: {currentConfig.mission.missionName} ({currentConfig.mission.type})");
            Debug.Log($"  Scene: {currentConfig.mission.sceneName}");
            Debug.Log($"  Weather: Rain={currentConfig.weather.rainIntensity}, Wind={currentConfig.weather.baseWindSpeed}m/s");

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"<color=red>[ConfigManager]</color> Error loading config: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// 应用当前配置到游戏系统
    /// </summary>
    public void ApplyCurrentConfig()
    {
        if (currentConfig == null)
        {
            Debug.LogWarning("<color=yellow>[ConfigManager]</color> No config loaded");
            return;
        }

        // 1. 应用任务配置
        ApplyMissionConfig();

        // 2. 应用天气配置
        ApplyWeatherConfig();

        Debug.Log($"<color=green>[ConfigManager]</color> Config applied: {currentConfig.configName}");
    }

    /// <summary>
    /// 应用任务配置到MissionManager
    /// </summary>
    void ApplyMissionConfig()
    {
        if (MissionManager.Instance == null)
        {
            Debug.LogWarning("<color=yellow>[ConfigManager]</color> MissionManager not found");
            return;
        }

        // 使用TaskSelectController的原始配置作为基础（保留prefab引用）
        MissionConfig config = TaskSelectController.missionConfig;

        if (config == null)
        {
            Debug.LogWarning("<color=yellow>[ConfigManager]</color> TaskSelectController.missionConfig is null, creating new config");
            config = ScriptableObject.CreateInstance<MissionConfig>();
        }
        else
        {
            Debug.Log($"<color=cyan>[ConfigManager]</color> Using existing MissionConfig as base, will override fields from JSON");
        }

        // 覆盖基础字段
        config.missionName = currentConfig.mission.missionName;
        config.description = currentConfig.mission.description;
        config.targetSceneName = currentConfig.mission.sceneName;

        // 根据任务类型覆盖特定字段
        switch (currentConfig.mission.type)
        {
            case "DisasterDetection":
                config.missionType = MissionType.DisasterDetection;

                // 转换火焰位置
                var firePositions = currentConfig.mission.disasterSettings.firePositions;
                config.fireSpawnPositions = firePositions.Select(p => p.ToVector3()).ToArray();

                config.numberOfFires = currentConfig.mission.disasterSettings.numberOfFires;
                config.photoDistance = currentConfig.mission.disasterSettings.photoDistance;
                config.requirePhotoConfirmation = currentConfig.mission.disasterSettings.requirePhoto;
                config.randomizeFirePositions = currentConfig.mission.disasterSettings.randomizePositions;
                break;

            case "Cargo":
                config.missionType = MissionType.Cargo;
                config.pickupLocation = currentConfig.mission.cargoSettings.pickupLocation.ToVector3();
                config.deliveryLocation = currentConfig.mission.cargoSettings.deliveryLocation.ToVector3();

                var lockerPositions = currentConfig.mission.cargoSettings.deliveryLockerPositions;
                config.deliveryLockerPositions = lockerPositions.Select(p => p.ToVector3()).ToArray();

                config.useRandomDeliveryPosition = currentConfig.mission.cargoSettings.useRandomDeliveryPosition;
                config.skipPickup = currentConfig.mission.cargoSettings.skipPickup;
                // prefab字段保留（deliveryLockerPrefab, cargoObjectPrefab等）
                Debug.Log($"<color=green>[ConfigManager]</color> Cargo config: deliveryLockerPrefab={(config.deliveryLockerPrefab != null ? "Exists" : "NULL")}");
                break;

            case "Waypoint":
                config.missionType = MissionType.Waypoint;

                var waypoints = currentConfig.mission.waypointSettings.waypointPositions;
                config.waypointPositions = waypoints.Select(p => p.ToVector3()).ToArray();

                config.reachThreshold = currentConfig.mission.waypointSettings.reachThreshold;
                break;
        }

        // 应用到MissionManager
        MissionManager.Instance.selectedMissionConfig = config;

        Debug.Log($"<color=cyan>[ConfigManager]</color> Mission config applied: {config.missionName}");
    }

    /// <summary>
    /// 应用天气配置到WeatherManager
    /// </summary>
    void ApplyWeatherConfig()
    {
        if (WeatherManager.Instance == null)
        {
            Debug.LogWarning("<color=yellow>[ConfigManager]</color> WeatherManager not found, will apply when scene loads");
            return;
        }

        WeatherManager.Instance.rainIntensity = currentConfig.weather.rainIntensity;
        WeatherManager.Instance.baseWindSpeed = currentConfig.weather.baseWindSpeed;
        WeatherManager.Instance.windDirection = currentConfig.weather.windDirection;
        WeatherManager.Instance.fogDensity = currentConfig.weather.fogDensity;
        WeatherManager.Instance.windShearExponent = currentConfig.weather.windShearExponent;
        WeatherManager.Instance.gustIntensity = currentConfig.weather.gustIntensity;

        Debug.Log($"<color=cyan>[ConfigManager]</color> Weather config applied: Rain={currentConfig.weather.rainIntensity}, Wind={currentConfig.weather.baseWindSpeed}m/s");
    }

    /// <summary>
    /// 获取配置文件列表（用于UI）
    /// </summary>
    public List<string> GetConfigFileNames()
    {
        if (availableConfigFiles.Count == 0)
        {
            ScanConfigFiles();
        }
        return new List<string>(availableConfigFiles);
    }

    /// <summary>
    /// 获取当前配置的预览信息
    /// </summary>
    public string GetConfigPreview()
    {
        if (currentConfig == null)
        {
            return "No config loaded";
        }

        string preview = $"<b>{currentConfig.configName}</b>\n";
        preview += $"Version: {currentConfig.version}\n";
        preview += $"\n<b>Mission:</b> {currentConfig.mission.missionName}\n";
        preview += $"Type: {currentConfig.mission.type}\n";
        preview += $"Scene: {currentConfig.mission.sceneName}\n";

        if (currentConfig.mission.type == "DisasterDetection")
        {
            preview += $"Fires: {currentConfig.mission.disasterSettings.numberOfFires}\n";
        }

        preview += $"\n<b>Weather:</b>\n";
        preview += $"Rain: {currentConfig.weather.rainIntensity * 100:F0}%\n";
        preview += $"Wind: {currentConfig.weather.baseWindSpeed} m/s @ {currentConfig.weather.windDirection}°\n";
        preview += $"Fog: {currentConfig.weather.fogDensity * 100:F0}%";

        return preview;
    }
}
