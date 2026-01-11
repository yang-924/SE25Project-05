using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;
using MissionSystem;

[System.Serializable]
public class ScenePreviewItem
{
    public string sceneName;
    public Sprite previewSprite;
}

public class SelectController : MonoBehaviour
{
    // ---------------- 场景预览 ----------------
    [Header("Scene Preview")]
    public Image previewSceneImage;                 // 场景预览框
    public List<ScenePreviewItem> scenePreviewList; // 场景名 → 预览图映射
    private Dictionary<string, Sprite> previewMap;  // 运行时字典

    private string[] sceneNames;                    // 当前任务允许的场景
    private int sceneIndex = 0;

    // ---------------- 无人机预览 ----------------
    [Header("Drone Preview")]
    public Image previewDroneImage;
    public Sprite[] dronePreviewSprites;
    public string[] droneNames;
    private int droneIndex = 0;

    // ---------------- 配置文件选择 ----------------
    [Header("Config Selection")]
    [Tooltip("配置文件下拉列表")]
    public TMP_Dropdown configDropdown;
    [Tooltip("配置预览文本")]
    public TextMeshProUGUI configPreviewText;
    [Tooltip("是否启用配置系统")]
    public bool useConfigSystem = true;

    void Start()
    {
        // 1. 构建场景预览图映射表
        previewMap = new Dictionary<string, Sprite>();
        foreach (var item in scenePreviewList)
        {
            if (!previewMap.ContainsKey(item.sceneName))
                previewMap.Add(item.sceneName, item.previewSprite);
        }

        // 2. 从任务选择界面获取允许的场景
        if (TaskSelectController.allowedScenes == null ||
            TaskSelectController.allowedScenes.Count == 0)
        {
            Debug.LogError("SceneSelectController: 没有从任务选择界面获取到允许的场景！");
            sceneNames = new string[0];
        }
        else
        {
            sceneNames = TaskSelectController.allowedScenes.ToArray();
        }

        // 3. 从任务选择界面获取允许的无人机并过滤（新增）

        // 5. 初始化配置系统
        if (useConfigSystem)
        {
            InitializeConfigSystem();
        }
        if (TaskSelectController.allowedDrones != null &&
            TaskSelectController.allowedDrones.Count > 0)
        {
            FilterDrones(TaskSelectController.allowedDrones);
        }

        // 4. 初始化预览
        UpdateScenePreview();
        UpdateDronePreview();
    }

    // 新增：过滤无人机列表
    void FilterDrones(List<string> allowedDroneNames)
    {
        Debug.Log($"[SelectController] FilterDrones called. Allowed drones from task: {string.Join(", ", allowedDroneNames)}");
        Debug.Log($"[SelectController] Current droneNames in SelectController: {string.Join(", ", droneNames)}");

        List<string> filteredNames = new List<string>();
        List<Sprite> filteredSprites = new List<Sprite>();

        for (int i = 0; i < droneNames.Length; i++)
        {
            if (allowedDroneNames.Contains(droneNames[i]))
            {
                filteredNames.Add(droneNames[i]);
                if (i < dronePreviewSprites.Length)
                {
                    filteredSprites.Add(dronePreviewSprites[i]);
                }
                Debug.Log($"[SelectController] Matched drone: {droneNames[i]}");
            }
            else
            {
                Debug.LogWarning($"[SelectController] Drone '{droneNames[i]}' not in allowed list, filtered out.");
            }
        }

        droneNames = filteredNames.ToArray();
        dronePreviewSprites = filteredSprites.ToArray();
        droneIndex = 0;

        Debug.Log($"[SelectController] Filtered drones: {droneNames.Length} available.");
    }

    // ---------------- 场景切换 ----------------
    public void NextScene()
    {
        if (sceneNames.Length == 0) return;
        sceneIndex = (sceneIndex + 1) % sceneNames.Length;
        UpdateScenePreview();
    }

    public void PrevScene()
    {
        if (sceneNames.Length == 0) return;
        sceneIndex = (sceneIndex - 1 + sceneNames.Length) % sceneNames.Length;
        UpdateScenePreview();
    }

    void UpdateScenePreview()
    {
        if (sceneNames.Length == 0)
            return;

        string currentScene = sceneNames[sceneIndex];

        if (previewMap.ContainsKey(currentScene))
        {
            previewSceneImage.sprite = previewMap[currentScene];
        }
        else
        {
            Debug.LogWarning($"没有找到场景 {currentScene} 的预览图");
            previewSceneImage.sprite = null;
        }
    }

    public void EnterScene()
    {
        if (sceneNames.Length == 0) return;

        string selectedScene = sceneNames[sceneIndex];
        string selectedDrone = (droneNames.Length > 0) ? droneNames[droneIndex] : "Default";

        MissionConfig configToUse = null;

        // 应用配置（如果启用了配置系统）
        if (useConfigSystem && ConfigManager.Instance != null && ConfigManager.Instance.currentConfig != null)
        {
            ConfigManager.Instance.ApplyCurrentConfig();
            configToUse = MissionManager.Instance.selectedMissionConfig; // 使用JSON配置生成的MissionConfig
            Debug.Log($"<color=green>[SelectController]</color> Using JSON config: {ConfigManager.Instance.currentConfig.configName}");
        }
        else
        {
            configToUse = TaskSelectController.missionConfig; // 使用原始MissionConfig
            Debug.Log($"<color=yellow>[SelectController]</color> Using TaskSelectController config: {configToUse?.missionName}");
        }

        // 检查config是否为null
        if (configToUse == null)
        {
            Debug.LogError("<color=red>[SelectController]</color> ===== 错误：没有可用的任务配置！=====");
            Debug.LogError("<color=red>[SelectController]</color> 请确保：");
            Debug.LogError("<color=red>[SelectController]</color> 1. 在TaskSelectController中分配了MissionConfig，或");
            Debug.LogError("<color=red>[SelectController]</color> 2. 启用了JSON配置系统并选择了配置文件");
            Debug.LogError("<color=red>[SelectController]</color> ===== 任务将无法正常运行！ =====");
        }

        // 调用桥接器加载场景和任务
        if (MissionBridge.Instance != null)
        {
            MissionBridge.Instance.LoadMissionScene(
                selectedScene,
                configToUse,
                selectedDrone
            );
        }
        else
        {
            Debug.LogError("[SelectController] MissionBridge not found! Falling back to direct scene load.");
            SceneManager.LoadScene(selectedScene);
        }
    }

    // ---------------- 无人机切换 ----------------
    public void NextDrone()
    {
        if (droneNames.Length == 0) return;
        droneIndex = (droneIndex + 1) % droneNames.Length;
        UpdateDronePreview();
    }

    public void PrevDrone()
    {
        if (droneNames.Length == 0) return;
        droneIndex = (droneIndex - 1 + droneNames.Length) % droneNames.Length;
        UpdateDronePreview();
    }

    void UpdateDronePreview()
    {
        if (previewDroneImage == null || dronePreviewSprites.Length == 0)
            return;

        previewDroneImage.sprite = dronePreviewSprites[droneIndex];
    }

    // ---------------- 配置系统 ----------------
    void InitializeConfigSystem()
    {
        // 确保ConfigManager存在
        if (ConfigManager.Instance == null)
        {
            GameObject configManagerObj = new GameObject("ConfigManager");
            configManagerObj.AddComponent<ConfigManager>();
            Debug.Log("<color=cyan>[SelectController]</color> Created ConfigManager");
        }

        // 扫描配置文件（根据任务类型筛选）
        List<string> configFiles;

        if (TaskSelectController.missionConfig != null)
        {
            // 使用当前任务的类型筛选配置文件
            MissionSystem.MissionType missionType = TaskSelectController.missionConfig.missionType;
            configFiles = ConfigManager.Instance.ScanConfigFilesByMissionType(missionType);
            Debug.Log($"<color=cyan>[SelectController]</color> Filtered configs for mission type: {missionType}");
        }
        else
        {
            // 没有任务配置时，显示所有配置
            configFiles = ConfigManager.Instance.ScanConfigFiles();
            Debug.LogWarning("<color=yellow>[SelectController]</color> No mission config found, showing all configs");
        }

        // 填充Dropdown
        if (configDropdown != null)
        {
            configDropdown.ClearOptions();

            if (configFiles.Count > 0)
            {
                configDropdown.AddOptions(configFiles);
                configDropdown.onValueChanged.AddListener(OnConfigSelected);

                // 加载第一个配置
                LoadConfigByIndex(0);
            }
            else
            {
                Debug.LogWarning("<color=yellow>[SelectController]</color> No config files found for current mission type.");
                configDropdown.AddOptions(new List<string> { "No configs available" });

                if (configPreviewText != null)
                {
                    if (TaskSelectController.missionConfig != null)
                    {
                        configPreviewText.text = $"No configuration files found for mission type: {TaskSelectController.missionConfig.missionType}\n\nPlease add matching JSON files to the Configs folder.";
                    }
                    else
                    {
                        configPreviewText.text = "No configuration files found.\nPlease add JSON files to the Configs folder.";
                    }
                }
            }
        }
        else
        {
            Debug.LogWarning("<color=yellow>[SelectController]</color> Config dropdown not assigned in Inspector");
        }
    }

    void OnConfigSelected(int index)
    {
        LoadConfigByIndex(index);
    }

    void LoadConfigByIndex(int index)
    {
        List<string> configFiles = ConfigManager.Instance.GetConfigFileNames();

        if (index >= 0 && index < configFiles.Count)
        {
            string fileName = configFiles[index];
            bool success = ConfigManager.Instance.LoadConfig(fileName);

            if (success && configPreviewText != null)
            {
                configPreviewText.text = ConfigManager.Instance.GetConfigPreview();
            }
        }
    }

    /// <summary>
    /// 应用当前配置（可由按钮调用）
    /// </summary>
    public void ApplyCurrentConfig()
    {
        if (ConfigManager.Instance != null && ConfigManager.Instance.currentConfig != null)
        {
            ConfigManager.Instance.ApplyCurrentConfig();
            Debug.Log("<color=green>[SelectController]</color> Configuration applied!");
        }
        else
        {
            Debug.LogWarning("<color=yellow>[SelectController]</color> No configuration loaded");
        }
    }

    /// <summary>
    /// 刷新配置文件列表（可由按钮调用）
    /// </summary>
    public void RefreshConfigList()
    {
        if (useConfigSystem)
        {
            InitializeConfigSystem();
            Debug.Log("<color=cyan>[SelectController]</color> Config list refreshed");
        }
    }

    // ---------------- 返回主界面 ----------------
    public void Return(string sceneName)
    {
        SceneManager.LoadScene(sceneName);
    }
}
