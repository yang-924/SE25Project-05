using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using MissionSystem;

public class TaskSelectController : MonoBehaviour
{
    public static string selectedTask;        // 当前任务名称
    public static List<string> allowedScenes; // 当前任务支持的场景列表
    public static List<string> allowedDrones; // 当前任务支持的无人机列表（新增）
    public static MissionConfig missionConfig; // 当前任务的配置（新增）

    // 任务数据结构（新增）
    [System.Serializable]
    public class TaskData
    {
        public string taskName;
        public List<string> scenes;
        public List<string> drones;
        public MissionConfig config; // 在 Inspector 中拖入对应的 MissionConfig
    }

    [Header("Task Configurations")]
    public List<TaskData> taskDataList = new List<TaskData>();

    // 运行时字典（从 List 构建）
    private Dictionary<string, TaskData> taskDataMap;

    void Start()
    {
        Debug.Log("[TaskSelectController] Start() called, building task dictionary...");

        // 构建运行时字典
        taskDataMap = new Dictionary<string, TaskData>();
        foreach (var data in taskDataList)
        {
            if (!taskDataMap.ContainsKey(data.taskName))
            {
                taskDataMap.Add(data.taskName, data);
                Debug.Log($"[TaskSelectController] Added task: {data.taskName}");
            }
        }

        Debug.Log($"[TaskSelectController] Dictionary built with {taskDataMap.Count} tasks.");
    }

    public void SelectTask(string taskName)
    {
        Debug.Log($"[TaskSelectController] SelectTask() called with parameter: '{taskName}'");

        selectedTask = taskName;

        // 从字典中获取任务数据
        if (taskDataMap.ContainsKey(taskName))
        {
            TaskData data = taskDataMap[taskName];
            allowedScenes = data.scenes;
            allowedDrones = data.drones;   // 新增
            missionConfig = data.config;   // 新增

            Debug.Log($"[TaskSelectController] Selected task: {taskName}, scenes: {data.scenes.Count}, drones: {data.drones.Count}");
        }
        else
        {
            Debug.LogError($"[TaskSelectController] Task '{taskName}' not found in taskDataMap!");
            allowedScenes = new List<string>();
            allowedDrones = new List<string>();
            missionConfig = null;
        }

        // 跳转到场景选择界面
        SceneManager.LoadScene("SceneSelectUI");
    }
}
