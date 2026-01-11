using UnityEngine;

/// <summary>
/// 相机持久化脚本
/// 让相机在场景切换时不被销毁
/// 挂载在相机的父物体上
/// </summary>
public class CameraPersistence : MonoBehaviour
{
    public static CameraPersistence Instance;

    void Awake()
    {
        // 单例模式：确保只有一个相机系统存在
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            Debug.Log("[CameraPersistence] Camera system will persist across scenes.");
        }
        else
        {
            Debug.LogWarning("[CameraPersistence] Duplicate camera system found, destroying.");
            Destroy(gameObject);
        }
    }
}
