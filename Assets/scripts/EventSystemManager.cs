using UnityEngine;
using UnityEngine.EventSystems;

/// <summary>
/// EventSystem 单例管理器
/// 防止场景切换时出现多个 EventSystem 或清理警告
/// </summary>
public class EventSystemManager : MonoBehaviour
{
    private static EventSystemManager instance;

    void Awake()
    {
        // 如果已经存在实例，销毁当前对象
        if (instance != null)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);

        // 确保有 EventSystem 组件
        if (GetComponent<EventSystem>() == null)
        {
            gameObject.AddComponent<EventSystem>();
        }

        // 确保有 StandaloneInputModule 组件
        if (GetComponent<StandaloneInputModule>() == null)
        {
            gameObject.AddComponent<StandaloneInputModule>();
        }

        Debug.Log("[EventSystemManager] Persistent EventSystem created.");
    }
}
