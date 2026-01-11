using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 信号延迟特效
/// 挂载在 FPV 相机上，根据无人机与基站的距离产生画面延迟。
/// </summary>
[RequireComponent(typeof(Camera))]
public class SignalLatencyEffect : MonoBehaviour
{
    [Header("Signal Settings")]
    [Tooltip("无人机的 Transform (用于计算距离)")]
    public Transform droneTransform;

    [Tooltip("基站位置 (如果为空，则使用脚本启动时的位置)")]
    public Transform baseStation;

    [Tooltip("每米距离产生的延迟 (秒)。例如 0.001 表示 1000米延迟 1秒。")]
    public float latencyPerMeter = 0.001f;

    [Tooltip("基础处理延迟 (秒)")]
    public float minDelay = 0.05f;

    [Tooltip("最大延迟上限 (秒)，防止内存占用过大")]
    public float maxDelay = 3.0f;

    [Header("Performance")]
    [Tooltip("降采样倍数 (1=原画质, 2=1/2分辨率, 4=1/4分辨率)。\n延迟画面通常不需要全高清，增加此值可大幅降低显存占用。")]
    [Range(1, 8)]
    public int downsampleFactor = 2;

    // 使用双向链表存储帧缓冲区，方便头部修剪
    private LinkedList<FrameData> frameBuffer = new LinkedList<FrameData>();
    // 对象池复用 RenderTexture，避免 GC 和 频繁申请显存
    private Queue<RenderTexture> rtPool = new Queue<RenderTexture>();

    private Vector3 startPosition;
    private float currentLatencyMs = 0f; // 当前延迟（毫秒）

    /// <summary>
    /// 获取当前信号延迟（毫秒）
    /// </summary>
    public float CurrentLatencyMs => currentLatencyMs;

    struct FrameData
    {
        public float timestamp;
        public RenderTexture rt;
    }

    void Start()
    {
        // 如果未手动赋值 droneTransform，自动查找无人机
        if (droneTransform == null)
        {
            // 方法1：通过 Tag 查找
            GameObject drone = GameObject.FindGameObjectWithTag("Player");
            if (drone != null)
            {
                droneTransform = drone.transform;
                Debug.Log("[SignalLatencyEffect] Auto-found drone by tag: Player");
            }
            else
            {
                // 方法2：通过 DroneController 组件查找
                DroneController controller = FindObjectOfType<DroneController>();
                if (controller != null)
                {
                    droneTransform = controller.transform;
                    Debug.Log("[SignalLatencyEffect] Auto-found drone by DroneController component");
                }
                else
                {
                    // 方法3：尝试获取根物体（最后的备选）
                    droneTransform = transform.root;
                    Debug.LogWarning("[SignalLatencyEffect] Using root transform as fallback. This may not be the drone!");
                }
            }
        }

        if (baseStation == null)
            startPosition = droneTransform.position;
        else
            startPosition = baseStation.position;
    }

    // OnRenderImage 在相机渲染完成后调用，允许我们修改最终图像
    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // 1. 计算当前应有的延迟
        Vector3 basePos = (baseStation != null) ? baseStation.position : startPosition;
        float distance = Vector3.Distance(droneTransform.position, basePos);
        float requiredDelay = Mathf.Clamp(minDelay + distance * latencyPerMeter, 0, maxDelay);
        currentLatencyMs = requiredDelay * 1000f; // 转换为毫秒

        // 2. 将当前帧存入缓冲区
        // 获取或创建 RT
        int w = source.width / downsampleFactor;
        int h = source.height / downsampleFactor;
        RenderTexture rt = GetRT(w, h);

        // 拷贝当前屏幕内容
        Graphics.Blit(source, rt);

        frameBuffer.AddLast(new FrameData
        {
            timestamp = Time.time,
            rt = rt
        });

        // 3. 在缓冲区中寻找符合延迟时间的帧
        float targetTime = Time.time - requiredDelay;
        FrameData frameToShow = frameBuffer.Last.Value; // 默认显示最新帧（如果缓冲区不足）

        var node = frameBuffer.First;
        while (node != null)
        {
            // 如果当前节点的时间戳 > 目标时间，说明我们已经“跨过”了目标时间点
            // 目标时间点应该在 node 和 node.Previous 之间
            // 我们选择显示 node.Previous (即比目标时间稍早一点点的帧，保证延迟不低于预期)
            if (node.Value.timestamp > targetTime)
            {
                if (node.Previous != null)
                {
                    frameToShow = node.Previous.Value;
                    // 清理掉比 frameToShow 更早的所有帧，因为时间是单向流动的，以后再也用不到它们了
                    PruneBuffer(node.Previous);
                }
                else
                {
                    // 缓冲区里最老的帧都比目标时间新（说明刚开始运行，或者延迟突然变大）
                    // 直接显示最老的帧
                    frameToShow = node.Value;
                }
                break;
            }

            // 如果遍历到了链表末尾，说明所有帧都比目标时间老（说明延迟很小，或者缓冲区刚建立）
            // 显示最新的一帧
            if (node.Next == null)
            {
                frameToShow = node.Value;
                PruneBuffer(node); // 清理掉之前的
                break;
            }

            node = node.Next;
        }

        // 4. 将选中的延迟帧渲染到屏幕
        Graphics.Blit(frameToShow.rt, destination);
    }

    /// <summary>
    /// 清理缓冲区，移除 keepNode 之前的所有节点
    /// </summary>
    void PruneBuffer(LinkedListNode<FrameData> keepNode)
    {
        while (frameBuffer.First != keepNode)
        {
            ReleaseRT(frameBuffer.First.Value.rt);
            frameBuffer.RemoveFirst();
        }
    }

    RenderTexture GetRT(int w, int h)
    {
        if (rtPool.Count > 0)
        {
            var rt = rtPool.Dequeue();
            // 如果分辨率变了（比如窗口大小改变），则销毁旧的创建新的
            if (rt.width != w || rt.height != h)
            {
                rt.Release();
                return new RenderTexture(w, h, 0);
            }
            return rt;
        }
        return new RenderTexture(w, h, 0);
    }

    void ReleaseRT(RenderTexture rt)
    {
        rtPool.Enqueue(rt);
    }

    void OnDestroy()
    {
        // 清理显存
        foreach (var frame in frameBuffer)
        {
            if (frame.rt != null) frame.rt.Release();
        }
        foreach (var rt in rtPool)
        {
            if (rt != null) rt.Release();
        }
    }
}
