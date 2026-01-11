using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System;

/// <summary>
/// 高性能飞行数据记录器
/// 功能：记录姿态、速度、方向、坐标等数据
/// 性能优化：
/// 1. 使用 Struct 减少 GC
/// 2. 使用 Buffer 缓存数据，批量处理
/// 3. 使用 Task.Run 异步写入文件，避免阻塞主线程
/// </summary>
public class FlightDataRecorder : MonoBehaviour
{
    [System.Serializable]
    public struct FlightData
    {
        public float time;
        public Vector3 position;
        public Vector3 rotation;
        public Vector3 velocity;
        public Vector3 direction;
        public float fps; // 每秒帧率
    }

    [Header("Settings")]
    [Tooltip("是否开启记录")]
    public bool isRecording = true;

    [Tooltip("文件名前缀")]
    public string fileNamePrefix = "FlightData";

    [Tooltip("缓冲区大小（条），每存满多少条数据写入一次硬盘")]
    public int bufferSize = 2000;

    [Header("Camera System")]
    [Tooltip("FPV Camera (第一人称)")]
    public Camera fpvCamera;
    [Tooltip("TPV Camera (第三人称)")]
    public Camera tpvCamera;
    [Tooltip("截图宽度")]
    public int snapshotWidth = 1920;
    [Tooltip("截图高度")]
    public int snapshotHeight = 1080;

    private List<FlightData> dataBuffer;
    private Rigidbody rb;
    private string logFolderPath; // 统一的日志文件夹路径
    private string csvFilePath;
    private string snapshotFolderPath;
    private float startTime;
    private bool isWriting = false; // 简单的锁标志

    // FPS 计算相关
    private float fpsUpdateInterval = 1.0f; // 每秒更新一次FPS
    private float fpsAccumulator = 0.0f;
    private int fpsFrameCount = 0;
    private float currentFPS = 0.0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("[FlightDataRecorder] No Rigidbody found! Recording disabled.");
            enabled = false;
            return;
        }

        // 自动查找相机
        AutoFindCameras();

        dataBuffer = new List<FlightData>(bufferSize);
        startTime = Time.time;

        // 1. 确定保存路径 (项目根目录下的data文件夹)
        // 路径示例: C:/Users/yang/Desktop/SE25Project-05/DronePlatform/data/FlightLogs
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        logFolderPath = Path.Combine(projectRoot, "data", "FlightLogs");
        if (!Directory.Exists(logFolderPath)) Directory.CreateDirectory(logFolderPath);

        // 创建截图子文件夹
        snapshotFolderPath = Path.Combine(logFolderPath, "Snapshots");
        if (!Directory.Exists(snapshotFolderPath)) Directory.CreateDirectory(snapshotFolderPath);

        // 2. 创建带时间戳的文件名
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        csvFilePath = Path.Combine(logFolderPath, $"{fileNamePrefix}_{timestamp}.csv");

        // 3. 写入 CSV 表头
        WriteCSVHeader();
        Debug.Log($"<color=cyan>[FlightDataRecorder]</color> Recording started. Path: {csvFilePath}");
    }

    void FixedUpdate()
    {
        if (!isRecording) return;

        // 采集数据 (Struct 是值类型，分配在栈上或作为 List 的连续内存，性能高)
        FlightData data = new FlightData
        {
            time = Time.time - startTime,
            position = transform.position,
            rotation = transform.eulerAngles,
            velocity = rb.velocity,
            direction = transform.forward,
            fps = currentFPS // 记录当前FPS
        };

        dataBuffer.Add(data);


        // 缓冲区满，触发写入
        if (dataBuffer.Count >= bufferSize)
        {
            FlushBuffer();
        }
    }

    void Update()
    {
        // 计算FPS
        fpsAccumulator += Time.unscaledDeltaTime;
        fpsFrameCount++;

        if (fpsAccumulator >= fpsUpdateInterval)
        {
            currentFPS = fpsFrameCount / fpsAccumulator;
            fpsAccumulator = 0.0f;
            fpsFrameCount = 0;
        }

        // 按键截图测试
        if (InputManager.instance.F12Input)
        {
            CaptureSnapshot(true, "Manual_FPV"); // F12 拍 FPV
            Debug.Log("Snapshot FPV Triggered");
        }
        if (Input.GetKeyDown(KeyCode.F11))
        {
            CaptureSnapshot(false, "Manual_TPV"); // F11 拍 TPV
            Debug.Log("Snapshot TPV Triggered");
        }
    }

    void OnDisable()
    {
        // 停止所有协程，防止场景切换时继续执行
        StopAllCoroutines();

        // 游戏结束或脚本禁用时，强制写入剩余数据
        FlushBuffer();
        Debug.Log($"<color=yellow>[FlightDataRecorder]</color> Recording stopped. Data saved.");
    }

    void OnDestroy()
    {
        // 确保清理资源
        StopAllCoroutines();
        FlushBuffer();
    }

    /// <summary>
    /// 自动查找场景中的 FPV 和 TPV 相机
    /// </summary>
    private void AutoFindCameras()
    {
        // 如果已手动赋值，跳过自动查找
        if (fpvCamera != null && tpvCamera != null)
        {
            Debug.Log("[FlightDataRecorder] Cameras already assigned manually.");
            return;
        }

        // 方法1: 通过脚本组件类型识别
        if (fpvCamera == null)
        {
            GimbalCamera gimbal = FindObjectOfType<GimbalCamera>();
            if (gimbal != null)
            {
                fpvCamera = gimbal.GetComponent<Camera>();
                Debug.Log("[FlightDataRecorder] Auto-found FPV camera by GimbalCamera component.");
            }
        }

        if (tpvCamera == null)
        {
            CameraController controller = FindObjectOfType<CameraController>();
            if (controller != null)
            {
                tpvCamera = controller.GetComponent<Camera>();
                Debug.Log("[FlightDataRecorder] Auto-found TPV camera by CameraController component.");
            }
        }

        // 方法2: 通过名称查找（如果方法1失败）
        if (fpvCamera == null || tpvCamera == null)
        {
            Camera[] allCameras = FindObjectsOfType<Camera>();
            foreach (Camera cam in allCameras)
            {
                string name = cam.gameObject.name.ToLower();

                if (fpvCamera == null && (name.Contains("fpv") || name.Contains("gimbal")))
                {
                    fpvCamera = cam;
                    Debug.Log($"[FlightDataRecorder] Auto-found FPV camera by name: {cam.gameObject.name}");
                }

                if (tpvCamera == null && (name.Contains("tpv") || name.Contains("follow") || name.Contains("third")))
                {
                    tpvCamera = cam;
                    Debug.Log($"[FlightDataRecorder] Auto-found TPV camera by name: {cam.gameObject.name}");
                }
            }
        }

        // 结果检查
        if (fpvCamera == null)
        {
            Debug.LogWarning("[FlightDataRecorder] FPV Camera not found! Snapshots may not work. Consider naming camera with 'FPV' or adding GimbalCamera component.");
        }
        if (tpvCamera == null)
        {
            Debug.LogWarning("[FlightDataRecorder] TPV Camera not found! Snapshots may not work. Consider naming camera with 'TPV' or adding CameraController component.");
        }
    }

    /// <summary>
    /// 外部调用：拍摄快照
    /// </summary>
    /// <param name="useFPV">true使用FPV相机，false使用TPV相机</param>
    /// <param name="suffix">文件名后缀（如 "Crash", "Land"）</param>
    public void CaptureSnapshot(bool useFPV, string suffix = "Manual")
    {
        Camera targetCam = useFPV ? fpvCamera : tpvCamera;
        if (targetCam == null)
        {
            Debug.LogWarning("[FlightDataRecorder] Target camera is null!");
            return;
        }
        StartCoroutine(CaptureCoroutine(targetCam, suffix));
    }

    private System.Collections.IEnumerator CaptureCoroutine(Camera cam, string suffix)
    {
        // 等待帧结束，确保渲染完成
        yield return new WaitForEndOfFrame();

        // 创建临时 RenderTexture
        RenderTexture rt = new RenderTexture(snapshotWidth, snapshotHeight, 24);
        // 临时替换相机的 TargetTexture
        RenderTexture originalTarget = cam.targetTexture;
        cam.targetTexture = rt;

        // 手动渲染一帧
        cam.Render();

        // 激活 RT 并读取像素
        RenderTexture.active = rt;
        Texture2D screenShot = new Texture2D(snapshotWidth, snapshotHeight, TextureFormat.RGB24, false);
        screenShot.ReadPixels(new Rect(0, 0, snapshotWidth, snapshotHeight), 0, 0);
        screenShot.Apply();

        // 恢复相机状态
        cam.targetTexture = originalTarget;
        RenderTexture.active = null;
        Destroy(rt);

        // 异步保存图片
        byte[] bytes = screenShot.EncodeToJPG();
        Destroy(screenShot); // 销毁 Texture2D 释放内存

        string filename = $"Snap_{DateTime.Now:yyyyMMdd_HHmmss_fff}_{suffix}.jpg";
        string fullPath = Path.Combine(snapshotFolderPath, filename);

        // 使用 Task 写入硬盘，避免卡顿（添加检查避免场景切换时继续执行）
        if (this != null && enabled)
        {
            Task.Run(() =>
            {
                try
                {
                    File.WriteAllBytes(fullPath, bytes);
                }
                catch (Exception e)
                {
                    // 捕获异常，防止崩溃
                }
            });
        }
    }

    private void WriteCSVHeader()
    {
        // CSV 格式表头
        string header = "Time(s),FPS,PosX,PosY,PosZ,RotX,RotY,RotZ,VelX,VelY,VelZ,DirX,DirY,DirZ\n";
        try
        {
            File.WriteAllText(csvFilePath, header);
        }
        catch (Exception e)
        {
            Debug.LogError($"[FlightDataRecorder] Failed to write header: {e.Message}");
            isRecording = false;
        }
    }

    private void FlushBuffer()
    {
        if (dataBuffer.Count == 0) return;

        // 将数据从 List 复制到数组，以便在后台线程安全使用
        // 这样主线程可以立即清空 List 继续记录下一帧，无需等待 IO
        FlightData[] dataToWrite = dataBuffer.ToArray();
        dataBuffer.Clear();

        // 启动后台任务进行文件写入
        Task.Run(() => WriteDataToFile(dataToWrite));
    }

    // 此方法在后台线程运行
    private void WriteDataToFile(FlightData[] data)
    {
        if (isWriting)
        {
            // 如果上一次写入还没完成（极少情况，除非硬盘极慢），为了防止文件占用冲突，可以简单的等待或丢弃
            // 这里选择简单的自旋等待一下
            System.Threading.Thread.Sleep(10);
        }

        isWriting = true;
        StringBuilder sb = new StringBuilder(data.Length * 100); // 预分配内存

        foreach (var d in data)
        {
            // 使用 F4 保留4位小数，兼顾精度和文件大小
            sb.Append(d.time.ToString("F4")).Append(",");
            sb.Append(d.fps.ToString("F2")).Append(","); // FPS保留2位小数

            sb.Append(d.position.x.ToString("F4")).Append(",");
            sb.Append(d.position.y.ToString("F4")).Append(",");
            sb.Append(d.position.z.ToString("F4")).Append(",");

            sb.Append(d.rotation.x.ToString("F4")).Append(",");
            sb.Append(d.rotation.y.ToString("F4")).Append(",");
            sb.Append(d.rotation.z.ToString("F4")).Append(",");

            sb.Append(d.velocity.x.ToString("F4")).Append(",");
            sb.Append(d.velocity.y.ToString("F4")).Append(",");
            sb.Append(d.velocity.z.ToString("F4")).Append(",");

            sb.Append(d.direction.x.ToString("F4")).Append(",");
            sb.Append(d.direction.y.ToString("F4")).Append(",");
            sb.Append(d.direction.z.ToString("F4")).Append("\n");
        }

        try
        {
            // 追加写入文件
            File.AppendAllText(csvFilePath, sb.ToString());
        }
        catch (Exception e)
        {
            // 注意：后台线程的 Log 不一定会立即显示在 Unity Console
            Debug.LogError($"[FlightDataRecorder] Write error: {e.Message}");
        }
        finally
        {
            isWriting = false;
        }
    }
}
