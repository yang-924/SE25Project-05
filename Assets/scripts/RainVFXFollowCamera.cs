using UnityEngine;
using UnityEngine.VFX;

/// <summary>
/// 让雨VFX跟随相机移动，实现无限降雨区域
/// 挂载在 RainVFX GameObject 上
/// </summary>
[RequireComponent(typeof(VisualEffect))]
public class RainVFXFollowCamera : MonoBehaviour
{
    [Header("跟随设置")]
    [Tooltip("要跟随的相机（如果为空，自动查找主相机）")]
    public Camera targetCamera;

    [Tooltip("雨区域在相机上方的高度偏移")]
    public float heightOffset = 20f;

    [Tooltip("是否平滑跟随")]
    public bool smoothFollow = true;

    [Tooltip("平滑跟随速度")]
    public float followSpeed = 5f;

    [Header("雨区域尺寸")]
    [Tooltip("雨区域大小（对应VFX Graph中的Box Size）")]
    public Vector3 rainAreaSize = new Vector3(50, 5, 50);

    [Header("调试")]
    public bool showGizmos = true;

    private VisualEffect vfx;
    private Transform cameraTransform;

    private void Start()
    {
        vfx = GetComponent<VisualEffect>();

        // 自动查找相机
        FindTargetCamera();
    }

    private void FindTargetCamera()
    {
        if (targetCamera == null)
        {
            // 首先尝试查找 Camera.main
            targetCamera = Camera.main;

            // 如果没找到，尝试查找FPV相机
            if (targetCamera == null)
            {
                GameObject fpvCameraObj = GameObject.Find("fpvCamera");
                if (fpvCameraObj != null)
                {
                    targetCamera = fpvCameraObj.GetComponent<Camera>();
                    Debug.Log($"<color=cyan>[RainVFXFollowCamera]</color> 找到FPV相机: fpvCamera");
                }
            }

            // 如果还是没找到，尝试查找任意激活的相机
            if (targetCamera == null)
            {
                Camera[] cameras = FindObjectsOfType<Camera>();
                if (cameras.Length > 0)
                {
                    targetCamera = cameras[0];
                    Debug.LogWarning($"<color=yellow>[RainVFXFollowCamera]</color> 未找到主相机或FPV相机，使用第一个找到的相机: {targetCamera.name}");
                }
            }

            if (targetCamera == null)
            {
                Debug.LogError("<color=red>[RainVFXFollowCamera]</color> 未找到任何相机！雨VFX跟随功能已禁用");
                enabled = false;
                return;
            }
        }

        cameraTransform = targetCamera.transform;

        Debug.Log($"<color=cyan>[RainVFXFollowCamera]</color> 雨VFX将跟随相机: <color=lime>{targetCamera.name}</color> (Tag: {targetCamera.tag})");
    }

    /// <summary>
    /// 允许外部在相机切换时重新查找目标相机
    /// </summary>
    public void RefreshCamera()
    {
        targetCamera = null;
        FindTargetCamera();
    }

    private void LateUpdate()
    {
        if (cameraTransform == null) return;

        // 计算目标位置（相机正上方 + 高度偏移）
        Vector3 targetPosition = cameraTransform.position + Vector3.up * heightOffset;

        // 可选：只跟随XZ平面，保持固定高度
        // targetPosition.y = heightOffset;

        // 移动VFX
        if (smoothFollow)
        {
            transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * followSpeed);
        }
        else
        {
            transform.position = targetPosition;
        }

        // 可选：更新VFX Graph中的Box Center属性（如果你暴露了这个参数）
        // vfx.SetVector3("BoxCenter", Vector3.zero); // 因为VFX现在跟随了，所以相对中心是(0,0,0)
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos) return;

        Gizmos.color = new Color(0, 1, 1, 0.3f);
        Gizmos.DrawWireCube(transform.position, rainAreaSize);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + Vector3.down * 50);
    }
}
