using UnityEngine;

/// <summary>
/// 火焰增长效果
/// 让ParticleSystem的size随时间逐渐增大到上限值
/// 模拟火势蔓延效果
/// </summary>
[RequireComponent(typeof(ParticleSystem))]
public class FireGrowthEffect : MonoBehaviour
{
    [Header("Growth Settings")]
    [Tooltip("初始大小倍率")]
    [Range(0.1f, 1f)]
    public float initialSizeMultiplier = 0.3f;

    [Tooltip("最大大小倍率")]
    [Range(1f, 5f)]
    public float maxSizeMultiplier = 2.5f;

    [Tooltip("增长到最大值所需时间（秒）")]
    [Range(5f, 300f)]
    public float growthDuration = 60f;

    [Header("Growth Curve")]
    [Tooltip("增长曲线（可自定义增长速度）")]
    public AnimationCurve growthCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

    [Header("Advanced")]
    [Tooltip("是否在达到最大值后保持")]
    public bool maintainMaxSize = true;

    [Tooltip("是否在开始时随机初始进度（模拟不同着火时间）")]
    public bool randomizeStartProgress = false;

    [Tooltip("随机初始进度范围（0-1）")]
    [Range(0f, 0.5f)]
    public float randomProgressRange = 0.2f;

    private ParticleSystem ps;
    private ParticleSystem.MainModule psMain;
    private float originalStartSize;
    private float currentTime = 0f;
    private bool isGrowing = true;

    void Start()
    {
        ps = GetComponent<ParticleSystem>();
        psMain = ps.main;

        // 记录原始大小
        originalStartSize = psMain.startSize.constant;

        // 随机初始进度
        if (randomizeStartProgress)
        {
            currentTime = Random.Range(0f, growthDuration * randomProgressRange);
        }

        // 设置初始大小
        float initialSize = originalStartSize * initialSizeMultiplier;
        psMain.startSize = initialSize;

        Debug.Log($"<color=orange>[FireGrowth]</color> {gameObject.name} 初始大小: {initialSize:F2}, 目标大小: {originalStartSize * maxSizeMultiplier:F2}, 增长时间: {growthDuration}s");
    }

    void Update()
    {
        if (!isGrowing) return;

        currentTime += Time.deltaTime;

        // 计算增长进度（0-1）
        float progress = Mathf.Clamp01(currentTime / growthDuration);

        // 应用增长曲线
        float curveValue = growthCurve.Evaluate(progress);

        // 插值计算当前大小
        float targetSize = Mathf.Lerp(
            originalStartSize * initialSizeMultiplier,
            originalStartSize * maxSizeMultiplier,
            curveValue
        );

        // 应用到粒子系统
        psMain.startSize = targetSize;

        // 检查是否达到最大值
        if (progress >= 1f)
        {
            if (maintainMaxSize)
            {
                isGrowing = false;
                Debug.Log($"<color=red>[FireGrowth]</color> {gameObject.name} 火势已达到最大！");
            }
            else
            {
                // 如果不保持最大值，重新开始（循环增长）
                currentTime = 0f;
            }
        }

        // 每10秒显示一次进度（调试用）
        if (Time.frameCount % 600 == 0 && isGrowing)
        {
            Debug.Log($"<color=yellow>[FireGrowth]</color> {gameObject.name} 增长进度: {progress * 100:F1}%, 当前大小: {targetSize:F2}");
        }
    }

    /// <summary>
    /// 获取当前增长进度（0-1）
    /// </summary>
    public float GetGrowthProgress()
    {
        return Mathf.Clamp01(currentTime / growthDuration);
    }

    /// <summary>
    /// 重置增长
    /// </summary>
    public void ResetGrowth()
    {
        currentTime = 0f;
        isGrowing = true;
        psMain.startSize = originalStartSize * initialSizeMultiplier;
        Debug.Log($"<color=cyan>[FireGrowth]</color> {gameObject.name} 火势增长已重置");
    }

    /// <summary>
    /// 立即设置为最大值
    /// </summary>
    public void SetToMaximum()
    {
        currentTime = growthDuration;
        psMain.startSize = originalStartSize * maxSizeMultiplier;
        isGrowing = false;
        Debug.Log($"<color=red>[FireGrowth]</color> {gameObject.name} 火势已设为最大");
    }

    /// <summary>
    /// 扑灭火焰（逐渐缩小）
    /// </summary>
    public void ExtinguishFire(float extinguishDuration = 3f)
    {
        StartCoroutine(ExtinguishCoroutine(extinguishDuration));
    }

    private System.Collections.IEnumerator ExtinguishCoroutine(float duration)
    {
        isGrowing = false;
        float startSize = psMain.startSize.constant;
        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = elapsed / duration;
            psMain.startSize = Mathf.Lerp(startSize, 0f, t);
            yield return null;
        }

        // 停止粒子系统
        ps.Stop();
        Debug.Log($"<color=cyan>[FireGrowth]</color> {gameObject.name} 火焰已扑灭");
    }

    // 可视化增长曲线（Scene视图中）
    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;

        // 在火焰上方显示进度条
        Vector3 position = transform.position + Vector3.up * 5f;
        float progress = GetGrowthProgress();

        // 进度条背景
        Gizmos.color = Color.black;
        Gizmos.DrawLine(position - Vector3.right * 2f, position + Vector3.right * 2f);

        // 进度条前景
        Gizmos.color = Color.Lerp(Color.yellow, Color.red, progress);
        Vector3 progressEnd = position - Vector3.right * 2f + Vector3.right * (4f * progress);
        Gizmos.DrawLine(position - Vector3.right * 2f, progressEnd);

        // 显示百分比文字（需要Handles）
#if UNITY_EDITOR
        UnityEditor.Handles.Label(position + Vector3.up * 0.5f, $"火势: {progress * 100:F0}%");
#endif
    }
}
