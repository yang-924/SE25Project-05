using UnityEngine;

/// <summary>
/// 天气对无人机的物理影响（独立组件，不修改 DroneController）
/// 挂载在无人机 GameObject 上即可生效
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class DroneWeatherEffect : MonoBehaviour
{
    [Header("风力影响设置")]
    [Tooltip("启用风力影响")]
    public bool enableWindEffect = true;
    
    [Tooltip("风力影响系数（越大影响越明显）")]
    [Range(0, 2.0f)]
    public float windForceMultiplier = 0.5f;
    
    [Tooltip("风阻系数")]
    [Range(0, 1.0f)]
    public float dragCoefficient = 0.1f;

    [Header("雨力影响设置")]
    [Tooltip("启用雨力影响")]
    public bool enableRainEffect = true;
    
    [Tooltip("雨对升力的影响系数（雨会降低升力）")]
    [Range(0, 1.0f)]
    public float rainLiftReduction = 0.2f;
    
    [Tooltip("雨对阻力的影响系数（雨会增加阻力）")]
    [Range(0, 1.0f)]
    public float rainDragIncrease = 0.3f;
    
    [Tooltip("雨滴冲击力（额外的向下力）")]
    [Range(0, 10.0f)]
    public float rainImpactForce = 2.0f;

    [Header("雾影响设置")]
    [Tooltip("启用雾影响（主要是传感器干扰，物理影响很小）")]
    public bool enableFogEffect = false;
    
    [Tooltip("雾对阻力的影响系数")]
    [Range(0, 0.5f)]
    public float fogDragIncrease = 0.1f;

    [Header("调试信息")]
    public bool showDebugInfo = false;

    private Rigidbody rb;
    private Vector3 currentWindForce;
    private float currentRainForce;
    private float originalDrag;
    private float originalAngularDrag;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        // 保存原始阻力值
        originalDrag = rb.drag;
        originalAngularDrag = rb.angularDrag;
        
        Debug.Log($"<color=cyan>[DroneWeatherEffect]</color> 已启动 - 无人机: {gameObject.name}");
    }

    private void FixedUpdate()
    {
        if (WeatherManager.Instance == null)
        {
            if (showDebugInfo && Time.frameCount % 300 == 0)
            {
                Debug.LogWarning("[DroneWeatherEffect] WeatherManager 未找到，天气效果未生效");
            }
            return;
        }

        // 重置阻力为原始值
        rb.drag = originalDrag;
        rb.angularDrag = originalAngularDrag;

        // 应用风力影响
        if (enableWindEffect)
        {
            ApplyWindEffect();
        }

        // 应用雨力影响
        if (enableRainEffect)
        {
            ApplyRainEffect();
        }

        // 应用雾影响
        if (enableFogEffect)
        {
            ApplyFogEffect();
        }

        // 调试信息
        if (showDebugInfo && Time.frameCount % 60 == 0)
        {
            Debug.Log($"<color=yellow>[DroneWeatherEffect]</color> 风力: {currentWindForce.magnitude:F2}N, 雨力: {currentRainForce:F2}N, Drag: {rb.drag:F3}");
        }
    }

    /// <summary>
    /// 应用风力效果
    /// </summary>
    private void ApplyWindEffect()
    {
        // 获取当前位置的风力向量
        Vector3 windVector = WeatherManager.Instance.GetWindAtPosition(transform.position);
        
        // 计算风力（考虑无人机的速度，相对风速）
        Vector3 relativeWind = windVector - rb.velocity;
        
        // 风阻力 = 0.5 * 空气密度 * 速度² * 阻力系数 * 面积
        // 这里简化为：风力 * 系数
        float windMagnitude = relativeWind.magnitude;
        Vector3 windForce = relativeWind.normalized * (windMagnitude * windMagnitude * dragCoefficient * windForceMultiplier);
        
        // 施加风力
        rb.AddForce(windForce, ForceMode.Force);
        
        // 增加动态阻力（风越大，阻力越大）
        float dynamicDrag = windMagnitude * dragCoefficient * 0.01f;
        rb.drag += dynamicDrag;
        
        currentWindForce = windForce;
    }

    /// <summary>
    /// 应用雨力效果
    /// </summary>
    private void ApplyRainEffect()
    {
        float rainIntensity = WeatherManager.Instance.rainIntensity;
        
        if (rainIntensity <= 0) 
        {
            currentRainForce = 0;
            return;
        }

        // 1. 雨滴冲击力（向下的额外力）
        Vector3 rainImpact = Vector3.down * rainImpactForce * rainIntensity;
        rb.AddForce(rainImpact, ForceMode.Force);
        
        // 2. 增加阻力（雨天飞行更费力）
        rb.drag += rainDragIncrease * rainIntensity;
        rb.angularDrag += rainDragIncrease * rainIntensity * 0.5f;
        
        // 3. 降低升力效果（通过增加向下的力来模拟）
        // 注意：这个效果应该在 DroneController 的升力计算中体现
        // 但由于我们不修改 DroneController，这里用额外的力来模拟
        float liftPenalty = rb.velocity.magnitude * rainLiftReduction * rainIntensity;
        rb.AddForce(Vector3.down * liftPenalty, ForceMode.Force);
        
        currentRainForce = rainImpact.magnitude + liftPenalty;
    }

    /// <summary>
    /// 应用雾影响效果
    /// </summary>
    private void ApplyFogEffect()
    {
        float fogDensity = WeatherManager.Instance.fogDensity;
        
        if (fogDensity <= 0) return;

        // 雾增加轻微阻力（空气湿度增加）
        rb.drag += fogDragIncrease * fogDensity;
    }

    /// <summary>
    /// 获取当前风力大小（供 UI 显示）
    /// </summary>
    public float GetCurrentWindMagnitude()
    {
        return currentWindForce.magnitude;
    }

    /// <summary>
    /// 获取当前雨力大小（供 UI 显示）
    /// </summary>
    public float GetCurrentRainForce()
    {
        return currentRainForce;
    }

    /// <summary>
    /// 实时调整风力影响（可以从 UI 调用）
    /// </summary>
    public void SetWindEffectMultiplier(float value)
    {
        windForceMultiplier = Mathf.Clamp(value, 0, 2.0f);
    }

    /// <summary>
    /// 实时调整雨力影响（可以从 UI 调用）
    /// </summary>
    public void SetRainEffectMultiplier(float liftReduction, float dragIncrease, float impactForce)
    {
        rainLiftReduction = Mathf.Clamp01(liftReduction);
        rainDragIncrease = Mathf.Clamp01(dragIncrease);
        rainImpactForce = Mathf.Clamp(impactForce, 0, 10f);
    }

    private void OnDrawGizmos()
    {
        if (!showDebugInfo || !Application.isPlaying) return;

        // 绘制风力向量
        if (enableWindEffect && currentWindForce.magnitude > 0.1f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, transform.position + currentWindForce * 0.1f);
        }
    }
}
