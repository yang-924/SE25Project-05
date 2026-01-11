using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering; // 用于控制 Volume
using UnityEngine.VFX;       // 引入 VFX Graph 命名空间

public class WeatherManager : MonoBehaviour
{
    public static WeatherManager Instance { get; private set; }

    [Header("Wind Physics (Simulation)")]
    [Tooltip("10米高度处的参考风速 (m/s)")]
    public float baseWindSpeed = 5.0f;

    [Tooltip("风向 (0-360度)")]
    [Range(0, 360)]
    public float windDirection = 0f;

    [Tooltip("风切变指数 (0.14:空旷, 0.3:城市, 0.4:密集建筑)")]
    public float windShearExponent = 0.3f;

    [Tooltip("阵风强度系数")]
    public float gustIntensity = 1.0f;

    [Tooltip("阵风变化频率")]
    public float gustFrequency = 0.5f;

    [Header("Rain Physics")]
    [Tooltip("雨量强度 (0-1)")]
    [Range(0, 1)]
    public float rainIntensity = 0.0f;

    [Header("Rain Visual Settings (VFX Graph)")]
    public VisualEffect rainVFX; // 引用 VFX Graph 组件
    public Color rainColor = new Color(0.8f, 0.8f, 0.8f, 0.5f);

    [Tooltip("视觉风力缩放系数：值越大，雨滴倾斜越明显")]
    [Range(0, 2.0f)]
    public float rainVisualWindScale = 0.5f;

    [Tooltip("VFX Graph 中的属性名称")]
    public string vfxSpawnRateProperty = "SpawnRate";
    public string vfxWindProperty = "WindVelocity";
    public string vfxColorProperty = "RainColor";

    [Header("Fog Physics")]
    [Tooltip("雾浓度 (0-1)")]
    [Range(0, 1)]
    public float fogDensity = 0.0f;

    [Header("Fog Visual Settings (VFX Graph)")]
    public VisualEffect fogVFX; // 引用雾的 VFX Graph
    public Color fogColor = new Color(0.7f, 0.7f, 0.7f, 0.1f);

    [Tooltip("VFX Graph 中的属性名称")]
    public string vfxFogDensityProperty = "FogDensity";
    public string vfxFogColorProperty = "FogColor";

    [Header("Legacy Visual References")]
    public ParticleSystem rainParticleSystem; // 旧版粒子系统(可选)
    public WindZone treeWindZone;             // 拖入场景中的 WindZone (影响树木)
    public Volume globalVolume;               // 拖入场景中的 Global Volume (HDRP)

    private void Awake()
    {
        // 单例 + 跨场景持久化（类似 MissionManager）
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject); // 防止场景切换时销毁
            Debug.Log("<color=green>[WeatherManager]</color> 已启动并设为跨场景持久化");
        }
        else
        {
            Debug.LogWarning("<color=yellow>[WeatherManager]</color> 检测到重复实例，销毁旧的");
            Destroy(gameObject);
        }
    }

    private void Start()
    {
        // 验证引用
        if (rainVFX != null)
        {
            Debug.Log($"<color=cyan>[WeatherManager]</color> RainVFX已连接: {rainVFX.name}");

            // 检查RainVFX是否是WeatherManager的子对象
            if (rainVFX.transform.parent != transform)
            {
                Debug.LogWarning($"<color=yellow>[WeatherManager]</color> RainVFX不是WeatherManager的子对象！" +
                               "\n这会导致跨场景引用丢失。建议操作：" +
                               "\n1. 在Hierarchy中将RainVFX拖动到WeatherManager下作为子对象" +
                               "\n2. 或者让RainVFX在代码中动态创建");
            }
        }
        else
        {
            Debug.LogError("<color=red>[WeatherManager]</color> RainVFX引用为空！" +
                          "\n可能原因：" +
                          "\n1. RainVFX不是WeatherManager的子对象，场景切换时被销毁" +
                          "\n2. VisualEffect组件未正确赋值" +
                          "\n\n解决方案：将RainVFX GameObject设置为WeatherManager的子对象");
        }

        if (fogVFX != null)
        {
            Debug.Log($"<color=cyan>[WeatherManager]</color> FogVFX已连接: {fogVFX.name}");
        }
    }

    private void Update()
    {
        UpdateVisuals();
    }

    /// <summary>
    /// 获取特定位置的风速向量 (包含风切变和阵风模型)
    /// </summary>
    public Vector3 GetWindAtPosition(Vector3 position)
    {
        // 1. 风切变模型 (Wind Shear)
        float h = Mathf.Max(position.y, 0.1f);
        float h_ref = 10.0f;
        float shearFactor = Mathf.Pow(h / h_ref, windShearExponent);

        // 2. 基础风向向量
        Vector3 direction = Quaternion.Euler(0, windDirection, 0) * Vector3.forward;

        // 3. 阵风模型 (Gusts - Perlin Noise)
        float timeInput = Time.time * gustFrequency;
        float posInput = (position.x + position.z) * 0.1f;
        float noise = Mathf.PerlinNoise(timeInput, posInput);
        float gustFactor = 1.0f + (noise - 0.5f) * 2.0f * gustIntensity;

        return direction * baseWindSpeed * shearFactor * gustFactor;
    }

    void UpdateVisuals()
    {
        // --- VFX Graph Control ---
        if (rainVFX != null)
        {
            // 1. 控制生成速率 (Spawn Rate)
            // VFX Graph 可以轻松处理数万个粒子，所以这里乘数可以很大
            rainVFX.SetFloat(vfxSpawnRateProperty, rainIntensity * 10000f);

            // 2. 控制风力 (Wind Velocity)
            // 我们使用基础风速，因为雨滴通常在高空，受地面阵风影响较小，或者你可以直接调用 GetWindAtPosition
            Vector3 windDir = Quaternion.Euler(0, windDirection, 0) * Vector3.forward;
            rainVFX.SetVector3(vfxWindProperty, windDir * baseWindSpeed * rainVisualWindScale);

            // 3. 控制颜色
            rainVFX.SetVector4(vfxColorProperty, rainColor);
        }

        // --- Legacy Particle System (如果还在用) ---
        if (rainParticleSystem != null)
        {
            var emission = rainParticleSystem.emission;
            emission.rateOverTime = rainIntensity * 2000f;
        }

        // --- Tree Wind Zone ---
        if (treeWindZone != null)
        {
            treeWindZone.windMain = baseWindSpeed * 0.1f;
            treeWindZone.windTurbulence = gustIntensity;
            treeWindZone.windPulseMagnitude = gustIntensity * 0.5f;
            treeWindZone.transform.rotation = Quaternion.Euler(0, windDirection, 0);
        }

        // --- Fog ---
        if (fogDensity > 0)
        {
            // 1. 基础全局雾 (RenderSettings) - 保证远处看不见
            RenderSettings.fog = true;
            RenderSettings.fogDensity = fogDensity * 0.05f;

            // 2. 局部体积雾 (VFX Graph) - 增加流动感和层次感
            if (fogVFX != null)
            {
                // 雾的生成率和透明度随密度变化
                fogVFX.SetFloat(vfxFogDensityProperty, fogDensity * 5000f);
                fogVFX.SetVector4(vfxFogColorProperty, fogColor);

                // 让雾也随风飘动 (复用雨的风力属性，或者新建一个)
                Vector3 windDir = Quaternion.Euler(0, windDirection, 0) * Vector3.forward;
                fogVFX.SetVector3(vfxWindProperty, windDir * baseWindSpeed * 0.5f); // 雾飘得慢一点
            }
        }
        else
        {
            RenderSettings.fog = false;
            if (fogVFX != null)
            {
                fogVFX.SetFloat(vfxFogDensityProperty, 0);
            }
        }
    }
}
