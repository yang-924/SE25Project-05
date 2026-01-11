using UnityEngine;

/// <summary>
/// 坐标转换工具：Unity世界坐标 ↔ 经纬度+海拔
/// </summary>
public class CoordinateConverter : MonoBehaviour
{
    [Header("参考点设置（世界原点对应的真实经纬度）")]
    [Tooltip("世界原点(0,0,0)对应的真实纬度")]
    public double referenceLatitude = 39.9042;  // 示例：北京天安门
    [Tooltip("世界原点(0,0,0)对应的真实经度")]
    public double referenceLongitude = 116.4074;
    [Tooltip("世界原点(0,0,0)对应的真实海拔(米)")]
    public float referenceAltitude = 0f;

    [Header("比例尺设置")]
    [Tooltip("Unity中1单位对应实际多少米")]
    public float unityToMeterScale = 1f;  // 默认1:1

    // 单例
    public static CoordinateConverter Instance { get; private set; }

    // 地球相关常量
    private const double EARTH_RADIUS_KM = 6371.0;  // 地球平均半径(km)
    private const double METERS_PER_DEGREE_LAT = 111320.0;  // 纬度1度≈111.32km

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    /// <summary>
    /// Unity世界坐标 → 经纬度+海拔
    /// </summary>
    /// <param name="worldPosition">Unity世界坐标</param>
    /// <returns>(纬度, 经度, 海拔)</returns>
    public (double latitude, double longitude, float altitude) WorldToGPS(Vector3 worldPosition)
    {
        // 转换为实际米数
        float eastMeters = worldPosition.x * unityToMeterScale;  // 东向(X)
        float northMeters = worldPosition.z * unityToMeterScale; // 北向(Z)
        float altitudeMeters = worldPosition.y * unityToMeterScale + referenceAltitude; // 海拔(Y)

        // 计算纬度偏移（纬度1度≈111.32km）
        double latOffset = northMeters / METERS_PER_DEGREE_LAT;
        double latitude = referenceLatitude + latOffset;

        // 计算经度偏移（经度1度≈111.32km * cos(纬度)）
        double metersPerDegreeLon = METERS_PER_DEGREE_LAT * System.Math.Cos(referenceLatitude * System.Math.PI / 180.0);
        double lonOffset = eastMeters / metersPerDegreeLon;
        double longitude = referenceLongitude + lonOffset;

        return (latitude, longitude, altitudeMeters);
    }

    /// <summary>
    /// 经纬度+海拔 → Unity世界坐标
    /// </summary>
    public Vector3 GPSToWorld(double latitude, double longitude, float altitude)
    {
        // 计算纬度差转米
        double latDiff = latitude - referenceLatitude;
        float northMeters = (float)(latDiff * METERS_PER_DEGREE_LAT);

        // 计算经度差转米
        double lonDiff = longitude - referenceLongitude;
        double metersPerDegreeLon = METERS_PER_DEGREE_LAT * System.Math.Cos(referenceLatitude * System.Math.PI / 180.0);
        float eastMeters = (float)(lonDiff * metersPerDegreeLon);

        // 海拔转高度
        float heightMeters = altitude - referenceAltitude;

        // 转换为Unity单位
        float x = eastMeters / unityToMeterScale;
        float y = heightMeters / unityToMeterScale;
        float z = northMeters / unityToMeterScale;

        return new Vector3(x, y, z);
    }

    /// <summary>
    /// 格式化经纬度字符串（度分秒格式）
    /// </summary>
    public static string FormatGPS(double latitude, double longitude, float altitude)
    {
        string latDir = latitude >= 0 ? "N" : "S";
        string lonDir = longitude >= 0 ? "E" : "W";

        latitude = System.Math.Abs(latitude);
        longitude = System.Math.Abs(longitude);

        // 转换为度分秒
        int latDeg = (int)latitude;
        double latMinDecimal = (latitude - latDeg) * 60;
        int latMin = (int)latMinDecimal;
        double latSec = (latMinDecimal - latMin) * 60;

        int lonDeg = (int)longitude;
        double lonMinDecimal = (longitude - lonDeg) * 60;
        int lonMin = (int)lonMinDecimal;
        double lonSec = (lonMinDecimal - lonMin) * 60;

        return $"{latDeg}°{latMin}'{latSec:F1}\"{latDir}, {lonDeg}°{lonMin}'{lonSec:F1}\"{lonDir}, Alt: {altitude:F1}m";
    }

    /// <summary>
    /// 格式化经纬度字符串（简洁小数格式）- 经纬度保留4位小数
    /// </summary>
    public static string FormatGPSSimple(double latitude, double longitude, float altitude)
    {
        return $"{latitude:F4}°, {longitude:F4}°, {altitude:F1}m";
    }

    /// <summary>
    /// 计算两个GPS坐标之间的距离(米) - 仅水平距离，不考虑海拔
    /// </summary>
    public static float DistanceBetweenGPS(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = (lat2 - lat1) * System.Math.PI / 180.0;
        double dLon = (lon2 - lon1) * System.Math.PI / 180.0;

        double a = System.Math.Sin(dLat / 2) * System.Math.Sin(dLat / 2) +
                   System.Math.Cos(lat1 * System.Math.PI / 180.0) * System.Math.Cos(lat2 * System.Math.PI / 180.0) *
                   System.Math.Sin(dLon / 2) * System.Math.Sin(dLon / 2);

        double c = 2 * System.Math.Atan2(System.Math.Sqrt(a), System.Math.Sqrt(1 - a));
        double distance = EARTH_RADIUS_KM * c * 1000; // 转为米

        return (float)distance;
    }

    /// <summary>
    /// 计算两个Unity世界坐标的水平距离（仅XZ平面，忽略Y轴高度）
    /// </summary>
    public static float HorizontalDistance(Vector3 pos1, Vector3 pos2)
    {
        Vector2 pos1_2D = new Vector2(pos1.x, pos1.z);
        Vector2 pos2_2D = new Vector2(pos2.x, pos2.z);
        return Vector2.Distance(pos1_2D, pos2_2D);
    }

    /// <summary>
    /// 计算两个位置的高度差（绝对值）
    /// </summary>
    public static float AltitudeDifference(Vector3 pos1, Vector3 pos2)
    {
        return Mathf.Abs(pos1.y - pos2.y);
    }
}
