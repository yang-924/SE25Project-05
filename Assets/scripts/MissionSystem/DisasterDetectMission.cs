using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace MissionSystem
{
    /// <summary>
    /// 灾害检测任务（火灾定位）
    /// 功能：在指定区域生成多个火焰点 → 靠近20m内拍照 → 发现所有火点完成任务
    /// </summary>
    public class DisasterDetectMission : MissionBase
    {
        [Header("Fire Points")]
        [Tooltip("所有活动的火焰点")]
        public List<FirePoint> firePoints = new List<FirePoint>();

        [Header("Detection Settings")]
        [Tooltip("检测半径（米）")]
        public float detectionRadius = 20f;
        [Tooltip("是否需要拍照确认")]
        public bool requirePhotoConfirmation = true;
        [Tooltip("拍照按键（已弃用，现使用InputManager统一管理F12键）")]
        [System.Obsolete("Use InputManager.instance.F12Input instead")]
        public KeyCode photoKey = KeyCode.F12;

        [Header("Mission Rules")]
        [Tooltip("是否需要发现所有火点")]
        public bool requireAllDetected = true;
        [Tooltip("最少需要发现的火点数量")]
        public int minimumDetections = 1;

        [Header("Debug")]
        public Color undetectedColor = Color.red;
        public Color detectedColor = Color.green;
        public float gizmoRadius = 2f;

        private Transform droneTransform;
        private int totalFiresDetected = 0;
        private float lastStatusLogTime = 0f;

        /// <summary>
        /// 火焰点数据类
        /// </summary>
        [System.Serializable]
        public class FirePoint
        {
            public GameObject fireObject;      // 火焰GameObject
            public Vector3 position;            // 位置
            public bool isDetected = false;     // 是否已发现
            public float detectionTime;         // 发现时间

            public FirePoint(GameObject obj, Vector3 pos)
            {
                fireObject = obj;
                position = pos;
            }
        }

        public override void BeginMission()
        {
            base.BeginMission();

            if (firePoints == null || firePoints.Count == 0)
            {
                Debug.LogError("<color=red>[DisasterDetectMission]</color> 没有火焰点！任务无法开始");
                FailMission("No fire points configured");
                return;
            }

            // 启动评分系统
            if (MissionScoreManager.Instance != null)
            {
                MissionScoreManager.Instance.StartMission(MissionType.DisasterDetection);
                Debug.Log($"<color=green>[DisasterDetectMission]</color> Score tracking started");
            }

            Debug.Log($"<color=orange>[DisasterDetectMission]</color> ===== 灾害检测任务开始 =====");
            Debug.Log($"<color=orange>[DisasterDetectMission]</color> 总火点数: {firePoints.Count}");
            Debug.Log($"<color=orange>[DisasterDetectMission]</color> 检测半径: {detectionRadius}m");
            Debug.Log($"<color=orange>[DisasterDetectMission]</color> 需要拍照: {requirePhotoConfirmation}");
            Debug.Log($"<color=yellow>[DisasterDetectMission]</color> 等待无人机生成...");

            NotifyObjective($"发现所有火灾点 (0/{firePoints.Count})");
        }

        public override void OnUpdate()
        {
            if (Status != MissionStatus.Running) return;

            // 查找无人机
            if (droneTransform == null)
            {
                GameObject drone = GameObject.FindGameObjectWithTag("Player");
                if (drone != null)
                {
                    droneTransform = drone.transform;
                    Debug.Log($"<color=green>[DisasterDetectMission]</color> 找到无人机: {drone.name}");
                    Debug.Log($"<color=green>[DisasterDetectMission]</color> 开始搜索火灾点，在{detectionRadius}m内按{photoKey}键（F12）拍照记录");
                    NotifyObjective($"搜索火灾点，靠近{detectionRadius}m内按F12拍照 ({totalFiresDetected}/{firePoints.Count})");
                }
                else
                {
                    return; // 继续等待
                }
            }

            // 检测火焰点
            CheckFireDetection();

            // 定期显示进度
            if (Time.time - lastStatusLogTime >= 5f)
            {
                lastStatusLogTime = Time.time;
                LogMissionStatus();
            }
        }

        private void CheckFireDetection()
        {
            if (droneTransform == null) return;

            Vector3 dronePos = droneTransform.position;

            // 遍历所有未发现的火点
            foreach (FirePoint firePoint in firePoints)
            {
                if (firePoint.isDetected) continue;

                float distance = Vector3.Distance(dronePos, firePoint.position);

                // 检查是否在检测范围内
                if (distance <= detectionRadius)
                {
                    // 如果需要拍照确认（使用 InputManager 统一输入系统）
                    if (requirePhotoConfirmation)
                    {
                        if (InputManager.instance != null && InputManager.instance.F12Input)
                        {
                            ConfirmFireDetection(firePoint, distance);
                        }
                    }
                    else
                    {
                        // 不需要拍照，直接确认
                        ConfirmFireDetection(firePoint, distance);
                    }
                }
            }
        }

        private void ConfirmFireDetection(FirePoint firePoint, float distance)
        {
            firePoint.isDetected = true;
            firePoint.detectionTime = Time.time;
            totalFiresDetected++;

            Debug.Log($"<color=green>[DisasterDetectMission]</color> ===== 发现火点 =====");
            Debug.Log($"<color=green>[DisasterDetectMission]</color> 位置: {firePoint.position}");
            Debug.Log($"<color=green>[DisasterDetectMission]</color> 距离: {distance:F1}m");
            Debug.Log($"<color=green>[DisasterDetectMission]</color> 进度: {totalFiresDetected}/{firePoints.Count}");

            NotifyObjective($"发现火点！({totalFiresDetected}/{firePoints.Count})");

            // 检查任务是否完成
            CheckMissionCompletion();
        }

        private void CheckMissionCompletion()
        {
            bool isComplete = false;

            if (requireAllDetected)
            {
                // 需要发现所有火点
                isComplete = totalFiresDetected >= firePoints.Count;
            }
            else
            {
                // 只需发现最少数量
                isComplete = totalFiresDetected >= minimumDetections;
            }

            if (isComplete)
            {
                Debug.Log($"<color=lime>[DisasterDetectMission]</color> ===== 所有火点已发现！=====");
                Debug.Log($"<color=lime>[DisasterDetectMission]</color> 总发现: {totalFiresDetected}/{firePoints.Count}");
                Debug.Log($"<color=lime>[DisasterDetectMission]</color> 任务完成！");

                // 计算精度得分（根据发现所有火点给满分）
                float accuracyScore = 40f;
                if (totalFiresDetected < firePoints.Count)
                {
                    // 未发现全部，按比例给分
                    accuracyScore = (totalFiresDetected / (float)firePoints.Count) * 40f;
                }

                // 完成评分
                if (MissionScoreManager.Instance != null)
                {
                    MissionScoreManager.Instance.SetAccuracyScore(accuracyScore);
                    // 参考时间 = 火点数 * 30秒
                    float referenceTime = firePoints.Count * 30f;
                    MissionScoreManager.Instance.CompleteMission(MissionType.DisasterDetection, referenceTime);
                    Debug.Log($"<color=green>[DisasterDetectMission]</color> Final Score: {MissionScoreManager.Instance.totalScore:F1}, Grade: {MissionScoreManager.Instance.grade}");
                }

                NotifyObjective($"任务完成！发现{totalFiresDetected}个火点");
                CompleteMission();
            }
        }

        private void LogMissionStatus()
        {
            int undetectedCount = firePoints.Count - totalFiresDetected;

            Debug.Log($"<color=yellow>[DisasterDetectMission]</color> ===== 任务进度 =====");
            Debug.Log($"<color=yellow>[DisasterDetectMission]</color> 已发现: <color=green>{totalFiresDetected}</color> | 未发现: <color=red>{undetectedCount}</color>");

            // 显示最近的未发现火点
            if (droneTransform != null)
            {
                var undetectedFires = firePoints.Where(f => !f.isDetected).ToList();
                if (undetectedFires.Count > 0)
                {
                    var nearestFire = undetectedFires
                        .OrderBy(f => Vector3.Distance(droneTransform.position, f.position))
                        .First();

                    float distance = Vector3.Distance(droneTransform.position, nearestFire.position);
                    Debug.Log($"<color=yellow>[DisasterDetectMission]</color> 最近火点距离: {distance:F1}m ({(distance <= detectionRadius ? "可拍照" : "太远")})");
                }
            }
        }

        /// <summary>
        /// 获取任务统计信息（供UI调用）
        /// </summary>
        public string GetProgressText()
        {
            return $"{totalFiresDetected}/{firePoints.Count}";
        }

        /// <summary>
        /// 获取最近的未发现火点距离（供UI调用）
        /// </summary>
        public float GetNearestUndetectedFireDistance()
        {
            if (droneTransform == null) return float.MaxValue;

            var undetectedFires = firePoints.Where(f => !f.isDetected).ToList();
            if (undetectedFires.Count == 0) return 0f;

            return undetectedFires
                .Min(f => Vector3.Distance(droneTransform.position, f.position));
        }

        /// <summary>
        /// 该任务不显示单一目标位置
        /// </summary>
        public bool ShouldShowTargetLocation()
        {
            return false;
        }

        private void OnDrawGizmos()
        {
            if (firePoints == null) return;

            foreach (FirePoint firePoint in firePoints)
            {
                // 根据检测状态改变颜色
                Gizmos.color = firePoint.isDetected ? detectedColor : undetectedColor;

                // 画火点位置
                Gizmos.DrawWireSphere(firePoint.position, gizmoRadius);
                Gizmos.DrawLine(firePoint.position, firePoint.position + Vector3.up * 10f);

                // 画检测范围
                Gizmos.color = new Color(Gizmos.color.r, Gizmos.color.g, Gizmos.color.b, 0.2f);
                DrawWireCircle(firePoint.position, detectionRadius, 24);
            }
        }

        private void DrawWireCircle(Vector3 center, float radius, int segments)
        {
            float angleStep = 360f / segments;

            for (int i = 0; i < segments; i++)
            {
                float angle1 = i * angleStep * Mathf.Deg2Rad;
                float angle2 = (i + 1) * angleStep * Mathf.Deg2Rad;

                Vector3 p1 = center + new Vector3(Mathf.Cos(angle1) * radius, 0, Mathf.Sin(angle1) * radius);
                Vector3 p2 = center + new Vector3(Mathf.Cos(angle2) * radius, 0, Mathf.Sin(angle2) * radius);

                Gizmos.DrawLine(p1, p2);
            }
        }
    }
}
