using UnityEngine;
using System.Collections.Generic;

namespace MissionSystem
{
    /// <summary>
    /// 侦察任务：到达指定地点并悬停观察
    /// 要求：位置精度、悬停稳定性
    /// </summary>
    public class ReconMission : MissionBase
    {
        [Header("Configuration")]
        public Transform playerTransform;
        public List<Transform> reconPoints; // 侦察点列表
        public float reachThreshold = 10f;   // 到达判定距离
        public float hoverTime = 3f;         // 需要悬停的时间（秒）
        public float hoverStabilityThreshold = 2f; // 悬停稳定性阈值（m）

        [Header("Runtime Info")]
        public int currentPointIndex = 0;
        private float hoverStartTime;
        private bool isHovering = false;
        private Vector3 hoverStartPosition;
        private float maxDeviationDuringHover;

        public override void BeginMission()
        {
            if (playerTransform == null)
            {
                GameObject drone = GameObject.FindGameObjectWithTag("Player");
                if (drone != null) playerTransform = drone.transform;
                else
                {
                    var controller = FindObjectOfType<DroneController>();
                    if (controller != null) playerTransform = controller.transform;
                }
            }

            if (playerTransform == null)
            {
                FailMission("No Player/Drone assigned!");
                return;
            }

            if (reconPoints == null || reconPoints.Count == 0)
            {
                FailMission("No recon points defined!");
                return;
            }

            currentPointIndex = 0;
            isHovering = false;

            // 启动评分系统
            if (MissionScoreManager.Instance != null)
            {
                MissionScoreManager.Instance.StartMission(MissionType.Recon);
            }

            base.BeginMission();
            UpdateObjectiveText();
        }

        public override void OnUpdate()
        {
            if (currentPointIndex >= reconPoints.Count)
            {
                // 计算精度得分
                CalculateAccuracyScore();

                // 完成评分
                if (MissionScoreManager.Instance != null)
                {
                    MissionScoreManager.Instance.CompleteMission(MissionType.Recon, reconPoints.Count * (hoverTime + 10f));
                }

                CompleteMission();
                return;
            }

            Transform target = reconPoints[currentPointIndex];
            if (target == null) return;

            float distance = Vector3.Distance(playerTransform.position, target.position);

            if (!isHovering)
            {
                // 检查是否到达侦察点
                if (distance <= reachThreshold)
                {
                    StartHovering();
                }
            }
            else
            {
                // 检查悬停稳定性
                float currentDeviation = Vector3.Distance(playerTransform.position, hoverStartPosition);
                maxDeviationDuringHover = Mathf.Max(maxDeviationDuringHover, currentDeviation);

                if (currentDeviation > hoverStabilityThreshold)
                {
                    // 漂移过大，重新开始悬停
                    Debug.LogWarning($"[ReconMission] Hover position deviated too much ({currentDeviation:F1}m), restarting hover timer");
                    StartHovering();
                }
                else if (Time.time - hoverStartTime >= hoverTime)
                {
                    // 悬停成功
                    Debug.Log($"[ReconMission] Completed recon point {currentPointIndex + 1}/{reconPoints.Count}");
                    currentPointIndex++;
                    isHovering = false;

                    if (currentPointIndex < reconPoints.Count)
                    {
                        UpdateObjectiveText();
                    }
                }
            }
        }

        private void StartHovering()
        {
            isHovering = true;
            hoverStartTime = Time.time;
            hoverStartPosition = playerTransform.position;
            maxDeviationDuringHover = 0f;
            Debug.Log($"[ReconMission] Started hovering at point {currentPointIndex + 1}");
            NotifyObjective($"Hovering at Point {currentPointIndex + 1}... ({hoverTime:F0}s)");
        }

        private void UpdateObjectiveText()
        {
            if (currentPointIndex < reconPoints.Count)
            {
                NotifyObjective($"Go to Recon Point {currentPointIndex + 1} and Hover");
            }
        }

        private void CalculateAccuracyScore()
        {
            // 根据悬停稳定性计算精度得分
            float accuracyScore = 40f;

            if (maxDeviationDuringHover <= 0.5f)
                accuracyScore = 40f;  // 完美悬停
            else if (maxDeviationDuringHover <= 1.0f)
                accuracyScore = 35f;  // 优秀
            else if (maxDeviationDuringHover <= 1.5f)
                accuracyScore = 30f;  // 良好
            else if (maxDeviationDuringHover <= 2.0f)
                accuracyScore = 25f;  // 及格
            else
                accuracyScore = 20f;  // 勉强

            if (MissionScoreManager.Instance != null)
            {
                MissionScoreManager.Instance.SetAccuracyScore(accuracyScore);
            }
        }

        /// <summary>
        /// 获取进度文本（供FlightInfoPanel使用）
        /// </summary>
        public string GetProgressText()
        {
            if (currentPointIndex >= reconPoints.Count)
            {
                return "Recon: Completed";
            }

            if (isHovering)
            {
                float remainingTime = hoverTime - (Time.time - hoverStartTime);
                return $"Recon: {currentPointIndex + 1}/{reconPoints.Count} | Hover: {remainingTime:F1}s";
            }
            else
            {
                Transform target = reconPoints[currentPointIndex];
                if (target != null)
                {
                    float distance = Vector3.Distance(playerTransform.position, target.position);
                    return $"Recon: {currentPointIndex + 1}/{reconPoints.Count} | Dist: {distance:F0}m";
                }
                else
                {
                    return $"Recon: {currentPointIndex + 1}/{reconPoints.Count}";
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (reconPoints == null || reconPoints.Count == 0) return;

            Gizmos.color = Color.cyan;
            for (int i = 0; i < reconPoints.Count; i++)
            {
                if (reconPoints[i] != null)
                {
                    Gizmos.DrawWireSphere(reconPoints[i].position, reachThreshold);
                    Gizmos.DrawWireSphere(reconPoints[i].position, hoverStabilityThreshold);
                }
            }
        }
    }
}
