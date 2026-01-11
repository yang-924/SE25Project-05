using UnityEngine;
using System.Collections.Generic;

namespace MissionSystem
{
    /// <summary>
    /// 航点任务：按顺序飞过一系列坐标点
    /// </summary>
    public class WaypointMission : MissionBase
    {
        [Header("Configuration")]
        public Transform playerTransform; // 无人机Transform
        public List<Transform> waypoints; // 航点列表
        public float reachThreshold = 5.0f; // 到达判定距离（米）

        [Header("Runtime Info")]
        public int currentWaypointIndex = 0;

        public override void BeginMission()
        {
            Debug.Log($"<color=cyan>[WaypointMission]</color> BeginMission called");

            if (playerTransform == null)
            {
                // 尝试自动查找标签为 Player 的物体，或者直接找 DroneController
                GameObject drone = GameObject.FindGameObjectWithTag("Player");
                if (drone != null) playerTransform = drone.transform;
                else
                {
                    // 尝试找 DroneController
                    var controller = FindObjectOfType<DroneController>();
                    if (controller != null) playerTransform = controller.transform;
                }
            }

            if (playerTransform == null)
            {
                Debug.LogError($"<color=red>[WaypointMission]</color> Failed to find player/drone!");
                FailMission("No Player/Drone assigned!");
                return;
            }

            if (waypoints == null || waypoints.Count == 0)
            {
                Debug.LogError($"<color=red>[WaypointMission]</color> No waypoints configured!");
                FailMission("No waypoints defined!");
                return;
            }

            currentWaypointIndex = 0;

            // 启动评分系统
            if (MissionScoreManager.Instance != null)
            {
                MissionScoreManager.Instance.StartMission(MissionType.Waypoint);
                Debug.Log($"<color=green>[WaypointMission]</color> Score tracking started");
            }
            else
            {
                Debug.LogWarning($"<color=yellow>[WaypointMission]</color> MissionScoreManager not found!");
            }

            base.BeginMission();
            UpdateObjectiveText();
            Debug.Log($"<color=green>[WaypointMission]</color> Mission started with {waypoints.Count} waypoints");
        }

        public override void OnUpdate()
        {
            if (currentWaypointIndex >= waypoints.Count)
            {
                Debug.Log($"<color=green>[WaypointMission]</color> All waypoints reached!");

                // 计算评分
                if (MissionScoreManager.Instance != null)
                {
                    // 参考时间 = 航点数 * 20秒
                    float referenceTime = waypoints.Count * 20f;
                    MissionScoreManager.Instance.CompleteMission(MissionType.Waypoint, referenceTime);
                    Debug.Log($"<color=green>[WaypointMission]</color> Score calculated. Grade: {MissionScoreManager.Instance.grade}");
                }

                CompleteMission();
                return;
            }

            Transform target = waypoints[currentWaypointIndex];
            if (target == null) return;

            float distance = Vector3.Distance(playerTransform.position, target.position);

            // 检查是否到达
            if (distance <= reachThreshold)
            {
                Debug.Log($"[WaypointMission] Reached waypoint {currentWaypointIndex + 1}/{waypoints.Count} (distance: {distance:F1}m)");
                currentWaypointIndex++;

                if (currentWaypointIndex >= waypoints.Count)
                {
                    Debug.Log($"<color=green>[WaypointMission]</color> All waypoints completed!");

                    // 计算精度得分（基于到达精度）
                    float accuracyScore = 20f; // 默认满分
                    if (MissionScoreManager.Instance != null)
                    {
                        MissionScoreManager.Instance.SetAccuracyScore(accuracyScore);
                    }

                    // 计算总分
                    if (MissionScoreManager.Instance != null)
                    {
                        float referenceTime = waypoints.Count * 20f;
                        MissionScoreManager.Instance.CompleteMission(MissionType.Waypoint, referenceTime);
                        Debug.Log($"<color=green>[WaypointMission]</color> Final Score: {MissionScoreManager.Instance.totalScore:F1}, Grade: {MissionScoreManager.Instance.grade}");
                    }

                    CompleteMission();
                }
                else
                {
                    UpdateObjectiveText();
                }
            }
        }

        private void UpdateObjectiveText()
        {
            if (currentWaypointIndex < waypoints.Count)
            {
                NotifyObjective($"Go to Waypoint {currentWaypointIndex + 1}");
            }
        }

        // 在编辑器中绘制航线
        private void OnDrawGizmos()
        {
            if (waypoints == null || waypoints.Count == 0) return;

            Gizmos.color = Color.yellow;
            for (int i = 0; i < waypoints.Count; i++)
            {
                if (waypoints[i] != null)
                {
                    Gizmos.DrawWireSphere(waypoints[i].position, reachThreshold);

                    if (i < waypoints.Count - 1 && waypoints[i + 1] != null)
                    {
                        Gizmos.DrawLine(waypoints[i].position, waypoints[i + 1].position);
                    }
                }
            }
        }
    }
}
