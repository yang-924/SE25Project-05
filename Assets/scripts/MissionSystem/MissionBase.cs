using UnityEngine;
using System;

namespace MissionSystem
{
    public enum MissionStatus
    {
        Idle,
        Running,
        Completed,
        Failed
    }

    /// <summary>
    /// 所有任务的基类
    /// </summary>
    public abstract class MissionBase : MonoBehaviour
    {
        [Header("Mission Info")]
        public string missionName = "New Mission";
        [TextArea]
        public string description = "Mission Description";

        public MissionStatus Status { get; protected set; } = MissionStatus.Idle;

        // 事件：任务状态变更
        public event Action<MissionBase> OnMissionStarted;
        public event Action<MissionBase, bool> OnMissionFinished; // bool success
        public event Action<string> OnObjectiveUpdated; // 进度更新消息

        /// <summary>
        /// 开始任务
        /// </summary>
        public virtual void BeginMission()
        {
            Status = MissionStatus.Running;
            Debug.Log($"[Mission] Started: {missionName}");
            OnMissionStarted?.Invoke(this);
        }

        /// <summary>
        /// 强制停止/失败任务
        /// </summary>
        public virtual void FailMission(string reason)
        {
            Debug.Log($"<color=red>[MissionBase]</color> FailMission called for '{missionName}', Current Status: {Status}");

            if (Status != MissionStatus.Running)
            {
                Debug.LogWarning($"<color=yellow>[MissionBase]</color> Cannot fail mission - not running (Status: {Status})");
                return;
            }

            Status = MissionStatus.Failed;
            Debug.LogWarning($"<color=red>[Mission]</color> Failed: {missionName}. Reason: {reason}");
            Debug.Log($"<color=red>[MissionBase]</color> Invoking OnMissionFinished event (success=false)");

            if (OnMissionFinished != null)
            {
                Debug.Log($"<color=red>[MissionBase]</color> OnMissionFinished has {OnMissionFinished.GetInvocationList().Length} subscriber(s)");
                OnMissionFinished.Invoke(this, false);
            }
            else
            {
                Debug.LogError($"<color=red>[MissionBase]</color> OnMissionFinished is NULL! No subscribers to handle mission failure!");
            }
        }

        /// <summary>
        /// 完成任务
        /// </summary>
        public virtual void CompleteMission()
        {
            Debug.Log($"<color=green>[MissionBase]</color> CompleteMission called for '{missionName}', Current Status: {Status}");

            if (Status != MissionStatus.Running)
            {
                Debug.LogWarning($"<color=yellow>[MissionBase]</color> Cannot complete mission - not running (Status: {Status})");
                return;
            }

            Status = MissionStatus.Completed;
            Debug.Log($"<color=green>[Mission]</color> Completed: {missionName}");
            Debug.Log($"<color=green>[MissionBase]</color> Invoking OnMissionFinished event (success=true)");

            if (OnMissionFinished != null)
            {
                Debug.Log($"<color=green>[MissionBase]</color> OnMissionFinished has {OnMissionFinished.GetInvocationList().Length} subscriber(s)");
                OnMissionFinished.Invoke(this, true);
            }
            else
            {
                Debug.LogError($"<color=red>[MissionBase]</color> OnMissionFinished is NULL! No subscribers to handle mission completion!");
            }
        }

        /// <summary>
        /// 每帧更新逻辑 (由 Manager 调用)
        /// </summary>
        public abstract void OnUpdate();

        protected void NotifyObjective(string message)
        {
            OnObjectiveUpdated?.Invoke(message);
        }
    }
}
