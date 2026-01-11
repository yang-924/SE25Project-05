using UnityEngine;

namespace MissionSystem
{
    /// <summary>
    /// 任务评分系统
    /// 为所有任务类型提供统一的评分机制
    /// </summary>
    public class MissionScoreManager : MonoBehaviour
    {
        public static MissionScoreManager Instance { get; private set; }

        [Header("Current Score")]
        public float totalScore = 100f;  // 总分（满分100）
        public string grade = "S";       // 评级：S, A, B, C, D

        [Header("Score Breakdown")]
        public float timeScore = 0f;         // 时间得分
        public float accuracyScore = 0f;     // 精度得分
        public float speedScore = 0f;        // 速度控制得分
        public float safetyScore = 0f;       // 安全得分（碰撞、坠毁）

        [Header("Mission Stats")]
        public float missionStartTime;
        public float missionDuration;
        public int collisionCount = 0;
        public bool crashed = false;

        // 速度相关统计
        public float targetSpeed = 0f;
        public float totalSpeedDeviation = 0f;
        public int speedSampleCount = 0;

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
        /// 开始任务计分
        /// </summary>
        public void StartMission(MissionType missionType)
        {
            ResetScore();
            missionStartTime = Time.time;
            Debug.Log($"<color=cyan>[ScoreManager]</color> Mission started: {missionType}");
        }

        /// <summary>
        /// 完成任务并计算最终分数
        /// </summary>
        public void CompleteMission(MissionType missionType, float referenceTime = 0f)
        {
            missionDuration = Time.time - missionStartTime;

            // 根据任务类型计算得分
            switch (missionType)
            {
                case MissionType.DisasterDetection:
                    CalculateDisasterScore(referenceTime);
                    break;
                case MissionType.Cargo:
                    CalculateCargoScore(referenceTime);
                    break;
                case MissionType.Waypoint:
                    CalculateWaypointScore(referenceTime);
                    break;
                case MissionType.Recon:
                    CalculateReconScore(referenceTime);
                    break;
                case MissionType.Patrol:
                    CalculatePatrolScore(referenceTime);
                    break;
            }

            // 计算总分和评级
            CalculateFinalScore();

            // 输出详细评分信息
            Debug.Log($"<color=cyan>========== 任务评分报告 ==========</color>");
            Debug.Log($"<color=cyan>[ScoreManager]</color> 任务类型: {missionType}");
            Debug.Log($"<color=cyan>[ScoreManager]</color> 任务时长: {missionDuration:F1}秒 (参考: {referenceTime:F1}秒)");
            Debug.Log($"<color=yellow>[ScoreManager]</color> 时间得分: {timeScore:F1}");
            Debug.Log($"<color=yellow>[ScoreManager]</color> 精度得分: {accuracyScore:F1}");
            Debug.Log($"<color=yellow>[ScoreManager]</color> 速度得分: {speedScore:F1}");
            Debug.Log($"<color=yellow>[ScoreManager]</color> 安全得分: {safetyScore:F1}");
            Debug.Log($"<color=green>[ScoreManager]</color> ===== 总分: {totalScore:F1} / 100 =====");
            Debug.Log($"<color=green>[ScoreManager]</color> ===== 评级: {grade} =====");
            Debug.Log($"<color=cyan>[ScoreManager]</color> 碰撞次数: {collisionCount}");
            Debug.Log($"<color=cyan>===================================</color>");
        }

        /// <summary>
        /// 灾害检测任务评分
        /// </summary>
        private void CalculateDisasterScore(float referenceTime)
        {
            // 时间得分 (40分)：参考时间内完成满分
            timeScore = CalculateTimeScore(referenceTime, 40f);

            // 精度得分 (40分)：拍照距离和角度
            accuracyScore = 40f; // 由DisasterDetectMission设置

            // 安全得分 (20分)
            safetyScore = CalculateSafetyScore(20f);
        }

        /// <summary>
        /// 货物运输任务评分
        /// </summary>
        private void CalculateCargoScore(float referenceTime)
        {
            // 时间得分 (50分)
            timeScore = CalculateTimeScore(referenceTime, 50f);

            // 安全得分 (30分)：不碰撞、平稳降落
            safetyScore = CalculateSafetyScore(30f);

            // 精度得分 (20分)：投放准确度
            accuracyScore = 20f; // 由CargoMission设置
        }

        /// <summary>
        /// 航点巡航任务评分
        /// </summary>
        private void CalculateWaypointScore(float referenceTime)
        {
            // 时间得分 (30分)
            timeScore = CalculateTimeScore(referenceTime, 30f);

            // 速度控制得分 (40分)：定速巡航精度
            speedScore = CalculateSpeedControlScore(40f);

            // 精度得分 (20分)：航点到达精度
            accuracyScore = 20f; // 由WaypointMission设置

            // 安全得分 (10分)
            safetyScore = CalculateSafetyScore(10f);
        }

        /// <summary>
        /// 侦察任务评分
        /// </summary>
        private void CalculateReconScore(float referenceTime)
        {
            // 时间得分 (40分)
            timeScore = CalculateTimeScore(referenceTime, 40f);

            // 精度得分 (40分)：悬停稳定性和位置精度
            accuracyScore = 40f; // 由ReconMission设置

            // 安全得分 (20分)
            safetyScore = CalculateSafetyScore(20f);
        }

        /// <summary>
        /// 巡逻任务评分
        /// </summary>
        private void CalculatePatrolScore(float referenceTime)
        {
            // 时间得分 (40分)
            timeScore = CalculateTimeScore(referenceTime, 40f);

            // 精度得分 (30分)
            accuracyScore = 30f;

            // 安全得分 (30分)
            safetyScore = CalculateSafetyScore(30f);
        }

        /// <summary>
        /// 计算时间得分
        /// </summary>
        private float CalculateTimeScore(float referenceTime, float maxScore)
        {
            if (referenceTime <= 0) return maxScore; // 无参考时间，给满分

            float ratio = referenceTime / missionDuration;

            if (ratio >= 1.0f) return maxScore;                    // 在参考时间内完成
            else if (ratio >= 0.8f) return maxScore * 0.9f;        // 超时20%以内
            else if (ratio >= 0.6f) return maxScore * 0.7f;        // 超时40%以内
            else if (ratio >= 0.4f) return maxScore * 0.5f;        // 超时60%以内
            else return maxScore * 0.3f;                           // 超时严重
        }

        /// <summary>
        /// 计算安全得分
        /// </summary>
        private float CalculateSafetyScore(float maxScore)
        {
            if (crashed) return 0f;

            float score = maxScore;
            score -= collisionCount * 5f; // 每次碰撞扣5分
            return Mathf.Max(0f, score);
        }

        /// <summary>
        /// 计算速度控制得分
        /// </summary>
        private float CalculateSpeedControlScore(float maxScore)
        {
            if (targetSpeed <= 0 || speedSampleCount == 0) return maxScore;

            // 平均速度偏差
            float avgDeviation = totalSpeedDeviation / speedSampleCount;
            float deviationPercent = avgDeviation / targetSpeed;

            if (deviationPercent <= 0.1f) return maxScore;              // 偏差10%以内满分
            else if (deviationPercent <= 0.2f) return maxScore * 0.85f; // 偏差20%以内
            else if (deviationPercent <= 0.3f) return maxScore * 0.7f;  // 偏差30%以内
            else if (deviationPercent <= 0.5f) return maxScore * 0.5f;  // 偏差50%以内
            else return maxScore * 0.3f;                                // 偏差严重
        }

        /// <summary>
        /// 计算最终总分和评级
        /// </summary>
        private void CalculateFinalScore()
        {
            totalScore = timeScore + accuracyScore + speedScore + safetyScore;
            totalScore = Mathf.Clamp(totalScore, 0f, 100f);

            // 评级
            if (totalScore >= 90f) grade = "S";
            else if (totalScore >= 80f) grade = "A";
            else if (totalScore >= 70f) grade = "B";
            else if (totalScore >= 60f) grade = "C";
            else grade = "D";
        }

        /// <summary>
        /// 记录碰撞
        /// </summary>
        public void RecordCollision()
        {
            collisionCount++;
            Debug.Log($"<color=yellow>[ScoreManager]</color> Collision recorded. Total: {collisionCount}");
        }

        /// <summary>
        /// 记录坠毁
        /// </summary>
        public void RecordCrash()
        {
            crashed = true;
            Debug.Log($"<color=red>[ScoreManager]</color> Crash recorded!");
        }

        /// <summary>
        /// 记录速度偏差（用于定速巡航）
        /// </summary>
        public void RecordSpeedDeviation(float currentSpeed, float targetSpeed)
        {
            if (targetSpeed > 0)
            {
                this.targetSpeed = targetSpeed;
                float deviation = Mathf.Abs(currentSpeed - targetSpeed);
                totalSpeedDeviation += deviation;
                speedSampleCount++;
            }
        }

        /// <summary>
        /// 设置精度得分（由各个任务调用）
        /// </summary>
        public void SetAccuracyScore(float score)
        {
            accuracyScore = Mathf.Clamp(score, 0f, 100f);
        }

        /// <summary>
        /// 重置分数
        /// </summary>
        private void ResetScore()
        {
            totalScore = 100f;
            grade = "S";
            timeScore = 0f;
            accuracyScore = 0f;
            speedScore = 0f;
            safetyScore = 0f;
            collisionCount = 0;
            crashed = false;
            targetSpeed = 0f;
            totalSpeedDeviation = 0f;
            speedSampleCount = 0;
        }

        /// <summary>
        /// 获取评分详情文本
        /// </summary>
        public string GetScoreDetails()
        {
            return $"Time: {timeScore:F1}\n" +
                   $"Accuracy: {accuracyScore:F1}\n" +
                   $"Speed Control: {speedScore:F1}\n" +
                   $"Safety: {safetyScore:F1}\n" +
                   $"-------------------\n" +
                   $"Total: {totalScore:F1}\n" +
                   $"Grade: {grade}";
        }
    }
}
