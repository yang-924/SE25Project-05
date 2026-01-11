using UnityEngine;
using TMPro;
using MissionSystem;

/// <summary>
/// 任务成功页面评分显示
/// 显示任务完成后的评分详情
/// </summary>
public class MissionScoreDisplay : MonoBehaviour
{
    [Header("Score UI Elements")]
    public TextMeshProUGUI gradeText;         // 评级显示（S/A/B/C/D）
    public TextMeshProUGUI totalScoreText;    // 总分显示
    public TextMeshProUGUI timeScoreText;     // 时间得分
    public TextMeshProUGUI accuracyScoreText; // 精度得分
    public TextMeshProUGUI speedScoreText;    // 速度得分
    public TextMeshProUGUI safetyScoreText;   // 安全得分
    public TextMeshProUGUI detailsText;       // 详细信息

    [Header("Optional")]
    public GameObject scorePanel;             // 评分面板（可选）

    private void Start()
    {
        DisplayScore();
    }

    /// <summary>
    /// 显示评分
    /// </summary>
    public void DisplayScore()
    {
        if (MissionScoreManager.Instance == null)
        {
            Debug.LogWarning("<color=yellow>[MissionScoreDisplay]</color> ScoreManager not found!");
            if (scorePanel != null) scorePanel.SetActive(false);
            return;
        }

        var scoreManager = MissionScoreManager.Instance;

        // 显示评级
        if (gradeText != null)
        {
            gradeText.text = scoreManager.grade;
            
            // 根据评级设置颜色
            switch (scoreManager.grade)
            {
                case "S":
                    gradeText.color = new Color(1f, 0.84f, 0f); // 金色
                    break;
                case "A":
                    gradeText.color = new Color(0f, 1f, 0.5f); // 青绿色
                    break;
                case "B":
                    gradeText.color = Color.green;
                    break;
                case "C":
                    gradeText.color = Color.yellow;
                    break;
                case "D":
                    gradeText.color = Color.red;
                    break;
            }
        }

        // 显示总分
        if (totalScoreText != null)
        {
            totalScoreText.text = $"{scoreManager.totalScore:F1}";
        }

        // 显示各项得分
        if (timeScoreText != null)
        {
            timeScoreText.text = $"Time: {scoreManager.timeScore:F1}";
        }

        if (accuracyScoreText != null)
        {
            accuracyScoreText.text = $"Accuracy: {scoreManager.accuracyScore:F1}";
        }

        if (speedScoreText != null)
        {
            speedScoreText.text = $"Speed Control: {scoreManager.speedScore:F1}";
        }

        if (safetyScoreText != null)
        {
            safetyScoreText.text = $"Safety: {scoreManager.safetyScore:F1}";
        }

        // 显示详细信息
        if (detailsText != null)
        {
            string details = $"Mission Duration: {scoreManager.missionDuration:F1}s\n";
            
            if (scoreManager.collisionCount > 0)
            {
                details += $"Collisions: {scoreManager.collisionCount}\n";
            }

            if (scoreManager.crashed)
            {
                details += "Status: Crashed\n";
            }

            if (scoreManager.speedSampleCount > 0)
            {
                float avgDeviation = scoreManager.totalSpeedDeviation / scoreManager.speedSampleCount;
                details += $"Avg Speed Deviation: {avgDeviation:F2} m/s\n";
            }

            detailsText.text = details;
        }

        // 启用评分面板
        if (scorePanel != null)
        {
            scorePanel.SetActive(true);
        }

        Debug.Log($"<color=green>[MissionScoreDisplay]</color> Displayed score: Grade {scoreManager.grade}, Score {scoreManager.totalScore:F1}");
    }

    /// <summary>
    /// 手动刷新评分显示（可由按钮调用）
    /// </summary>
    public void RefreshScore()
    {
        DisplayScore();
    }
}
