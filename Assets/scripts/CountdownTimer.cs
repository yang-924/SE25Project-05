using UnityEngine;
using TMPro;

public class CountdownTimer : MonoBehaviour
{
    public float totalTime = 300f; // 任务总时间（秒），例如 300 秒 = 5 分钟
    public TextMeshProUGUI countdownText;

    private float remainingTime;

    void Start()
    {
        remainingTime = totalTime;
        UpdateTimerUI();
    }

    void Update()
    {
        if (remainingTime > 0)
        {
            remainingTime -= Time.deltaTime;
            if (remainingTime < 0)
                remainingTime = 0;

            UpdateTimerUI();
        }
    }

    void UpdateTimerUI()
    {
        int minutes = Mathf.FloorToInt(remainingTime / 60);
        int seconds = Mathf.FloorToInt(remainingTime % 60);

        countdownText.text = $"{minutes:00}:{seconds:00}";
    }
}
