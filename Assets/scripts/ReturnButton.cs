using UnityEngine;
using UnityEngine.SceneManagement;

public class ReturnButton : MonoBehaviour
{
    [Header("返回的场景名（留空则返回上一场景）")]
    public string targetSceneName = "";

    public void OnReturn()
    {
        if (!string.IsNullOrEmpty(targetSceneName))
        {
            // 返回指定场景
            SceneManager.LoadScene(targetSceneName);
        }
        else
        {
            // 返回上一场景（如果你有记录）
            string lastScene = PlayerPrefs.GetString("LastScene", "");
            if (!string.IsNullOrEmpty(lastScene))
            {
                SceneManager.LoadScene(lastScene);
            }
            else
            {
                Debug.LogWarning("没有记录上一场景，无法返回");
            }
        }
    }
}
