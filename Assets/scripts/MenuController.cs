using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuController : MonoBehaviour
{
    public void GoToScene(string sceneName)
    {
        SceneManager.LoadScene(sceneName);
    }

    public void ExitApp() 
    { 
        #if UNITY_EDITOR 
            UnityEditor.EditorApplication.isPlaying = false; // 编辑器中停止运行 
        #else 
            Application.Quit(); // 真机或打包后退出程序 
        #endif 
    }
}
