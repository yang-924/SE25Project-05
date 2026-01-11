using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using TMPro;

public class LoginRegisterUI : MonoBehaviour
{
    [Header("UI Panels")]
    [SerializeField] private GameObject loginPanel;
    [SerializeField] private GameObject registerPanel;
    
    [Header("Login Panel")]
    [SerializeField] private TMP_InputField loginUsernameInput;
    [SerializeField] private TMP_InputField loginPasswordInput;
    [SerializeField] private Button loginButton;
    [SerializeField] private Button toRegisterButton;
    [SerializeField] private TextMeshProUGUI loginMessageText;
    
    [Header("Register Panel")]
    [SerializeField] private TMP_InputField registerUsernameInput;
    [SerializeField] private TMP_InputField registerPasswordInput;
    [SerializeField] private TMP_InputField registerConfirmPasswordInput;
    [SerializeField] private Button registerButton;
    [SerializeField] private Button backToLoginButton;
    [SerializeField] private TextMeshProUGUI registerMessageText;
    
    [Header("Settings")]
    [SerializeField] private string mainSceneName = "MainScene";
    
    void Start()
    {
        InitializeUI();
        ShowLoginPanel();
    }
    
    private void InitializeUI()
    {
        // Login panel events
        loginButton.onClick.AddListener(OnLoginClicked);
        toRegisterButton.onClick.AddListener(ShowRegisterPanel);
        
        // Register panel events
        registerButton.onClick.AddListener(OnRegisterClicked);
        backToLoginButton.onClick.AddListener(ShowLoginPanel);
        
        // Enter key support for input fields
        loginPasswordInput.onSubmit.AddListener(_ => OnLoginClicked());
        registerConfirmPasswordInput.onSubmit.AddListener(_ => OnRegisterClicked());
    }
    
    #region Panel Switching
    
    private void ShowLoginPanel()
    {
        loginPanel.SetActive(true);
        registerPanel.SetActive(false);
        ClearMessageTexts();
    }
    
    private void ShowRegisterPanel()
    {
        loginPanel.SetActive(false);
        registerPanel.SetActive(true);
        ClearRegisterInputs();
        ClearMessageTexts();
    }
    
    #endregion
    
    #region Button Click Events
    
    private void OnLoginClicked()
    {
        string username = loginUsernameInput.text.Trim();
        string password = loginPasswordInput.text;
        
        // Input validation
        if (string.IsNullOrEmpty(username))
        {
            ShowLoginMessage("Please enter username", Color.red);
            return;
        }
        
        if (string.IsNullOrEmpty(password))
        {
            ShowLoginMessage("Please enter password", Color.red);
            return;
        }
        
        // Attempt login
        if (LocalDataManager.Instance.Login(username, password))
        {
            ShowLoginMessage("Login successful!", Color.green);
            
            // Delay 1 second before loading main scene
            Invoke(nameof(LoadMainScene), 1f);
        }
        else
        {
            ShowLoginMessage("Invalid username or password", Color.red);
        }
    }
    
    private void OnRegisterClicked()
    {
        string username = registerUsernameInput.text.Trim();
        string password = registerPasswordInput.text;
        string confirmPassword = registerConfirmPasswordInput.text;
        
        // Input validation
        if (string.IsNullOrEmpty(username))
        {
            ShowRegisterMessage("Please enter username", Color.red);
            return;
        }
        
        if (username.Length < 3)
        {
            ShowRegisterMessage("Username must be at least 3 characters", Color.red);
            return;
        }
        
        if (string.IsNullOrEmpty(password))
        {
            ShowRegisterMessage("Please enter password", Color.red);
            return;
        }
        
        if (password.Length < 6)
        {
            ShowRegisterMessage("Password must be at least 6 characters", Color.red);
            return;
        }
        
        if (password != confirmPassword)
        {
            ShowRegisterMessage("Passwords do not match", Color.red);
            return;
        }
        
        // Attempt registration
        if (LocalDataManager.Instance.RegisterUser(username, password))
        {
            ShowRegisterMessage("Registration successful! Returning to login page", Color.green);
            
            // Delay 1.5 seconds before returning to login page with username filled
            Invoke(nameof(ReturnToLoginWithUsername), 1.5f);
        }
        else
        {
            ShowRegisterMessage("Registration failed. Username may already exist", Color.red);
        }
    }
    
    #endregion
    
    #region Helper Methods
    
    private void ReturnToLoginWithUsername()
    {
        string username = registerUsernameInput.text.Trim();
        ShowLoginPanel();
        loginUsernameInput.text = username;
        loginPasswordInput.text = "";
    }
    
    private void LoadMainScene()
    {
        if (!string.IsNullOrEmpty(mainSceneName))
        {
            SceneManager.LoadScene(mainSceneName);
        }
    }
    
    private void ShowLoginMessage(string message, Color color)
    {
        if (loginMessageText != null)
        {
            loginMessageText.text = message;
            loginMessageText.color = color;
        }
    }
    
    private void ShowRegisterMessage(string message, Color color)
    {
        if (registerMessageText != null)
        {
            registerMessageText.text = message;
            registerMessageText.color = color;
        }
    }
    
    private void ClearMessageTexts()
    {
        if (loginMessageText != null)
            loginMessageText.text = "";
        
        if (registerMessageText != null)
            registerMessageText.text = "";
    }
    
    private void ClearRegisterInputs()
    {
        if (registerUsernameInput != null)
            registerUsernameInput.text = "";
        
        if (registerPasswordInput != null)
            registerPasswordInput.text = "";
        
        if (registerConfirmPasswordInput != null)
            registerConfirmPasswordInput.text = "";
    }
    
    #endregion
    
    #region Public Methods
    
    public void ClearAllUserData()
    {
        LocalDataManager.Instance.ClearAllData();
    }
    
    #endregion
}