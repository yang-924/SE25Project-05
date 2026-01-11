using System.IO;
using System.Collections.Generic;
using UnityEngine;
using System;

[System.Serializable]
public class UserData
{
    public string username;
    public string passwordHash;  // 存储哈希值，不存明文
    public DateTime registerDate;
    
    public UserData(string username, string password)
    {
        this.username = username;
        this.passwordHash = HashPassword(password);
        this.registerDate = DateTime.Now;
    }
    
    // 验证密码
    public bool VerifyPassword(string password)
    {
        return passwordHash == HashPassword(password);
    }
    
    // 简单的密码哈希
    private string HashPassword(string password)
    {
        // 使用简单的哈希，实际项目建议用更安全的方法
        int hash = password.GetHashCode();
        return hash.ToString();
    }
}

[System.Serializable]
public class UserDatabase
{
    public List<UserData> users = new List<UserData>();
}

public class LocalDataManager : MonoBehaviour
{
    private static LocalDataManager instance;
    public static LocalDataManager Instance => instance;
    
    private const string USER_DATA_FILE = "userdata.json";
    private UserDatabase userDatabase;
    
    void Awake()
    {
        Debug.Log($"用户数据保存路径: {Application.persistentDataPath}");
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }
        
        instance = this;
        DontDestroyOnLoad(gameObject);
        LoadUserData();
    }
    
    // 加载用户数据
    private void LoadUserData()
    {
        string filePath = Path.Combine(Application.persistentDataPath, USER_DATA_FILE);
        
        if (File.Exists(filePath))
        {
            try
            {
                string json = File.ReadAllText(filePath);
                userDatabase = JsonUtility.FromJson<UserDatabase>(json);
            }
            catch
            {
                userDatabase = new UserDatabase();
            }
        }
        else
        {
            userDatabase = new UserDatabase();
        }
    }
    
    // 保存用户数据
    private void SaveUserData()
    {
        try
        {
            string filePath = Path.Combine(Application.persistentDataPath, USER_DATA_FILE);
            string json = JsonUtility.ToJson(userDatabase, true);
            File.WriteAllText(filePath, json);
        }
        catch { }
    }
    
    // 注册新用户
    public bool RegisterUser(string username, string password)
    {
        if (string.IsNullOrWhiteSpace(username) || string.IsNullOrWhiteSpace(password))
            return false;
        
        if (password.Length < 6)
            return false;
        
        if (UserExists(username))
            return false;
        
        UserData newUser = new UserData(username, password);
        userDatabase.users.Add(newUser);
        SaveUserData();
        return true;
    }
    
    // 用户登录
    public bool Login(string username, string password)
    {
        UserData user = GetUser(username);
        if (user == null)
            return false;
        
        return user.VerifyPassword(password);
    }
    
    // 检查用户是否存在
    public bool UserExists(string username)
    {
        return userDatabase.users.Exists(u => u.username == username);
    }
    
    // 获取用户数据
    private UserData GetUser(string username)
    {
        return userDatabase.users.Find(u => u.username == username);
    }
    
    // 清除所有数据（用于测试）
    public void ClearAllData()
    {
        string filePath = Path.Combine(Application.persistentDataPath, USER_DATA_FILE);
        if (File.Exists(filePath))
            File.Delete(filePath);
        
        userDatabase = new UserDatabase();
    }
}