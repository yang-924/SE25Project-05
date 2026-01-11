# DisasterDetection任务配置指南 + Cargo跳转问题排查

## 🎯 DisasterDetection任务配置

### 1. 创建MissionConfig资源

在Unity编辑器中：
1. **Project窗口** → 右键 → `Create` → `MissionSystem` → `Mission Config`
2. **命名**：`DisasterDetect_City`（或其他名称）
3. **选中配置文件，在Inspector中设置**：

```yaml
基础配置:
  Mission Name: "城市火灾检测"
  Description: "在城市中发现所有起火点并拍照记录"
  Target Scene Name: "City"  # 或 "Forest"
  Mission Type: DisasterDetection ← 选择这个！

灾害检测配置（在自定义Editor中会自动显示）:
  Fire Prefab: [拖入火焰prefab]
  Fire Spawn Positions: 
    - Size: 5-10（配置多个候选位置）
    - Element 0: (100, 5, 50)
    - Element 1: (200, 5, 100)
    - Element 2: (150, 5, 150)
    - ...（根据场景大小配置）
  
  Number Of Fires: 3  # 每次生成3个火点
  Randomize Fire Positions: ✓  # 随机选择
  Photo Distance: 20  # 20m内有效
  Require Photo Confirmation: ✓  # 需要按P键拍照
```

### 2. 配置MissionManager（TaskSelectUI场景）

选中TaskSelectUI场景中的`MissionManager` GameObject：

```yaml
Success Scene Name: "Success"  ← 必须设置！
Fail Scene Name: "Fail"        ← 必须设置！
Transition Delay: 2            ← 延迟2秒跳转
```

### 3. 测试流程

1. **启动TaskSelectUI场景**
2. **选择DisasterDetection任务**
3. **进入游戏场景**，观察日志：
   ```
   [MissionManager] 创建灾害检测任务...
   [MissionManager] 随机选择了 3 个火点位置
   [MissionManager] 生成火点 1: (100, 5, 50)
   [DisasterDetectMission] ===== 灾害检测任务开始 =====
   [DisasterDetectMission] 总火点数: 3
   ```

4. **飞向火焰**，距离20m内按 **P键** 拍照
5. **观察UI**：`🔥 起火点: 1/3 | 最近: 45m`
6. **发现所有火点后**，2秒后自动跳转到Success场景

### 4. UI显示对比

**CargoMission**:
```
Target: 39.9044°, 116.4133° (H:503m, ΔAlt:3m)
```

**DisasterDetectMission**:
```
🔥 起火点: 2/5 | 最近: 127m
```

---

## ⚠️ Cargo场景跳转问题排查

### 问题现象
- Cargo任务可以正常拾取、投放
- 投放成功后日志显示"任务完成"
- 但是**没有跳转到Success场景**

### 排查步骤

#### 步骤1：检查MissionManager配置

1. **打开TaskSelectUI场景**
2. **选中MissionManager GameObject**
3. **在Inspector中检查**：
   ```
   [MissionManager Component]
   ├── Success Scene Name: "Success" ← 必须填写！
   ├── Fail Scene Name: "Fail"       ← 必须填写！
   └── Transition Delay: 2
   ```

**如果为空**：手动填写`Success`和`Fail`（必须与Build Settings中的场景名称一致）

---

#### 步骤2：检查Build Settings

1. **Unity菜单** → `File` → `Build Settings`
2. **确认以下场景已添加**：
   ```
   Scenes In Build:
   ✓ TaskSelectUI
   ✓ City (或其他游戏场景)
   ✓ Success  ← 必须存在！
   ✓ Fail     ← 必须存在！
   ```

**如果缺少**：
- 点击`Add Open Scenes`
- 或拖拽场景文件到列表中

---

#### 步骤3：测试场景跳转

运行以下测试代码（在Unity Console中或创建测试脚本）：

```csharp
// 测试场景跳转
using UnityEngine.SceneManagement;

// 检查场景是否存在
int sceneCount = SceneManager.sceneCountInBuildSettings;
for (int i = 0; i < sceneCount; i++)
{
    string scenePath = SceneUtility.GetScenePathByBuildIndex(i);
    string sceneName = System.IO.Path.GetFileNameWithoutExtension(scenePath);
    Debug.Log($"场景 {i}: {sceneName} ({scenePath})");
}

// 尝试直接跳转（在游戏运行时）
SceneManager.LoadScene("Success");
```

---

#### 步骤4：检查事件订阅

在Cargo任务完成时，观察Console日志：

**正常流程应该显示**：
```
[CargoMission] ===== 货物投放成功 =====
[CargoMission] 任务即将完成...
[Mission] Completed: 货物运输任务  ← MissionBase.CompleteMission()
[MissionManager] 任务成功: 货物运输任务  ← OnMissionFinishedHandler()
[MissionManager] 2秒后跳转到场景: Success  ← TransitionToResultScene()
[MissionManager] 正在加载场景: Success
```

**如果缺少后面几行**，说明事件没有触发，可能原因：
1. `MissionManager.StartMission()` 没有订阅事件
2. 检查 `OnMissionFinished -= OnMissionFinishedHandler` 是否过早清理

---

#### 步骤5：检查协程是否运行

在`MissionManager.cs`中添加调试日志：

```csharp
private IEnumerator TransitionToResultScene(bool success)
{
    string targetScene = success ? successSceneName : failSceneName;

    Debug.Log($"<color=yellow>[MissionManager]</color> {transitionDelay}秒后跳转到场景: {targetScene}");
    Debug.Log($"<color=yellow>[MissionManager]</color> 当前场景: {SceneManager.GetActiveScene().name}");
    Debug.Log($"<color=yellow>[MissionManager]</color> 协程开始等待...");

    yield return new WaitForSeconds(transitionDelay);

    Debug.Log($"<color=green>[MissionManager]</color> 等待完成，开始加载场景: {targetScene}");
    
    // 检查场景是否存在
    if (Application.CanStreamedLevelBeLoaded(targetScene))
    {
        Debug.Log($"<color=green>[MissionManager]</color> 场景存在，开始加载...");
        SceneManager.LoadScene(targetScene);
    }
    else
    {
        Debug.LogError($"<color=red>[MissionManager]</color> 场景不存在: {targetScene}");
    }
}
```

---

#### 步骤6：手动强制跳转测试

在Cargo任务完成时，按 **F1键** 手动触发跳转：

在`CargoMission.cs`的`CheckDelivery()`中添加：

```csharp
// 临时测试代码
if (Input.GetKeyDown(KeyCode.F1))
{
    Debug.Log("<color=cyan>[Test]</color> F1键按下，强制跳转到Success场景");
    UnityEngine.SceneManagement.SceneManager.LoadScene("Success");
}
```

**如果F1可以跳转**，说明场景配置正常，问题在于事件流程。

---

### 最可能的原因

根据代码分析，最可能的原因是：

**MissionManager的successSceneName/failSceneName字段在Inspector中为空**

#### 解决方案：

1. **打开TaskSelectUI场景**
2. **选中MissionManager**
3. **手动填写字段**：
   - Success Scene Name: `Success`
   - Fail Scene Name: `Fail`
4. **保存场景**（Ctrl+S）
5. **重新测试**

---

### 检查清单

在测试前，确保以下都已完成：

- [ ] TaskSelectUI场景中有MissionManager GameObject
- [ ] MissionManager挂载了MissionManager.cs脚本
- [ ] successSceneName = "Success"（在Inspector中填写）
- [ ] failSceneName = "Fail"（在Inspector中填写）
- [ ] Success.unity 和 Fail.unity 存在于Assets/Scenes文件夹
- [ ] 两个场景已添加到Build Settings
- [ ] Cargo任务可以正常完成（日志显示"任务完成"）
- [ ] 观察Console有完整的跳转日志

---

## 🐛 常见错误

### 错误1：场景名称拼写错误
```
❌ successSceneName: "success"  （小写s）
✅ successSceneName: "Success"  （大写S，必须与文件名完全一致）
```

### 错误2：场景未添加到Build Settings
```
Error: Scene 'Success' couldn't be loaded because it has not been added to the build settings
```
**解决**：File → Build Settings → Add Open Scenes

### 错误3：MissionManager在场景切换时被销毁
```
[MissionManager] 检测到重复实例，销毁旧的
```
**检查**：City场景中不应该有另一个MissionManager对象（只在TaskSelectUI中有一个）

### 错误4：事件未订阅
```
// 只显示：
[CargoMission] 任务即将完成...
[Mission] Completed: 货物运输任务

// 缺少：
[MissionManager] 任务成功: 货物运输任务  ← 这行缺失说明事件没触发
```

**检查代码**：
```csharp
// MissionManager.StartMission() 中必须有：
currentMission.OnMissionFinished += OnMissionFinishedHandler;
```

---

## 🎮 完整测试流程

### Cargo任务测试
1. ▶️ 运行TaskSelectUI场景
2. 🎯 选择Cargo任务
3. ✈️ 进入游戏场景，找到无人机
4. 📦 按N键投放货物（在范围内）
5. ⏱️ 观察日志和UI
6. ✅ 2秒后跳转到Success场景

### DisasterDetection任务测试
1. ▶️ 运行TaskSelectUI场景
2. 🎯 选择DisasterDetection任务
3. ✈️ 进入游戏场景，找到火焰（Scene视图中红色Gizmo）
4. 📸 靠近20m内按P键拍照
5. 🔥 观察UI：`🔥 起火点: 1/3 | 最近: 45m`
6. 🔍 找到所有火点后
7. ✅ 2秒后跳转到Success场景

---

## 📝 配置文件示例

### DisasterDetect_City.asset
```yaml
m_Name: DisasterDetect_City
missionName: "城市火灾检测"
description: "在城市中搜索并记录所有起火点"
targetSceneName: "City"
missionType: 4  # DisasterDetection

firePrefab: {fileID: ..., guid: ...}
fireSpawnPositions:
- {x: 100, y: 5, z: 50}
- {x: 200, y: 5, z: 100}
- {x: 150, y: 5, z: 150}
- {x: 80, y: 5, z: 200}
- {x: 250, y: 5, z: 80}

numberOfFires: 3
randomizeFirePositions: 1
detectionRadius: 20
photoDistance: 20
requirePhotoConfirmation: 1
```

### CargoDelivery_City.asset
```yaml
m_Name: CargoDelivery_City
missionName: "货物运输"
description: "将货物运送到指定快递柜"
targetSceneName: "City"
missionType: 1  # Cargo

deliveryLockerPrefab: {fileID: ..., guid: ...}
deliveryLockerPositions:
- {x: 500, y: 0, z: 500}
- {x: 600, y: 0, z: 400}
- {x: 450, y: 0, z: 550}

useRandomDeliveryPosition: 1
skipPickup: 1
```

---

## 💡 调试技巧

### 实时查看任务状态

在游戏运行时，选中MissionManager：
```
Inspector → Debug模式:
├── currentMission: CargoMission (CargoMission)
├── Status: Running
├── successSceneName: "Success"
└── failSceneName: "Fail"
```

### Console过滤

使用Console的Filter功能：
- `[MissionManager]` - 只看任务管理日志
- `[CargoMission]` - 只看货物任务日志
- `[DisasterDetectMission]` - 只看火灾任务日志
- `跳转` - 查看跳转相关日志

### 强制日志

在关键位置添加显眼的日志：
```csharp
Debug.Log($"<color=red>█████████ CompleteMission() 被调用！█████████</color>");
Debug.Log($"<color=red>█████████ OnMissionFinishedHandler() 被调用！█████████</color>");
Debug.Log($"<color=red>█████████ 开始协程跳转！█████████</color>");
```

这样即使日志很多也能快速找到关键信息。
