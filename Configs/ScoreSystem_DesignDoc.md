# 评分系统设计说明与问题排查指南

## 📋 评分系统设计原理

### 为什么需要在每个任务中单独调用评分？

**MissionScoreManager** 是一个**通用框架**，类似于一个计分板，它知道如何计算分数，但**不知道任务的具体细节**。

```
┌─────────────────────────────────────────┐
│  MissionScoreManager (计分框架)         │
│  - 知道评分规则和公式                    │
│  - 不知道任务何时开始/结束               │
│  - 不知道任务的具体完成情况               │
└─────────────────────────────────────────┘
           ↑ 需要任务告诉它 ↑
┌─────────────────────────────────────────┐
│  具体任务 (WaypointMission, 等)         │
│  - 知道任务何时开始                      │
│  - 知道任务何时完成                      │
│  - 知道任务的精度/速度等细节              │
│  - 必须主动调用评分系统的方法             │
└─────────────────────────────────────────┘
```

### 评分系统是如何细分的？

评分系统**已经根据任务类型细分**，看这段代码：

```csharp
// 在 MissionScoreManager.CompleteMission() 中
switch (missionType)
{
    case MissionType.DisasterDetection:
        CalculateDisasterScore(referenceTime);  // 🔥 火灾专用评分
        break;
    case MissionType.Cargo:
        CalculateCargoScore(referenceTime);     // 📦 货物专用评分
        break;
    case MissionType.Waypoint:
        CalculateWaypointScore(referenceTime);  // ✈️ 航点专用评分
        break;
    case MissionType.Recon:
        CalculateReconScore(referenceTime);     // 🔍 侦察专用评分
        break;
}
```

**每个任务类型都有不同的评分权重**：
- 灾害检测：时间40分 + 精度40分 + 安全20分
- 货物运输：时间50分 + 安全30分 + 精度20分
- 航点巡航：时间30分 + 速度40分 + 精度20分 + 安全10分
- 侦察任务：时间40分 + 精度40分 + 安全20分

### 为什么不能自动调用？

**问题**：评分系统无法自动知道：
1. ❌ 任务什么时候真正开始了（玩家可能还在准备）
2. ❌ 任务什么时候真正完成了（不同任务有不同的完成条件）
3. ❌ 任务的精度如何（火点全找到了吗？悬停稳定吗？）
4. ❌ 当前速度是否符合要求（定速巡航是否达标？）

**解决方案**：每个任务必须主动告诉评分系统：

```csharp
// 1. 任务开始时
BeginMission() {
    MissionScoreManager.Instance.StartMission(MissionType.Waypoint);
}

// 2. 任务进行中（定速巡航任务）
OnUpdate() {
    MissionScoreManager.Instance.RecordSpeedDeviation(currentSpeed, targetSpeed);
}

// 3. 任务完成时
CompleteMission() {
    MissionScoreManager.Instance.SetAccuracyScore(40f);  // 告诉精度得分
    MissionScoreManager.Instance.CompleteMission(MissionType.Waypoint, referenceTime);
}
```

---

## 🔍 场景跳转问题排查指南

### 问题现象
任务完成后无法跳转到Success/Fail页面

### 可能原因及排查步骤

#### ✅ 步骤1：检查MissionManager配置

**在Hierarchy中找到MissionManager对象，检查Inspector：**

```
MissionManager (GameObject)
└── MissionManager (Component)
    ├── Success Scene Name: "Success"  ← 必须填写！
    ├── Fail Scene Name: "Fail"        ← 必须填写！
    └── Transition Delay: 2.0          ← 延迟时间
```

**常见错误**：
- ❌ Success/Fail Scene Name 为空
- ❌ 场景名称拼写错误
- ❌ 场景未添加到Build Settings

#### ✅ 步骤2：检查Build Settings

打开 **File → Build Settings**，确认：
- ✅ Success 场景在列表中
- ✅ Fail 场景在列表中
- ✅ 场景名称与MissionManager配置一致

#### ✅ 步骤3：查看Console日志

运行游戏后，任务完成时应该看到以下日志序列：

**正常流程**：
```
[MissionBase] CompleteMission called for 'XXX', Current Status: Running
[MissionBase] Invoking OnMissionFinished event (success=true)
[MissionBase] OnMissionFinished has 1 subscriber(s)
[MissionManager] ===== OnMissionFinishedHandler CALLED =====
[MissionManager] Mission: XXX, Success: true
[MissionManager] ===== TransitionToResultScene Coroutine Started =====
[MissionManager] Target Scene: Success
[MissionManager] Waiting 2 seconds before scene transition...
[MissionManager] Delay complete. Now loading scene: Success
[MissionManager] SceneManager.LoadScene() called successfully
```

**异常情况及含义**：

| 日志 | 问题 | 解决方法 |
|------|------|---------|
| ❌ `OnMissionFinished is NULL!` | 没有订阅者监听任务完成事件 | 检查MissionManager是否在场景中 |
| ❌ `Target scene name is NULL or EMPTY!` | Success/Fail场景名未配置 | 在MissionManager Inspector中填写 |
| ❌ `Failed to load scene 'XXX'` | 场景不存在或未添加到Build Settings | 添加场景到Build Settings |
| ⚠️ `Cannot complete mission - not running (Status: XXX)` | 任务状态不对，可能已经完成过 | 检查是否多次调用CompleteMission |

#### ✅ 步骤4：检查事件订阅

在MissionManager的`InitializeMission()`方法中，应该有：

```csharp
currentMission.OnMissionFinished += OnMissionFinishedHandler;
```

**排查方法**：
1. 打开 [MissionManager.cs](c:\Users\yang\Desktop\SE25Project-05\DronePlatform\Assets\scripts\MissionSystem\MissionManager.cs)
2. 搜索 `OnMissionFinished +=`
3. 确认任务创建后正确订阅了事件

#### ✅ 步骤5：检查MissionBase状态

任务必须处于 `Running` 状态才能完成：

```csharp
public enum MissionStatus
{
    NotStarted,  // 未开始
    Running,     // 运行中 ← 只有这个状态可以完成
    Completed,   // 已完成
    Failed       // 已失败
}
```

**如果任务状态不对**：
- 检查 `BeginMission()` 是否被正确调用
- 检查是否有其他代码错误地修改了状态

---

## 🐛 调试技巧

### 1. 测试简单任务
创建一个最简单的测试任务：

```csharp
public class TestMission : MissionBase
{
    public override void BeginMission()
    {
        base.BeginMission();
        Debug.Log("TEST: Mission started");
        
        // 3秒后自动完成
        StartCoroutine(AutoComplete());
    }

    IEnumerator AutoComplete()
    {
        yield return new WaitForSeconds(3f);
        Debug.Log("TEST: Completing mission...");
        CompleteMission();
    }

    public override void OnUpdate() { }
}
```

### 2. 手动触发完成
在任意脚本中添加：

```csharp
void Update()
{
    if (Input.GetKeyDown(KeyCode.F9))
    {
        var mission = MissionManager.Instance?.currentMission;
        if (mission != null)
        {
            Debug.Log("Manual complete triggered!");
            mission.CompleteMission();
        }
    }
}
```

按F9手动完成任务，观察日志。

### 3. 检查场景加载
创建一个简单的场景加载测试：

```csharp
void Update()
{
    if (Input.GetKeyDown(KeyCode.F10))
    {
        Debug.Log("Loading Success scene...");
        SceneManager.LoadScene("Success");
    }
}
```

如果F10能跳转但任务完成不能，说明问题在事件订阅。

---

## 📊 完整事件流程图

```
1. 任务开始
   ↓
BeginMission()
   ├─ base.BeginMission() → Status = Running
   ├─ MissionScoreManager.StartMission() ← 启动评分
   └─ 初始化任务数据

2. 任务进行中
   ↓
OnUpdate() 每帧调用
   ├─ 检测任务目标
   ├─ RecordSpeedDeviation() ← 记录速度偏差
   └─ RecordCollision() ← 记录碰撞

3. 任务完成
   ↓
CompleteMission()
   ├─ 检查 Status == Running ? ✓
   ├─ Status = Completed
   ├─ SetAccuracyScore() ← 设置精度得分
   ├─ MissionScoreManager.CompleteMission() ← 计算总分
   ├─ OnMissionFinished?.Invoke(this, true) ← 触发事件
   │   ↓
   │   MissionManager.OnMissionFinishedHandler()
   │   ↓
   │   TransitionToResultScene(true)
   │   ↓
   │   WaitForSeconds(2)
   │   ↓
   │   SceneManager.LoadScene("Success")
   └─ 完成！
```

---

## ✅ 检查清单

运行游戏前，确认以下所有项：

- [ ] MissionManager在Hierarchy中存在
- [ ] Success Scene Name = "Success"
- [ ] Fail Scene Name = "Fail"
- [ ] Success场景在Build Settings中
- [ ] Fail场景在Build Settings中
- [ ] Console日志级别设置为显示所有信息
- [ ] 任务脚本调用了StartMission()
- [ ] 任务脚本调用了CompleteMission()
- [ ] 任务状态为Running时才调用CompleteMission

---

## 📝 总结

### 评分系统设计
- ✅ 已经按任务类型细分（每种任务不同评分权重）
- ✅ 采用主动调用模式（任务告诉评分系统，而不是自动检测）
- ✅ 灵活可扩展（每个任务可以自定义精度计算方式）

### 场景跳转问题
- 现在添加了详细日志，可以精确定位问题
- 大多数问题来自配置缺失（Scene Name未填写、场景未添加）
- 少数问题来自事件未订阅或状态错误

按照上述检查清单逐项排查，应该能快速找到问题！
