# 配置文件系统说明

## 📁 文件夹位置
此文件夹位于项目根目录 `DronePlatform/Configs/`，包含所有任务和天气配置文件。

## 📄 配置文件列表

### 🔥 火灾探测任务
1. **CityFire_Sunny.json**
   - 场景：City
   - 火点数量：3个
   - 天气：晴天（无雨，微风5m/s）

2. **CityFire_Storm.json**
   - 场景：City
   - 火点数量：3个
   - 天气：暴风雨（最大雨强1.0，强风25m/s，浓雾0.5）

3. **CityFire_Rain.json**
   - 场景：City
   - 火点数量：5个
   - 天气：小雨（雨强0.3，风速10m/s，轻雾0.2）

4. **ForestFire_Windy.json**
   - 场景：Forest
   - 火点数量：4个
   - 天气：大风（风速15m/s，无雨，轻雾0.15）

### 📦 货物运输任务
5. **CargoDelivery_Sunny.json**
   - 场景：City
   - 任务类型：货物运输（Cargo）
   - 天气：晴天（微风3m/s）

## 🎮 Unity Editor 配置步骤

### 1. 场景选择界面配置（SceneSelectUI）

在 SelectController 脚本的 Inspector 面板中：

#### Config Selection 区域：
- **Use Config System**: ✅ 勾选启用
- **Config Dropdown**: 拖入场景中的 TMP_Dropdown 组件
- **Config Preview Text**: 拖入预览文本的 TextMeshProUGUI 组件

### 2. UI布局建议

```
SceneSelectUI Canvas
├── Scene Preview (现有)
├── Drone Preview (现有)
├── Config Selection Panel [新增]
│   ├── Label: "Configuration"
│   ├── TMP_Dropdown (configDropdown)
│   └── Preview Text (configPreviewText)
└── Confirm Button (现有)
```

### 3. Dropdown事件配置
- Dropdown的 OnValueChanged 事件会自动连接到 SelectController.OnConfigSelected
- 无需手动配置事件

## 🔧 配置文件格式

```json
{
  "configName": "配置名称",
  "mission": {
    "type": "任务类型（DisasterDetection/Cargo/Waypoint/Patrol）",
    "disasterSettings": {
      "fireCount": 火点数量,
      "firePoints": [
        {"x": 坐标X, "y": 坐标Y, "z": 坐标Z}
      ],
      "detectionRange": 探测范围,
      "photoMode": "拍照模式（Manual/Auto）",
      "needPhotoCount": 需要拍照数量
    }
  },
  "weather": {
    "rainIntensity": 雨强度（0-1）,
    "baseWindSpeed": 基础风速,
    "windDirection": {"x": 风向X, "y": 风向Y, "z": 风向Z},
    "fogDensity": 雾密度（0-1）
  }
}
```

## 🚀 使用流程

1. **选择场景** → 点击左右箭头选择城市/森林场景
2. **选择无人机** → 点击左右箭头选择无人机型号
3. **选择配置** → 从Dropdown中选择任务配置
4. **查看预览** → 预览文本会显示：
   - 配置名称
   - 任务类型和参数
   - 天气条件
5. **确认进入** → 点击Confirm按钮，系统自动应用配置并加载场景

## ⚙️ 配置系统功能

### 自动功能：
- ✅ 启动时自动扫描 Configs 文件夹
- ✅ 自动填充 Dropdown 列表
- ✅ 选择配置时自动加载并显示预览
- ✅ 确认进入时自动应用配置到 MissionManager 和 WeatherManager

### 手动功能：
- `ApplyCurrentConfig()` - 立即应用当前配置（可绑定到按钮）
- `RefreshConfigList()` - 刷新配置文件列表（添加新文件后使用）

## 🔍 调试信息

配置系统会输出以下日志：
- `[ConfigManager] Found X config files` - 扫描到的配置数量
- `[ConfigManager] Loaded config: <name>` - 成功加载配置
- `[SelectController] Applied config: <name>` - 应用配置到系统
- `[ConfigManager] Applied mission config` - 任务参数已应用
- `[ConfigManager] Applied weather config` - 天气参数已应用

## 📝 添加新配置

1. 复制现有JSON文件
2. 修改参数（任务类型、火点位置、天气条件等）
3. 保存到 Configs 文件夹
4. （可选）在游戏中点击刷新按钮，或重启游戏
5. 新配置会自动出现在Dropdown中

## ⚠️ 注意事项

1. **文件名规范**：建议使用 `TaskName_Weather.json` 格式
2. **坐标范围**：确保火点坐标在场景有效范围内
3. **参数限制**：
   - rainIntensity: 0-1
   - windSpeed: 建议0-30
   - fogDensity: 0-1
   - detectionRange: 建议5-50米

4. **JSON语法**：确保JSON格式正确，可使用在线验证工具检查

## 🎯 快速测试

1. 打开 SceneSelectUI 场景
2. 运行游戏
3. Dropdown应显示5个配置选项
4. 选择 "CityFire_Storm" 
5. 预览文本应显示暴风雨配置
6. 点击Confirm进入游戏
7. 验证：大雨、强风、3个火点
