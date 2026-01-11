using UnityEngine;
using UnityEditor;

namespace MissionSystem
{
#if UNITY_EDITOR
    [CustomEditor(typeof(MissionConfig))]
    public class MissionConfigEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            MissionConfig config = (MissionConfig)target;
            serializedObject.Update();

            // 基本信息（始终显示）
            EditorGUILayout.LabelField("Basic Info", EditorStyles.boldLabel);
            config.missionName = EditorGUILayout.TextField("Mission Name", config.missionName);
            config.description = EditorGUILayout.TextArea(config.description, GUILayout.Height(60));
            config.icon = (Sprite)EditorGUILayout.ObjectField("Icon", config.icon, typeof(Sprite), false);

            EditorGUILayout.Space();

            // 场景设置
            EditorGUILayout.LabelField("Scene", EditorStyles.boldLabel);
            config.targetSceneName = EditorGUILayout.TextField("Target Scene Name", config.targetSceneName);

            EditorGUILayout.Space();

            // 任务类型
            EditorGUILayout.LabelField("Mission Type", EditorStyles.boldLabel);
            config.missionType = (MissionType)EditorGUILayout.EnumPopup("Mission Type", config.missionType);

            EditorGUILayout.Space();

            // 根据任务类型显示对应字段
            switch (config.missionType)
            {
                case MissionType.Waypoint:
                    DrawWaypointFields(config);
                    break;

                case MissionType.Cargo:
                    DrawCargoFields(config);
                    break;

                case MissionType.DisasterDetection:
                    DrawDisasterFields(config);
                    break;

                case MissionType.Patrol:
                    DrawPatrolFields(config);
                    break;

                case MissionType.Recon:
                    EditorGUILayout.HelpBox("Recon mission configuration not implemented yet.", MessageType.Info);
                    break;
            }

            // 保存更改
            if (GUI.changed)
            {
                EditorUtility.SetDirty(config);
                serializedObject.ApplyModifiedProperties();
            }
        }

        void DrawWaypointFields(MissionConfig config)
        {
            EditorGUILayout.LabelField("Waypoint Mission Data", EditorStyles.boldLabel);

            config.reachThreshold = EditorGUILayout.FloatField("Reach Threshold", config.reachThreshold);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Waypoint Positions");

            // 数组大小控制
            int newSize = EditorGUILayout.IntField("Size", config.waypointPositions?.Length ?? 0);
            if (newSize != (config.waypointPositions?.Length ?? 0))
            {
                System.Array.Resize(ref config.waypointPositions, newSize);
            }

            // 显示每个航点
            if (config.waypointPositions != null)
            {
                for (int i = 0; i < config.waypointPositions.Length; i++)
                {
                    config.waypointPositions[i] = EditorGUILayout.Vector3Field($"Waypoint {i}", config.waypointPositions[i]);
                }
            }
        }

        void DrawCargoFields(MissionConfig config)
        {
            EditorGUILayout.LabelField("Cargo Mission Data", EditorStyles.boldLabel);

            config.pickupLocation = EditorGUILayout.Vector3Field("Pickup Location", config.pickupLocation);
            config.deliveryLocation = EditorGUILayout.Vector3Field("Delivery Location", config.deliveryLocation);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Cargo Setup", EditorStyles.miniBoldLabel);

            config.cargoChildName = EditorGUILayout.TextField("Cargo Child Name", config.cargoChildName);
            config.skipPickup = EditorGUILayout.Toggle("Skip Pickup (已装载)", config.skipPickup);

            EditorGUILayout.HelpBox("货物子物体名称（在无人机prefab中）。如果货物已绑定在无人机上，勾选'Skip Pickup'跳过拾取阶段。", MessageType.Info);

            config.cargoMarkerPrefab = (GameObject)EditorGUILayout.ObjectField("Cargo Marker Prefab", config.cargoMarkerPrefab, typeof(GameObject), false);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Delivery Locker Setup", EditorStyles.miniBoldLabel);

            config.deliveryLockerPrefab = (GameObject)EditorGUILayout.ObjectField("Delivery Locker Prefab", config.deliveryLockerPrefab, typeof(GameObject), false);
            config.useRandomDeliveryPosition = EditorGUILayout.Toggle("Random Delivery Position", config.useRandomDeliveryPosition);

            if (config.useRandomDeliveryPosition)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Delivery Locker Positions (Random)");

                int newSize = EditorGUILayout.IntField("Position Count", config.deliveryLockerPositions?.Length ?? 0);
                if (newSize != (config.deliveryLockerPositions?.Length ?? 0))
                {
                    System.Array.Resize(ref config.deliveryLockerPositions, newSize);
                }

                if (config.deliveryLockerPositions != null)
                {
                    for (int i = 0; i < config.deliveryLockerPositions.Length; i++)
                    {
                        config.deliveryLockerPositions[i] = EditorGUILayout.Vector3Field($"Position {i}", config.deliveryLockerPositions[i]);
                    }
                }

                EditorGUILayout.HelpBox("快递柜会在这些位置中随机选择一个生成。deliveryLocation 会被自动设置为选中的位置。", MessageType.Info);
            }
            else
            {
                EditorGUILayout.HelpBox("快递柜会在 Delivery Location 位置生成。", MessageType.Info);
            }
        }

        void DrawPatrolFields(MissionConfig config)
        {
            EditorGUILayout.LabelField("Patrol Mission Data", EditorStyles.boldLabel);

            config.patrolLoops = EditorGUILayout.IntField("Patrol Loops", config.patrolLoops);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Patrol Points");

            // 数组大小控制
            int newSize = EditorGUILayout.IntField("Size", config.patrolPoints?.Length ?? 0);
            if (newSize != (config.patrolPoints?.Length ?? 0))
            {
                System.Array.Resize(ref config.patrolPoints, newSize);
            }

            // 显示每个巡逻点
            if (config.patrolPoints != null)
            {
                for (int i = 0; i < config.patrolPoints.Length; i++)
                {
                    config.patrolPoints[i] = EditorGUILayout.Vector3Field($"Patrol Point {i}", config.patrolPoints[i]);
                }
            }
        }

        void DrawDisasterFields(MissionConfig config)
        {
            EditorGUILayout.LabelField("Disaster Detection Mission Data", EditorStyles.boldLabel);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Fire Settings", EditorStyles.miniBoldLabel);

            config.firePrefab = (GameObject)EditorGUILayout.ObjectField("Fire Prefab", config.firePrefab, typeof(GameObject), false);

            EditorGUILayout.HelpBox("火焰prefab（可选）。如果为空，系统会生成红色球体作为标记。", MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Spawn Configuration", EditorStyles.miniBoldLabel);

            config.numberOfFires = EditorGUILayout.IntField("Number Of Fires", Mathf.Max(1, config.numberOfFires));
            config.randomizeFirePositions = EditorGUILayout.Toggle("Randomize Positions", config.randomizeFirePositions);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Fire Spawn Positions");

            // 数组大小控制
            int newSize = EditorGUILayout.IntField("Position Count", config.fireSpawnPositions?.Length ?? 0);
            if (newSize != (config.fireSpawnPositions?.Length ?? 0))
            {
                System.Array.Resize(ref config.fireSpawnPositions, newSize);
            }

            // 显示每个火点位置
            if (config.fireSpawnPositions != null)
            {
                for (int i = 0; i < config.fireSpawnPositions.Length; i++)
                {
                    config.fireSpawnPositions[i] = EditorGUILayout.Vector3Field($"Fire Position {i}", config.fireSpawnPositions[i]);
                }
            }

            if (config.randomizeFirePositions)
            {
                EditorGUILayout.HelpBox($"系统会从上述 {config.fireSpawnPositions?.Length ?? 0} 个位置中随机选择 {config.numberOfFires} 个生成火点。", MessageType.Info);
            }
            else
            {
                EditorGUILayout.HelpBox($"系统会按顺序在前 {config.numberOfFires} 个位置生成火点。", MessageType.Info);
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Detection Settings", EditorStyles.miniBoldLabel);

            config.photoDistance = EditorGUILayout.FloatField("Photo Distance (米)", Mathf.Max(1f, config.photoDistance));
            config.requirePhotoConfirmation = EditorGUILayout.Toggle("Require Photo (F12键)", config.requirePhotoConfirmation);

            if (config.requirePhotoConfirmation)
            {
                EditorGUILayout.HelpBox("玩家需要在 20m 范围内按 F12 键（FPV截图键）拍照才能确认发现火点。", MessageType.Info);
            }
            else
            {
                EditorGUILayout.HelpBox("进入检测范围自动记录，无需拍照。", MessageType.Info);
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Mission Summary", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                $"任务配置：\n" +
                $"• 总候选位置：{config.fireSpawnPositions?.Length ?? 0} 个\n" +
                $"• 生成火点数：{config.numberOfFires} 个\n" +
                $"• 检测方式：{(config.requirePhotoConfirmation ? "拍照确认" : "自动检测")}\n" +
                $"• 检测半径：{config.photoDistance} 米\n" +
                $"• 完成条件：发现所有 {config.numberOfFires} 个火点",
                MessageType.None);
        }
    }
#endif
}
