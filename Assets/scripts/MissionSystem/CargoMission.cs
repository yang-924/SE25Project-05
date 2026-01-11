using UnityEngine;
using System.Collections.Generic;

namespace MissionSystem
{
    /// <summary>
    /// 货物运输任务
    /// 功能：拾取货物 → 运输到目标点 → 投放货物 → 完成
    /// </summary>
    public class CargoMission : MissionBase
    {
        [Header("Cargo Points")]
        [Tooltip("货物拾取点")]
        public Transform pickupPoint;
        [Tooltip("货物投放点")]
        public Transform deliveryPoint;

        [Header("Cargo Settings")]
        [Tooltip("货物对象（场景中的实际物体）")]
        public GameObject cargoObject;
        [Tooltip("跳过拾取阶段（货物已绑定在无人机上）")]
        public bool skipPickup = false;
        [Tooltip("拾取距离阈值（米）")]
        public float pickupDistance = 3f;
        [Tooltip("投放距离阈值（米）")]
        public float deliveryDistance = 10f;
        [Tooltip("投放高度限制（低于此高度才能投放）")]
        public float deliveryMaxHeight = 10f;
        [Tooltip("最大投放尝试次数（超过后任务失败）")]
        public int maxDropAttempts = 1;

        [Header("Visualization")]
        public Color pickupColor = Color.green;
        public Color deliveryColor = Color.blue;
        public float gizmoRadius = 2f;

        private enum CargoState
        {
            WaitingForPickup,   // 等待拾取
            Carrying,           // 正在运输
            WaitingForDelivery, // 等待投放
            Delivered           // 已投放
        }

        private CargoState currentState = CargoState.WaitingForPickup;
        private bool cargoAttached = false;
        private Vector3 cargoLocalOffset;
        private Transform droneTransform;
        private float lastDeliveryCheckTime = 0f; // 上次检查投放条件的时间
        private bool cargoChecked = false; // 是否已检查过货物状态
        private int failedDropAttempts = 0; // 失败的投放尝试次数
        private bool isProcessingDrop = false; // 是否正在处理投放（防止重复触发）
        private bool lastNInputState = false; // 上一帧的N键状态（用于边缘检测）

        public override void BeginMission()
        {
            base.BeginMission();

            // 开始评分
            MissionScoreManager.Instance.StartMission(MissionType.Cargo);
            Debug.Log("<color=cyan>[CargoMission]</color> 评分系统已启动");

            // 不在这里查找无人机，延迟到 Update 中
            // 因为 MissionBridge 可能还没生成无人机

            if (pickupPoint == null || deliveryPoint == null)
            {
                Debug.LogError("[CargoMission] Pickup or Delivery point not assigned!");
                FailMission("Missing pickup or delivery points");
                return;
            }

            currentState = CargoState.WaitingForPickup;
            Debug.Log($"<color=cyan>[CargoMission]</color> ===== 任务开始 =====");
            Debug.Log($"<color=cyan>[CargoMission]</color> 拾取点: {pickupPoint.position}");
            Debug.Log($"<color=cyan>[CargoMission]</color> 投放点: {deliveryPoint.position}");
            Debug.Log($"<color=yellow>[CargoMission]</color> 等待无人机生成...");
        }

        public override void OnUpdate()
        {
            if (Status != MissionStatus.Running) return;

            // 自动查找无人机（延迟查找，等待 MissionBridge 生成）
            if (droneTransform == null)
            {
                GameObject drone = GameObject.FindGameObjectWithTag("Player");
                if (drone != null)
                {
                    droneTransform = drone.transform;
                    Debug.Log($"<color=green>[CargoMission]</color> 找到无人机: {drone.name}");
                    Debug.Log($"<color=green>[CargoMission]</color> 当前状态: 等待拾取 - 请飞向拾取点（绿色标记）");
                    NotifyObjective("请飞往拾取点获取货物");
                }
                else
                {
                    return; // 继续等待
                }
            }

            // 自动查找货物对象（只执行一次）
            if (!cargoChecked)
            {
                cargoChecked = true; // 标记已检查

                if (cargoObject == null)
                {
                    Debug.Log($"<color=yellow>[CargoMission]</color> 货物对象为空，开始查找...");
                    // 优先在无人机子物体中查找
                    foreach (Transform child in droneTransform)
                    {
                        if (child.name.ToLower().Contains("cargo") &&
                            !child.name.ToLower().Contains("pickup") &&
                            !child.name.ToLower().Contains("delivery"))
                        {
                            cargoObject = child.gameObject;
                            Debug.Log($"<color=cyan>[CargoMission]</color> 在无人机子物体中找到货物: {child.name}");
                            skipPickup = true; // 自动启用跳过拾取
                            break;
                        }
                    }

                    // 备选：场景中独立的货物对象
                    if (cargoObject == null)
                    {
                        GameObject[] allObjects = FindObjectsOfType<GameObject>();
                        foreach (GameObject obj in allObjects)
                        {
                            if (obj.name.ToLower().Contains("cargo") &&
                                !obj.name.ToLower().Contains("pickup") &&
                                !obj.name.ToLower().Contains("delivery") &&
                                !obj.name.ToLower().Contains("point"))
                            {
                                cargoObject = obj;
                                Debug.Log($"<color=cyan>[CargoMission]</color> 在场景中找到货物对象: {obj.name}");
                                break;
                            }
                        }
                    }
                }
                else
                {
                    Debug.Log($"<color=cyan>[CargoMission]</color> 货物对象已由 MissionManager 设置: {cargoObject.name}");
                }

                // 检测货物是否已经是无人机子物体
                if (cargoObject != null && cargoObject.transform.IsChildOf(droneTransform))
                {
                    Debug.Log($"<color=yellow>[CargoMission]</color> 货物已绑定在无人机上，自动跳过拾取阶段");
                    skipPickup = true;
                    cargoAttached = true;
                    cargoLocalOffset = cargoObject.transform.localPosition;
                }
            }

            // 如果跳过拾取，直接进入运输状态
            if (skipPickup && currentState == CargoState.WaitingForPickup)
            {
                currentState = CargoState.Carrying;
                Debug.Log($"<color=green>[CargoMission]</color> 跳过拾取阶段，直接进入运输状态");
                Debug.Log($"<color=cyan>[CargoMission]</color> 货物已装载: {cargoObject?.name ?? "NULL"}");
                Debug.Log($"<color=cyan>[CargoMission]</color> 投放点位置: {deliveryPoint.position}");
                NotifyObjective("货物已装载，请飞往投放点");
            }

            switch (currentState)
            {
                case CargoState.WaitingForPickup:
                    CheckPickup();
                    break;

                case CargoState.Carrying:
                    UpdateCargoPosition();
                    // 检查是否到达投放区域
                    CheckDelivery();
                    break;

                case CargoState.WaitingForDelivery:
                    CheckDelivery();
                    break;

                case CargoState.Delivered:
                    // 任务已在CheckDelivery中完成，此处无需操作
                    break;
            }
        }

        void CheckPickup()
        {
            if (cargoObject == null)
            {
                Debug.LogWarning("[CargoMission] Cargo object is null, cannot pickup!");
                return;
            }

            float distance = Vector3.Distance(droneTransform.position, pickupPoint.position);

            // 每5秒显示一次距离提示
            if (Time.frameCount % 300 == 0)
            {
                Debug.Log($"<color=yellow>[CargoMission]</color> 离拾取点距离: {distance:F1}m / {pickupDistance}m");
            }

            if (distance <= pickupDistance)
            {
                // 拾取货物
                AttachCargo();
                currentState = CargoState.WaitingForDelivery;
                Debug.Log($"<color=green>[CargoMission]</color> ===== 货物已拾取 =====");
                Debug.Log($"<color=green>[CargoMission]</color> 当前状态: 等待投放 - 请飞向投放点（蓝色标记）");
                NotifyObjective("货物已拾取！请运送到目标点，按N键投放");
            }
        }

        void AttachCargo()
        {
            if (cargoObject == null) return;

            Debug.Log($"<color=cyan>[CargoMission]</color> 正在附着货物: {cargoObject.name}");

            // 方案1：设置为无人机的子物体
            cargoObject.transform.SetParent(droneTransform);

            // 记录局部偏移（通常在无人机下方）
            cargoLocalOffset = new Vector3(0f, -1f, 0f);
            cargoObject.transform.localPosition = cargoLocalOffset;
            cargoObject.transform.localRotation = Quaternion.identity;

            // 禁用物理（如果有 Rigidbody）
            Rigidbody rb = cargoObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = true;
                rb.useGravity = false;
                Debug.Log("<color=cyan>[CargoMission]</color> 货物物理已禁用");
            }

            cargoAttached = true;
            Debug.Log($"<color=green>[CargoMission]</color> 货物附着成功！本地偏移: {cargoLocalOffset}");
        }

        void UpdateCargoPosition()
        {
            // 如果货物被其他脚本（如DroneController.DropBox）投放
            if (cargoAttached && cargoObject != null && cargoObject.transform.parent != droneTransform)
            {
                currentState = CargoState.Carrying;
                cargoAttached = false;
            }
        }

        void CheckDelivery()
        {
            // 只计算水平距离（XZ平面，忽略高度）
            Vector3 dronePos = droneTransform.position;
            Vector3 targetPos = deliveryPoint.position;

            Vector2 dronePos2D = new Vector2(dronePos.x, dronePos.z);
            Vector2 targetPos2D = new Vector2(targetPos.x, targetPos.z);
            float horizontalDistance = Vector2.Distance(dronePos2D, targetPos2D);

            // 计算高度差（海拔）
            float heightDiff = dronePos.y - targetPos.y;
            float heightAbs = Mathf.Abs(heightDiff);

            // 每5秒显示一次状态提示
            if (Time.time - lastDeliveryCheckTime >= 5f)
            {
                lastDeliveryCheckTime = Time.time;

                bool distanceOK = horizontalDistance <= deliveryDistance;
                bool heightOK = heightAbs <= deliveryMaxHeight;

                string distanceStatus = distanceOK ? "<color=green>✓</color>" : "<color=red>✗</color>";
                string heightStatus = heightOK ? "<color=green>✓</color>" : "<color=red>✗</color>";

                Debug.Log($"<color=yellow>[CargoMission]</color> 投放条件: 水平距离 {distanceStatus} {horizontalDistance:F1}/{deliveryDistance}m | 高度差 {heightStatus} {heightAbs:F1}/{deliveryMaxHeight}m | 按N键投放");
            }

            // 检测按键投放（使用 InputManager 统一输入系统，边缘检测防止重复触发）
            bool currentNInput = InputManager.instance != null && InputManager.instance.NInput;
            bool nKeyPressed = currentNInput && !lastNInputState; // 边缘检测：当前帧按下，上一帧未按下
            lastNInputState = currentNInput;

            if (nKeyPressed && !isProcessingDrop && currentState != CargoState.Delivered)
            {
                isProcessingDrop = true; // 标记正在处理，防止重复
                Debug.Log($"<color=cyan>[CargoMission]</color> ===== 检测到N键按下（边缘触发） =====");
                Debug.Log($"<color=cyan>[CargoMission]</color> 按键时刻的实际距离:");
                Debug.Log($"<color=cyan>[CargoMission]</color>   - 水平距离: {horizontalDistance:F2}m (要求≤{deliveryDistance}m) - {(horizontalDistance <= deliveryDistance ? "✓" : "✗")}");
                Debug.Log($"<color=cyan>[CargoMission]</color>   - 高度差: {heightAbs:F2}m (要求≤{deliveryMaxHeight}m) - {(heightAbs <= deliveryMaxHeight ? "✓" : "✗")}");
                Debug.Log($"<color=cyan>[CargoMission]</color>   - 无人机位置: {dronePos}");
                Debug.Log($"<color=cyan>[CargoMission]</color>   - 目标位置: {targetPos}");

                if (horizontalDistance <= deliveryDistance && heightAbs <= deliveryMaxHeight)
                {
                    // 投放货物
                    DetachCargo();
                    currentState = CargoState.Delivered;
                    Debug.Log($"<color=green>[CargoMission]</color> ===== 货物投放成功 =====");
                    Debug.Log($"<color=green>[CargoMission]</color> 水平距离: {horizontalDistance:F1}m, 高度差: {heightDiff:F1}m");

                    // 计算精度分数（30pts满分）: 0-2m完美(30), 2-4m优秀(25), 4-6m良好(20), >6m及格(15)
                    float accuracyScore = 30f;
                    if (horizontalDistance <= 2f) accuracyScore = 30f;
                    else if (horizontalDistance <= 4f) accuracyScore = 25f;
                    else if (horizontalDistance <= 6f) accuracyScore = 20f;
                    else accuracyScore = 15f;

                    // 调用评分系统（如果存在）
                    if (MissionScoreManager.Instance != null)
                    {
                        MissionScoreManager.Instance.SetAccuracyScore(accuracyScore);
                        Debug.Log($"<color=cyan>[CargoMission]</color> 精度得分: {accuracyScore}/30 (距离: {horizontalDistance:F1}m)");

                        Debug.Log($"<color=cyan>[CargoMission]</color> 调用评分系统...");
                        MissionScoreManager.Instance.CompleteMission(MissionType.Cargo, referenceTime: 120f);
                        Debug.Log("<color=cyan>[CargoMission]</color> 评分计算完成");
                    }
                    else
                    {
                        Debug.LogWarning("<color=yellow>[CargoMission]</color> MissionScoreManager not found, skipping score calculation");
                    }

                    Debug.Log($"<color=green>[CargoMission]</color> 任务即将完成...");
                    NotifyObjective("货物投放成功！任务完成");

                    Debug.Log($"<color=cyan>[CargoMission]</color> 调用CompleteMission...");
                    CompleteMission();
                }
                else
                {
                    failedDropAttempts++;

                    string reason = "";
                    if (horizontalDistance > deliveryDistance)
                        reason += $"水平距离过远({horizontalDistance:F1}m > {deliveryDistance}m) ";
                    if (heightAbs > deliveryMaxHeight)
                        reason += $"高度差过大({heightAbs:F1}m > {deliveryMaxHeight}m)";

                    Debug.LogError($"<color=red>[CargoMission] ===== 投放失败 ({failedDropAttempts}/{maxDropAttempts}) =====</color>");
                    Debug.LogError($"<color=red>[CargoMission]</color> 原因: {reason}");
                    Debug.LogError($"<color=yellow>[CargoMission]</color> 当前状态: 距离={horizontalDistance:F1}m (需要≤{deliveryDistance}m), 高度差={heightAbs:F1}m (需要≤{deliveryMaxHeight}m)");

                    if (failedDropAttempts >= maxDropAttempts)
                    {
                        Debug.LogError($"<color=red>[CargoMission]</color> 投放尝试次数超过限制！任务失败！");
                        NotifyObjective($"任务失败：投放失败{maxDropAttempts}次");
                        FailMission($"投放失败{maxDropAttempts}次: {reason}");
                    }
                    else
                    {
                        NotifyObjective($"无法投放：{reason} (剩余{maxDropAttempts - failedDropAttempts}次机会)");
                    }
                }

                // 重置处理标记（在下一帧允许再次尝试）
                isProcessingDrop = false;
            }
        }

        void DetachCargo()
        {
            if (cargoObject == null)
            {
                Debug.LogWarning("[CargoMission] Cannot detach: cargo object is null!");
                return;
            }

            Debug.Log($"<color=cyan>[CargoMission]</color> 正在分离货物: {cargoObject.name}");

            // 解除父子关系
            cargoObject.transform.SetParent(null);

            // 启用物理
            Rigidbody rb = cargoObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;
                rb.useGravity = true;

                // 继承无人机的速度
                Rigidbody droneRb = droneTransform.GetComponent<Rigidbody>();
                if (droneRb != null)
                {
                    rb.velocity = droneRb.velocity;
                    Debug.Log($"<color=cyan>[CargoMission]</color> 货物继承速度: {droneRb.velocity}");
                }

                Debug.Log("<color=cyan>[CargoMission]</color> 货物物理已启用，开始下落");
            }

            cargoAttached = false;
            Debug.Log($"<color=green>[CargoMission]</color> 货物已成功分离！位置: {cargoObject.transform.position}");
        }

        void OnDrawGizmos()
        {
            // 拾取点
            if (pickupPoint != null)
            {
                Gizmos.color = pickupColor;
                Gizmos.DrawWireSphere(pickupPoint.position, gizmoRadius);
                Gizmos.DrawLine(pickupPoint.position, pickupPoint.position + Vector3.up * 5f);
            }

            // 投放点
            if (deliveryPoint != null)
            {
                Gizmos.color = deliveryColor;
                Gizmos.DrawWireSphere(deliveryPoint.position, gizmoRadius);
                Gizmos.DrawLine(deliveryPoint.position, deliveryPoint.position + Vector3.up * 5f);

                // 投放区域圆柱体
                Gizmos.color = new Color(deliveryColor.r, deliveryColor.g, deliveryColor.b, 0.3f);
                DrawWireCylinder(deliveryPoint.position, deliveryDistance, deliveryMaxHeight);
            }
        }

        void DrawWireCylinder(Vector3 center, float radius, float height)
        {
            int segments = 20;
            float angleStep = 360f / segments;

            for (int i = 0; i < segments; i++)
            {
                float angle1 = i * angleStep * Mathf.Deg2Rad;
                float angle2 = (i + 1) * angleStep * Mathf.Deg2Rad;

                Vector3 p1Bottom = center + new Vector3(Mathf.Cos(angle1) * radius, 0, Mathf.Sin(angle1) * radius);
                Vector3 p2Bottom = center + new Vector3(Mathf.Cos(angle2) * radius, 0, Mathf.Sin(angle2) * radius);
                Vector3 p1Top = p1Bottom + Vector3.up * height;
                Vector3 p2Top = p2Bottom + Vector3.up * height;

                Gizmos.DrawLine(p1Bottom, p2Bottom);
                Gizmos.DrawLine(p1Top, p2Top);
                Gizmos.DrawLine(p1Bottom, p1Top);
            }
        }
    }
}
