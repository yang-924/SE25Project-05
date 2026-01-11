using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public GameObject obj;
    public Vector3 offsetPosition; // 相机相对于目标的偏移位置

    void Start()
    {
        // 如果未手动赋值 obj，自动查找无人机
        if (obj == null)
        {
            // 方法1：通过 Tag 查找
            obj = GameObject.FindGameObjectWithTag("Player");
            if (obj != null)
            {
                Debug.Log("[CameraController] Auto-found drone by tag: Player");
            }
            else
            {
                // 方法2：通过 DroneController 组件查找
                DroneController controller = FindObjectOfType<DroneController>();
                if (controller != null)
                {
                    obj = controller.gameObject;
                    Debug.Log("[CameraController] Auto-found drone by DroneController component");
                }
            }
        }

        if (obj == null)
        {
            Debug.LogWarning("[CameraController] No target found! Camera will not follow anything. Make sure drone has 'Player' tag or DroneController component.");
        }
    }

    // Update is called once per frame
    void LateUpdate() // 改为 LateUpdate 以确保在目标移动后更新相机
    {
        if (obj != null)
        {
            transform.position = offsetPosition + obj.transform.position;
            transform.LookAt(obj.transform.position);
        }
    }
}
