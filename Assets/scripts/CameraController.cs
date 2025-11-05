using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public GameObject obj;
    public Vector3 offsetPosition; // 相机相对于目标的偏移位置
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = offsetPosition+obj.transform.position;
        transform.LookAt(obj.transform.position);
    }
}
