using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class DroneController : MonoBehaviour
{
    // Start is called before the first frame update
    //public GameObject wing0;
    public GameObject mainDrone;
    public GameObject box;

    public LayerMask groundLayer;

    public GameObject[] wings = new GameObject[4];

    public float[] speeds = new float[4] { 100.0f, 100.0f, 100.0f, 100.0f };
    public Vector3 rotate = Vector3.zero;


    public float riseAccelaration = 9.81f;
    public float airResistence = 0.05f;
    public Vector3 moveAt = Vector3.zero;

    private Vector3 wingOffset;
    private float wingLength;
    private float wingInertia;
    private float bodyInertia;
    private Vector3 inertia;
    private Vector3 last_velocity = Vector3.zero;
    //private float hoverSpeed;
    private float clock = 0.0f;
    public float k = 0.0002f;
    public float M = 10.0f;
    public float maxSpeed = 15.0f;

    public float point_Kp;
    public float point_Kd;

    public float rotate_Kp;
    public float rotate_Kd;

    public float move_Kp;
    public float move_Kd;

    public float move_to_Kp;
    public float move_to_Kd;

    public float baisc_speed = 1f;
    public float spinSpeed = 45f;

    private Vector3 hoverPoint;
    private bool isHovering = false;
    private Vector3 rotatePoint;
    private bool isStopRotating = false;

    private bool isTakeoffEnabled = false;
    private bool[] keyPressed = new bool[4];
    private bool isGrounded = true;
    private bool isAutomaticTtakeoff = true;
    private Vector3 automaticTakeoffHoverPoint;
    private bool isAutomaticTakeoffPocessEnd = false;
    public float automaticTakeoffDistance = 9f;
    private bool isOperatingPocessEnd = false;
    public float automaticLandingDistance = 5f;
    private bool isAutomaticLandingPocessEnd = false;
    private Vector3 automaticLandingHoverPoint;
    private bool hasMove = false;
    private bool isDrop = false;
    private bool isDropEnd = false;
    public float dropGroundIgnoreTime = 10.0f;
    private float dropDoneTime = -999f;

    private Vector3 IDir = Vector3.zero;
    private Vector3 KDir = Vector3.zero;
    private Vector3 JDir = Vector3.zero;
    private Vector3 LDir = Vector3.zero;

    private float desiredYaw;
    public float yawRate = 0.01f;
    private float centerOffset = 0f;

    [Header("Weather Disturbance")]
    [Tooltip("风扰动力矩强度（模拟不均匀风力对机翼的影响）")]
    public float windTurbulenceStrength = 0.5f;
    [Tooltip("雨扰动力矩强度（模拟雨滴不均匀冲击）")]
    public float rainTurbulenceStrength = 1.0f;


    void Start()
    {
        if (wings[0] == null)
        {
            return;
        }
        else
        {
            var rb = GetComponent<Rigidbody>();
            rb.drag = 0.2f;
            rb.angularDrag = 0.4f;
            // calculate wing offset, size and inertia

            Vector3 position = transform.position;
            Vector3 scale = transform.localScale;
            desiredYaw = Mathf.Atan2(transform.forward.x, transform.forward.z);
            IDir = transform.forward;
            KDir = -transform.forward;
            JDir = -transform.right;
            LDir = transform.right;
            /*            Vector3 localPosition = wings[0].transform.localPosition;
                        Vector3 worldPosition = position + Vector3.Scale(localPosition, scale);
                        wingOffset = worldPosition - position;
                        Debug.Log(wingOffset);*/
            wingOffset = Vector3.Scale(
                wings[0].transform.localPosition,
                transform.localScale
            );
            Debug.Log(wingOffset);
            rotatePoint = transform.forward;
            Collider collider = wings[0].GetComponent<Collider>();
            if (collider != null)
            {
                Vector3 size = collider.bounds.size;
                wingLength = size.x;
            }
            else
            {
                wingLength = -1;
            }

            inertia = CalculateInertia();

            for (int i = 0; i < keyPressed.Length; i++)
            {
                keyPressed[i] = false;
            }
            automaticTakeoffHoverPoint = transform.position;
            automaticTakeoffHoverPoint.y += automaticTakeoffDistance;
            hoverPoint = automaticTakeoffHoverPoint;
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        clock += Time.deltaTime;

        Clear();

        if (!isAutomaticTakeoffPocessEnd)
        {
            automaticTakeoffPocess();
            return;
        }
        if (!isOperatingPocessEnd)
        {
            OperatingPocess();
            return;
        }
        if (!isAutomaticLandingPocessEnd)
        {
            AutomaticLandingPocess();
            return;
        }
    }

    float CalculateTheta(Vector2 vec1, Vector2 vec2)
    {
        if (vec1.magnitude == 0.0f || vec2.magnitude == 0.0f)
        {
            return 0.0f;
        }

        Vector2 vec = new Vector2(1.0f, 0.0f);

        float theta1 = Mathf.Acos(Vector2.Dot(vec1, vec) / (vec1.magnitude * vec.magnitude)) * Mathf.Rad2Deg;
        if (vec1.y < 0.0f) theta1 = 360.0f - theta1;
        float theta2 = Mathf.Acos(Vector2.Dot(vec2, vec) / (vec2.magnitude * vec.magnitude)) * Mathf.Rad2Deg;
        if (vec2.y < 0.0f) theta2 = 360.0f - theta2;

        float delta_theta = theta1 - theta2;
        if (Mathf.Abs(delta_theta) > 180.0f)
        {
            if (delta_theta < 0.0f)
            {
                delta_theta += 360.0f;
            }
            else
            {
                delta_theta -= 360.0f;
            }
        }

        return delta_theta * Mathf.Deg2Rad;
    }


    void MoveToWithVelocity(Vector3 position, Vector3 velocity)
    {
        Rigidbody rb = GetComponent<Rigidbody>();

        Vector3 position_error = position - transform.position;
        Vector3 localVelocity = rb.velocity;
        //Vector3 velocity_error = velocity - rb.velocity;
        Vector3 velocity_error = velocity - localVelocity;

        float Kp = move_to_Kp;
        float Kd = move_to_Kd;

        Vector3 expect_velocity = (Kp * position_error + Kd * velocity_error) + velocity;


        MoveAt(expect_velocity);
    }

    void MoveAt(Vector3 velocity)
    {
        if (velocity.magnitude > maxSpeed)
        {
            velocity = velocity.normalized * maxSpeed;
        }

        moveAt = velocity;

        Rigidbody rb = GetComponent<Rigidbody>();

        //Vector3 velocity_error = velocity - rb.velocity;
        Vector3 localVelocity = rb.velocity;
        //Debug.Log("local v: "+localVelocity);

        Vector3 velocity_error = velocity - localVelocity;
        Vector3 acceleration_error = -(rb.velocity - last_velocity) / Time.deltaTime;

        float Kp = move_Kp;
        float Kd = move_Kd;

        Vector3 acc = Kp * velocity_error + Kd * acceleration_error;
        if (acc.magnitude > 10.0f)
        {
            acc = acc.normalized * 10.0f;
        }

        //Vector3 real_acc = acc - Physics.gravity + airResistence * localVelocity.normalized;
        Vector3 real_acc = acc - Physics.gravity;
        RotateTo(real_acc);
        riseAccelaration = real_acc.magnitude;

        //Debug.Log(rb.velocity);
    }

    void PointTo(Vector3 forward)
    {
        Rigidbody rb = GetComponent<Rigidbody>();

        float angular_error_y = -CalculateTheta(new Vector2(forward.x, forward.z), new Vector2(transform.forward.x, transform.forward.z));

        float angular_velocity_error_y = -rb.angularVelocity.y;

        float Kp = point_Kp;
        float Kd = point_Kd;

        float rotate_y = Kp * angular_error_y + Kd * angular_velocity_error_y;
        this.rotate.y = rotate_y;
    }

    void RotateTo(Vector3 up)
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 localUp = up;
        Vector3 droneUp = transform.up;

        float angular_error_x = CalculateTheta(new Vector2(localUp.z, localUp.y), new Vector2(droneUp.z, droneUp.y));

        float angular_error_z = -CalculateTheta(new Vector2(localUp.x, localUp.y), new Vector2(droneUp.x, droneUp.y));

        Vector3 angular_error = new Vector3(angular_error_x, 0.0f, angular_error_z);
        //Vector3 local_angular_error = GetWorldVector(angular_error);
        //Debug.Log("error z" + angular_error_z);

        float angular_velocity_error_x = rb.angularVelocity.x;
        float angular_velocity_error_z = rb.angularVelocity.z;

        Vector3 angular_velocity_error = new Vector3(angular_velocity_error_x, 0.0f, angular_velocity_error_z);
        //Debug.Log("v error z" + angular_velocity_error_z);

        float Kp = rotate_Kp;
        float Kd = rotate_Kd;

        Vector3 rotate_vec = Kp * angular_error + Kd * angular_velocity_error;
        float rotate_x = Kp * angular_error_x + Kd * angular_velocity_error_x;
        float rotate_z = Kp * angular_error_z + Kd * angular_velocity_error_z;



        this.rotate.x = rotate_vec.x;
        this.rotate.z = rotate_vec.z;
    }

    void YawRateControl(float targetYawRate)
    {
        Rigidbody rb = GetComponent<Rigidbody>();

        float yawRateError = targetYawRate - rb.angularVelocity.y;

        float Kp = rotate_Kp;
        float Kd = rotate_Kd;

        float yawAccelCmd = Kp * yawRateError - Kd * rb.angularVelocity.y;

        rotate.y = yawAccelCmd;
    }
    void UpdateHorizontalDirs()
    {
        Vector3 forwardFlat = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
        Vector3 rightFlat = Vector3.ProjectOnPlane(transform.right, Vector3.up).normalized;

        IDir = forwardFlat;
        KDir = -forwardFlat;
        JDir = -rightFlat;
        LDir = rightFlat;
        /*        Debug.Log("test");
                Debug.Log(IDir);

                Debug.Log(transform.forward);*/
    }

    void Clear()
    {
        this.riseAccelaration = 0;
        this.rotate = Vector3.zero;
    }

    void Rise(float acceleration)
    {
        this.riseAccelaration = acceleration;
    }

    void Rotate(Vector3 rotate)
    {
        this.rotate = rotate;
    }

    float Speed2Force(float speed)
    {
        float forceDirection = (speed > 0.0f) ? 1 : -1;
        float force = forceDirection * k * Mathf.Pow(speed, 2);
        return force;
    }
    float Force2Speed(float force)
    {
        float forceDirection = ((force > 0.0f) ? 1 : -1);
        float speed = forceDirection * Mathf.Sqrt(Mathf.Abs(force) / k);
        return speed;
    }

    void ApplyAirResistance()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 velocity = rb.velocity;

        // --- Weather Simulation Integration ---
        Vector3 windVelocity = Vector3.zero;
        float rainIntensity = 0f;

        // Check if WeatherManager exists
        if (WeatherManager.Instance != null)
        {
            windVelocity = WeatherManager.Instance.GetWindAtPosition(transform.position);
            rainIntensity = WeatherManager.Instance.rainIntensity;
        }

        // 1. Aerodynamics (Wind + Resistance)
        // Calculate relative air speed (Wind - Drone Velocity)
        Vector3 relativeVelocity = windVelocity - velocity;

        // Force = Coefficient * v * |v|
        // This calculates both drag (when moving against still air) and wind push (when air moves against drone)
        Vector3 airForce = airResistence * relativeVelocity * relativeVelocity.magnitude;
        rb.AddForce(airForce, ForceMode.Force);

        // 2. Rain Effects (位置影响很小，主要用于视觉反馈)
        if (rainIntensity > 0)
        {
            // Rain impact force (downward with noise) - 减弱位置影响
            float rainForce = rainIntensity * 2.0f * Random.Range(0.8f, 1.2f);
            rb.AddForce(Vector3.down * rainForce, ForceMode.Force);
        }

        // 3. Weather-Induced Attitude Disturbance (姿态扰动)
        // 模拟风和雨对不同机翼的不均匀作用，导致晃动但不漂移
        float totalDisturbance = 0f;

        // 风扰动：基于风速计算
        if (windVelocity.magnitude > 0.1f)
        {
            // 风速越大，扰动越强
            float windDisturbance = windVelocity.magnitude * windTurbulenceStrength;
            totalDisturbance += windDisturbance;
        }

        // 雨扰动：基于雨强度
        if (rainIntensity > 0.01f)
        {
            float rainDisturbance = rainIntensity * rainTurbulenceStrength;
            totalDisturbance += rainDisturbance;
        }

        // 应用随机力矩（模拟某个机翼受力突变）
        if (totalDisturbance > 0)
        {
            // 使用Perlin噪声生成更自然的扰动
            float time = Time.time * 2.0f; // 扰动频率
            float noiseX = Mathf.PerlinNoise(time, 0f) - 0.5f;
            float noiseY = Mathf.PerlinNoise(time + 100f, 0f) - 0.5f;
            float noiseZ = Mathf.PerlinNoise(time + 200f, 0f) - 0.5f;

            Vector3 disturbanceTorque = new Vector3(noiseX, noiseY, noiseZ) * totalDisturbance;
            rb.AddTorque(disturbanceTorque, ForceMode.Force);
        }
    }
    void ApplyForce()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        //float p = 0.1f;
        //rb.AddForce(-p*rb.velocity,ForceMode.Force);
        for (int i = 0; i < wings.Length; i++)
        {
            //RotateAt(wings[i], speeds[i]);
            float torque = M * wingOffset.magnitude * Mathf.Pow(speeds[i], 2);
            float direction = (speeds[i] > 0 ? 1 : -1) * ((i % 2 == 0) ? 1 : -1);
            rb.AddTorque(torque * direction * transform.up, ForceMode.Force);
            //Debug.Log(torque * direction * transform.up);
            Vector3 force = transform.up.normalized * Speed2Force(speeds[i]);
            //Debug.Log(force);
            wings[i].GetComponent<Rigidbody>().AddForce(force, ForceMode.Force);
        }
    }

    void ApplyVision()
    {
        for (int i = 0; i < wings.Length; i++)
        {
            float direction = (i % 2 == 0) ? 1 : -1;
            Rigidbody rb = wings[i].GetComponent<Rigidbody>();
            rb.angularVelocity = speeds[i] * transform.up * direction;
            //rb.inertiaTensor = new Vector3(rb.inertiaTensor.x,rb.mass * wingLength * wingLength / 12,rb.inertiaTensor.z);
        }
    }

    void automaticTakeoff()
    {
        MoveToWithVelocity(automaticTakeoffHoverPoint, Vector3.zero);
    }

    void automaticTakeoffPocess()
    {
        if (isGrounded)
        {
            CheckTakeoffKeys();
        }
        if (!isTakeoffEnabled)
        {
            return;
        }
        automaticTakeoff();
        Solve();
        ApplyAirResistance();
        ApplyForce();
        ApplyVision();
        if (transform.position.y >= (automaticTakeoffHoverPoint.y - 2f))
        {
            isAutomaticTakeoffPocessEnd = true;
            isOperatingPocessEnd = false;
        }
    }

    void OperatingPocess()
    {
        UpdateHorizontalDirs();
        Vector3 directionXYZ = Vector3.zero;
        rotate = Vector3.zero;
        Vector3 rotateXYZ = Vector3.zero;
        float yawInput = 0f;
        if (InputManager.instance.WInput) directionXYZ += transform.up;
        if (InputManager.instance.SInput) directionXYZ -= transform.up;
        if (InputManager.instance.AInput) yawInput = -1f;
        if (InputManager.instance.DInput) yawInput = 1f;
        if (InputManager.instance.IInput) directionXYZ += IDir;
        if (InputManager.instance.KInput) directionXYZ += KDir;
        if (InputManager.instance.JInput) directionXYZ += JDir;
        if (InputManager.instance.LInput) directionXYZ += LDir;
        if (InputManager.instance.NInput) isDrop = true;
        /*        if (Input.GetKey(KeyCode.I)) directionXYZ += IDir;
                if (Input.GetKey(KeyCode.K)) directionXYZ += KDir;
                if (Input.GetKey(KeyCode.J)) directionXYZ += JDir;
                if (Input.GetKey(KeyCode.L)) directionXYZ += LDir;
                if (Input.GetKey(KeyCode.W)) directionXYZ += transform.up;
                if (Input.GetKey(KeyCode.S)) directionXYZ -= transform.up;
                if (Input.GetKey(KeyCode.A)) yawInput = -1f;
                if (Input.GetKey(KeyCode.D)) yawInput = 1f;
                if (Input.GetKey(KeyCode.N)) isDrop = true;*/
        desiredYaw += yawInput * yawRate * Time.deltaTime;
        hasMove = directionXYZ != Vector3.zero;
        //bool hasRotate = rotateXYZ != Vector3.zero; ;
        bool hasRotate = yawInput != 0;
        float maxYawRate = 1.0f;
        if (!hasMove)
        {
            if (!isHovering)
            {
                hoverPoint = transform.position;
                isHovering = true;
            }

            MoveToWithVelocity(hoverPoint, Vector3.zero);
        }
        else
        {
            isHovering = false;

            Vector3 v = baisc_speed * directionXYZ.normalized;
            MoveAt(v);
        }

        if (!hasRotate)
        {
            if (!isStopRotating)
            {
                rotatePoint = IDir;
                isStopRotating = true;
            }
            PointTo(rotatePoint);

        }
        else
        {
            isStopRotating = false;
            /*            Vector3 yawForward = new Vector3(Mathf.Sin(desiredYaw), 0f, Mathf.Cos(desiredYaw));
                        PointTo(yawForward);
                        Debug.Log("自旋");
                        Debug.Log(yawForward);
                        Debug.Log(rotatePoint);*/
            //transform.Rotate(rotateXYZ * spinSpeed * Time.deltaTime, Space.Self);
            YawRateControl(yawInput * maxYawRate);

        }
        if (isDrop)
        {
            DropBox();

        }

        Solve();
        ApplyAirResistance();
        ApplyForce();
        ApplyVision();
        if (isDropEnd)
        {
            if (Time.time - dropDoneTime > dropGroundIgnoreTime)
                CheckGroundDistance();
        }


    }

    void CheckGroundDistance()
    {
        RaycastHit hit;
        Vector3 rayOrigin = transform.position;
        rayOrigin.y += centerOffset;
        if (Physics.Raycast(rayOrigin, Vector3.down, out hit, automaticLandingDistance, groundLayer))
        {
            isOperatingPocessEnd = true;
            automaticLandingHoverPoint = transform.position;
            automaticLandingHoverPoint.y = automaticLandingHoverPoint.y - hit.distance;
            isAutomaticLandingPocessEnd = false;
        }

    }

    void AutomaticLandingPocess()
    {
        MoveToWithVelocity(automaticLandingHoverPoint, Vector3.zero);
        Solve();
        ApplyAirResistance();
        ApplyForce();
        ApplyVision();
        if ((transform.position.y) <= automaticLandingHoverPoint.y + 0.3)
        {
            isAutomaticLandingPocessEnd = true;
            NewProcess();

        }
    }

    void NewProcess()
    {
        isAutomaticTakeoffPocessEnd = false;
        automaticTakeoffHoverPoint = transform.position;
        automaticTakeoffHoverPoint.y += automaticTakeoffDistance;
        hoverPoint = automaticTakeoffHoverPoint;
        for (int i = 0; i < keyPressed.Length; i++)
        {
            keyPressed[i] = false;
        }
        isGrounded = true;
        isTakeoffEnabled = false;
    }

    void DropBox()
    {
        if (isDropEnd)
        {
            return;
        }
        if (box == null)
        {
            return;
        }
        Rigidbody rb = box.GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = box.AddComponent<Rigidbody>();
        }

        rb.useGravity = true;
        rb.isKinematic = false;
        BoxCollider boxCol = box.GetComponent<BoxCollider>();
        if (boxCol == null)
        {
            boxCol = box.AddComponent<BoxCollider>();
            boxCol.size = new Vector3(0.09f, 0.04f, 0.07f);
        }

        float boxHeight = boxCol.size.y;


        BoxCollider droneCol = mainDrone.GetComponent<BoxCollider>();
        if (droneCol != null)
        {
            float oldSizeY = droneCol.size.y;
            float newSizeY = Mathf.Max(0.01f, oldSizeY - boxHeight);

            centerOffset = (oldSizeY - newSizeY) * 0.5f;

            droneCol.size = new Vector3(
                droneCol.size.x,
                newSizeY,
                droneCol.size.z
            );

            droneCol.center += new Vector3(0f, centerOffset, 0f);
        }

        Rigidbody droneRb = GetComponent<Rigidbody>();
        if (droneRb != null)
        {
            rb.velocity = droneRb.velocity;
            rb.angularVelocity = droneRb.angularVelocity;
        }
        isDropEnd = true;
        dropDoneTime = Time.time;
    }


    void Hover()
    {
        Vector3 up = Vector3.up;
        Vector3 droneUp = transform.up;

        float g = Mathf.Abs(Physics.gravity.y);
        float cosine = Vector3.Dot(up, droneUp) / (up.magnitude * droneUp.magnitude);
        float a = g / cosine;

        Rise(a);
    }

    private void CheckTakeoffKeys()
    {

        /*keyPressed[0] = Input.GetKey(KeyCode.S);  
        keyPressed[1] = Input.GetKey(KeyCode.D);  
        keyPressed[2] = Input.GetKey(KeyCode.J);  
        keyPressed[3] = Input.GetKey(KeyCode.K);*/
        keyPressed[0] = InputManager.instance.SInput;
        keyPressed[1] = InputManager.instance.DInput;
        keyPressed[2] = InputManager.instance.JInput;
        keyPressed[3] = InputManager.instance.KInput;

        bool allKeysPressed = true;
        for (int i = 0; i < keyPressed.Length; i++)
        {
            if (!keyPressed[i])
            {
                allKeysPressed = false;
                break;
            }
        }

        if (allKeysPressed)
        {
            if (!isTakeoffEnabled)
            {
                isTakeoffEnabled = true;
                isGrounded = false;
                Debug.Log("起飞已启用！");

            }
        }

    }

    Vector3 CalculateInertia()
    {
        Vector3 inertia = GetComponent<Rigidbody>().inertiaTensor;

        return inertia;
    }

    void Solve()    // rotate & riseAcceleratoin => speeds
    {
        Vector3 rotateBody = transform.InverseTransformDirection(rotate);
        float tanAlpha = Mathf.Abs(wingOffset.z / wingOffset.x);
        float tanAlpha_pow = tanAlpha * tanAlpha;
        float sinAlpha = Mathf.Sqrt(tanAlpha_pow / (1 + tanAlpha_pow));
        float cosAlpha = Mathf.Sqrt(1 / (1 + tanAlpha_pow));
        float distance = wingOffset.magnitude;

        float[] forces = new float[4] { 0.0f, 0.0f, 0.0f, 0.0f };
        float[] k = new float[4];
        //Debug.Log(inertia);
        /*        k[0] = GetComponent<Rigidbody>().mass * riseAccelaration;
                k[1] = (inertia.x * rotate.x) / (distance * sinAlpha);
                k[2] = (inertia.z * rotate.z) / (distance * cosAlpha);
                k[3] = (inertia.y * rotate.y) * this.k / (distance * M);*/
        k[0] = GetComponent<Rigidbody>().mass * riseAccelaration;
        k[1] = (inertia.x * rotateBody.x) / (distance * sinAlpha);
        k[2] = (inertia.z * rotateBody.z) / (distance * cosAlpha);
        k[3] = (inertia.y * rotateBody.y) * this.k / (distance * M);
        /*        Debug.Log("test");
                Debug.Log(rotate);
                Debug.Log(k[0]);
                Debug.Log(k[1]);
                Debug.Log(k[2]);
                Debug.Log(k[3]);*/
        forces[0] = (k[0] + k[1] - k[2] + k[3]) / 4;
        forces[1] = (k[0] - k[1] - k[2] - k[3]) / 4;
        forces[2] = (k[0] - k[1] + k[2] + k[3]) / 4;
        forces[3] = (k[0] + k[1] + k[2] - k[3]) / 4;
        /*        Debug.Log("k");
                Debug.Log(forces[0]);
                Debug.Log(forces[1]);
                Debug.Log(forces[2]);
                Debug.Log(forces[3]);*/
        /*        for (int i = 0; i < 4; i++)
                {
                    forces[i] = Mathf.Max(forces[i], 0f);

                }*/
        for (int i = 0; i < 4; i++)
        {
            speeds[i] = Force2Speed(forces[i]);
        }

        last_velocity = GetComponent<Rigidbody>().velocity;
    }





}

