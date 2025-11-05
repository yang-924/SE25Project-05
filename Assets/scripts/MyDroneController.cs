using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class MyDroneController : MonoBehaviour
{
    // Start is called before the first frame update
    //public GameObject wing0;

    public GameObject[] wings = new GameObject[4];
    //public GameObject wing1;
    //public GameObject wing2;
    //public GameObject wing3;
    //public GameObject wing4;
    public float[] speeds = new float[4] { 100.0f, 100.0f, 100.0f, 100.0f };
    public Vector3 rotate = Vector3.zero;
    //public float wing1_speed = 100.0f;
    //public float wing2_speed = 100.0f;
    //public float wing3_speed = 100.0f;
    //public float wing4_speed = 100.0f;

    public float riseAccelaration = 9.81f;

    private Vector3 wingOffset;
    private float wingLength;
    private float wingInertia;
    private float bodyInertia;
    private Vector3 inertia;
    //private float hoverSpeed;
    private float clock = 0.0f;
    public float k = 0.0001f;
    public float M = 10.0f;

    //Rigidbody wing1_rigid;
    //Rigidbody wing2_rigid;
    //Rigidbody wing3_rigid;
    //Rigidbody wing4_rigid;

    public Quaternion originAngle;
    public Quaternion currentAngle;
    public float angleDiffX;
    public float angleDiffY;
    public float angleDiffZ;

    public float setAngle = 0.3f;
    public float maxSpeedAdjustment = 1f;


    //每个方向使用自己的count和error：自体旋转 = 0,垂直旋转 = 1,水平旋转 = 2
    public int[] count = new int[3] { 0, 0, 0 };
    public float[] error_total = new float[3] { 0, 0, 0 };
    public float[] error_last = new float[3] { 0,0,0};
    //每个方向的state：front = 0,back = 1,left = 2,right = 3，reset：4
    public int stateH = 4;
    public int stateV = 4;
    public int resetState = 4;

    public float airResistence = 0.05f;

    Vector3 currentEulerAngle = Vector3.zero;

    /*    int count0 = 0;
        public float error_total0 = 0f;
        public float error_last0 = 0f;*/

    void Start()
    {
        if (wings[0] == null)
        {
            return;
        }
        else
        {
            // calculate wing offset, size and inertia
            Vector3 position = transform.position;
            Vector3 scale = transform.localScale;
            originAngle = transform.rotation;
            Vector3 localPosition = wings[0].transform.localPosition;
            Vector3 worldPosition = position + Vector3.Scale(localPosition, scale);
            wingOffset = worldPosition - position;

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

            //float WingMass = wings[0].GetComponent<Rigidbody>().mass;
            //wingInertia = WingMass * Mathf.Pow(wingOffset.magnitude, 2) + WingMass * Mathf.Pow(wingLength, 2) / 12;

            //float droneBodyMass = GetComponent<Rigidbody>().mass;
            //float droneBodySize = GetComponent<Collider>().bounds.size.x;
            //bodyInertia = droneBodyMass * Mathf.Pow(droneBodySize, 2) / 2;  // take drone as a plate
        }
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 velocity=GetComponent<Rigidbody>().velocity;
        Vector3 angle = transform.up;
    }

    private void FixedUpdate()
    {
        clock += Time.deltaTime;
        //Clear();

        //Rise(riseAccelaration);
        //Vector2 directionXZ = GetComponent<FindPath>().directionXZ;
        //Rotate(directionXZ);
        Hover();

        Solve();
        ApplyAirResistance();
        ApplyForce();
        ApplyVision();
        /*Debug.Log(clock);
        Debug.Log(GetComponent<Rigidbody>().angularVelocity * Mathf.Rad2Deg);*/
        //ApplySpeed();
        //ApplyAngularMomentum();
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

    void Rotate(Vector2 rotate)
    {
        //以相机视角，水平15°为向左运动，机身逆时针旋转15°，水平345°为向右运动，机身顺时针旋转15°
        //以相机视角，垂直15°为向前运动，机头下低旋转15°，垂直345°为向后运动，机头上抬旋转15°
        //以相机视角，从上往下看，自旋15°为逆时针旋转15°，自旋345°为顺时针旋转15°
        GetEulerAngleDiff();
        // this.rotate = HorizontalController(15f);
        // this.rotate = VerticalController(345f);
        //this.rotate = SpinController(345f);
        print(rotate);
        this.rotate = Vector3.zero;
        if (rotate.y < 0)
            //if (Input.GetKey(KeyCode.A))
        {
            if (stateH != 2)
            {
                clearError(2);
                stateH = 2;
            }
            this.rotate += HorizontalController(15f);
        }
        if (rotate.y > 0)
           // if (Input.GetKey(KeyCode.D))
        {
            if (stateH != 1)
            {
                clearError(2);
                stateH = 1;
            }
            this.rotate += HorizontalController(345f);
        }
        if (rotate.x > 0)
           // if (Input.GetKey(KeyCode.W))
        {
            if (stateV != 2)
            {
                clearError(1);
                stateV = 2;
            }
            this.rotate += VerticalController(15f);

        }
        if (rotate.x < 0)
           // if (Input.GetKey(KeyCode.S))
        {
            if (stateV != 1)
            {
                clearError(1);
                stateV = 1;
            }
            this.rotate += VerticalController(345f); ;

        }
        if (Input.GetKey(KeyCode.J))
        {
            clearError(0);
            this.rotate += SpinController(345f);
        }
        if (Input.GetKey(KeyCode.L))
        {
            clearError(0);
            this.rotate += SpinController(15f);
        }
        if (!Input.anyKey)
        {
            // ResetState();
        }
        //this.rotate = RightController(0.15f);
        //this.rotate = FrontController(0.15f);
        //this.rotate = BackController(0.15f);
        //this.rotate = anticlockwiseController(0.5f);
        //this.rotate = clockwiseController(0.5f);
    }

    /*void ResetState()
    {
        this.rotate = Vector3.zero;
        if (state != 4)
        {
            clearError(2);
            clearError(1);
            state = 4;
        }
        *//* if ((state == 0 || state == 1) && resetState == 4)
         {
             clearError(1);
             resetState = 1;
         }
         if (resetState == 2)
         {
             state = 4;
             this.rotate += HorizontalController(0f);
         }
         if (resetState == 1)
         {
             state = 4;
             this.rotate += VerticalController(0f);
         }*//*
        if (currentAngle.x != 0)
        {
            this.rotate += VerticalController(0f);
        }
        if (currentAngle.z != 0)
        {
            Debug.Log(111);
            this.rotate += HorizontalController(0f);
        }

    }*/

    void clearError(int n) 
    {
            count[n] = 0;
            error_last[n] = 0;
            error_total[n] = 0;

    }


    void GetEulerAngleDiff()
    {
        currentEulerAngle = transform.rotation.eulerAngles;

    }

    Vector3 HorizontalController(float setAngle)
    {
        float currentEulerAngleZ = currentEulerAngle.z;
        float clockwiseAngle = 0f;
        float anticlockwiseAngle = 0f;
        bool turnClockwise = false;
        bool turnAntiClockwise = false;
        float angleDiff = 0f;
        if (currentEulerAngleZ < setAngle){
            anticlockwiseAngle = setAngle - currentEulerAngleZ;
            clockwiseAngle = 360f - (setAngle - currentEulerAngleZ);
        } else {
            anticlockwiseAngle = 360f - (currentEulerAngleZ - setAngle);
            clockwiseAngle = currentEulerAngleZ - setAngle;
        }
        if (clockwiseAngle > anticlockwiseAngle)
        {
            turnClockwise = true;
            angleDiff = anticlockwiseAngle;
            // Debug.Log(true);
        } else {
            turnAntiClockwise = true;
            angleDiff = clockwiseAngle;
            // Debug.Log(false);
        }
        // Debug.Log(angleDiff);
        float proportional = (angleDiff / 180f) * maxSpeedAdjustment * 5;


        error_total[2] += angleDiff;
        float integral = error_total[2] ;
        float derivative = 0;
        if (count[2] != 0)
        {
            derivative = (angleDiff - error_last[2])* 2;

        }
        count[2]++;
        error_last[2] = angleDiff;

        //float adjustment = proportional + derivative + integral;
        float adjustment = proportional + derivative;
        // Debug.Log(proportional);
                // Debug.Log(integral);
                // Debug.Log(derivative);
        if (turnClockwise == true)
        {
            // Correcting forward tilt
            return new Vector3(adjustment, 0, 0);
        }
        if (turnAntiClockwise == true)
        {
            // Correcting backward tilt
            return new Vector3(-adjustment, 0, 0);
        }
        return new Vector3(0, 0, 0);
    }


    Vector3 VerticalController(float setAngle)
    {
        float currentEulerAngleX = currentEulerAngle.x;

        float clockwiseAngle = 0f;
        float anticlockwiseAngle = 0f;
        bool turnClockwise = false;
        bool turnAntiClockwise = false;
        float angleDiff = 0f;
        if (currentEulerAngleX < setAngle)
        {
            anticlockwiseAngle = setAngle - currentEulerAngleX;
            clockwiseAngle = 360f - (setAngle - currentEulerAngleX);
        }
        else
        {
            anticlockwiseAngle = 360f - (currentEulerAngleX - setAngle);
            clockwiseAngle = currentEulerAngleX - setAngle;
        }
        if (clockwiseAngle > anticlockwiseAngle)
        {
            turnClockwise = true;
            angleDiff = anticlockwiseAngle;
           // Debug.Log(true);
        }
        else
        {
            turnAntiClockwise = true;

            angleDiff = clockwiseAngle;
            //Debug.Log(false);
        }
        // Debug.Log(angleDiff);
        float proportional = (angleDiff / 180f) * maxSpeedAdjustment * 5;


        error_total[1] += angleDiff;
        float integral = error_total[1];
        float derivative = 0;
        if (count[1] != 0)
        {
            derivative = (angleDiff - error_last[1]) * 2;

        }
        count[1]++;
        error_last[1] = angleDiff;

        //float adjustment = proportional + derivative + integral;
        float adjustment = proportional + derivative;
        // Debug.Log(proportional);
        // Debug.Log(integral);
        // Debug.Log(derivative);
        if (turnClockwise == true)
        {
            // Correcting forward tilt
            return new Vector3(0, 0, -adjustment);
        }
        if (turnAntiClockwise == true)
        {
            // Correcting backward tilt
            return new Vector3(0, 0, adjustment);
        }
        return new Vector3(0, 0, 0);
    }


    Vector3 SpinController(float setAngle)
    {
        float currentEulerAngleY = currentEulerAngle.y;
        float clockwiseAngle = 0f;
        float anticlockwiseAngle = 0f;
        bool turnClockwise = false;
        bool turnAntiClockwise = false;
        float angleDiff = 0f;
        if (currentEulerAngleY < setAngle)
        {
            anticlockwiseAngle = setAngle - currentEulerAngleY;
            clockwiseAngle = 360f - (setAngle - currentEulerAngleY);
        }
        else
        {
            anticlockwiseAngle = 360f - (currentEulerAngleY - setAngle);
            clockwiseAngle = currentEulerAngleY - setAngle;
        }
        if (clockwiseAngle > anticlockwiseAngle)
        {
            turnClockwise = true;
            angleDiff = anticlockwiseAngle;
            // Debug.Log(true);
        }
        else
        {
            turnAntiClockwise = true;
            angleDiff = clockwiseAngle;
            // Debug.Log(false);
        }
        // Debug.Log(angleDiff);
        float proportional = (angleDiff / 180f) * maxSpeedAdjustment * 5;


        error_total[0] += angleDiff;
        float integral = error_total[0];
        float derivative = 0;
        if (count[0] != 0)
        {
            derivative = (angleDiff - error_last[0]) * 2;

        }
        count[0]++;
        error_last[0] = angleDiff;

        //float adjustment = proportional + derivative + integral;
        float adjustment = proportional + derivative;
        // Debug.Log(proportional);
        // Debug.Log(integral);
        // Debug.Log(derivative);
        if (turnClockwise == true)
        {
            // Correcting forward tilt
            return new Vector3(0, adjustment, 0);
        }
        if (turnAntiClockwise == true)
        {
            // Correcting backward tilt
            return new Vector3(0, -adjustment, 0);
        }
        return new Vector3(0, 0, 0);
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


    void ApplyForce()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        for (int i = 0; i < wings.Length; i++)
        {
            //RotateAt(wings[i], speeds[i]);
            float torque = M * wingOffset.magnitude * Mathf.Pow(speeds[i], 2);
            float direction = (speeds[i] > 0 ? 1 : -1) * ((i % 2 == 0) ? 1 : -1);
            rb.AddTorque(torque * direction * transform.up, ForceMode.Force);
            //Debug.Log(torque * direction * transform.up);
            Vector3 force = transform.up.normalized * Speed2Force(speeds[i]);
            /*Debug.Log(force);*/
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

    void Hover()
    {
        Vector3 up = Vector3.up;
        Vector3 droneUp = transform.up;

        float g = Mathf.Abs(Physics.gravity.y);
        float cosine = Vector3.Dot(up, droneUp) / (up.magnitude * droneUp.magnitude);
        float a = g / cosine;

        Rise(a);
    }

    //void RotateAt(GameObject obj, float rotateSpeed)
    //{
    //    Rigidbody rigidbody = obj.GetComponent<Rigidbody>();
    //    rigidbody.angularVelocity = transform.up * rotateSpeed;
    //    rigidbody.AddForce(transform.up.normalized * Speed2Force(rotateSpeed), ForceMode.Force);
    //}

    //void ApplySpeed()
    //{
    //    for (int i = 0; i < wings.Length; i++)
    //    {
    //        RotateAt(wings[i], speeds[i]);
    //    }
    //}

    //void ApplyAngularMomentum()
    //{
    //    float totalWingsAngularMomentum = 0;
    //    for (int i = 0; i < wings.Length; i++)
    //    {
    //        float direction = (wings[i].transform.eulerAngles.x < 180) ? 1 : -1;    // if the wing spin counter-clockwise.
    //        float angularSpeed = speeds[i] * Mathf.Deg2Rad;
    //        totalWingsAngularMomentum += wingInertia * direction * angularSpeed;
    //    }

    //    float droneBodyAngularSpeed = -totalWingsAngularMomentum / bodyInertia;
    //    Rigidbody rb = GetComponent<Rigidbody>();
    //    rb.angularVelocity = transform.up * droneBodyAngularSpeed * Mathf.Rad2Deg;
    //}

    Vector3 CalculateInertia()
    {
        Vector3 inertia = GetComponent<Rigidbody>().inertiaTensor;

        return inertia;
    }

    void Solve()    // rotate & riseAcceleratoin => speeds
    {
        float tanAlpha = Mathf.Abs(wingOffset.z / wingOffset.x);
        float tanAlpha_pow = tanAlpha * tanAlpha;
        float sinAlpha = Mathf.Sqrt(tanAlpha_pow / (1 + tanAlpha_pow));
        float cosAlpha = Mathf.Sqrt(1 / (1 + tanAlpha_pow));
        float distance = wingOffset.magnitude;

        float[] forces = new float[4] { 0.0f, 0.0f, 0.0f, 0.0f };
        float[] k = new float[4];
        //Debug.Log(inertia);
        k[0] = GetComponent<Rigidbody>().mass * riseAccelaration;
        k[1] = (inertia.x * rotate.x) / (distance * sinAlpha);
        k[2] = (inertia.z * rotate.z) / (distance * cosAlpha);
        k[3] = (inertia.y * rotate.y) * this.k / (distance * M);
        forces[0] = (k[0] + k[1] - k[2] + k[3]) / 4;
        forces[1] = (k[0] - k[1] - k[2] - k[3]) / 4;
        forces[2] = (k[0] - k[1] + k[2] + k[3]) / 4;
        forces[3] = (k[0] + k[1] + k[2] - k[3]) / 4;
        for (int i = 0; i < 4; i++)
        {
            speeds[i] = Force2Speed(forces[i]);
        }
    }

    void ApplyAirResistance()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 velocity = rb.velocity;
        Vector3 resistenceForce = -airResistence * velocity;
        rb.AddForce(resistenceForce, ForceMode.Force);
    }
}
