using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class DroneController : MonoBehaviour
{
    // Start is called before the first frame update
    //public GameObject wing0;
    public GameObject mainDrone;
    public GameObject target;
    public Vector3 offset = new Vector3(5.0f, 5.0f, 5.0f);

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
    void Start()
    {
        if (wings[0] == null)
        {
            return;
        }
        else
        {
            var rb = GetComponent<Rigidbody>();
            rb.drag = 0.2f;          // 线阻尼
            rb.angularDrag = 0.4f;   // 角阻尼
            // calculate wing offset, size and inertia
            Vector3 position = transform.position;
            Vector3 scale = transform.localScale;

            Vector3 localPosition = wings[0].transform.localPosition;
            Vector3 worldPosition = position + Vector3.Scale(localPosition, scale);
            wingOffset = worldPosition - position;
            hoverPoint = transform.position;
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
        PointTo(rotatePoint);

        float horizontal = Input.GetAxis("Horizontal"); // A/D键 -> -1 ~ 1
        float vertical = Input.GetAxis("Vertical");     // W/S键 -> -1 ~ 1

        Vector3 directionXYZ = Vector3.zero;
        rotate = Vector3.zero;
        Vector3 rotateXYZ = Vector3.zero;
        // 水平移动：a/d 控制左右，w/s 控制前后
        if (Input.GetKey(KeyCode.W)) directionXYZ += transform.forward;
        if (Input.GetKey(KeyCode.S)) directionXYZ -= transform.forward;
        if (Input.GetKey(KeyCode.A)) directionXYZ -= transform.right;
        if (Input.GetKey(KeyCode.D)) directionXYZ += transform.right;

        // 旋转：q/e 控制绕Y轴旋转
        /*        if (Input.GetKey(KeyCode.Q)) isRight = -1f;
                else if (Input.GetKey(KeyCode.E)) isRight = 1f;*/
        // 旋转：q/e 控制绕Y轴旋转
        if (Input.GetKey(KeyCode.Q)) rotateXYZ = Vector3.down;
        if (Input.GetKey(KeyCode.E)) rotateXYZ = Vector3.up;



        // 垂直移动：j/k 控制上升下降
        if (Input.GetKey(KeyCode.J)) directionXYZ += transform.up;     // 上升
        if (Input.GetKey(KeyCode.K)) directionXYZ -= transform.up;     // 下降

        bool hasMove = directionXYZ != Vector3.zero;
        bool hasRotate = rotateXYZ != Vector3.zero; ;

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
                rotatePoint = transform.forward;
                isStopRotating = true;
            }
            PointTo(rotatePoint);
        }
        else
        {
            isStopRotating = false;
            transform.Rotate(rotateXYZ * spinSpeed * Time.deltaTime, Space.Self);
        }

        Solve();
        ApplyAirResistance();
        ApplyForce();
        ApplyVision();

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

   



    void Follow(GameObject obj)
    {
        if (obj == null) return;

        MoveToWithVelocity(obj.transform.position + offset, obj.GetComponent<DroneController>().moveAt);
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
        if (acc.magnitude > 5.0f)
        {
            acc = acc.normalized * 5.0f;
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
        Vector3 resistenceForce = -airResistence * velocity * velocity.magnitude;
        rb.AddForce(resistenceForce, ForceMode.Force);
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



    void Hover()
    {
        Vector3 up = Vector3.up;
        Vector3 droneUp = transform.up;

        float g = Mathf.Abs(Physics.gravity.y);
        float cosine = Vector3.Dot(up, droneUp) / (up.magnitude * droneUp.magnitude);
        float a = g / cosine;

        Rise(a);
    }

    

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

        last_velocity = GetComponent<Rigidbody>().velocity;
    }
}
