
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
public class CarControl : MonoBehaviour
{
    [Header("Engine and Steering")]
    public float enginePower = 2000f;
    public float turnSpeed = 25f;
    public float turnSmoothness = 5f;
    public float brakePower = 3000f;

    [Header("Original Vehicle Setup")]
    public Transform[] wheels;
    public Transform[] wheelMeshes;
    public Transform centerOfMass;
    public GameObject steeringWheel;

    [Header("ML-Agents Control")]
    public bool useAgentControl = false;
    public float agentSteering = 0f;
    public float agentThrottle = 0f;

    private Rigidbody rb;
    private float currentTurnAngle = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        if (centerOfMass != null)
        {
            rb.centerOfMass =
                transform.InverseTransformPoint(
                    centerOfMass.position
                );
        }
    }

    void FixedUpdate()
    {
        float steering;
        float throttle;

        if (useAgentControl)
        {
            steering = Mathf.Clamp(
                agentSteering, -1f, 1f
            );

            throttle = Mathf.Clamp(
                agentThrottle, -1f, 1f
            );
        }
        else
        {
            GetKeyboardInput(
                out steering,
                out throttle
            );
        }

        ApplyControls(steering, throttle);
    }

    private void GetKeyboardInput(
        out float steering,
        out float throttle)
    {
        steering = 0f;
        throttle = 0f;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;

        if (keyboard != null)
        {
            if (keyboard.leftArrowKey.isPressed)
                steering = -1f;

            if (keyboard.rightArrowKey.isPressed)
                steering = 1f;

            if (keyboard.upArrowKey.isPressed)
                throttle = 1f;

            if (keyboard.downArrowKey.isPressed)
                throttle = -1f;
        }

#elif ENABLE_LEGACY_INPUT_MANAGER

        if (Input.GetKey(KeyCode.LeftArrow))
            steering = -1f;

        if (Input.GetKey(KeyCode.RightArrow))
            steering = 1f;

        if (Input.GetKey(KeyCode.UpArrow))
            throttle = 1f;

        if (Input.GetKey(KeyCode.DownArrow))
            throttle = -1f;
#endif
    }

    private void ApplyControls(
        float steering,
        float throttle)
    {
        float targetAngle =
            steering * turnSpeed;

        currentTurnAngle = Mathf.Lerp(
            currentTurnAngle,
            targetAngle,
            Time.fixedDeltaTime * turnSmoothness
        );

        if (steeringWheel != null)
        {
            steeringWheel.transform.localEulerAngles =
                new Vector3(
                    -64f,
                    0f,
                    currentTurnAngle * 3f
                );
        }

        if (wheels == null)
            return;

        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] == null)
                continue;

            WheelCollider wheel =
                wheels[i].GetComponent<WheelCollider>();

            if (wheel == null)
                continue;

            // Original front-wheel steering
            if (i < 2)
                wheel.steerAngle = currentTurnAngle;
            else
                wheel.steerAngle = 0f;

            // Original motor system
            wheel.motorTorque =
                throttle * enginePower;

            // Automatic braking for AI
            bool braking =
                useAgentControl &&
                Mathf.Abs(throttle) < 0.05f;

            wheel.brakeTorque =
                braking ? brakePower : 0f;

            // Original wheel mesh animation
            if (wheelMeshes != null &&
                i < wheelMeshes.Length &&
                wheelMeshes[i] != null)
            {
                wheel.GetWorldPose(
                    out Vector3 position,
                    out Quaternion rotation
                );

                wheelMeshes[i].position = position;
                wheelMeshes[i].rotation = rotation;
            }
        }
    }

    public void SetAgentInput(
        float steering,
        float throttle)
    {
        agentSteering =
            Mathf.Clamp(steering, -1f, 1f);

        agentThrottle =
            Mathf.Clamp(throttle, -1f, 1f);
    }

    public void ResetControls()
    {
        agentSteering = 0f;
        agentThrottle = 0f;
        currentTurnAngle = 0f;

        if (wheels == null)
            return;

        foreach (Transform wheelTransform in wheels)
        {
            if (wheelTransform == null)
                continue;

            WheelCollider wheel =
                wheelTransform.GetComponent<WheelCollider>();

            if (wheel == null)
                continue;

            wheel.motorTorque = 0f;
            wheel.brakeTorque = 0f;
            wheel.steerAngle = 0f;
        }
    }
}
