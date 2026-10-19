using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    [Header("Controls")]
    public bool useAgentControl = true;

    [Range(-1, 1)] public float agentPitch;
    [Range(-1, 1)] public float agentRoll;
    [Range(-1, 1)] public float agentLift;

    [Header("Stabilization")]
    public float maximumTiltAngle = 20f;
    public float attitudeStrength = 9f;
    public float angularDamping = 5f;
    public float horizontalDrag = 1.2f;
    public float maxHorizontalBrake = 4f;

    [Header("Altitude Hold")]
    public float climbSpeed = 2.5f;
    public float altitudeStrength = 3f;
    public float verticalDamping = 3.5f;
    public float maximumVerticalCorrection = 7f;

    // Used only after Approach succeeds in inference mode.
    [Header("Automatic Position Hold")]
    public float hoverPositionGain = 1.4f;
    public float hoverVelocityGain = 2.4f;
    public float hoverMaxHorizontalSpeed = 2f;
    public float hoverMaxHorizontalAcceleration = 4f;
    public float hoverHeightGain = 1.2f;
    public float hoverMaxVerticalSpeed = 1f;

    private Rigidbody rb;
    private float targetAltitude;
    private bool positionHoldActive;
    private Vector3 positionHoldTarget;

    public void SetPositionHoldTarget(Vector3 target)
    {
        positionHoldTarget = target;
        positionHoldActive = true;
    }

    public void DisablePositionHold()
    {
        positionHoldActive = false;
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;

        rb.constraints |= RigidbodyConstraints.FreezeRotationY;
        targetAltitude = rb.position.y;
    }

    private void OnEnable()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        targetAltitude = rb.position.y;
    }

    public void SetAgentInput(float pitch, float roll, float lift)
    {
        agentPitch = Mathf.Clamp(pitch, -1f, 1f);
        agentRoll = Mathf.Clamp(roll, -1f, 1f);
        agentLift = Mathf.Clamp(lift, -1f, 1f);
    }

    public void ResetControls()
    {
        agentPitch = 0f;
        agentRoll = 0f;
        agentLift = 0f;

        if (rb != null)
            targetAltitude = rb.position.y;
    }

    private void FixedUpdate()
    {
        if (rb.isKinematic)
            return;

        float pitch = agentPitch;
        float roll = agentRoll;
        float lift = agentLift;

        if (!useAgentControl)
        {
#if ENABLE_INPUT_SYSTEM
            var k = Keyboard.current;

            if (k != null)
            {
                pitch =
                    (k.wKey.isPressed ? 1f : 0f) -
                    (k.sKey.isPressed ? 1f : 0f);

                roll =
                    (k.dKey.isPressed ? 1f : 0f) -
                    (k.aKey.isPressed ? 1f : 0f);

                lift =
                    (k.rKey.isPressed ? 1f : 0f) -
                    (k.fKey.isPressed ? 1f : 0f);
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            pitch =
                (Input.GetKey(KeyCode.W) ? 1f : 0f) -
                (Input.GetKey(KeyCode.S) ? 1f : 0f);

            roll =
                (Input.GetKey(KeyCode.D) ? 1f : 0f) -
                (Input.GetKey(KeyCode.A) ? 1f : 0f);

            lift =
                (Input.GetKey(KeyCode.R) ? 1f : 0f) -
                (Input.GetKey(KeyCode.F) ? 1f : 0f);
#endif
        }

        // When position hold is active, override the model's input with
        // a position/velocity controller. Keep the original force physics below.
        if (positionHoldActive)
        {
            Vector3 holdPositionError = positionHoldTarget - rb.position;

            Vector3 holdHorizontalError = Vector3.ProjectOnPlane(
                holdPositionError, Vector3.up);

            Vector3 holdHorizontalVelocity = Vector3.ProjectOnPlane(
                rb.linearVelocity, Vector3.up);

            Vector3 holdDesiredVelocity = Vector3.ClampMagnitude(
                holdHorizontalError * hoverPositionGain,
                hoverMaxHorizontalSpeed);

            Vector3 holdDesiredAcceleration = Vector3.ClampMagnitude(
                (holdDesiredVelocity - holdHorizontalVelocity) * hoverVelocityGain,
                hoverMaxHorizontalAcceleration);

            // These directions match the existing pitch/roll convention:
            // positive pitch tilts toward local +Z, positive roll toward local +X.
            Quaternion holdYawRotation = Quaternion.Euler(
                0f, rb.rotation.eulerAngles.y, 0f);

            Vector3 holdLocalAcceleration =
                Quaternion.Inverse(holdYawRotation) * holdDesiredAcceleration;

            float holdGravity = Mathf.Max(0.1f, Physics.gravity.magnitude);
            float holdTiltLimit = Mathf.Max(0.1f, maximumTiltAngle);

            pitch = Mathf.Clamp(
                Mathf.Atan2(holdLocalAcceleration.z, holdGravity) *
                Mathf.Rad2Deg / holdTiltLimit,
                -1f, 1f);

            roll = Mathf.Clamp(
                Mathf.Atan2(holdLocalAcceleration.x, holdGravity) *
                Mathf.Rad2Deg / holdTiltLimit,
                -1f, 1f);

            float holdVerticalSpeed = Mathf.Clamp(
                holdPositionError.y * hoverHeightGain,
                -hoverMaxVerticalSpeed,
                hoverMaxVerticalSpeed);

            lift = Mathf.Clamp(
                holdVerticalSpeed / Mathf.Max(0.1f, climbSpeed),
                -1f, 1f);
        }

        // Original attitude stabilization.
        Quaternion desired = Quaternion.Euler(
            pitch * maximumTiltAngle,
            transform.eulerAngles.y,
            -roll * maximumTiltAngle);

        Quaternion error = desired * Quaternion.Inverse(rb.rotation);

        if (error.w < 0f)
        {
            error = new Quaternion(
                -error.x, -error.y, -error.z, -error.w);
        }

        error.ToAngleAxis(out float angle, out Vector3 axis);

        if (angle > 180f)
            angle -= 360f;

        Vector3 correction = Vector3.zero;

        if (axis.sqrMagnitude > 0.0001f && !float.IsNaN(axis.x))
        {
            correction = axis.normalized *
                (angle * Mathf.Deg2Rad * attitudeStrength);
        }

        rb.AddTorque(
            correction - rb.angularVelocity * angularDamping,
            ForceMode.Acceleration);

        // Original vertical-speed control.
        float desiredVerticalSpeed = lift * climbSpeed;

        float accelerationY = Mathf.Clamp(
            (desiredVerticalSpeed - rb.linearVelocity.y) * verticalDamping,
            -maximumVerticalCorrection,
            maximumVerticalCorrection);

        float upright = Mathf.Max(
            Vector3.Dot(transform.up, Vector3.up), 0.3f);

        float thrust = Mathf.Max(
            0f,
            (Physics.gravity.magnitude + accelerationY) / upright);

        rb.AddForce(transform.up * thrust, ForceMode.Acceleration);

        // Original horizontal braking.
        Vector3 horizontal = Vector3.ProjectOnPlane(
            rb.linearVelocity, Vector3.up);

        Vector3 braking = Vector3.ClampMagnitude(
            -horizontal * horizontalDrag,
            maxHorizontalBrake);

        rb.AddForce(braking, ForceMode.Acceleration);
    }
}
