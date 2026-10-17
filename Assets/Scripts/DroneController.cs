
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

    private Rigidbody rb;
    private float targetAltitude;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;

        rb.constraints |=
            RigidbodyConstraints.FreezeRotationY;

        targetAltitude = rb.position.y;
    }

    private void OnEnable()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        targetAltitude = rb.position.y;
    }

    public void SetAgentInput(
        float pitch,
        float roll,
        float lift)
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

        // Automatically return toward level flight
        // when pitch and roll are released.

        Quaternion desired = Quaternion.Euler(
            pitch * maximumTiltAngle,
            transform.eulerAngles.y,
            -roll * maximumTiltAngle
        );

        Quaternion error =
            desired * Quaternion.Inverse(rb.rotation);

        if (error.w < 0f)
        {
            error = new Quaternion(
                -error.x,
                -error.y,
                -error.z,
                -error.w
            );
        }

        error.ToAngleAxis(
            out float angle,
            out Vector3 axis
        );

        if (angle > 180f)
            angle -= 360f;

        Vector3 correction = Vector3.zero;

        if (axis.sqrMagnitude > 0.0001f &&
            !float.IsNaN(axis.x))
        {
            correction =
                axis.normalized *
                (angle * Mathf.Deg2Rad *
                 attitudeStrength);
        }

        rb.AddTorque(
            correction -
            rb.angularVelocity * angularDamping,
            ForceMode.Acceleration
        );

        // R/F changes target altitude.
        // Releasing R/F holds the target altitude.

        targetAltitude +=
            lift * climbSpeed *
            Time.fixedDeltaTime;

        float accelerationY = Mathf.Clamp(
            (targetAltitude - rb.position.y) *
            altitudeStrength -
            rb.linearVelocity.y * verticalDamping,
            -maximumVerticalCorrection,
            maximumVerticalCorrection
        );

        float upright = Mathf.Max(
            Vector3.Dot(
                transform.up,
                Vector3.up
            ),
            0.3f
        );

        float thrust = Mathf.Max(
            0f,
            (Physics.gravity.magnitude +
             accelerationY) / upright
        );

        rb.AddForce(
            transform.up * thrust,
            ForceMode.Acceleration
        );

        // Gradually brake horizontal sliding.

        Vector3 horizontal =
            Vector3.ProjectOnPlane(
                rb.linearVelocity,
                Vector3.up
            );

        Vector3 braking =
            Vector3.ClampMagnitude(
                -horizontal * horizontalDrag,
                maxHorizontalBrake
            );

        rb.AddForce(
            braking,
            ForceMode.Acceleration
        );
    }
}
