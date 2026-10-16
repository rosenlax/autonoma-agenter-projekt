
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    [Header("Original Flight Physics")]
    public float thrustChangeRate = 10f;
    public float maximumThrustOffset = 20f;
    public float thrustReturnRate = 10f;
    public float tiltChangeRate = 30f;
    public float tiltStabilizationTorque = 20f;
    public float angularDamping = 5f;
    public float maximumTiltAngle = 40f;
    public float minimumCosine = 0.1f;

    [Header("ML-Agents")]
    public bool useAgentControl = false;

    [Range(-1f, 1f)]
    public float agentPitch = 0f;

    [Range(-1f, 1f)]
    public float agentRoll = 0f;

    [Range(-1f, 1f)]
    public float agentLift = 0f;

    private Rigidbody rb;
    private float thrustOffset;
    private float targetPitch;
    private float targetRoll;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();

        rb.constraints |=
            RigidbodyConstraints.FreezeRotationY;
    }

    private void FixedUpdate()
    {
        if (useAgentControl)
        {
            UpdateAgentControls();
        }
        else
        {
            UpdateThrustOffset();
            UpdateTargetTilt();
        }

        ApplyThrust();
        ApplyTiltTorque();
        LimitTiltAngle();
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

    private void UpdateAgentControls()
    {
        targetPitch =
            agentPitch * maximumTiltAngle;

        targetRoll =
            agentRoll * maximumTiltAngle;

        thrustOffset =
            agentLift * maximumThrustOffset;
    }

    private bool KeyPressed(KeyCode key)
    {
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;

        if (keyboard != null)
        {
            switch (key)
            {
                case KeyCode.W:
                    return keyboard.wKey.isPressed;
                case KeyCode.S:
                    return keyboard.sKey.isPressed;
                case KeyCode.A:
                    return keyboard.aKey.isPressed;
                case KeyCode.D:
                    return keyboard.dKey.isPressed;
                case KeyCode.R:
                    return keyboard.rKey.isPressed;
                case KeyCode.F:
                    return keyboard.fKey.isPressed;
            }
        }

        return false;
#elif ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(key);
#else
        return false;
#endif
    }

    private void UpdateThrustOffset()
    {
        if (KeyPressed(KeyCode.R))
        {
            thrustOffset +=
                thrustChangeRate * Time.fixedDeltaTime;
        }
        else if (KeyPressed(KeyCode.F))
        {
            thrustOffset -=
                thrustChangeRate * Time.fixedDeltaTime;
        }
        else
        {
            thrustOffset = Mathf.MoveTowards(
                thrustOffset,
                0f,
                thrustReturnRate * Time.fixedDeltaTime
            );
        }

        thrustOffset = Mathf.Clamp(
            thrustOffset,
            -maximumThrustOffset,
            maximumThrustOffset
        );
    }

    private void UpdateTargetTilt()
    {
        if (KeyPressed(KeyCode.W))
        {
            targetPitch +=
                tiltChangeRate * Time.fixedDeltaTime;
        }

        if (KeyPressed(KeyCode.S))
        {
            targetPitch -=
                tiltChangeRate * Time.fixedDeltaTime;
        }

        if (KeyPressed(KeyCode.D))
        {
            targetRoll -=
                tiltChangeRate * Time.fixedDeltaTime;
        }

        if (KeyPressed(KeyCode.A))
        {
            targetRoll +=
                tiltChangeRate * Time.fixedDeltaTime;
        }

        targetPitch = Mathf.Clamp(
            targetPitch,
            -maximumTiltAngle,
            maximumTiltAngle
        );

        targetRoll = Mathf.Clamp(
            targetRoll,
            -maximumTiltAngle,
            maximumTiltAngle
        );
    }

    private void ApplyThrust()
    {
        float cosine = Vector3.Dot(
            transform.up.normalized,
            Vector3.up
        );

        cosine = Mathf.Max(
            cosine,
            minimumCosine
        );

        float hoverThrust =
            (rb.mass * Physics.gravity.magnitude)
            / cosine;

        float thrustForce =
            hoverThrust + thrustOffset;

        rb.AddForce(
            transform.up * thrustForce,
            ForceMode.Force
        );
    }

    private void ApplyTiltTorque()
    {
        Vector3 localAngularVelocity =
            transform.InverseTransformDirection(
                rb.angularVelocity
            );

        Vector3 localAngles =
            GetLocalTiltAngles();

        Vector3 localTorque = new Vector3(
            (targetPitch - localAngles.x)
                * tiltStabilizationTorque
                - localAngularVelocity.x
                * angularDamping,

            0f,

            (targetRoll - localAngles.z)
                * tiltStabilizationTorque
                - localAngularVelocity.z
                * angularDamping
        );

        rb.AddRelativeTorque(
            localTorque,
            ForceMode.Force
        );
    }

    private Vector3 GetLocalTiltAngles()
    {
        Quaternion localRotation =
            transform.parent == null
                ? rb.rotation
                : Quaternion.Inverse(
                    transform.parent.rotation
                ) * rb.rotation;

        Vector3 localAngles =
            localRotation.eulerAngles;

        localAngles.x =
            NormalizeAngle(localAngles.x);

        localAngles.z =
            NormalizeAngle(localAngles.z);

        return localAngles;
    }

    private void LimitTiltAngle()
    {
        Quaternion localRotation =
            transform.parent == null
                ? rb.rotation
                : Quaternion.Inverse(
                    transform.parent.rotation
                ) * rb.rotation;

        Vector3 localAngles =
            localRotation.eulerAngles;

        localAngles.x =
            NormalizeAngle(localAngles.x);

        localAngles.z =
            NormalizeAngle(localAngles.z);

        float limitedX = Mathf.Clamp(
            localAngles.x,
            -maximumTiltAngle,
            maximumTiltAngle
        );

        float limitedZ = Mathf.Clamp(
            localAngles.z,
            -maximumTiltAngle,
            maximumTiltAngle
        );

        if (Mathf.Approximately(
                localAngles.x, limitedX) &&
            Mathf.Approximately(
                localAngles.z, limitedZ))
        {
            return;
        }

        Quaternion limitedRotation =
            Quaternion.Euler(
                limitedX,
                localRotation.eulerAngles.y,
                limitedZ
            );

        rb.MoveRotation(
            transform.parent == null
                ? limitedRotation
                : transform.parent.rotation
                    * limitedRotation
        );
    }

    private float NormalizeAngle(float angle)
    {
        return angle > 180f
            ? angle - 360f
            : angle;
    }

    public void ResetControls()
    {
        agentPitch = 0f;
        agentRoll = 0f;
        agentLift = 0f;

        targetPitch = 0f;
        targetRoll = 0f;
        thrustOffset = 0f;
    }
}
