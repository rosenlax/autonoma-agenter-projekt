
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneController))]
public class DroneAgent : Agent
{
    [Header("Landing Target")]
    public Transform landingTarget;
    public RoverAgent rover;

    [Header("Training Mode")]
    public bool trainingMode = true;
    public bool roverReadyAtStart = true;

    [Header("Navigation")]
    public float hoverHeight = 4f;
    public float maxDistance = 100f;
    public float maxSpeed = 15f;

    [Header("Landing Conditions")]
    public float landingRadius = 0.7f;
    public float landingHeightTolerance = 0.35f;
    public float landingSpeed = 0.6f;

    [Header("Rewards")]
    public float progressReward = 0.1f;
    public float successReward = 5f;
    public float failurePenalty = -2f;
    public float stepPenalty = -0.0005f;

    private Rigidbody rb;
    private DroneController controller;

    private Vector3 startPosition;
    private Quaternion startRotation;
    private float previousDistance;

    private bool hasLanded;

    public bool HasLanded
    {
        get { return hasLanded; }
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        controller = GetComponent<DroneController>();

        startPosition = transform.position;
        startRotation = transform.rotation;

        controller.useAgentControl = true;
        controller.ResetControls();
    }

    private bool RoverReady()
    {
        if (trainingMode)
            return roverReadyAtStart;

        return rover != null && rover.HasArrived;
    }

    private Vector3 GetNavigationTarget()
    {
        if (landingTarget == null)
            return transform.position;

        if (RoverReady())
        {
            return landingTarget.position;
        }

        return landingTarget.position
            + Vector3.up * hoverHeight;
    }

    public override void OnEpisodeBegin()
    {
        hasLanded = false;

        controller.useAgentControl = true;
        controller.ResetControls();

        rb.isKinematic = false;

        transform.SetPositionAndRotation(
            startPosition,
            startRotation
        );

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        previousDistance = Vector3.Distance(
            transform.position,
            GetNavigationTarget()
        );
    }

    public override void CollectObservations(
        VectorSensor sensor)
    {
        if (landingTarget == null)
        {
            for (int i = 0; i < 11; i++)
                sensor.AddObservation(0f);

            return;
        }

        Vector3 difference =
            GetNavigationTarget()
            - transform.position;

        Vector3 localTarget =
            transform.InverseTransformDirection(
                difference
            );

        // 3: relative target position
        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localTarget / maxDistance,
                1f
            )
        );

        // 3: relative velocity
        Vector3 localVelocity =
            transform.InverseTransformDirection(
                rb.linearVelocity
            );

        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localVelocity / maxSpeed,
                1f
            )
        );

        // 3: orientation relative to gravity
        sensor.AddObservation(
            transform.InverseTransformDirection(
                Vector3.up
            )
        );

        // 1: rover ready status
        sensor.AddObservation(
            RoverReady() ? 1f : 0f
        );

        // 1: horizontal distance
        Vector3 flatDifference = difference;
        flatDifference.y = 0f;

        sensor.AddObservation(
            Mathf.Clamp01(
                flatDifference.magnitude / maxDistance
            )
        );
    }

    public override void OnActionReceived(
        ActionBuffers actions)
    {
        if (hasLanded)
        {
            controller.ResetControls();
            return;
        }

        float pitch = Mathf.Clamp(
            actions.ContinuousActions[0],
            -1f, 1f
        );

        float roll = Mathf.Clamp(
            actions.ContinuousActions[1],
            -1f, 1f
        );

        float lift = Mathf.Clamp(
            actions.ContinuousActions[2],
            -1f, 1f
        );

        controller.SetAgentInput(
            pitch,
            roll,
            lift
        );

        if (landingTarget == null)
            return;

        Vector3 target = GetNavigationTarget();

        float distance = Vector3.Distance(
            transform.position,
            target
        );

        float progress =
            previousDistance - distance;

        AddReward(progress * progressReward);
        AddReward(stepPenalty);

        previousDistance = distance;

        Vector3 difference =
            landingTarget.position
            - transform.position;

        float horizontalDistance =
            new Vector2(
                difference.x,
                difference.z
            ).magnitude;

        float verticalDistance =
            Mathf.Abs(difference.y);

        float speed =
            rb.linearVelocity.magnitude;

        bool stable =
            Vector3.Dot(
                transform.up,
                Vector3.up
            ) > 0.9f;

        // Landing is only permitted
        // when the rover is ready
        if (RoverReady() &&
            horizontalDistance < landingRadius &&
            verticalDistance < landingHeightTolerance &&
            speed < landingSpeed &&
            stable)
        {
            AddReward(successReward);

            controller.ResetControls();

            if (trainingMode)
            {
                EndEpisode();
            }
            else
            {
                hasLanded = true;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.isKinematic = true;

                Debug.Log(
                    "DRONE LANDED ON ROVER!"
                );
            }

            return;
        }

        // Failure when too far away
        if (distance > maxDistance)
        {
            AddReward(failurePenalty);

            if (trainingMode)
                EndEpisode();

            return;
        }

        // Failure when falling below
        // the intended landing height
        if (transform.position.y <
            landingTarget.position.y - 1f)
        {
            AddReward(failurePenalty);

            if (trainingMode)
                EndEpisode();

            return;
        }

        // Failure when tilted too far
        if (Vector3.Dot(
            transform.up,
            Vector3.up
        ) < 0.4f)
        {
            AddReward(failurePenalty);

            if (trainingMode)
                EndEpisode();
        }
    }

    public override void Heuristic(
        in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        actions[0] = 0f;
        actions[1] = 0f;
        actions[2] = 0f;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;

        if (keyboard != null)
        {
            if (keyboard.wKey.isPressed)
                actions[0] = 1f;
            if (keyboard.sKey.isPressed)
                actions[0] = -1f;

            if (keyboard.aKey.isPressed)
                actions[1] = 1f;
            if (keyboard.dKey.isPressed)
                actions[1] = -1f;

            if (keyboard.rKey.isPressed)
                actions[2] = 1f;
            if (keyboard.fKey.isPressed)
                actions[2] = -1f;
        }
#elif ENABLE_LEGACY_INPUT_MANAGER
        if (Input.GetKey(KeyCode.W))
            actions[0] = 1f;
        if (Input.GetKey(KeyCode.S))
            actions[0] = -1f;

        if (Input.GetKey(KeyCode.A))
            actions[1] = 1f;
        if (Input.GetKey(KeyCode.D))
            actions[1] = -1f;

        if (Input.GetKey(KeyCode.R))
            actions[2] = 1f;
        if (Input.GetKey(KeyCode.F))
            actions[2] = -1f;
#endif
    }

    protected override void OnDisable()
    {
        if (controller != null)
        {
            controller.ResetControls();
            controller.useAgentControl = false;
        }

        if (rb != null)
            rb.isKinematic = false;

        base.OnDisable();
    }
}
