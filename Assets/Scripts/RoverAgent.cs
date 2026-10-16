
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CarControl))]
public class RoverAgent : Agent
{
    [Header("Target")]
    public Transform meetingPoint;

    [Header("Operating Mode")]
    public bool demonstrationMode = false;

    [Header("Success Conditions")]
    public float successDistance = 2f;
    public float stopSpeed = 0.5f;

    [Header("Training Settings")]
    public float maxDistance = 100f;
    public float maxSpeed = 20f;
    public int maxIdleSteps = 400;

    [Header("Rewards")]
    public float progressReward = 0.2f;
    public float successReward = 5f;
    public float failurePenalty = -1f;
    public float stepPenalty = -0.001f;

    private CarControl car;
    private Rigidbody rb;

    private Vector3 startPosition;
    private Quaternion startRotation;

    private float previousDistance;
    private float bestDistance;
    private int idleSteps;

    private bool hasArrived = false;

    // Can be read by the drone agent later
    public bool HasArrived
    {
        get { return hasArrived; }
    }

    public override void Initialize()
    {
        car = GetComponent<CarControl>();
        rb = GetComponent<Rigidbody>();

        startPosition = transform.position;
        startRotation = transform.rotation;

        car.useAgentControl = true;
        car.ResetControls();
    }

    public override void OnEpisodeBegin()
    {
        hasArrived = false;
        idleSteps = 0;

        car.useAgentControl = true;
        car.ResetControls();

        transform.SetPositionAndRotation(
            startPosition,
            startRotation
        );

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (meetingPoint != null)
        {
            previousDistance = GetDistance();
            bestDistance = previousDistance;
        }
        else
        {
            previousDistance = 0f;
            bestDistance = 0f;
            Debug.LogWarning(
                "RoverAgent: MeetingPoint saknas!"
            );
        }
    }

    public override void CollectObservations(
        VectorSensor sensor)
    {
        // IMPORTANT:
        // Preserve the same 5 observations
        // used when training Rover.onnx.

        if (meetingPoint == null)
        {
            for (int i = 0; i < 5; i++)
                sensor.AddObservation(0f);

            return;
        }

        Vector3 direction =
            meetingPoint.position - transform.position;

        direction.y = 0f;

        Vector3 localDirection =
            transform.InverseTransformDirection(
                direction.normalized
            );

        // 2 observations: direction
        sensor.AddObservation(localDirection.x);
        sensor.AddObservation(localDirection.z);

        // 1 observation: distance
        sensor.AddObservation(
            Mathf.Clamp01(
                GetDistance() / maxDistance
            )
        );

        // 2 observations: velocity
        Vector3 localVelocity =
            transform.InverseTransformDirection(
                rb.linearVelocity
            );

        sensor.AddObservation(
            Mathf.Clamp(
                localVelocity.x / maxSpeed,
                -1f, 1f
            )
        );

        sensor.AddObservation(
            Mathf.Clamp(
                localVelocity.z / maxSpeed,
                -1f, 1f
            )
        );
    }

    public override void OnActionReceived(
        ActionBuffers actions)
    {
        // Prevent further AI movement after arrival
        if (hasArrived && demonstrationMode)
        {
            car.ResetControls();
            return;
        }

        float steering = Mathf.Clamp(
            actions.ContinuousActions[0],
            -1f, 1f
        );

        float throttle = Mathf.Clamp(
            actions.ContinuousActions[1],
            -1f, 1f
        );

        car.SetAgentInput(steering, throttle);

        if (meetingPoint == null)
            return;

        float distance = GetDistance();
        float speed = rb.linearVelocity.magnitude;

        // Reward progress toward MeetingPoint
        float progress = previousDistance - distance;

        AddReward(progress * progressReward);
        AddReward(stepPenalty);

        previousDistance = distance;

        // Successful arrival
        if (distance < successDistance &&
            speed < stopSpeed)
        {
            AddReward(successReward);
            car.ResetControls();

            if (demonstrationMode)
            {
                hasArrived = true;

                // Keep rover stationary for docking
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.isKinematic = true;

                Debug.Log(
                    "ROVER ARRIVED! Waiting for drone."
                );

                return;
            }

            EndEpisode();
            return;
        }

        // Detect lack of progress
        if (distance < bestDistance - 0.1f)
        {
            bestDistance = distance;
            idleSteps = 0;
        }
        else
        {
            idleSteps++;
        }

        // Training failures
        if (idleSteps > maxIdleSteps ||
            distance > maxDistance ||
            Vector3.Dot(
                transform.up,
                Vector3.up
            ) < 0.3f)
        {
            AddReward(failurePenalty);
            car.ResetControls();

            if (!demonstrationMode)
            {
                EndEpisode();
            }
            else
            {
                // Keep trying during demonstration
                idleSteps = 0;
                bestDistance = distance;
            }
        }
    }

    private float GetDistance()
    {
        Vector3 difference =
            meetingPoint.position - transform.position;

        difference.y = 0f;

        return difference.magnitude;
    }

    public override void Heuristic(
        in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        actions[0] = 0f;
        actions[1] = 0f;

#if ENABLE_INPUT_SYSTEM

        Keyboard keyboard = Keyboard.current;

        if (keyboard != null)
        {
            if (keyboard.leftArrowKey.isPressed)
                actions[0] = -1f;

            if (keyboard.rightArrowKey.isPressed)
                actions[0] = 1f;

            if (keyboard.upArrowKey.isPressed)
                actions[1] = 1f;

            if (keyboard.downArrowKey.isPressed)
                actions[1] = -1f;
        }

#elif ENABLE_LEGACY_INPUT_MANAGER

        if (Input.GetKey(KeyCode.LeftArrow))
            actions[0] = -1f;

        if (Input.GetKey(KeyCode.RightArrow))
            actions[0] = 1f;

        if (Input.GetKey(KeyCode.UpArrow))
            actions[1] = 1f;

        if (Input.GetKey(KeyCode.DownArrow))
            actions[1] = -1f;

#endif
    }

    protected override void OnDisable()
    {
        if (car != null)
        {
            car.ResetControls();
            car.useAgentControl = false;
        }

        if (rb != null)
        {
            rb.isKinematic = false;
        }

        base.OnDisable();
    }
}
