
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

    [Header("Success Conditions")]
    public float successDistance = 3f;
    public float stopSpeed = 0.5f;

    [Header("Training Area")]
    public float maxDistance = 300f;
    public float maxSpeed = 20f;

    [Header("Rewards")]
    public float progressReward = 0.02f;
    public float successReward = 3f;
    public float failurePenalty = -1f;
    public float stepPenalty = -0.0005f;

    private CarControl car;
    private Rigidbody rb;

    private Vector3 startPosition;
    private Quaternion startRotation;

    private float previousDistance;

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
        car.useAgentControl = true;
        car.ResetControls();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.SetPositionAndRotation(
            startPosition,
            startRotation
        );

        rb.Sleep();
        rb.WakeUp();

        if (meetingPoint != null)
        {
            previousDistance = GetDistanceToTarget();
        }
        else
        {
            previousDistance = 0f;
        }
    }

    public override void CollectObservations(
        VectorSensor sensor)
    {
        if (meetingPoint == null)
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            return;
        }

        // Direction to MeetingPoint
        Vector3 direction =
            meetingPoint.position - transform.position;

        Vector3 localDirection =
            transform.InverseTransformDirection(
                direction.normalized
            );

        // 3 observations: target direction
        sensor.AddObservation(localDirection);

        // 1 observation: normalized distance
        float distance = GetDistanceToTarget();

        sensor.AddObservation(
            Mathf.Clamp01(distance / maxDistance)
        );

        // 1 observation: forward speed
        float forwardSpeed =
            transform.InverseTransformDirection(
                rb.linearVelocity
            ).z;

        sensor.AddObservation(
            Mathf.Clamp(
                forwardSpeed / maxSpeed,
                -1f,
                1f
            )
        );
    }

    public override void OnActionReceived(
        ActionBuffers actions)
    {
        // Continuous action 0: steering
        float steering = Mathf.Clamp(
            actions.ContinuousActions[0],
            -1f,
            1f
        );

        // Continuous action 1: throttle
        float throttle = Mathf.Clamp(
            actions.ContinuousActions[1],
            -1f,
            1f
        );

        car.SetAgentInput(steering, throttle);

        if (meetingPoint == null)
            return;

        float distance = GetDistanceToTarget();

        // Reward movement toward MeetingPoint
        float progress = previousDistance - distance;

        AddReward(progress * progressReward);
        AddReward(stepPenalty);

        previousDistance = distance;

        // Successful arrival and stop
        if (distance < successDistance &&
            rb.linearVelocity.magnitude < stopSpeed)
        {
            AddReward(successReward);

            car.ResetControls();
            EndEpisode();
            return;
        }

        // Failure: too far from target
        if (distance > maxDistance)
        {
            AddReward(failurePenalty);

            car.ResetControls();
            EndEpisode();
            return;
        }

        // Failure: rover overturned
        float upright = Vector3.Dot(
            transform.up,
            Vector3.up
        );

        if (upright < 0.3f)
        {
            AddReward(failurePenalty);

            car.ResetControls();
            EndEpisode();
        }
    }

    private float GetDistanceToTarget()
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

        base.OnDisable();
    }
}
