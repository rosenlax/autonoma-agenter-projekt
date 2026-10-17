
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
    public enum TrainingStage
    {
        Approach = 0,
        Hover = 1,
        Landing = 2
    }

    [Header("Landing Target")]
    public Transform landingTarget;
    public RoverAgent rover;

    [Header("Training")]
    public bool trainingMode = true;
    public TrainingStage trainingStage =
        TrainingStage.Approach;
    public bool roverReadyAtStart = true;

    [Header("Navigation")]
    public float hoverHeight = 3f;
    public float maxDistance = 100f;
    public float maxSpeed = 15f;

    [Header("Approach")]
    public float approachRadius = 1.5f;
    public float approachMaxSpeed = 3f;

    [Header("Hover")]
    public float hoverRadius = 0.8f;
    public float hoverSpeed = 1f;
    public int requiredHoverSteps = 50;

    [Header("Landing")]
    public float landingRadius = 0.7f;
    public float landingHeightTolerance = 0.35f;
    public float landingSpeed = 0.6f;
    public int requiredLandingSteps = 10;

    [Header("Rewards")]
    public float progressReward = 0.15f;
    public float successReward = 5f;
    public float failurePenalty = -2f;
    public float stepPenalty = -0.0005f;

    [Header("Smooth Flight Rewards")]
    public float actionChangePenalty = 0.0005f;
    public float actionStrengthPenalty = 0.0001f;
    public float nearTargetSpeedPenalty = 0.0003f;
    public float stableNearTargetReward = 0.001f;

    private Rigidbody rb;
    private DroneController controller;

    private Vector3 startPosition;
    private Quaternion startRotation;

    private float previousDistance;
    private Vector3 previousAction;
    private bool hasPreviousAction;

    private int stableSteps;
    private bool hasLanded;
    private bool episodeFinished;

    public bool HasLanded => hasLanded;

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

    private Vector3 GetHoverTarget()
    {
        if (landingTarget == null)
            return transform.position;

        return landingTarget.position +
               Vector3.up * hoverHeight;
    }

    private Vector3 GetNavigationTarget()
    {
        Vector3 hoverTarget = GetHoverTarget();

        if (landingTarget == null)
            return hoverTarget;

        if (trainingStage != TrainingStage.Landing)
            return hoverTarget;

        if (!RoverReady())
            return hoverTarget;

        Vector3 difference =
            landingTarget.position - transform.position;

        Vector2 horizontal = new Vector2(
            difference.x,
            difference.z
        );

        if (horizontal.magnitude > hoverRadius)
            return hoverTarget;

        return landingTarget.position;
    }

    public override void OnEpisodeBegin()
    {
        hasLanded = false;
        episodeFinished = false;
        stableSteps = 0;

        previousAction = Vector3.zero;
        hasPreviousAction = false;

        rb.isKinematic = false;

        transform.SetPositionAndRotation(
            startPosition,
            startRotation
        );

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        controller.useAgentControl = true;

        // Reset altitude hold AFTER repositioning.
        controller.ResetControls();

        previousDistance = Vector3.Distance(
            transform.position,
            GetNavigationTarget()
        );
    }

    public override void CollectObservations(
        VectorSensor sensor)
    {
        // Always 11 observations.

        if (landingTarget == null)
        {
            for (int i = 0; i < 11; i++)
                sensor.AddObservation(0f);

            return;
        }

        Vector3 difference =
            GetNavigationTarget() - transform.position;

        Vector3 localTarget =
            transform.InverseTransformDirection(
                difference
            );

        // 1-3: target direction and distance
        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localTarget / maxDistance,
                1f
            )
        );

        Vector3 localVelocity =
            transform.InverseTransformDirection(
                rb.linearVelocity
            );

        // 4-6: velocity
        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localVelocity / maxSpeed,
                1f
            )
        );

        // 7-9: orientation
        sensor.AddObservation(
            transform.InverseTransformDirection(
                Vector3.up
            )
        );

        // 10: rover ready
        sensor.AddObservation(
            RoverReady() ? 1f : 0f
        );

        // 11: horizontal target distance
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
        if (episodeFinished || hasLanded)
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

        Vector3 currentAction =
            new Vector3(pitch, roll, lift);

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

        float speed = rb.linearVelocity.magnitude;

        float upright = Vector3.Dot(
            transform.up,
            Vector3.up
        );

        // 1. Reward actual progress toward target.
        float progress =
            previousDistance - distance;

        AddReward(progress * progressReward);
        AddReward(stepPenalty);

        previousDistance = distance;

        // 2. Penalize rapid changes in AI commands.
        // Small penalty: do not prevent necessary turns.
        if (hasPreviousAction)
        {
            float change =
                Vector3.Distance(
                    currentAction,
                    previousAction
                );

            AddReward(
                -change * actionChangePenalty
            );
        }

        // 3. Slightly discourage extreme commands.
        AddReward(
            -currentAction.sqrMagnitude *
            actionStrengthPenalty
        );

        previousAction = currentAction;
        hasPreviousAction = true;

        // 4. Encourage stability near the goal.
        float proximity = 1f -
            Mathf.Clamp01(distance / 5f);

        if (upright > 0.9f)
        {
            AddReward(
                proximity * stableNearTargetReward
            );
        }

        // 5. Slow down as target gets closer.
        if (distance < 3f)
        {
            AddReward(
                -speed * nearTargetSpeedPenalty
            );
        }

        switch (trainingStage)
        {
            case TrainingStage.Approach:
                CheckApproach(distance, speed, upright);
                break;

            case TrainingStage.Hover:
                CheckHover(distance, speed, upright);
                break;

            case TrainingStage.Landing:
                CheckLanding(speed, upright);
                break;
        }

        if (episodeFinished)
            return;

        // Failure: flies too far.
        if (distance > maxDistance)
        {
            FailEpisode();
            return;
        }

        // Failure: falls far below landing point.
        if (transform.position.y <
            landingTarget.position.y - 2f)
        {
            FailEpisode();
            return;
        }

        // Failure: overturns.
        if (upright < 0.3f)
        {
            FailEpisode();
        }
    }

    private void CheckApproach(
        float distance,
        float speed,
        float upright)
    {
        bool reachedTarget =
            distance < approachRadius &&
            speed < approachMaxSpeed &&
            upright > 0.8f;

        if (reachedTarget)
            CompleteStage();
    }

    private void CheckHover(
        float distance,
        float speed,
        float upright)
    {
        bool stable =
            distance < hoverRadius &&
            speed < hoverSpeed &&
            upright > 0.9f;

        if (stable)
        {
            stableSteps++;
            AddReward(0.003f);
        }
        else
        {
            stableSteps = 0;
        }

        if (stableSteps >= requiredHoverSteps)
            CompleteStage();
    }

    private void CheckLanding(
        float speed,
        float upright)
    {
        if (!RoverReady())
        {
            stableSteps = 0;
            return;
        }

        Vector3 difference =
            landingTarget.position - transform.position;

        float horizontalDistance =
            new Vector2(
                difference.x,
                difference.z
            ).magnitude;

        float verticalDistance =
            Mathf.Abs(difference.y);

        bool stableLanding =
            horizontalDistance < landingRadius &&
            verticalDistance < landingHeightTolerance &&
            speed < landingSpeed &&
            upright > 0.9f;

        if (stableLanding)
        {
            stableSteps++;
            AddReward(0.01f);
        }
        else
        {
            stableSteps = 0;
        }

        if (stableSteps >= requiredLandingSteps)
            CompleteStage();
    }

    private void RecordResult(bool success)
    {
        // Visible in TensorBoard custom statistics.
        Academy.Instance.StatsRecorder.Add(
            "Drone/SuccessRate",
            success ? 1f : 0f
        );
    }

    private void CompleteStage()
    {
        if (episodeFinished)
            return;

        episodeFinished = true;

        AddReward(successReward);
        RecordResult(true);

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

            Debug.Log("DRONE STAGE COMPLETED!");
        }
    }

    private void FailEpisode()
    {
        if (episodeFinished)
            return;

        episodeFinished = true;

        AddReward(failurePenalty);
        RecordResult(false);

        controller.ResetControls();

        if (trainingMode)
            EndEpisode();
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

            if (keyboard.dKey.isPressed)
                actions[1] = 1f;

            if (keyboard.aKey.isPressed)
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

        if (Input.GetKey(KeyCode.D))
            actions[1] = 1f;

        if (Input.GetKey(KeyCode.A))
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
