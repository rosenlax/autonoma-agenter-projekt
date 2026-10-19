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
    public TrainingStage trainingStage = TrainingStage.Approach;
    public bool roverReadyAtStart = true;

    [Header("Navigation")]
    public float hoverHeight = 3f;
    public float maxDistance = 100f;
    public float maxSpeed = 15f;

    [Header("Observation Precision")]
    public float observationPositionScale = 40f;
    public float observationVelocityScale = 5f;

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

    [Header("Automatic Landing (Inference Only)")]
    [Tooltip("How quickly the drone lowers its landing target, in metres per second.")]
    public float landingDescentSpeed = 0.45f;

    [Tooltip("Final height of the DRONE CENTRE above DroneLandingPoint. Use about 0.22 if the point is on the flatbed surface; use 0 if the point is already at the drone-centre resting height.")]
    public float landingFinalHeightOffset = 0.22f;

    [Tooltip("For isolated landing tests only. Disable for the final rover + drone demonstration.")]
    public bool simulateRoverArrivalForTesting = false;

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
    private TrainingStage startingStage;

    private float previousDistance;
    private Vector3 previousAction;
    private bool hasPreviousAction;

    private int stableSteps;
    private bool hasLanded;
    private bool episodeFinished;
    private float landingDescentHeight;

    public bool HasLanded => hasLanded;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        controller = GetComponent<DroneController>();

        startPosition = transform.position;
        startRotation = transform.rotation;
        startingStage = trainingStage;

        controller.useAgentControl = true;
        controller.DisablePositionHold();
        controller.ResetControls();
    }

    private bool RoverReady()
    {
        // Preserve the behavior used to train the Approach model.
        if (trainingStage != TrainingStage.Landing)
            return true;

        if (trainingMode)
            return roverReadyAtStart;

        return HasRoverArrivedForLanding();
    }

    private bool HasRoverArrivedForLanding()
    {
        return (rover != null && rover.HasArrived) ||
               simulateRoverArrivalForTesting;
    }

    private Vector3 GetHoverTarget()
    {
        if (landingTarget == null)
            return transform.position;

        return landingTarget.position + Vector3.up * hoverHeight;
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

        Vector3 difference = landingTarget.position - transform.position;

        Vector2 horizontal = new Vector2(difference.x, difference.z);

        if (horizontal.magnitude > hoverRadius)
            return hoverTarget;

        return landingTarget.position;
    }

    public override void OnEpisodeBegin()
    {
        // An inference retry must start from the Inspector's original stage.
        if (!trainingMode)
            trainingStage = startingStage;

        hasLanded = false;
        episodeFinished = false;
        stableSteps = 0;
        landingDescentHeight = hoverHeight;

        previousAction = Vector3.zero;
        hasPreviousAction = false;

        rb.isKinematic = false;
        transform.SetPositionAndRotation(startPosition, startRotation);
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        controller.useAgentControl = true;
        controller.DisablePositionHold();
        controller.ResetControls();

        // Allow an optional direct Hover test in inference mode.
        if (!trainingMode &&
            trainingStage == TrainingStage.Hover &&
            landingTarget != null)
        {
            controller.SetPositionHoldTarget(GetHoverTarget());
        }

        previousDistance = Vector3.Distance(
            transform.position, GetNavigationTarget());
    }

    // In inference mode, position hold flies Hover and the controlled descent.
    // The trained policy still controls Approach without any changes.
    private void FixedUpdate()
    {
        if (trainingMode || episodeFinished ||
            landingTarget == null || controller == null)
            return;

        if (trainingStage == TrainingStage.Hover)
        {
            controller.SetPositionHoldTarget(GetHoverTarget());
        }
        else if (trainingStage == TrainingStage.Landing)
        {
            // Abort the descent if the rover is no longer ready.
            if (!HasRoverArrivedForLanding())
            {
                trainingStage = TrainingStage.Hover;
                stableSteps = 0;
                controller.SetPositionHoldTarget(GetHoverTarget());
                Debug.LogWarning("ROVER NOT READY: Returning to HOVER.");
                return;
            }

            Vector3 landingDifference =
                landingTarget.position - rb.position;
            float horizontalError = new Vector2(
                landingDifference.x, landingDifference.z).magnitude;
            float horizontalSpeed = new Vector2(
                rb.linearVelocity.x, rb.linearVelocity.z).magnitude;

            // Avoid descending beside the truck: hold the current height
            // until centred over the intended landing point and slowed down.
            if (horizontalError < landingRadius * 0.75f &&
                horizontalSpeed < hoverSpeed)
            {
                landingDescentHeight = Mathf.MoveTowards(
                    landingDescentHeight,
                    Mathf.Max(0f, landingFinalHeightOffset),
                    Mathf.Max(0f, landingDescentSpeed) * Time.fixedDeltaTime);
            }

            controller.SetPositionHoldTarget(
                landingTarget.position + Vector3.up * landingDescentHeight);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Exactly the same 11 observations as the trained Approach model.
        if (landingTarget == null)
        {
            for (int i = 0; i < 11; i++)
                sensor.AddObservation(0f);

            return;
        }

        Vector3 difference = GetNavigationTarget() - transform.position;

        Vector3 localTarget = transform.InverseTransformDirection(difference);

        // 1-3: target direction and distance.
        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localTarget / observationPositionScale, 1f));

        Vector3 localVelocity = transform.InverseTransformDirection(
            rb.linearVelocity);

        // 4-6: velocity.
        sensor.AddObservation(
            Vector3.ClampMagnitude(
                localVelocity / observationVelocityScale, 1f));

        // 7-9: orientation.
        sensor.AddObservation(
            transform.InverseTransformDirection(Vector3.up));

        // 10: rover ready.
        sensor.AddObservation(RoverReady() ? 1f : 0f);

        // 11: horizontal target distance.
        Vector3 flatDifference = difference;
        flatDifference.y = 0f;

        sensor.AddObservation(
            Mathf.Clamp01(flatDifference.magnitude / observationPositionScale));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (episodeFinished || hasLanded)
        {
            controller.ResetControls();
            return;
        }

        float pitch = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float roll = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float lift = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        Vector3 currentAction = new Vector3(pitch, roll, lift);

        // In Hover, the controller overrides these commands each FixedUpdate.
        controller.SetAgentInput(pitch, roll, lift);

        if (landingTarget == null)
            return;

        Vector3 target = GetNavigationTarget();
        float distance = Vector3.Distance(transform.position, target);
        float speed = rb.linearVelocity.magnitude;
        float upright = Vector3.Dot(transform.up, Vector3.up);

        float progress = previousDistance - distance;
        AddReward(progress * progressReward);
        AddReward(stepPenalty);
        previousDistance = distance;

        if (hasPreviousAction)
        {
            float change = Vector3.Distance(currentAction, previousAction);
            AddReward(-change * actionChangePenalty);
        }

        AddReward(-currentAction.sqrMagnitude * actionStrengthPenalty);

        previousAction = currentAction;
        hasPreviousAction = true;

        float proximity = 1f - Mathf.Clamp01(distance / 5f);

        if (upright > 0.9f)
            AddReward(proximity * stableNearTargetReward);

        if (distance < 5f)
        {
            AddReward(-distance * 0.001f);
            AddReward(-speed * 0.001f);
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

        if (distance > maxDistance)
        {
            FailEpisode();
            return;
        }

        if (transform.position.y < landingTarget.position.y - 2f)
        {
            FailEpisode();
            return;
        }

        if (upright < 0.3f)
            FailEpisode();
    }


private void CheckApproach(float distance, float speed, float upright)
{
    // Keep original training conditions unchanged.
    // This preserves compatibility with the trained model.
    if (trainingMode)
    {
        bool reachedTarget =
            distance < approachRadius &&
            speed < approachMaxSpeed &&
            upright > 0.8f;

        if (reachedTarget)
            CompleteStage();

        return;
    }

    // INFERENCE ONLY:
    // Switch to automatic position hold earlier
    // to brake before reaching the truck.
    float earlyHoverDistance = 5f;

    if (distance >= earlyHoverDistance)
        return;

    // Hand over control to the hover controller.
    trainingStage = TrainingStage.Hover;
    stableSteps = 0;

    controller.SetPositionHoldTarget(GetHoverTarget());

    Debug.Log(
        "DRONE APPROACH HANDOFF! Switching to automatic HOVER."
    );
}


    private void CheckHover(float distance, float speed, float upright)
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
        {
            if (trainingMode)
            {
                CompleteStage();
            }
            else
            {
                if (stableSteps == requiredHoverSteps)
                    Debug.Log("DRONE HOVER STABLE! Waiting for rover.");

                if (HasRoverArrivedForLanding())
                {
                    trainingStage = TrainingStage.Landing;
                    stableSteps = 0;

                    // Start from the actual altitude to avoid a sudden
                    // downward change in the position-hold target.
                    landingDescentHeight = Mathf.Max(
                        Mathf.Max(0f, landingFinalHeightOffset),
                        transform.position.y - landingTarget.position.y);

                    controller.SetPositionHoldTarget(
                        landingTarget.position + Vector3.up * landingDescentHeight);

                    Debug.Log("ROVER READY! DRONE STARTING CONTROLLED DESCENT.");
                }
            }
        }
    }

    private void CheckLanding(float speed, float upright)
    {
        if (!RoverReady())
        {
            stableSteps = 0;
            return;
        }

        Vector3 difference = landingTarget.position - transform.position;

        float horizontalDistance = new Vector2(
            difference.x, difference.z).magnitude;

        // The landing point normally marks the bed surface, while the
        // drone's transform is at its centre (half its collider height up).
        float landingCentreY = landingTarget.position.y +
            (trainingMode ? 0f : Mathf.Max(0f, landingFinalHeightOffset));
        float verticalDistance = Mathf.Abs(
            transform.position.y - landingCentreY);

        // Do not claim landing before the target has reached the final height.
        bool descentFinished = landingDescentHeight <=
            Mathf.Max(0f, landingFinalHeightOffset) + 0.05f;

        bool stableLanding =
            (trainingMode || descentFinished) &&
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
        Academy.Instance.StatsRecorder.Add(
            "Drone/SuccessRate", success ? 1f : 0f);
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
            // Inference freezes only on actual Landing completion.
            hasLanded = true;
            controller.DisablePositionHold();

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.isKinematic = true;

            Debug.Log("DRONE LANDED! Stable at the truck landing point.");
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
        controller.DisablePositionHold();

        if (!trainingMode)
            Debug.LogWarning("DRONE FAILED! Restarting episode.");

        EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
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
            controller.DisablePositionHold();
            controller.ResetControls();
            controller.useAgentControl = false;
        }

        if (rb != null)
            rb.isKinematic = false;

        base.OnDisable();
    }
}
