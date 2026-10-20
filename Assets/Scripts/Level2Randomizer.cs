
using UnityEngine;

[DefaultExecutionOrder(-10000)]
public class Level2Randomizer : MonoBehaviour
{
    [Header("Scene References")]
    public Terrain terrain;
    public Transform truck;
    public Transform drone;
    public Transform meetingPoint;

    private bool heightsSaved;
    private float savedTruckHeight;
    private float savedDroneHeight;
    private float savedMeetingHeight;

    [Header("Randomization")]
    public float edgeMargin = 25f;

    [Header("Level 5 Obstacle-Safe Spawning")]
    public bool avoidObstacleSpawns = false;
    public float goalClearance = 12f;
    public float truckClearance = 4f;
    public float droneClearance = 3f;

    [Header("Training Setup")]
    public bool placeTruckAtGoalForDroneTraining = false;

    public Vector2 truckDistanceRange =
        new Vector2(25f, 45f);

    public Vector2 droneDistanceRange =
        new Vector2(20f, 40f);

    private void Awake()
    {
        RandomizePositions();
    }

    public void RandomizePositions()
    {
        if (terrain == null ||
            truck == null ||
            drone == null ||
            meetingPoint == null)
        {
            Debug.LogError(
                "Level2Randomizer: Assign all references!"
            );
            return;
        }

        Vector3 terrainSize = terrain.terrainData.size;

        if (terrainSize.x <= edgeMargin * 2f ||
            terrainSize.z <= edgeMargin * 2f)
        {
            Debug.LogError(
                "Level2Randomizer: Terrain is too small for the margin."
            );
            return;
        }

        // Save original heights above the terrain.
        if (!heightsSaved)
        {
            savedTruckHeight =
                truck.position.y - GroundY(truck.position);

            savedDroneHeight =
                drone.position.y - GroundY(drone.position);

            savedMeetingHeight =
                meetingPoint.position.y -
                GroundY(meetingPoint.position);

            heightsSaved = true;
        }

        float truckHeight = savedTruckHeight;
        float droneHeight = savedDroneHeight;
        float meetingHeight = savedMeetingHeight;

        // Find a random, clear MeetingPoint.
        Vector2 goalXZ = Vector2.zero;
        bool goalFound = false;

        for (int attempt = 0; attempt < 100; attempt++)
        {
            Vector2 candidate = RandomTerrainPoint();

            if (!avoidObstacleSpawns ||
                IsSpawnClear(candidate, goalClearance))
            {
                goalXZ = candidate;
                goalFound = true;
                break;
            }
        }

        if (!goalFound)
        {
            Debug.LogError(
                "Level2Randomizer: No clear MeetingPoint found."
            );
            return;
        }

        // Find the truck starting position.
        Vector2 truckXZ;

        if (placeTruckAtGoalForDroneTraining)
        {
            // Keep truck at MeetingPoint while
            // training the drone.
            truckXZ = goalXZ;
        }
        else
        {
            if (!TryRandomPointNear(
                goalXZ,
                truckDistanceRange,
                out truckXZ,
                truckClearance))
            {
                Debug.LogError(
                    "Level2Randomizer: Could not place the truck."
                );
                return;
            }
        }

        // Find a clear drone starting position
        // relative to MeetingPoint.
        Vector2 droneXZ;

        if (!TryRandomPointNear(
            goalXZ,
            droneDistanceRange,
            out droneXZ,
            droneClearance))
        {
            Debug.LogError(
                "Level2Randomizer: Could not place the drone."
            );
            return;
        }

        // Convert 2D positions to world positions.
        Vector3 goalPosition = WorldPosition(
            goalXZ,
            meetingHeight
        );

        Vector3 truckPosition = WorldPosition(
            truckXZ,
            truckHeight
        );

        Vector3 dronePosition = WorldPosition(
            droneXZ,
            Mathf.Max(6f, droneHeight)
        );

        // Face the truck toward MeetingPoint.
        Vector3 truckDirection =
            goalPosition - truckPosition;

        truckDirection.y = 0f;

        Quaternion truckRotation = Quaternion.identity;

        if (truckDirection.sqrMagnitude > 0.0001f)
        {
            truckRotation =
                Quaternion.LookRotation(truckDirection);
        }

        // Face the drone toward MeetingPoint.
        Vector3 droneDirection =
            goalPosition - dronePosition;

        droneDirection.y = 0f;

        Quaternion droneRotation = Quaternion.identity;

        if (droneDirection.sqrMagnitude > 0.0001f)
        {
            droneRotation =
                Quaternion.LookRotation(droneDirection);
        }

        // Apply the randomized positions.
        SetPose(
            meetingPoint,
            goalPosition,
            meetingPoint.rotation
        );

        SetPose(
            truck,
            truckPosition,
            truckRotation
        );

        SetPose(
            drone,
            dronePosition,
            droneRotation
        );


        Physics.SyncTransforms();

        // Level 5: place a new obstacle after
        // the drone and MeetingPoint are randomized.
        Level5ObstaclePlacer obstaclePlacer =
            GetComponent<Level5ObstaclePlacer>();

        if (obstaclePlacer != null)
        {
            obstaclePlacer.PlaceObstacle();
        }

        Debug.Log(
            "LEVEL 2: New random positions generated!"
        );

    }

    private float GroundY(Vector3 position)
    {
        return terrain.SampleHeight(position) +
               terrain.transform.position.y;
    }

    private Vector3 WorldPosition(
        Vector2 xz,
        float heightAboveGround)
    {
        Vector3 position = new Vector3(
            xz.x,
            0f,
            xz.y
        );

        position.y =
            GroundY(position) + heightAboveGround;

        return position;
    }

    private Vector2 RandomTerrainPoint()
    {
        Vector3 origin = terrain.transform.position;
        Vector3 size = terrain.terrainData.size;

        return new Vector2(
            Random.Range(
                origin.x + edgeMargin,
                origin.x + size.x - edgeMargin
            ),
            Random.Range(
                origin.z + edgeMargin,
                origin.z + size.z - edgeMargin
            )
        );
    }

    private bool TryRandomPointNear(
        Vector2 center,
        Vector2 distanceRange,
        out Vector2 point,
        float clearance)
    {
        Vector3 origin = terrain.transform.position;
        Vector3 size = terrain.terrainData.size;

        for (int attempt = 0; attempt < 100; attempt++)
        {
            float angle = Random.Range(
                0f,
                Mathf.PI * 2f
            );

            float distance = Random.Range(
                distanceRange.x,
                distanceRange.y
            );

            Vector2 candidate = center +
                new Vector2(
                    Mathf.Cos(angle),
                    Mathf.Sin(angle)
                ) * distance;

            bool inside =
                candidate.x >= origin.x + edgeMargin &&
                candidate.x <= origin.x + size.x - edgeMargin &&
                candidate.y >= origin.z + edgeMargin &&
                candidate.y <= origin.z + size.z - edgeMargin;

            if (inside &&
                (!avoidObstacleSpawns ||
                 IsSpawnClear(candidate, clearance)))
            {
                point = candidate;
                return true;
            }
        }

        point = Vector2.zero;
        return false;
    }


    private bool IsSpawnClear(Vector2 xz, float clearance)
    {
        int obstacleLayer = LayerMask.NameToLayer("Obstacle");

        if (obstacleLayer < 0)
        {
            Debug.LogError("Obstacle layer missing!");
            return false;
        }

        // Check the space from ground level upward
        // so buildings cannot contain the spawn location.
        Vector3 bottom = WorldPosition(xz, 1f);
        Vector3 top = WorldPosition(xz, 25f);

        return !Physics.CheckCapsule(
            bottom,
            top,
            Mathf.Max(0.1f, clearance),
            1 << obstacleLayer,
            QueryTriggerInteraction.Ignore
        );
    }


    private void SetPose(
        Transform target,
        Vector3 position,
        Quaternion rotation)
    {
        target.SetPositionAndRotation(
            position,
            rotation
        );

        Rigidbody body =
            target.GetComponent<Rigidbody>();

        if (body != null)
        {
            body.position = position;
            body.rotation = rotation;

            if (!body.isKinematic)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
    }
}
