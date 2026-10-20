
using UnityEngine;

public class Level5ObstaclePlacer : MonoBehaviour
{
    [Header("Scene References")]
    public Terrain terrain;
    public Transform drone;
    public Transform meetingPoint;
    public Transform trainingObstacle;

    [Header("Random Placement")]
    public float lateralVariation = 1.5f;
    public int maxAttempts = 30;

    public void PlaceObstacle()
    {
        if (terrain == null ||
            drone == null ||
            meetingPoint == null ||
            trainingObstacle == null)
        {
            Debug.LogError(
                "Level5ObstaclePlacer: Assign all references!"
            );
            return;
        }

        BoxCollider box =
            trainingObstacle.GetComponent<BoxCollider>();

        if (box == null)
        {
            Debug.LogError(
                "TrainingObstacle needs a Box Collider!"
            );
            return;
        }

        // Temporarily exclude our own obstacle
        // from the placement collision check.
        box.enabled = false;
        Physics.SyncTransforms();

        Vector3 direction =
            meetingPoint.position - drone.position;

        direction.y = 0f;

        if (direction.sqrMagnitude < 36f)
        {
            trainingObstacle.gameObject.SetActive(false);
            return;
        }

        float distance = direction.magnitude;
        direction.Normalize();

        Vector3 sideways = new Vector3(
            -direction.z,
            0f,
            direction.x
        );

        Vector3 scale = trainingObstacle.lossyScale;

        Vector3 halfSize = new Vector3(
            Mathf.Abs(box.size.x * scale.x) * 0.5f,
            Mathf.Abs(box.size.y * scale.y) * 0.5f,
            Mathf.Abs(box.size.z * scale.z) * 0.5f
        );

        int obstacleMask =
            LayerMask.GetMask("Obstacle");

        for (int i = 0; i < maxAttempts; i++)
        {
            float fraction = i == 0
                ? 0.5f
                : Random.Range(0.4f, 0.6f);

            float sidewaysOffset = i == 0
                ? 0f
                : Random.Range(
                    -lateralVariation,
                    lateralVariation
                );

            Vector3 position =
                drone.position +
                direction * distance * fraction +
                sideways * sidewaysOffset;

            float groundY =
                terrain.SampleHeight(position) +
                terrain.transform.position.y;

            position.y =
                groundY + halfSize.y -
                box.center.y * scale.y;

            Vector3 colliderCenter =
                position +
                trainingObstacle.rotation *
                Vector3.Scale(box.center, scale);



            Vector3 toGoal = position - meetingPoint.position;
            toGoal.y = 0f;

            Vector3 toDrone = position - drone.position;
            toDrone.y = 0f;

            // Include the obstacle's physical width in the clearance.
            float obstacleRadius = Mathf.Max(
                halfSize.x,
                halfSize.z
            );

            // Keep the truck landing area clear.
            float requiredGoalDistance = 7f + obstacleRadius;

            // Give the drone room to start flying.
            float requiredDroneDistance = 5f + obstacleRadius;

            if (toGoal.magnitude < requiredGoalDistance ||
                toDrone.magnitude < requiredDroneDistance)
            {
                continue;
            }



            bool blocked = Physics.CheckBox(
                colliderCenter,
                halfSize,
                trainingObstacle.rotation,
                obstacleMask,
                QueryTriggerInteraction.Ignore
            );

            if (blocked)
                continue;

            trainingObstacle.position = position;
            trainingObstacle.gameObject.SetActive(true);
            box.enabled = true;

            Physics.SyncTransforms();

            Debug.Log(
                "LEVEL 5: Training obstacle repositioned!"
            );

            return;
        }

        // No safe position was found.
        // Skip this obstacle for the current episode.
        trainingObstacle.gameObject.SetActive(false);

        Debug.LogWarning(
            "LEVEL 5: Could not find a clear obstacle position."
        );
    }
}
