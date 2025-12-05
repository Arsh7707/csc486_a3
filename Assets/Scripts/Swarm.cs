using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;

        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;

        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 10f;
    public float maxSpeed = 4f;
    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;
    public float separationWeight = 1.4f;
    public float alignmentWeight = 0.4f;
    public float cohesionWeight = 0.6f;
    public float wanderWeight = 0.25f;
    public float obstacleWeight = 1.75f;
    public float goalWeight = 1f;

    public float neighbourDistance = 2.2f;
    public float initializationRadius = 4.5f;
    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;
    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    // PATH FOLLOWING
    private NavMeshPath boidZeroPath;
    private bool boidZeroNavigatingTowardGoal = false;
    private int currentCorner = 0;

    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        InitBoids();
    }

    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        for (int i = 0; i < numberOfBoids; i++)
        {
            Vector3 pos = transform.position + Random.insideUnitSphere * initializationRadius;

            float yaw = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
            Vector3 fwd = Quaternion.Euler(0f, yaw, 0f) * Vector3.forward;
            fwd.Normalize();

            Vector3 vel = fwd * Random.Range(0.1f * maxSpeed, 0.55f * maxSpeed);

            boids[i].position = pos;
            boids[i].forward = fwd;
            boids[i].velocity = vel;

            boidObjects[i] = Instantiate(boidPrefab, pos, Quaternion.LookRotation(fwd, Vector3.up), transform);
        }
    }

    private void ResetBoidForces()
    {
        for (int i = 0; i < numberOfBoids; i++)
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;

            boids[i].currentTotalForce = Vector3.zero;
        }
    }

    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0) return;

        ResetBoidForces();
        HandlePathFollowing();
        ComputeRules();
        IntegrateMotion();
    }

    private void HandlePathFollowing()
    {
        if (!boidZeroNavigatingTowardGoal || boidZeroPath == null)
            return;

        // Path invalid
        if (boidZeroPath.status != NavMeshPathStatus.PathComplete ||
            boidZeroPath.corners == null ||
            boidZeroPath.corners.Length < 2)
        {
            boidZeroNavigatingTowardGoal = false;
            currentCorner = 0;
            return;
        }

        NavMeshHit hit;
        if (!NavMesh.SamplePosition(boids[0].position, out hit, 2.0f, NavMesh.AllAreas))
            return;

        float dist = Vector3.Distance(hit.position, boidZeroPath.corners[currentCorner]);

        // Switch corners
        if (dist < 1.0f)
        {
            currentCorner++;

            if (currentCorner >= boidZeroPath.corners.Length)
            {
                boidZeroNavigatingTowardGoal = false;
                currentCorner = 0;
                return;
            }
        }
    }

    private void ComputeRules()
    {
        int count = boids.Length;

        for (int i = 0; i < count; i++)
        {
            int neighbourCount = 0;

            Vector3 alignmentAccum = Vector3.zero;
            Vector3 cohesionAccum = Vector3.zero;
            Vector3 separationAccum = Vector3.zero;


            for (int j = 0; j < count; j++)
            {
                if (i == j) continue;

                Vector3 diff = boids[j].position - boids[i].position;
                float sqr = diff.sqrMagnitude;

                if (sqr < sqrNeighbourDistance && Vector3.Dot(diff, boids[i].forward) > 0f)
                {
                    neighbourCount++;

                    alignmentAccum += boids[j].velocity;
                    cohesionAccum += boids[j].position;

                    float dist = Mathf.Sqrt(sqr) + 1e-5f;
                    separationAccum += (boids[i].position - boids[j].position) / dist;
                }
            }

            Vector3 alignmentRule = Vector3.zero;
            Vector3 cohesionRule = Vector3.zero;
            Vector3 separationRule = Vector3.zero;
            Vector3 wanderRule = Vector3.zero;

            if (neighbourCount > 0)
            {
                alignmentRule = alignmentAccum / neighbourCount;
                cohesionRule = (cohesionAccum / neighbourCount) - boids[i].position;
                separationRule = separationAccum.normalized;
            }
            else
            {
                wanderRule = boids[i].velocity;
            }

            boids[i].alignment = alignmentRule;
            boids[i].cohesion = cohesionRule;
            boids[i].separation = separationRule;

            Vector3 obstacleRule = Vector3.zero;
            Collider[] cols = Physics.OverlapSphere(boids[i].position, obstacleCheckRadius);

            foreach (var col in cols)
            {
                Vector3 closest = col.ClosestPoint(boids[i].position);
                Vector3 normal = boids[i].position - closest;


                if (normal.sqrMagnitude < 0.0001f)
                {
                    normal = boids[i].position - col.bounds.center;
                }

                normal = normal.normalized;
                obstacleRule += normal * 2.0f;
            }

            boids[i].obstacle = obstacleRule;

            Vector3 total = Vector3.zero;

            if (neighbourCount > 0)
            {
                total += separationWeight * ((separationRule * boidForceScale) - boids[i].velocity);
                total += alignmentWeight * ((alignmentRule * boidForceScale) - boids[i].velocity);
                total += cohesionWeight * ((cohesionRule * boidForceScale) - boids[i].velocity);
            }
            else
            {
                total += wanderWeight * ((wanderRule * boidForceScale) - boids[i].velocity);
            }

            total += obstacleWeight * ((obstacleRule * boidForceScale) - boids[i].velocity);


            if (i == 0 && boidZeroNavigatingTowardGoal && currentCorner < boidZeroPath.corners.Length)
            {
                Vector3 goalVec = boidZeroPath.corners[currentCorner] - boids[i].position;
                total += goalWeight * ((goalVec * boidForceScale) - boids[i].velocity);
            }

            boids[i].currentTotalForce = total;
        }
    }

    private void IntegrateMotion()
    {
        float dt = Time.fixedDeltaTime;

        for (int i = 0; i < numberOfBoids; i++)
        {
            Vector3 accel = boids[i].currentTotalForce;

            float maxAccel = 20f;
            if (accel.magnitude > maxAccel)
                accel = accel.normalized * maxAccel;

            boids[i].velocity += accel * dt;

            if (boids[i].velocity.magnitude > maxSpeed)
                boids[i].velocity = boids[i].velocity.normalized * maxSpeed;

            boids[i].position += boids[i].velocity * dt;

            if (boids[i].velocity.sqrMagnitude > 1e-6f)
                boids[i].forward = boids[i].velocity.normalized;

            boidObjects[i].position = boids[i].position;

            boidObjects[i].rotation = Quaternion.Slerp(
                boidObjects[i].rotation,
                Quaternion.LookRotation(boids[i].forward, Vector3.up),
                rotationSpeed * Time.fixedDeltaTime
            );
        }
    }


    public void SetGoal(Vector3 goal)
    {
        if (boidZeroNavigatingTowardGoal)
            return;

        boidZeroPath = new NavMeshPath();

        NavMeshHit startHit, endHit;

        if (!NavMesh.SamplePosition(boids[0].position, out startHit, 5f, NavMesh.AllAreas))
            return;

        if (!NavMesh.SamplePosition(goal, out endHit, 5f, NavMesh.AllAreas))
            return;

        if (!NavMesh.CalculatePath(startHit.position, endHit.position, NavMesh.AllAreas, boidZeroPath))
            return;

        if (boidZeroPath.status != NavMeshPathStatus.PathComplete ||
            boidZeroPath.corners.Length < 2)
            return;

        currentCorner = 1;
        boidZeroNavigatingTowardGoal = true;
    }
}
