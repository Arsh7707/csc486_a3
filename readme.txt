BOIDS Assignment 3

Author: Arshjot Singh
Student ID: V00977388
Course: CSC 486A


1. Boid Initialization

 Correct and automatic initialization of the boids.
    - All boids are spawned in InitBoids() inside Swarm.cs.
    - Positions are initialized randomly within a sphere of radius `initializationRadius` around the Swarm GameObject using Random.insideUnitSphere.
    - Each boid’s forward heading is randomized within ±`initializationForwardRandomRange` degrees around the Y-axis using a yaw rotation.
    - Initial velocity is aligned with the forward direction and scaled to a random fraction of maxSpeed.


2. Resetting Values at Top of Simulation Loop

 Correct handling of resetting values.
    - ResetBoidForces() is called at the beginning of FixedUpdate().
    - Alignment, cohesion, separation, obstacle, and currentTotalForce are reset to Vector3.zero for all boids each physics step.

3. Neighbour List and Vision (Distance + Dot Product)

 Correct implementation of neighbour detection.
    - In ComputeRules(), for each boid i, all other boids j are checked.
    - Distance check: squared distance < sqrNeighbourDistance.
    - Vision check: Vector3.Dot(toNeighbour, boids[i].forward) > 0f implements a 180-degree FOV.
    - Only neighbours that pass both checks contribute to separation, alignment, and cohesion.

4. Separation Rule

 Correct implementation of separation.
    - For neighbours, a vector from neighbour to current boid is accumulated inversely weighted by distance.
    - After accumulation, the result is normalized and used as the separation rule.
    - The separation rule is applied as a steering force via:
      separationWeight * ((separationRule * boidForceScale) - velocity).

5. Alignment Rule

 Correct implementation of alignment.
    - Alignment is computed as the average of neighbour velocities.
    - This average is used as a target direction for velocity.
    - Applied through the steering formulation:
      alignmentWeight * ((alignmentRule * boidForceScale) - velocity).


6. Cohesion Rule

 Correct implementation of cohesion.
    - Cohesion is calculated as (average neighbour position - current position).
    - This vector points toward the centre of mass of nearby boids.
    - Applied using the standard steering formula:
      cohesionWeight * ((cohesionRule * boidForceScale) - velocity).


7. No-Neighbour Wander Rule

 Correct implementation of wander.
    - If a boid has zero neighbours, alignment, separation and cohesion are not used.
    - Instead, a wander rule is used equal to the boid’s current velocity.
    - This wander rule is applied as:
      wanderWeight * ((wanderRule * boidForceScale) - velocity).


8. Obstacles Rule

 Correct implementation of obstacle avoidance.
    - Obstacles are detected using Physics.OverlapSphere around each boid with radius obstacleCheckRadius.
    - For each collider, ClosestPoint is used to get the nearest point on the collider.
    - A normal vector from the closest point to the boid is computed.
    - If the boid is inside the collider (degenerate normal), the normal is reconstructed using the collider bounds center.
    - All normals are accumulated and used as the obstacle avoidance rule.
    - Applied as a steering force with obstacleWeight and boidForceScale.


9. World Boundary Rule

 World boundaries included as part of obstacle rule.
    - The assignment’s world bounds are enforced logically by using obstacle avoidance against colliders and/or environment geometry at the world edges.
    - The same obstacle rule mechanism is used so that boids are pushed away from geometry at the edges of the navigable world.
    - If the testcase scene contains explicit colliders for boundaries, they are handled exactly like other obstacles.


10. Total Force Accumulation

Correct accumulation of total forces.
    - For each boid, separation, alignment, cohesion, wander (exclusive with neighbours), obstacle, and (for boidZero) goal following are combined in ComputeRules().
    - Each rule uses the standard steering pattern:
      ω_k * ((ruleVector * boidForceScale) - velocity).
    - The sum of these contributions is stored as currentTotalForce for each boid.


11. SetGoal Implementation

 SetGoal implemented correctly.
    - SetGoal(Vector3 goal) samples the nearest NavMesh positions to both boidZero and the target goal using NavMesh.SamplePosition.
    - NavMesh.CalculatePath is used to compute the shortest path between these points.
    - The resulting corners are stored in boidZeroPath.
    - Navigation is only started if a valid complete path with at least two corners is found.
    - The function is non-interruptible while a path is active (ignores new goals until finished).


boidZero Path Handling and Corner Bookkeeping

 boidZero handled correctly.
    - HandlePathFollowing() checks that boidZero is navigating and that the path is valid.
    - The current boidZero position is projected onto the NavMesh via NavMesh.SamplePosition.
    - Distance to the current corner is measured; if within 1 unit, currentCorner is incremented.
    - When currentCorner >= number of corners, the path is finished; navigating is set to false and corner index reset.
    - boidZero uses a goal-following rule in ComputeRules() that steers toward the current path corner.
    - All bookkeeping for path start, progression, and completion is handled cleanly.


13. Boid Mesh / Animated Object Updates

[✔] Boid objects follow particle positions and headings.
    - After integration, each boid’s Transform (boidObjects[i]) position is set to the particle position.
    - Forward direction is derived from velocity and used in Quaternion.LookRotation.
    - Slerp is used with rotationSpeed to smoothly rotate the mesh toward the boid’s forward direction.


14. Symplectic Euler Integration

Symplectic Euler is used for integration.
    - In IntegrateMotion():
      1. Velocity is updated using currentTotalForce * dt (acceleration, with optional clamping).
      2. Position is then updated using the new velocity * dt.
    - This ordering (update velocity first, then position) corresponds to Symplectic Euler.


15. Simulator Loop and Timing

Simulator loop updates boids correctly.
    - The simulation loop is implemented inside FixedUpdate(), which matches Unity’s physics timestep.
    - At each FixedUpdate():
      - Forces are reset.
      - Path-following bookkeeping for boidZero is updated.
      - Neighbour and rule computations are performed.
      - Symplectic Euler integration is applied to all boids.
      - Animated mesh transforms are updated.
    - This follows the simulator structure taught in lecture.


16. Default Testcase Values

 Default testcase values preserved.
    - Public parameter defaults are kept reasonable and in line with the testcase expectations.
    - No additional components or external packages are used beyond the allowed AI Navigation package.


In order to run the program, please drag and drop SampleScene in the heirarchy!!


