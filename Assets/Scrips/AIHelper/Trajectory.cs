using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathCreation;
using Logging;
using Analysis;
using MathNet.Numerics;

[RequireComponent(typeof(PathCreator))]
public class Trajectory : MonoBehaviour
{
    [Header("Debug options")]
    public bool debug = false; //True is the function optimize has been called since the last modification of the waypoints.
    public bool showGraph = false;
    public bool showWaypoints = false;


    private bool bezierForm = false; //True is the function optimize has been called since the last modification of the waypoints.
    private List<Vector3> waypoints = new List<Vector3>();

    private List<float[]> speedAtDistance;
    private List<float[]> targetSpeedAtDistance;

    TerrainManager terrain_manager;

    private PathCreator pathCreator;
    private List<Node> nodes;

    private int subSamplingX = 2;
    private int subSamplingZ = 2;


    void Start()
    {
        pathCreator = GetComponent<PathCreator>();

        findShortestPath();

        OptimizeTrajectory();

        Debug.Log("Travel time: " + ComputeTravelTime());
    }


    private void findShortestPath()
    {
        terrain_manager = GameObject.FindObjectOfType<TerrainManager>();


        (Node entryNode, Node goalNode) = createGraph();

        // Vector2Int startNode = terrain_manager.myInfo.coordinatesToNode(terrain_manager.myInfo.start_pos);

        Node[] pathNodes = AStar.ComputeShortestPath(entryNode, goalNode);

        addPointToTrajectory(terrain_manager.myInfo.start_pos);
        for (int i = 1; i < pathNodes.Length - 1; i++)
        {
            addPointToTrajectory(pathNodes[i].position);
        }
        addPointToTrajectory(terrain_manager.myInfo.goal_pos);
    }

    public void setTrajectoryAsSuccessiveWaypoints(Vector3[] waypoints)
    {
        bezierForm = false;
        this.waypoints = new List<Vector3>(waypoints);
    }

    public void addPointToTrajectory(Vector3 point)
    {
        bezierForm = false;
        waypoints.Add(point);
    }

    private (Node, Node) createGraph()
    {
        Node startNode = new Node(terrain_manager.myInfo.start_pos);
        Node goalNode = new Node(terrain_manager.myInfo.goal_pos);
        nodes = new List<Node>();
        nodes.Add(startNode);
        nodes.Add(goalNode);

        float[,] traversability = terrain_manager.myInfo.traversability;
        for (int i = 1; i < traversability.GetLength(0) - 1; i++)
        {
            for (int j = 1; j < traversability.GetLength(1) - 1; j++)
            {
                if (traversability[i, j] < 0.5) continue; //Traversable so no corner

                for (int u = 0; u < 4; u++)
                {
                    Vector2Int neighborSides = new Vector2Int((u / 2) * 2 - 1, (u % 2) * 2 - 1);
                    if (traversability[i + neighborSides.x, j] > 0.5 ||
                        traversability[i, j + neighborSides.y] > 0.5 ||
                        traversability[i + neighborSides.x, j + neighborSides.y] > 0.5) continue;

                    Vector3 position = terrain_manager.myInfo.nodeToCoordinates(new Vector2Int(i, j)) +
                                        new Vector3(terrain_manager.myInfo.getCubeSize()[0] / 2 * neighborSides.x, 0, terrain_manager.myInfo.getCubeSize()[1] / 2 * neighborSides.y);
                    Debug.DrawLine(position, position + 5 * new Vector3(neighborSides.x, 0f, neighborSides.y), Color.green, 5000f);

                    if (Physics.Raycast(position, new Vector3(neighborSides.x, 0f, neighborSides.y), 5f)) continue;

                    nodes.Add(new Node(position + 5 * new Vector3(neighborSides.x, 0f, neighborSides.y)));
                }

            }
        }

        List<Node> visited = new List<Node>();
        Queue<Node> worklist = new Queue<Node>();

        Node currentNode = startNode;
        visited.Add(currentNode);
        worklist.Enqueue(currentNode);

        while (worklist.Count != 0)
        {
            currentNode = worklist.Dequeue();

            foreach (Node neighbor in nodes)
            {
                if (!currentNode.neighbors.Contains(neighbor) && !Physics.Raycast(currentNode.position, neighbor.position - currentNode.position, Vector3.Distance(currentNode.position, neighbor.position)))
                {
                    visited.Add(neighbor);
                    worklist.Enqueue(neighbor);

                    currentNode.neighbors.Add(neighbor);
                }
            }
        }

        return (startNode, goalNode);
    }




    #region Getters and accesseurs =====================================================================================================


    public Vector3 getTangentAhead(Vector3 position, float speed)
    {
        Vector3 tangentAtClosestPoint = Vector3.zero;

        if (!bezierForm)
        {
            Vector3 closestPoint = Vector3.positiveInfinity;
            for (int i = 0; i < waypoints.Count - 1; i++) //We loop through all the segment and find the closest point on each of those segments. Then we keep the best.
            {
                Vector3 line = (waypoints[i + 1] - waypoints[i]); //Line on which to project
                Vector3 distanceFromWaypoint = Vector3.Project(position - waypoints[i], line.normalized);
                Vector3 projectedPoint = waypoints[i] + distanceFromWaypoint;

                if (Vector3.Distance(projectedPoint, position) <= Vector3.Distance(closestPoint, position) && distanceFromWaypoint.magnitude < line.magnitude)
                {
                    closestPoint = projectedPoint;
                    tangentAtClosestPoint = line.normalized;
                }
            }
        }
        else
        {
            float distance = pathCreator.path.GetClosestDistanceAlongPath(position);
            distance = distance + 5f + 0.1f * speed; // + speed * 0.5f; //The faster we go, the further away we look for the tangent
            tangentAtClosestPoint = pathCreator.path.GetDirection(distance / pathCreator.path.length);
        }


        return tangentAtClosestPoint;
    }

    public Vector3 getClosestPoint(Vector3 position)
    {
        Vector3 closestPoint = Vector3.positiveInfinity;
        if (!bezierForm)
        {
            for (int i = 0; i < waypoints.Count - 1; i++) //We loop through all the segment and find the closest point on each of those segments. Then we keep the best.
            {
                Vector3 line = (waypoints[i + 1] - waypoints[i]); //Line on which to project
                Vector3 distanceFromWaypoint = Vector3.Project(position - waypoints[i], line.normalized);
                Vector3 projectedPoint = waypoints[i] + distanceFromWaypoint;

                if (Vector3.Distance(projectedPoint, position) <= Vector3.Distance(closestPoint, position) && distanceFromWaypoint.magnitude < line.magnitude)
                {
                    closestPoint = projectedPoint;
                }
            }
        }
        else
        {
            closestPoint = pathCreator.path.GetClosestPointOnPath(position);
        }

        return closestPoint;
    }

    public (float, float) GetSpeedAtPosition(Vector3 position)
    {
        float distanceFromStart = pathCreator.path.GetClosestDistanceAlongPath(position);
        Debug.Log("Distance: " + distanceFromStart);

        for (int i = 1; i < targetSpeedAtDistance.Count; i++)
        {
            if (targetSpeedAtDistance[i][0] > distanceFromStart)
            {
                return (distanceFromStart, targetSpeedAtDistance[i][1]);
            }
        }
        return (distanceFromStart, -1f);
    }

    #endregion

    #region Trajectory optimization ==========================================================================

    public void OptimizeTrajectory()
    {
        bezierForm = true;
        //Transform the series of waypoints in a smooth series of bezier
        fromWaypointsToBezier();

        gradientMinimization();
    }

    private void fromWaypointsToBezier()
    {
        List<Vector3> controlPoints = new List<Vector3>();
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            controlPoints.Add(waypoints[i]);
            Vector3 middlePoint = (waypoints[i] + waypoints[i + 1]) / 2;
            controlPoints.Add(middlePoint);
        }
        controlPoints.Add(waypoints[waypoints.Count - 1]);

        BezierPath bezierPath = new BezierPath(controlPoints, false, PathSpace.xz);
        bezierPath.ControlPointMode = BezierPath.ControlMode.Aligned;
        bezierPath.MovePoint(1, terrain_manager.myInfo.start_pos + new Vector3(0, 0, 5));

        pathCreator.bezierPath = bezierPath;
    }

    private void gradientMinimization()
    {
        //TODO use math.NET to do gradient descent
        // FindMinimum.OfFunctionGradientConstrained()
    }


    //From the curvature profile (curvature as a function of the distance from the start) 
    //and with the car properties we can compute an estimation of the travel time of the curve (necessary for later optimization)
    public float ComputeTravelTime()
    {
        List<float[]> radiusData = pathCreator.bezierPath.RadiusProfile();

        List<float[]> maxSpeedData = new List<float[]>();
        for (int i = 0; i < radiusData.Count; i++)
        {
            maxSpeedData.Add(new float[] { radiusData[i][0], SpeedAtRadius(radiusData[i][1]) });
        }
        maxSpeedData[0] = new float[] { radiusData[0][0], 0f };
        List<float> constraintData = MathHelper.BestFunctionWithSlopeConstraints(maxSpeedData, AccelerationAtSpeed, DeccelerationAtSpeed);

        for (int i = 0; i < radiusData.Count; i++)
        {
            radiusData[i] = new float[] { radiusData[i][0], radiusData[i][1], maxSpeedData[i][1], constraintData[i] };
        }

        targetSpeedAtDistance = MathHelper.pieceWiseConstantFromSpeedProfile(radiusData);
        LogRadiusHistogram(radiusData, "speedData");
        LogRadiusHistogram(targetSpeedAtDistance, "targetSpeedData");
        speedAtDistance = radiusData;

        float travelTime = 0f;
        for (int i = 0; i < constraintData.Count - 1; i++)
        {
            if (constraintData[i] > 2f) //To prevent the start of the car to artificially increase the travel time (speed of zero => infinite time)
            {
                travelTime += (radiusData[i + 1][0] - radiusData[i][0]) / (constraintData[i]); //The distance between consecutive steps / the speed at the step
            }
        }
        return travelTime;
    }

    public float GetCurvature(Vector3 position)
    {
        float time = pathCreator.path.GetClosestTimeOnPath(position);
        float curvature = pathCreator.path.GetCurvature(time, EndOfPathInstruction.Stop);

        return curvature;
    }
    #endregion

    #region Car properties =====================================================================================

    private float[] sampleCurvatures = new float[] { 32.8f, 21.8f, 16.28f, 12.95f, 10.71f, 9.10f, 7.89f, 6.93f, 6.15f };
    public float SpeedAtRadius(float radius)
    {
        radius = Mathf.Clamp(radius, 6f, 33f); //Range of values in which the speed-curvature function is valid

        //Equation between the curvature (in meter) and the speed (in m/s). Found experimentally.
        return 9.56f + 0.931f * radius - 5.94E-3f * radius * radius;
    }

    public float AccelerationAtSpeed(float speed)
    {
        float accel = 0f;

        if (speed > 5 && speed < 49)
        {
            accel = 6.78f - 0.104f * speed;
        }
        else if (speed <= 5)
        {
            accel = 0.22f + 2.25f * speed - 0.187f * speed * speed;
        }
        return accel;
    }

    public float DeccelerationAtSpeed(float speed)
    {
        // return 3.12f + 5.92f * speed;
        return 0.01f;
    }
    #endregion





    #region Logging and gizmos

    public void LogRadiusHistogram(List<float[]> radiusData, string filename = "export")
    {
        DataLogger dataLogger = new DataLogger(radiusData);
        dataLogger.SaveToFile("Time, Radius, MaxSpeed, ActualSpeed", filename);
    }

    public void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            if (showWaypoints)
            {
                Vector3 old_wp = waypoints[0];
                foreach (var wp in waypoints)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawLine(old_wp, wp);
                    old_wp = wp;

                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(wp, 2f);

                }
            }

            if (showGraph)
            {
                foreach (var node in nodes)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(node.position, 2f);

                    foreach (Node neighbor in node.neighbors)
                    {
                        Gizmos.DrawLine(node.position, neighbor.position);
                    }
                }
            }


        }

    }
    #endregion
}
