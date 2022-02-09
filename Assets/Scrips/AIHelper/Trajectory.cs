using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathCreation;
using Logging;
using Analysis;

[RequireComponent(typeof(PathCreator))]
public class Trajectory : MonoBehaviour
{
    public bool debug = false; //True is the function optimize has been called since the last modification of the waypoints.


    private bool optimized = false; //True is the function optimize has been called since the last modification of the waypoints.
    private List<Vector3> waypoints = new List<Vector3>();

    private List<float[]> speedAtDistance;
    private List<float[]> targetSpeedAtDistance;

    TerrainManager terrain_manager;

    private PathCreator pathCreator;


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


        if (!debug)
        {
            Vector2Int startNode = terrain_manager.myInfo.coordinatesToNode(terrain_manager.myInfo.start_pos);
            Vector2Int goalNode = terrain_manager.myInfo.coordinatesToNode(terrain_manager.myInfo.goal_pos);
            Vector2Int[] pathNodes = AStar.ComputeShortestPath(terrain_manager.myInfo.traversability, startNode, goalNode);

            addPointToTrajectory(terrain_manager.myInfo.start_pos);
            for (int i = 1; i < pathNodes.Length - 1; i++)
            {
                addPointToTrajectory(terrain_manager.myInfo.nodeToCoordinates(pathNodes[i]));
            }
            addPointToTrajectory(terrain_manager.myInfo.goal_pos);
        }
        else
        {
            //Fake Trajectory for debug
            addPointToTrajectory(new Vector3(100, 0, 100));
            addPointToTrajectory(new Vector3(100, 0, 130));
            addPointToTrajectory(new Vector3(100, 0, 160));
            addPointToTrajectory(new Vector3(100, 0, 190));
            addPointToTrajectory(new Vector3(100, 0, 220));
            addPointToTrajectory(new Vector3(100, 0, 250));
        }
    }

    public void setTrajectoryAsSuccessiveWaypoints(Vector3[] waypoints)
    {
        optimized = false;
        this.waypoints = new List<Vector3>(waypoints);
    }

    public void addPointToTrajectory(Vector3 point)
    {
        optimized = false;
        waypoints.Add(point);
    }

    #region Getters and accesseurs =====================================================================================================


    public Vector3 getTangentAhead(Vector3 position, float speed)
    {
        Vector3 tangentAtClosestPoint = Vector3.zero;

        if (!optimized)
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
            float time = pathCreator.path.GetClosestTimeOnPath(position);
            time = time + speed * 0.001f; //The faster we go, the further away we look for the tangent
            tangentAtClosestPoint = pathCreator.path.GetDirection(time);
        }


        return tangentAtClosestPoint;
    }

    public Vector3 getClosestPoint(Vector3 position)
    {
        Vector3 closestPoint = Vector3.positiveInfinity;
        if (!optimized)
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

    //From the curvature profile (curvature as a function of the distance from the start) 
    //and with the car properties we can compute an estimation of the travel time of the curve (necessary for later optimization)
    public float ComputeTravelTime()
    {
        List<float[]> radiusData;
        radiusData = pathCreator.bezierPath.RadiusProfile();

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

    #region Trajectory optimization ==========================================================================

    public void OptimizeTrajectory()
    {
        optimized = true;
        //Transform the series of waypoints in a smooth series of bezier
        fromWaypointsToBezier();


    }

    private void fromWaypointsToBezier()
    {
        List<Vector3> controlPoints = new List<Vector3>();
        controlPoints.Add(waypoints[0]);
        for (int i = 1; i < waypoints.Count - 1; i++)
        {
            Vector3 middlePoint = (waypoints[i] + waypoints[i + 1]) / 2;
            controlPoints.Add(middlePoint);
        }
        controlPoints.Add(waypoints[waypoints.Count - 1]);

        BezierPath bezierPath = new BezierPath(controlPoints, false, PathSpace.xz);
        bezierPath.ControlPointMode = BezierPath.ControlMode.Aligned;
        bezierPath.MovePoint(1, terrain_manager.myInfo.start_pos + new Vector3(0, 0, 5));

        pathCreator.bezierPath = bezierPath;
    }

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
        return 2f;
    }

    public float SpeedAtPosition(Vector3 position)
    {
        float distanceFromStart = pathCreator.path.GetClosestDistanceAlongPath(position);
        Debug.Log("Distance: " + distanceFromStart);

        //TODO check if it works: it doesn't (to make it work try to make the target change by step instead of using the profile directly)
        float[] distances = new float[speedAtDistance.Count];
        for (int i = 0; i < speedAtDistance.Count; i++)
        {
            distances[i] = speedAtDistance[i][0];
        }
        int index = MathHelper.DichotomicSearch(distances, distanceFromStart);
        Debug.Log("Index: " + index);

        return speedAtDistance[index][3];
    }

    public float SpeedAtPosition2(Vector3 position)
    {
        float distanceFromStart = pathCreator.path.GetClosestDistanceAlongPath(position);

        for (int i = 1; i < targetSpeedAtDistance.Count; i++)
        {
            if (targetSpeedAtDistance[i][0] > distanceFromStart)
            {
                return targetSpeedAtDistance[i][1];
            }
        }
        return -1f;
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

    }
    #endregion
}
