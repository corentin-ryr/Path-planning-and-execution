using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathCreation;
using Logging;

[RequireComponent(typeof(PathCreator))]
public class Trajectory : MonoBehaviour
{
    public bool debug = false; //True is the function optimize has been called since the last modification of the waypoints.


    private bool optimized = false; //True is the function optimize has been called since the last modification of the waypoints.
    private List<Vector3> waypoints = new List<Vector3>();

    TerrainManager terrain_manager;

    private PathCreator pathCreator;


    void Start()
    {
        pathCreator = GetComponent<PathCreator>();

        findShortestPath();

        OptimizeTrajectory();

        Debug.Log("Travel time: " + GetTravelTime());
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

    #region Getters and accesseurs


    public Vector3 getTangent(Vector3 position)
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

    public float GetTravelTime()
    {
        return pathCreator.bezierPath.TravelTime(SpeedAtCurvature);
    }

    #endregion

    #region Trajectory optimization ==========================================================================

    public void OptimizeTrajectory()
    {
        optimized = true;
        //Transform the series of waypoints in a smooth series of bezier
        fromWaypointsToBezier();

        pathCreator.bezierPath.ControlPointMode = BezierPath.ControlMode.Aligned;


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
        pathCreator.bezierPath = bezierPath;
    }

    private float[] sampleCurvatures = new float[] { 32.8f, 21.8f, 16.28f, 12.95f, 10.71f, 9.10f, 7.89f, 6.93f, 6.15f };
    public float SpeedAtCurvature(float curvature)
    {
        //Equation between the curvature (in meter) and the speed (in m/s). Found experimentally.
        return 21.4f + 2.08f * curvature - 0.0133f * curvature * curvature;
    }


    #endregion

    #region Logging and gizmos

    public void LogCurvatureHistogram() {
        DataLogger dataLogger = new DataLogger();
        //TODO
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
