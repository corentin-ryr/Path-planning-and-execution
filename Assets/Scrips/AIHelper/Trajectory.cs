using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathCreation;

[RequireComponent(typeof(PathCreator))]
public class Trajectory : MonoBehaviour
{
    private bool optimized = false; //True is the function optimize has been called since the last modification of the waypoints.
    private List<Vector3> waypoints = new List<Vector3>();

    private PathCreator pathCreator;

    void Start()
    {
        pathCreator = GetComponent<PathCreator>();
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

    public float GetTravelTime() {
        return pathCreator.bezierPath.TravelTime();
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


    #endregion

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
}
