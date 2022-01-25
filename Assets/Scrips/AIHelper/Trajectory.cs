using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Trajectory
{
    private List<Vector3> waypoints;

    public void setTrajectoryAsSuccessiveWaypoints(Vector3[] waypoints)
    {
        this.waypoints = new List<Vector3>(waypoints);
    }

    public void addPointToTrajectory(Vector3 point){
        waypoints.Add(point);
    }

    public Vector3 getTangent(Vector3 position)
    {
        return new Vector3(0, 0, 0); // TODO
    }

    public Vector3 getClosestPoint(Vector3 position)
    {
        Vector3 closestPoint = Vector3.positiveInfinity;

        for (int i = 0; i < waypoints.Count - 1; i++) //We loop through all the segment and find the closest point on each of those segments. Then we keep the best.
        {
            Vector3 line = (waypoints[i + 1] - waypoints[i]); //Line on which to project
            Vector3 distanceFromWaypoint = Vector3.Project(position - waypoints[i], line.normalized);
            Vector3 projectedPoint = waypoints[i] + distanceFromWaypoint;

            if (Vector3.Distance(projectedPoint, position) < Vector3.Distance(closestPoint, position) && distanceFromWaypoint.magnitude < line.magnitude)
            {
                closestPoint = projectedPoint;
            }
        }

        return closestPoint;
    }
}
