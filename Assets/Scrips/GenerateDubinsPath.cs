// Based on impementation by Erik Nordeus
// Source: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//from assets.scripts import DubinsPathPart as dpp;
import DubinsPathPart;
import DubinsMath;

public class GenerateDubinsPath : MonoBehaviour
{
    public Vector3 startLeftCircle;
    public Vector3 startRightCircle;
    public Vector3 goalLeftCircle;
    public Vector3 goalRightCircle;

    Vector3 startPos;
    Vector3 goalPos;

    // Is in radians
    float startHeading;
    float goalHeading;

    List<DubinsPathPart> pathDataList = new List<DubinsPathPart>();

    public List<DubinsPathPart> GetDubinsPaths(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading)
    {
        this.startPos = startPos;
        this.goalPos = goalPos;
        this.startHeading = startHeading;
        this.goalHeading = goalHeading;

        pathDataList.Clear();
        PositionLeftRightCircles();
        CalculatePathLengths();

        if pathDataList.Count > 0)
        {
            // Shortest path first
            pathDataList.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));
            GeneratePathCoordinates();
            return pathDataList;
        }
        // No paths found
        return null;
    }


    void PositionLeftRightCirtcles()
    {
        goalRightCircle = DubinsMath.rightCirclePos(goalPos, goalHeading);
        goalLeftCircle = DubinsMath.leftCirclePos(goalPos, goalHeading);
        startRightCircle = DubinsMath.rightCirclePos(startPos, startHeading);
        startLeftCircle = DubinsMath.leftCirclePos(startPos, startHeading);
    }

    void CalculatePathLengths()
    {
        // RSR (if not on the same position)
        if (startRightCircle.x != goalRightCircle.x && startRightCircle.z != goalRightCircle.z)
        {
            getRSR_length();
        }

        // LSL
        if (startRightCircle.x != goalRightCircle.x && startRightCircle.z != goalRightCircle.z)
        {
            getLSL_length();
        }

        // RSL, LSR (if circles don't intersect)
        // Squaring to get absolute value (I suppose. Perhaps changeable then)
        float comparisonSqr = DubinsMath.turningRadius * 2f * DubinsMath.turningRadius * 2f;

        // RSL
        if ((startRightCircle - goalLeftCircle).sqrMagnitude > comparisonSqr)
        {
            getRSL_length();
        }

        // LSR
        if ((startRightCircle - goalLeftCircle).sqrMagnitude > comparisonSqr)
        {
            getLSR_length();
        }

        // For the CCC class, the distance between circles have to be less than 4*r
        comparisonSqr = 4f * DubinsMath.turningRadius * 4f * DubinsMath.turningRadius;

        // RLR
        if ((startRightCircle - goalLeftCircle).magnitude > comparisonSqr)
        {
            getRLR_length();
        }

        // LRL
        if ((startRightCircle - goalLeftCircle).sqrMagnitude < comparisonSqr)
        {
            getLRL_length();
        }

    }


    void getRSR_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.outerTangent(startRightCircle, goalRightCircle, false, out startTangent, out goalTangent);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startRightCircle, startPos, startTangent, false);
        float length2 = (startTangent - goalTangent).magnitude;
        float length3 = DubinsMath.ArcLength(goalRightCircle, goalTangent, goalPos, false);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = false;
        pathData.SetIfTurningRight(true, false, true);

        pathDataList.Add(pathData);
    }

    void getLSL_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.outerTangent(startLeftCircle, goalLeftCircle, false, out startTangent, out goalTangent);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startLeftCircle, startPos, startTangent, true);
        float length2 = (startTangent - goalTangent).magnitude;
        float length3 = DubinsMath.ArcLength(goalLeftCircle, goalTangent, goalPos, true);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = false;
        pathData.SetIfTurningRight(false, false, false);

        pathDataList.Add(pathData);


    }

    void getRSL_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.innerTangent(startRightCircle, goalLeftCircle, false, out startTangent, out goalTangent);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startRightCircle, startPos, startTangent, false);
        float length2 = (startTangent - goalTangent).magnitude;
        float length3 = DubinsMath.ArcLength(goalLeftCircle, goalTangent, goalPos, true);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = false;
        pathData.SetIfTurningRight(true, false, false);

        pathDataList.Add(pathData);


    }

    void getLSR_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;

        DubinsMath.innerTangent(startLeftCircle, goalRightCircle, false, out startTangent, out goalTangent);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startLeftCircle, startPos, startTangent, true);
        float length2 = (startTangent - goalTangent).magnitude;
        float length3 = DubinsMath.ArcLength(goalRightCircle, goalTangent, goalPos, false);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = false;
        pathData.SetIfTurningRight(false, false, true);

        pathDataList.Add(pathData);
    }

    void getLRL_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;
        Vector3 middleCircle = Vector3.zero;

        DubinsMath.getCCCTangents(startRightCircle, goalRightCircle, false, out startTangent, out goalTangent, out middleCircle);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startRightCircle, startPos, startTangent, false);
        float length2 = DubinsMath.ArcLength(middleCircle, startTangent, goalTangent, true);
        float length3 = DubinsMath.ArcLength(goalRightCircle, goalTangent, goalPos, false);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = true;
        pathData.SetIfTurningRight(true, false, true);

        pathDataList.Add(pathData);
    }

    void getRLR_length()
    {
        Vector3 startTangent = Vector3.zero;
        Vector3 goalTangent = Vector3.zero;
        Vector3 middleCircle = Vector3.zero;

        DubinsMath.getCCCTangents(startleftCircle, goalLeftCircle, false, out startTangent, out goalTangent, out middleCircle);

        // Calculate lengths
        float length1 = DubinsMath.ArcLength(startLeftCircle, startPos, startTangent, true);
        float length2 = DubinsMath.ArcLength(middleCircle, startTangent, goalTangent, false);
        float length3 = DubinsMath.ArcLength(goalLeftCircle, goalTangent, goalPos, true);

        DubinsPathPart pathData = new DubinsPathPart(length1, length2, length3, startTangent, goalTangent, null); // Replace null with pathType
        pathData.segment2Turning = true;
        pathData.SetIfTurningRight(false, true, false);

        pathDataList.Add(pathData);
    }

    // Generate final path

    void GeneratePathCoordinates()
    {
        for (int i = 0; i < pathDataList.Count; i++)
        {
            getTotalPath(pathDataList[i]);
        }
    }

    void getTotalPath(DubinsPathPart pathData)
    {
        List<Vector3> finalPath = new List<Vector3>();

        Vector3 currentPos = startPos;
        float theta = startHeading;

        finalPath.Add(currentPos);

        int segments = 0;

        // First
        segments = Mathf.FloorToInt(pathData.length1 / DubinsMath.distanceSegment);

        DubinsMath.ADDCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            true,
            pathData.segment1TurningRight);


        // Second
        segments = Mathf.FloorToInt(pathData.length2 / DubinsMath.distanceSegment);

        DubinsMath.ADDCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            pathData.segment2Turning,
            pathData.segment2TurningRight);

        // Third
        segments = Mathf.FloorToInt(pathData.length3 / DubinsMath.distanceSegment);

        DubinsMath.ADDCoordinatesToPath(
            ref currentPos,
            ref theta,
            finalPath,
            segments,
            true,
            pathData.segment3TurningRight);

        // Add final goal coordinate
        finalPath.Add(new Vector3(goalPos.x, currentPos.y, goalPos.z));

        pathData.pathCoordinates = finalPath;

    }

}
