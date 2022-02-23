using System.Collections;
using System.Collections.Generic;
using System;
using MathNet.Numerics.LinearAlgebra;
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
    public bool optimize = true;
    public bool showPath;


    [Header("Optimization parameters")]
    public int nbIter = 500;
    public int nbEpochs;


    private bool bezierForm = false; //True is the function optimize has been called since the last modification of the waypoints.
    private List<UnityEngine.Vector3> waypoints = new List<UnityEngine.Vector3>();

    private List<double[]> targetSpeedAtDistance;

    TerrainManager terrain_manager;

    private PathCreator pathCreator;
    private List<Node> nodes;
    // private VertexPath path;

    private float carWidth = 5f;
    private int currentSampledPoint;

    void Start()
    {
        pathCreator = GetComponent<PathCreator>();

        findShortestPath();

        OptimizeTrajectory();
    }

    void OnEnable()
    {
        EventManager.FinishedOptimizing += OnStartDriving;
    }
    void OnDisable()
    {
        EventManager.FinishedOptimizing -= OnStartDriving;
    }

    public void OnStartDriving()
    {
        Debug.Log("Travel time: " + ComputeTravelTime());
        ComputeTargetSpeeds();
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

    public void setTrajectoryAsSuccessiveWaypoints(UnityEngine.Vector3[] waypoints)
    {
        bezierForm = false;
        this.waypoints = new List<UnityEngine.Vector3>(waypoints);
    }

    public void addPointToTrajectory(UnityEngine.Vector3 point)
    {
        bezierForm = false;
        waypoints.Add(point);
    }

    private (Node, Node) createGraph()
    {
        //Create the nodes
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

                    // Vector3 temp = terrain_manager.myInfo.getCubeSize() / 2 * neighborSides;

                    Vector2 offset = terrain_manager.myInfo.getCubeSize() / 2 * (Vector2)neighborSides;
                    Vector3 position = terrain_manager.myInfo.nodeToCoordinates(new Vector2Int(i, j)) + new Vector3(offset.x, 0f, offset.y);

                    if (showGraph) Debug.DrawLine(position + Vector3.up * 0.1f, position + 5 * new Vector3(neighborSides.x, 0f, neighborSides.y) + Vector3.up * 0.1f, Color.green);

                    if (Physics.Raycast(position, new Vector3(neighborSides.x, 0f, neighborSides.y), 5f)) continue;

                    nodes.Add(new Node(position + 5 * new Vector3(neighborSides.x, 0f, neighborSides.y)));
                }

            }
        }

        //Link the nodes
        GameObject[] cubes = GameObject.FindGameObjectsWithTag("Box");

        Vector3 initialScale = cubes[0].transform.localScale;
        foreach (GameObject cube in cubes)
        {
            cube.transform.localScale = initialScale + new Vector3(carWidth / 2, 0, carWidth / 2);
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
                if (!currentNode.neighbors.Contains(neighbor) && !Physics.Raycast(currentNode.position, neighbor.position - currentNode.position, UnityEngine.Vector3.Distance(currentNode.position, neighbor.position)))
                {
                    visited.Add(neighbor);
                    worklist.Enqueue(neighbor);

                    currentNode.neighbors.Add(neighbor);
                }
            }
        }

        foreach (GameObject cube in cubes)
        {
            cube.transform.localScale = initialScale;
        }


        return (startNode, goalNode);
    }




    #region Getters and accesseurs =====================================================================================================

    public UnityEngine.Vector3 getTangentAhead(UnityEngine.Vector3 position, float speed)
    {
        UnityEngine.Vector3 tangentAtClosestPoint = UnityEngine.Vector3.zero;

        if (!bezierForm)
        {
            UnityEngine.Vector3 closestPoint = UnityEngine.Vector3.positiveInfinity;
            for (int i = 0; i < waypoints.Count - 1; i++) //We loop through all the segment and find the closest point on each of those segments. Then we keep the best.
            {
                UnityEngine.Vector3 line = (waypoints[i + 1] - waypoints[i]); //Line on which to project
                UnityEngine.Vector3 distanceFromWaypoint = UnityEngine.Vector3.Project(position - waypoints[i], line.normalized);
                UnityEngine.Vector3 projectedPoint = waypoints[i] + distanceFromWaypoint;

                if (UnityEngine.Vector3.Distance(projectedPoint, position) <= UnityEngine.Vector3.Distance(closestPoint, position) && distanceFromWaypoint.magnitude < line.magnitude)
                {
                    closestPoint = projectedPoint;
                    tangentAtClosestPoint = line.normalized;
                }
            }
        }
        else
        {
            float distance = pathCreator.path.GetClosestDistanceAlongPath(position);
            // distance = distance + 0.1f * speed; //The faster we go, the further away we look for the tangent
            distance = distance + Mathf.Clamp(0.5f / speed, 0, 10); //The faster we go, the further away we look for the tangent
            tangentAtClosestPoint = pathCreator.path.GetDirection(distance / pathCreator.path.length);
        }


        return tangentAtClosestPoint;
    }

    public UnityEngine.Vector3 getClosestPoint(UnityEngine.Vector3 position)
    {
        UnityEngine.Vector3 closestPoint = UnityEngine.Vector3.positiveInfinity;
        if (!bezierForm)
        {
            for (int i = 0; i < waypoints.Count - 1; i++) //We loop through all the segment and find the closest point on each of those segments. Then we keep the best.
            {
                UnityEngine.Vector3 line = (waypoints[i + 1] - waypoints[i]); //Line on which to project
                UnityEngine.Vector3 distanceFromWaypoint = UnityEngine.Vector3.Project(position - waypoints[i], line.normalized);
                UnityEngine.Vector3 projectedPoint = waypoints[i] + distanceFromWaypoint;

                if (UnityEngine.Vector3.Distance(projectedPoint, position) <= UnityEngine.Vector3.Distance(closestPoint, position) && distanceFromWaypoint.magnitude < line.magnitude)
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

    public (float, float) GetSpeedAtPosition(UnityEngine.Vector3 position)
    {
        float distanceFromStart = pathCreator.path.GetClosestDistanceAlongPath(position);

        for (int i = 1; i < targetSpeedAtDistance.Count; i++)
        {
            if (targetSpeedAtDistance[i][0] > distanceFromStart)
            {
                return (distanceFromStart, (float)targetSpeedAtDistance[i][1]);
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
        if (optimize)
        {
            fromWaypointsToBezier();
            StartCoroutine(gradientMinimization2());
        }
        else
        {
            EventManager.OnFinishedOptimizing();
        }
    }

    private void fromWaypointsToBezier()
    {
        List<UnityEngine.Vector3> controlPoints = new List<UnityEngine.Vector3>();
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            controlPoints.Add(waypoints[i]);
            if (Vector3.Distance(waypoints[i], waypoints[i + 1]) > 25)
            {
                UnityEngine.Vector3 middlePoint = (waypoints[i] + waypoints[i + 1]) / 2;
                controlPoints.Add(middlePoint);
            }
        }
        controlPoints.Add(waypoints[waypoints.Count - 1]);

        BezierPath bezierPath = new BezierPath(controlPoints, false, PathSpace.xz);
        bezierPath.ControlPointMode = BezierPath.ControlMode.Aligned;
        bezierPath.MovePoint(1, terrain_manager.myInfo.start_pos + new UnityEngine.Vector3(0, 0, 5));

        pathCreator.bezierPath = bezierPath;
        pathCreator.showPath = showPath;
    }

    private double costFunction(Vector<double> input)
    {
        setPathFromVector(input);

        double penalty = ComputeObstaclePenalty();
        double travelTime = ComputeTravelTime();

        return travelTime + penalty;
    }

    #region First optimization test ===============================================================================
    private void gradientMinimization()
    {
        //TODO use math.NET to do gradient descent
        try
        {
            FindMinimum.OfFunction(costFunction, getPathVector(), 1E-7, nbIter);
        }
        catch (System.Exception)
        {
            Debug.Log("Exceeded iteration numbers");
        }

        // Vector<double> features = getPathVector();
        // setPathFromVector(features);
    }

    private Vector<double> getPathVector()
    {
        //We cant move the first and last points, we cant change the first point orientation.
        //The features are the position of the points (2 doubles for each points), 
        // the orientation of the tangent (the control points are aligned so only 1 double) 
        // and the length of the control point (1 double per control point)
        int nbPoints = pathCreator.bezierPath.NumAnchorPoints;

        int nbFeatures = (nbPoints - 2) * 2 + (nbPoints - 1) + (nbPoints - 1) * 2; //The position of each point (except start and finish) + orientation (except start) + 
        Vector<double> features = Vector<double>.Build.Dense(nbFeatures);

        int vectorPosition = 0;
        for (int i = 1; i < nbPoints - 1; i++)
        {
            Vector3[] segmentPoints = pathCreator.bezierPath.GetPointsInSegment(i);

            vectorPosition = (i - 1) * 2;
            features[vectorPosition] = segmentPoints[0].x;
            features[vectorPosition + 1] = segmentPoints[0].z;
        }

        for (int i = 1; i < nbPoints; i++)
        {
            Vector3 anchorPoint = pathCreator.bezierPath.GetPoint(i * 3);
            Vector3 controlPoint = pathCreator.bezierPath.GetPoint(i * 3 - 1);

            vectorPosition = (nbPoints - 2) * 2 + i - 1;
            features[vectorPosition] = Vector3.SignedAngle(Vector3.forward, controlPoint - anchorPoint, Vector3.up) / 360;
        }

        vectorPosition = (nbPoints - 2) * 2 + (nbPoints - 1);
        for (int i = 0; i < nbPoints; i++)
        {
            Vector3 anchorPoint = pathCreator.bezierPath.GetPoint(i * 3);
            if (i == 0)
            {
                Vector3 controlPoint = pathCreator.bezierPath.GetPoint(i * 3 + 1);

                features[vectorPosition] = (anchorPoint - controlPoint).magnitude;
                vectorPosition++;
            }
            else if (i == nbPoints - 1)
            {
                Vector3 controlPoint = pathCreator.bezierPath.GetPoint(i * 3 - 1);

                features[vectorPosition] = (anchorPoint - controlPoint).magnitude;
                vectorPosition++;
            }
            else
            {
                Vector3 controlPoint1 = pathCreator.bezierPath.GetPoint(i * 3 - 1);
                Vector3 controlPoint2 = pathCreator.bezierPath.GetPoint(i * 3 + 1);

                features[vectorPosition] = (anchorPoint - controlPoint1).magnitude;
                vectorPosition++;
                features[vectorPosition] = (anchorPoint - controlPoint2).magnitude;
                vectorPosition++;
            }
        }

        return features;
    }

    private void setPathFromVector(Vector<double> features)
    {
        int nbPoints = pathCreator.bezierPath.NumAnchorPoints;

        for (int i = 0; i < nbPoints - 1; i += 2) //We first set the anchor points correctly
        {
            pathCreator.bezierPath.SetPoint((i + 1) * 3, new Vector3((float)features[i * 2], 0, (float)features[i * 2 + 1]));
        }

        //Then we set the control points
        Vector3 anchorPoint = pathCreator.bezierPath.GetPoint(0);
        Vector3 controlPoint = pathCreator.bezierPath.GetPoint(1);
        pathCreator.bezierPath.SetPoint(1, anchorPoint + (controlPoint - anchorPoint).normalized * (float)features[((nbPoints - 2) * 2 + nbPoints - 1)]);
        for (int i = 1; i < nbPoints - 1; i++) //Loop through every regular point (with two control points)
        {
            anchorPoint = pathCreator.bezierPath.GetPoint(i * 3);
            float orientation = (float)features[(nbPoints - 2) * 2 + i - 1] * 360;
            float magnitude1 = (float)features[(nbPoints - 2) * 2 + nbPoints - 1 + 1 + (i - 1) * 2];
            float magnitude2 = (float)features[(nbPoints - 2) * 2 + nbPoints - 1 + 1 + (i - 1) * 2 + 1];

            pathCreator.bezierPath.SetPoint(i * 3 - 1, anchorPoint + SetVectorFromAngle(orientation) * magnitude1);
            pathCreator.bezierPath.SetPoint(i * 3 + 1, anchorPoint - SetVectorFromAngle(orientation) * magnitude2);
        }
    }
    #endregion


    #region Second optimization test ==========================================================================

    private IEnumerator gradientMinimization2()
    {
        //Number of epochs (we sample a point each time and optimize the proximity)
        for (int i = 0; i < nbEpochs; i++)
        {
            try
            {
                FindMinimum.OfFunction(costFunction2, getVectorFromPoint(UnityEngine.Random.Range(1, pathCreator.bezierPath.NumAnchorPoints - 1)), 1E-7, nbIter);
            }
            catch (System.Exception)
            {
                Debug.Log("Exceeded iteration numbers");
            }
            Debug.Log("epoch " + i);
            Debug.Log("Penalty: " + ComputeObstaclePenalty());
            ComputeTravelTime();

            yield return null;

        }

        EventManager.OnFinishedOptimizing();

        // setPathFromVector2(getVectorFromPoint(3));
        // yield return null;
    }

    private double costFunction2(Vector<double> input)
    {
        setPathFromVector2(input);

        double penalty = ComputeObstaclePenalty();
        double travelTime = ComputeTravelTime();
        return travelTime + penalty;
    }

    private Vector<double> getVectorFromPoint(int pointIndex)
    {
        //We cant move the first and last points, we cant change the first point orientation.
        //The features are the position of point (2 doubles), 
        // the orientation of the tangent (the control points are aligned so only 1 double) 
        // and the length of the control point (1 double per control point)
        currentSampledPoint = pointIndex;
        List<double> features = new List<double>();

        //Adding the position of the anchorpoint
        Vector3 segmentPoints = pathCreator.bezierPath.GetPoint(pointIndex * 3);
        features.Add(segmentPoints.x);
        features.Add(segmentPoints.z);

        //Adding the orientation of the 3 points (or 2 if its the first)
        Vector3 anchorPoint;
        Vector3 controlPoint;
        if (pointIndex > 1)
        {
            anchorPoint = pathCreator.bezierPath.GetPoint((pointIndex - 1) * 3);
            controlPoint = pathCreator.bezierPath.GetPoint((pointIndex - 1) * 3 - 1);
            features.Add(Vector3.SignedAngle(Vector3.forward, controlPoint - anchorPoint, Vector3.up) / 360);
        }

        anchorPoint = pathCreator.bezierPath.GetPoint(pointIndex * 3);
        controlPoint = pathCreator.bezierPath.GetPoint(pointIndex * 3 - 1);
        features.Add(Vector3.SignedAngle(Vector3.forward, controlPoint - anchorPoint, Vector3.up) / 360);

        anchorPoint = pathCreator.bezierPath.GetPoint((pointIndex + 1) * 3);
        controlPoint = pathCreator.bezierPath.GetPoint((pointIndex + 1) * 3 - 1);
        features.Add(Vector3.SignedAngle(Vector3.forward, controlPoint - anchorPoint, Vector3.up) / 360);

        //Adding the magnitude of the 4 points
        anchorPoint = pathCreator.bezierPath.GetPoint((pointIndex - 1) * 3);
        controlPoint = pathCreator.bezierPath.GetPoint((pointIndex - 1) * 3 + 1);
        features.Add((anchorPoint - controlPoint).magnitude);

        anchorPoint = pathCreator.bezierPath.GetPoint(pointIndex * 3);
        controlPoint = pathCreator.bezierPath.GetPoint(pointIndex * 3 - 1);
        features.Add((anchorPoint - controlPoint).magnitude);
        controlPoint = pathCreator.bezierPath.GetPoint(pointIndex * 3 + 1);
        features.Add((anchorPoint - controlPoint).magnitude);

        anchorPoint = pathCreator.bezierPath.GetPoint((pointIndex + 1) * 3);
        controlPoint = pathCreator.bezierPath.GetPoint((pointIndex + 1) * 3 - 1);
        features.Add((anchorPoint - controlPoint).magnitude);

        return Vector<double>.Build.DenseOfArray(features.ToArray());
    }

    private void setPathFromVector2(Vector<double> features)
    {
        pathCreator.bezierPath.SetPoint(currentSampledPoint * 3, new Vector3((float)features[0], 0, (float)features[1]));
        //Then we set the control points (4 to 5 control points to set)

        Vector3 anchorPoint;
        Vector3 controlPoint;
        float orientation;
        float magnitude;
        if (currentSampledPoint == 1)
        {
            anchorPoint = pathCreator.bezierPath.GetPoint(0);
            controlPoint = pathCreator.bezierPath.GetPoint(1);
            pathCreator.bezierPath.SetPoint(1, anchorPoint + (controlPoint - anchorPoint).normalized * (float)features[4]);
        }
        else
        {
            anchorPoint = pathCreator.bezierPath.GetPoint((currentSampledPoint - 1) * 3);
            orientation = (float)features[2] * 360;
            magnitude = (float)features[5];
            pathCreator.bezierPath.SetPoint((currentSampledPoint - 1) * 3 + 1, anchorPoint - SetVectorFromAngle(orientation) * magnitude);
            magnitude = (pathCreator.bezierPath.GetPoint((currentSampledPoint - 1) * 3 - 1) - anchorPoint).magnitude;
            pathCreator.bezierPath.SetPoint((currentSampledPoint - 1) * 3 - 1, anchorPoint + SetVectorFromAngle(orientation) * magnitude);
        }
        int isFirstPoint = currentSampledPoint == 1 ? 1 : 0;

        anchorPoint = pathCreator.bezierPath.GetPoint(currentSampledPoint * 3);
        orientation = (float)features[3 - isFirstPoint] * 360;
        magnitude = (float)features[6 - isFirstPoint];
        pathCreator.bezierPath.SetPoint(currentSampledPoint * 3 - 1, anchorPoint + SetVectorFromAngle(orientation) * magnitude);
        magnitude = (float)features[7 - isFirstPoint];
        pathCreator.bezierPath.SetPoint(currentSampledPoint * 3 + 1, anchorPoint - SetVectorFromAngle(orientation) * magnitude);

        anchorPoint = pathCreator.bezierPath.GetPoint((currentSampledPoint + 1) * 3);
        orientation = (float)features[4 - isFirstPoint] * 360;
        magnitude = (float)features[8 - isFirstPoint];
        pathCreator.bezierPath.SetPoint((currentSampledPoint + 1) * 3 - 1, anchorPoint + SetVectorFromAngle(orientation) * magnitude);
        if (currentSampledPoint != pathCreator.bezierPath.NumAnchorPoints - 1)
        {
            magnitude = (pathCreator.bezierPath.GetPoint((currentSampledPoint + 1) * 3 + 1) - anchorPoint).magnitude;
            pathCreator.bezierPath.SetPoint((currentSampledPoint + 1) * 3 + 1, anchorPoint - SetVectorFromAngle(orientation) * magnitude);
        }
    }

    #endregion

    #endregion

    private Vector3 SetVectorFromAngle(float angle)
    {
        var rotation = Quaternion.Euler(0, angle, 0);
        return rotation * Vector3.forward;
    }


    //From the curvature profile (curvature as a function of the distance from the start) 
    //and with the car properties we can compute an estimation of the travel time of the curve (necessary for later optimization)
    public double ComputeTravelTime()
    {
        List<double[]> radiusData = ComputeSpeedData();
        double lowRadiusPenalty = 0;

        double travelTime = 0;
        for (int i = 0; i < radiusData.Count - 1; i++)
        {
            if (radiusData[i][3] > 2f) //To prevent the start of the car to artificially increase the travel time (speed of zero => infinite time)
            {
                travelTime += (radiusData[i + 1][0] - radiusData[i][0]) / (radiusData[i][3]); //The distance between consecutive steps / the speed at the step
            }
            if (radiusData[i][1] < 7f)
            {
                lowRadiusPenalty += (radiusData[i + 1][0] - radiusData[i][0]) / (radiusData[i][1] + 0.5);
            }
        }

        if (debug) Debug.Log(Math.Exp(lowRadiusPenalty * 2));
        return travelTime + Math.Exp(lowRadiusPenalty * 2);
    }

    public double ComputeObstaclePenalty()
    {
        double minDst = 0;
        double totalDst = 0;
        Vector3 badPoint = Vector3.zero;

        //For each sampled point on the vertexPath, we check its distance to the obstacles.
        Vector3[] points = pathCreator.path.localPoints;
        Vector2[] v2Points = new Vector2[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            v2Points[i] = new Vector2(points[i].x, points[i].z);

            // if (i >= 1) Debug.DrawLine(points[i - 1], points[i]);
        }
        Vector2 cubeSize = (terrain_manager.myInfo.getCubeSize() + new Vector2(carWidth, carWidth)) / 2;

        foreach (Vector2 point in v2Points)
        {
            for (int i = 0; i < terrain_manager.myInfo.traversability.GetLength(0); i++)
            {
                for (int j = 0; j < terrain_manager.myInfo.traversability.GetLength(1); j++)
                {
                    if (terrain_manager.myInfo.traversability[i, j] < 0.5)
                    {
                        continue;
                    }
                    Vector3 obstaclePosition = terrain_manager.myInfo.nodeToCoordinates(new Vector2Int(i, j));

                    Vector2 positionInBox = point - new Vector2(obstaclePosition.x, obstaclePosition.z);
                    Vector2 offset = new Vector2(Mathf.Abs(positionInBox.x), Mathf.Abs(positionInBox.y)) - cubeSize;

                    Vector2 posInsideBox = Vector2.Min(offset, Vector2.zero);
                    float dstInsideBox = posInsideBox.x < posInsideBox.y ? posInsideBox.y : posInsideBox.x; //Distance <= 0 (0 if outside of the cube and negative if inside)

                    totalDst += dstInsideBox;
                    if (dstInsideBox < minDst)
                    {
                        minDst = dstInsideBox;
                        badPoint = new Vector3(point.x, 0, point.y);
                    }
                }
            }
        }

        // Debug.Log(minDst);
        // Debug.Log(badPoint);
        // Debug.DrawLine(Vector3.zero, badPoint, Color.blue, 100);
        // return 1500 * (1 - Math.Exp(minDst / 2));
        // return 1500 * (1 - Math.Exp(totalDst / 20));
        return Math.Exp(-totalDst / 10) + Math.Exp(-minDst * 5);
    }

    public void ComputeTargetSpeeds()
    {
        List<double[]> radiusData = ComputeSpeedData();

        targetSpeedAtDistance = MathHelper.pieceWiseConstantFromSpeedProfile(radiusData);
        LogRadiusHistogram(radiusData, "speedData");
        LogRadiusHistogram(targetSpeedAtDistance, "targetSpeedData");
    }

    private List<double[]> ComputeSpeedData() //Return list of elements: 1) distance from start, 2) radius, 3) max speed at radius, 4) actual speed
    {
        List<double[]> radiusData = pathCreator.bezierPath.RadiusProfile();

        List<double[]> maxSpeedData = new List<double[]>();
        for (int i = 0; i < radiusData.Count; i++)
        {
            maxSpeedData.Add(new double[] { radiusData[i][0], SpeedAtRadius(radiusData[i][1]) });
        }
        maxSpeedData[0] = new double[] { radiusData[0][0], 0f };
        List<double> constraintData = MathHelper.BestFunctionWithSlopeConstraints(maxSpeedData, AccelerationAtSpeed, DeccelerationAtSpeed);

        for (int i = 0; i < radiusData.Count; i++)
        {
            radiusData[i] = new double[] { radiusData[i][0], radiusData[i][1], maxSpeedData[i][1], constraintData[i] };
        }

        return radiusData;
    }


    #region Car properties =====================================================================================

    public double SpeedAtRadius(double radius)
    {
        // radius = radius < 6 ? 6 : radius;
        radius = radius > 33 ? 33 : radius;//Range of values in which the speed-curvature function is valid

        if (radius < 7)
        {
            return 0.05 * radius;
        }
        else
        {
            //Equation between the curvature (in meter) and the speed (in m/s). Found experimentally.
            return (8.72 + 0.988 * radius - 6.94E-3 * radius * radius) * 0.8; //The 0.9 is a safety constant
        }


    }

    public double AccelerationAtSpeed(double speed)
    {
        double accel = 0;

        if (speed > 5 && speed < 49)
        {
            accel = 6.78 - 0.104 * speed;
        }
        else if (speed <= 5)
        {
            accel = 0.22 + 2.25 * speed - 0.187 * speed * speed;
        }
        return accel;
    }

    public double DeccelerationAtSpeed(double speed)
    {
        // return 3.12f + 5.92f * speed;
        return 0.3;
    }
    #endregion





    #region Logging and gizmos =================================================================================

    public void LogRadiusHistogram(List<double[]> radiusData, string filename = "export")
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
                UnityEngine.Vector3 old_wp = waypoints[0];
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
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(node.position, 1f);

                    foreach (Node neighbor in node.neighbors)
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawLine(node.position + Vector3.up * 0.1f, neighbor.position + Vector3.up * 0.1f);
                    }
                }
            }


        }

    }
    #endregion
}
