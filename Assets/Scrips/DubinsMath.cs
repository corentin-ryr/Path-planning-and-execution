
//  // Based on impementation by Erik Nordeus
// // Source: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/

// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public static class NewBehaviourScript : MonoBehaviour
// {
//     public static float distanceSegment = 0.05f;    // Size of path segments
//     public static float turningRadius = 10f;    // PLACEHOLDER, replace with actual maximum turning radius


//     public static Vector3 rightCirclePos(Vector3 carPos, float heading)
//     {   // Places circle left of position
//         Vector3 rightCirclePos = Vector3.zero;
//         rightCirclePos.x = carPos.x + turningRadius * Mathf.Sin(heading + (Math.PI / 2f));
//         rightCirclePos.z = carPos.z + turningRadius * Mathf.Cos(heading + (Math.PI / 2f));

//         return rightCirclePos;
//     }

//     public static Vector3 leftCirclePos(Vector3 carPos, float heading)
//     {   // Places circle right of position
//         Vector3 leftCirclePos = Vector3.zero;
//         leftCirclePos.x = carPos.x + turningRadius * Mathf.Sin(heading - (Math.PI / 2f));
//         leftCirclePos.z = carPos.z + turningRadius * Mathf.Cos(heading - (Math.PI / 2f));

//         return leftCirclePos;
//     }

//     // LSL or RSR
//     public static void outerTangent(
//         Vector3 startCircle,
//         Vector3 goalCirtcle,
//         bool isBottom,
//         out Vector3 startTangent,
//         out Vector3 goalTangent)
//     {
//         float theta = 90f * Mathf.Deg2Rad;  // Angle to the first tagent

//         theta += Mathf.Atan2(goalCircle.z - startCircle.z, goalCircle.x, startCircle.x);

//         if (isBottom) {
//             theta += Mathf.PI;  // Add 180 degress to put tagent at bottom
//         }

//         float T1x = startCircle.x + turningRadius * Mathf.Cos(theta);
//         float T1z = startCircle.z + turningRadius * Mathf.Sin(theta);

//         // Gets direction vector
//         Vector3 dirVec = goalCircle - startCircle;

//         // Tagent connecting start and goal circles
//         float T2x = T1x + dirVec.x;
//         float T2z = T1z - dirVec.z;

//         // Start and end points of tagent 
//         startTangent = new Vector3(T1x, 0f, T1z);
//         goalTagent = new Vector3(T2x, 0f, T2z);

//     }

//     public static void innerTangent(
//         Vector3 startCircle,
//         Vector3 goalCirtcle,
//         bool isBottom,
//         out Vector3 startTangent,
//         out Vector3 goalTangent)
//     {
//         fload D = (startCircle - goalCirtcle).magnitude;

//         float theta = Mathf.Acos((2f * turningRadius) / D);

//         if (isBottom)
//         {
//             theta *= -1f;   // Same as Mathf.PI (I suppose) // We could use either
//         }

//         theta += Mathf.Atan2(goalCirtcle.z - startCircle.z, goalCirtcle.x - startCircle.x);

//         float T1x = startCircle.x + turningRadius * Mathf.Cos(theta);
//         float T1z = startCircle.z + turningRadius * Mathf.Sin(theta);

//         // This is to get the diagonal tangent end point
//         // It would be a straight line if we moved the starting point two radiuses
//         float T1x_temp = startCircle.x + 2f * turningRadius * Mathf.Cos(theta);
//         float T1z_temp = startCircle.z + 2f * turningRadius * Mathf.Sin(theta);

//         Vector3 dirVec = goalCirtcle - new Vector3(T1x_temp, 0f, T1z_temp);

//         float T2x = T1x + dirVec.x;
//         float T2z = T1z + dirVec.z;

//         startTangent = new Vector3(T1x, 0f, T1z);
//         goalTangent = new Vector3(T2x, 0f, T2z);
//     }

//     public static void getCCCTangents(
//         Vector3 startCircle,
//         Vector3 goalCircle,
//         bool isLRL,
//         out Vector3 startTangent,
//         out Vector3 goalTangent,
//         out Vector3 middleCircle)
//     {
//         // This is for the CCC class (curve, curve, curve)

//         float D = (startCircle - goalCircle).magnitude;
//         float theta = Mahtf.Acos(D / 4f * turningRadius); // Angle between goal and 3rd circles

//         Vector3 V1 = goalCircle - startCircle;

//         if (isLRL)
//         {
//             theta = Mathf.Atan2(V1.z, V1.x) + theta;
//         }
//         else
//         {
//             theta = Mathf.Atan2(V1.z, V1.x) - theta;
//         }

//         // Position of the third circle
//         float x = startCircle.x + 2f * turningRadius * Mathf.Cos(theta);
//         float y = startCircle.y;
//         float z = startCircle.z + 2f * turningRadius * Mathf.Sin(theta);

//         middleCircle = new Vector3(x, y, z);

//         startTangent = middleCircle + V2 * turningRadius;
//         goalTangent = middleCircle + V3 * turningRadius;
//     }


//     public static void ArcLength(
//         Vector3 circlePos,
//         Vector3 startPos,
//         Vector3 goalPos,
//         bool isLeftCircle)
//     {
//         Vector3 V1 = startPos - circlePos;
//         Vector3 V2 = goalPos - circlePos;

//         float theta = Mathf.Atan2(V2.z, V2.x) - Mathf.Atan2(V1.z, V1.x);

//         if (theta < 0f && isLeftCircle)
//         {
//             theta += 2f * Mathf.PI;
//         } else if (theta > 0 && !isLeftCircle)
//         {
//             theta -= 2f * Mathf.PI;
//         }

//         float arcLength = Mathf.Abs(theta * turningRadius);

//         return arcLength;
//     }

//     public static void ADDCoordinatesToPath(
//         ref Vector3 currentPos,
//         ref float theta,
//         List<Vector3> finalPath,
//         int segments,
//         bool isTurning,
//         bool isTurningRight)
//     {
//         currentPos.x += driveDistance * Mathf.Sin(theta);
//         currentPos.z += driveDistance * Math.Cos(theta);

//         if (isTurning)
//         {
//             float turnParam = 1f;
            
//             if (isTurningRight)
//             {
//                 turnParam = -1f;
//             }

//             theta += (driveDistance / turningRadius) * turnParam;
//         }

//         finalPath.Add(currentPos);
//     }
// }