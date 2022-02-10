// // Based on impementation by Erik Nordeus
// // Source: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/

// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class DubinsPathPart : MonoBehaviour
// {
//     // This will hold one path. It can therefore be sortable

//     public float totalLength;
//     public float length1;
//     public float length2;
//     public float length3;

//     public Vector3 tangent1;
//     public Vector3 tangent2;

//     public PathType pathType;
//     public List<Vector3> pathCoordinates;

//     public bool segment2Turning;

//     public bool segment1TurningRight;
//     public bool segment2TurningRight;
//     public bool segment3TurningRight;

//     public DubinsPathPart(float length1, float length2, float length3, Vector3 tangent1, Vector3 tangent2,PathType pathType)
//     {
//         this.totalLength = length1 + length2 + length3;
//         this.length1 = length1;
//         this.length2 = length2;
//         this.length3 = length3;

//         this.tangent1 = tangent1;
//         this.tangent2 = tangent2;

//         this.pathType = pathType;
//     }

//     public void SetIfTurningRight(bool segment1TurningRight, bool segment2TurningRight, bool segment3TurningRight)
//     {
//         this.segment1TurningRight = segment1TurningRight;
//         this.segment2TurningRight = segment2TurningRight;
//         this.segment3TurningRight = segment3TurningRight;
//     }

// }
