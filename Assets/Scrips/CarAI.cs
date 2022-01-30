using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {

        [Header("Parameters")]
        public float k = 1f;
        public float ks = 0f;
        public bool debug = true;

        [Header("References")]
        public Trajectory trajectory;

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;


        private float currentThrottle = 0.1f;
        private float currentSteering = 0f;


        private void Start()
        {

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...


            #region Create the path ===============================================================================

            if (!debug)
            {
                Vector2Int startNode = terrain_manager.myInfo.coordinatesToNode(terrain_manager.myInfo.start_pos);
                Vector2Int goalNode = terrain_manager.myInfo.coordinatesToNode(terrain_manager.myInfo.goal_pos);
                Vector2Int[] pathNodes = AStar.ComputeShortestPath(terrain_manager.myInfo.traversability, startNode, goalNode);

                trajectory.addPointToTrajectory(terrain_manager.myInfo.start_pos);
                for (int i = 1; i < pathNodes.Length - 1; i++)
                {
                    trajectory.addPointToTrajectory(terrain_manager.myInfo.nodeToCoordinates(pathNodes[i]));
                }
                trajectory.addPointToTrajectory(terrain_manager.myInfo.goal_pos);
            }
            else
            {
                //Fake Trajectory for debug
                trajectory.addPointToTrajectory(new Vector3(100, 0, 100));
                trajectory.addPointToTrajectory(new Vector3(100, 0, 130));
                trajectory.addPointToTrajectory(new Vector3(100, 0, 160));
                trajectory.addPointToTrajectory(new Vector3(100, 0, 190));
                trajectory.addPointToTrajectory(new Vector3(100, 0, 220));
                trajectory.addPointToTrajectory(new Vector3(100, 0, 250));
            }


            trajectory.OptimizeTrajectory();
            
            Debug.Log("Travel time: " + trajectory.GetTravelTime());
            #endregion


        }


        private void FixedUpdate()
        {


            // // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                // Debug.Log("Did Hit");
            }

            computeSteering();
            computeThrottle(20f);

            // this is how you control the car
            m_Car.Move(currentSteering, currentThrottle, 0f, 0f);


        }

        private void computeThrottle(float targetSpeed)
        {

            float speed = m_Car.CurrentSpeed;

            if (targetSpeed > speed)
            {
                currentThrottle = 0.5f;
            }
            else
            {
                currentThrottle = 0f;
            }

            Debug.Log("Speed: " + speed);
        }

        private void computeSteering()
        {
            Vector3 currentPosition = transform.position + new Vector3(0, 0, 1.27f);

            Vector3 trajectoryTangent = trajectory.getTangent(currentPosition);

            float psi = Vector3.Angle(transform.forward, trajectoryTangent);
            psi = Vector3.Cross(transform.forward, trajectoryTangent).y < 0 ? -psi : psi;

            Vector3 closestpoint = trajectory.getClosestPoint(currentPosition);
            float distanceToClosestPoint = Vector3.Distance(currentPosition, closestpoint);
            float crossTrack = Mathf.Atan((k * distanceToClosestPoint) / (m_Car.CurrentSpeed + ks)) * 360 / Mathf.PI;
            crossTrack = Vector3.Cross(transform.forward, closestpoint - currentPosition).y < 0 ? -crossTrack : crossTrack;

            currentSteering = Mathf.Clamp(psi + crossTrack, -25, 25) / 25;
        }




        #region Gizmos ========================================================================================
        void OnDrawGizmos()
        {
            if (Application.isPlaying)
            {
                // trajectory.OnDrawGizmos();
            }
        }

        #endregion
    }
}
