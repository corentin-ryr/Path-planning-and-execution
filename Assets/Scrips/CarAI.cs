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
        public float gainDerivative = 1f;
        public float gainIntegral = 1f;
        public float gainProportional = 1f;
        public bool debug = true;

        [Header("References")]
        public Trajectory trajectory;

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        PidController controller = new PidController(1f, 1f, 0.1f, 1f, -1f);



        private float currentThrottle = 0.1f;
        private float currentSteering = 0f;


        private void Start()
        {

            // get the car controller
            m_Car = GetComponent<CarController>();

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
            computeThrottle();

            // this is how you control the car
            m_Car.Move(currentSteering, currentThrottle, currentThrottle, 0f);


        }

        // Legacy
        // private void computeThrottle()
        // {

        //     //Get the curvature at the closest point
        //     Vector3 currentPosition = transform.position + new Vector3(0, 0, 1.27f);
        //     Vector3 closestpoint = trajectory.getClosestPoint(currentPosition);
        //     // float curvature = trajectory.GetCurvature(closestpoint);

        //     float targetSpeed = trajectory.SpeedAtPosition(currentPosition) / 2.23693629f;
        //     targetSpeed = Mathf.Max(targetSpeed, 5f);
        //     float speed = m_Car.CurrentSpeed;

        //     if (targetSpeed > speed)
        //     {
        //         currentThrottle = 0.5f;
        //     }
        //     else
        //     {
        //         currentThrottle = 0f;
        //     }

        //     Debug.Log("Target speed: " + targetSpeed);
        //     Debug.Log("Speed: " + speed);
        // }

        private void computeThrottle()
        {
            Vector3 currentPosition = transform.position + new Vector3(0, 0, 1.27f);
            Vector3 closestpoint = trajectory.getClosestPoint(currentPosition);

            float targetSpeed = trajectory.SpeedAtPosition(currentPosition);//* 2.23693629f;
            targetSpeed = Mathf.Max(targetSpeed, 5f);

            controller.GainDerivative = gainDerivative;
            controller.GainIntegral = gainIntegral;
            controller.GainProportional = gainProportional;


            controller.SetPoint = targetSpeed;

            float currentSpeed = m_Car.CurrentSpeed;
            currentSpeed = m_Car.Backing ? -currentSpeed : currentSpeed;
            controller.ProcessVariable = currentSpeed;
            currentThrottle = (float)controller.ControlVariable(System.TimeSpan.FromSeconds(Time.fixedDeltaTime));

            Debug.Log("Target speed: " + targetSpeed);
            Debug.Log("Speed: " + currentSpeed);

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
