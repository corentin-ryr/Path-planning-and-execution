using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Logging;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {

        [Header("PID controller parameters")]
        public float gainDerivative = 1f;
        public float gainIntegral = 1f;
        public float gainProportional = 1f;

        [Header("Stanley controller parameters")]
        public float k = 1f;
        public float ks = 0f;

        [Header("Tracking score parameters")]
        public float distanceRelaxation = 1f;
        public bool debug = true;

        [Header("References")]
        public Trajectory trajectory;


        //Internal references
        private CarController m_Car; // the car controller we want to use
        private PidController controller = new PidController(1f, 1f, 0.1f, 1f, -1f);

        //Internal variables
        private float currentThrottle = 0.1f;
        private float currentSteering = 0f;
        private float trackTrajectoryScore = float.MaxValue; //Lower is better

        private Vector3 closestpoint;
        private Vector3 currentPosition;

        List<double[]> recordedSpeed = new List<double[]>();


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
            currentPosition = transform.position + new Vector3(0, 0, 1.27f);
            closestpoint = trajectory.getClosestPoint(currentPosition);


            computeSteering();
            computeThrottle();

            // this is how you control the car
            m_Car.Move(currentSteering, currentThrottle, currentThrottle, 0f);

            if (Input.GetKey("s"))
            {
                trajectory.LogRadiusHistogram(recordedSpeed, "actualSpeed");
            }
        }



        private void computeThrottle()
        {

            (float distanceFromStart, float targetSpeed) = trajectory.GetSpeedAtPosition(closestpoint);

            // targetSpeed = Mathf.Max(targetSpeed, 5f); //The minimum target speed is 5 because otherwise it doesn't start (TODO to be removed after the improved target speed profile ?)
            targetSpeed = trackTrajectoryScore > 0.4 ? 10f : targetSpeed; //We limit the speed of the car if it goes to far away from the trajectory

            controller.GainDerivative = gainDerivative;
            controller.GainIntegral = gainIntegral;
            controller.GainProportional = gainProportional;


            controller.SetPoint = targetSpeed * 2.23693629f; //The target speed in car speed unit (MPH)

            float currentSpeed = m_Car.CurrentSpeed; //Also in car unit
            currentSpeed = m_Car.Backing ? -currentSpeed : currentSpeed;
            controller.ProcessVariable = currentSpeed;
            currentThrottle = (float)controller.ControlVariable(System.TimeSpan.FromSeconds(Time.fixedDeltaTime));

            Debug.Log("Target speed (in game unit): " + targetSpeed);
            Debug.Log("Speed (in game unit): " + currentSpeed / 2.23693629f); //Divided to have game units
            recordedSpeed.Add(new double[] { distanceFromStart, currentSpeed / 2.23693629f });
        }

        private void computeSteering()
        {
            Vector3 trajectoryTangent = trajectory.getTangentAhead(closestpoint, m_Car.CurrentSpeed / 2.23693629f);
            Debug.DrawRay(closestpoint, trajectoryTangent * 2, Color.red);

            float psi = Vector3.Angle(transform.forward, trajectoryTangent);
            psi = Vector3.Cross(transform.forward, trajectoryTangent).y < 0 ? -psi : psi;


            float carDistanceToTrajectory = Vector3.Distance(currentPosition, closestpoint);
            float crossTrack = Mathf.Atan((k * carDistanceToTrajectory) / (m_Car.CurrentSpeed / 2.23693629f + ks)) * 360 / Mathf.PI;
            crossTrack = Vector3.Cross(transform.forward, closestpoint - currentPosition).y < 0 ? -crossTrack : crossTrack;

            currentSteering = Mathf.Clamp(psi + crossTrack, -25, 25) / 25;

            trackTrajectoryScore = ((1 - Mathf.Exp(-carDistanceToTrajectory / distanceRelaxation)) + Mathf.Abs(psi / 180)) / 2;
        }


        void OnDrawGizmos()
        {
            Gizmos.DrawSphere(currentPosition, 0.1f);
            Debug.DrawLine(currentPosition, closestpoint, Color.blue);
        }


    }
}
