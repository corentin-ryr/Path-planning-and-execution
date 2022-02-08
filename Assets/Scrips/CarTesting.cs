using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityStandardAssets.Vehicles.Car;
using Logging;

public enum Scenario
{
    accelerationProfile,
    speedStability,
    maxTurnProfile,
    responseTime,
    brakeTime
}

[RequireComponent(typeof(CarController))]
public class CarTesting : MonoBehaviour
{

    [Header("Testing parameters")]
    public Scenario scenario;
    public float targetSpeed = 20f;
    public float steering = 0;
    public float gainDerivative = 1f;
    public float gainIntegral = 1f;
    public float gainProportional = 1f;


    private List<float[]> keyFrames = new List<float[]>(); //First is time and second is speed, third is "isSkidding"
    private CarController m_Car; // the car controller we want to use
    PidController controller = new PidController(1f, 1f, 0.1f, 1f, -1f);


    // Start is called before the first frame update
    void Start()
    {
        m_Car = GetComponent<CarController>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Time.time < 2)
        {
            return;
        }

        switch (scenario)
        {
            case Scenario.accelerationProfile:
                m_Car.Move(0f, 1f, 0f, 0f);
                keyFrames.Add(new float[] { Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0 });
                break;

            case Scenario.speedStability:
                m_Car.Move(0f, computeThrottle(targetSpeed), 0f, 0f);
                keyFrames.Add(new float[] { Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0 });
                break;

            case Scenario.maxTurnProfile:
                // steering = Mathf.Clamp(((int)(Time.time - 10) / 5) * 0.1f, 0, 1);
                targetSpeed = 70 + (Time.time - 3) * 0.2f;

                m_Car.Move(steering, legacyComputeThrottle(targetSpeed), 0f, 0f);
                keyFrames.Add(new float[] { Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0 });

                print(m_Car.Skidding);
                break;

            case Scenario.responseTime:
                m_Car.Move(0f, computeThrottle(targetSpeed), computeThrottle(targetSpeed), 0f);
                keyFrames.Add(new float[] { Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0 });

                if (Time.time > 15)
                {
                    targetSpeed = 100;
                }
                break;

            case Scenario.brakeTime:
                if (Time.time < 10)
                {
                    m_Car.Move(0f, computeThrottle(targetSpeed), computeThrottle(targetSpeed), 0f);
                }
                else
                {
                    m_Car.Move(0f, computeThrottle(0), computeThrottle(0), 0f);
                }
                keyFrames.Add(new float[] { Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0 });
                break;


            default:
                break;
        }


        if (Input.GetKey("s"))
        {
            DataLogger dataLogger = new DataLogger(keyFrames);
            dataLogger.SaveToFile(scenario.ToString() + " " + targetSpeed.ToString() + "\nTime,Value,Skidding");
        }
    }

    private float computeThrottle(float targetSpeed)
    {
        controller.GainDerivative = gainDerivative;
        controller.GainIntegral = gainIntegral;
        controller.GainProportional = gainProportional;


        controller.SetPoint = targetSpeed;

        float currentSpeed = m_Car.CurrentSpeed;
        currentSpeed = m_Car.Backing ? -currentSpeed : currentSpeed;
        controller.ProcessVariable = currentSpeed;
        float currentThrottle = (float)controller.ControlVariable(System.TimeSpan.FromSeconds(Time.fixedDeltaTime));

        Debug.Log("Target speed: " + targetSpeed);
        Debug.Log("Speed: " + currentSpeed);
        return currentThrottle;
    }

    private float legacyComputeThrottle(float targetSpeed)
    {
        float currentThrottle;
        float speed = m_Car.CurrentSpeed;

        if (targetSpeed > speed)
        {
            currentThrottle = 1f;
        }
        else
        {
            currentThrottle = 0f;
        }
        return currentThrottle;
    }


}
