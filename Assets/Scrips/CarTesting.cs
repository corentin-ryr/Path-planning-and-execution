using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Text;
using System.IO;
using UnityStandardAssets.Vehicles.Car;

public enum Scenario
{
    accelerationProfile,
    speedStability,
    maxTurnProfile,
    responseTime
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


    private List<(float, float, int)> keyFrames = new List<(float, float, int)>(); //First is time and second is speed, third is "isSkidding"
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
                keyFrames.Add((Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0));
                break;

            case Scenario.speedStability:
                m_Car.Move(0f, computeThrottle(targetSpeed), 0f, 0f);
                keyFrames.Add((Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0));
                break;

            case Scenario.maxTurnProfile:
                // steering = Mathf.Clamp(((int)(Time.time - 10) / 5) * 0.1f, 0, 1);
                targetSpeed = 70 + (Time.time - 3) * 0.2f;

                m_Car.Move(steering, legacyComputeThrottle(targetSpeed), 0f, 0f);
                keyFrames.Add((Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0));

                print(m_Car.Skidding);
                break;

            case Scenario.responseTime:
                m_Car.Move(0f, computeThrottle(targetSpeed), computeThrottle(targetSpeed), 0f);
                keyFrames.Add((Time.time, m_Car.CurrentSpeed, m_Car.Skidding ? 1 : 0));

                if (Time.time > 15)
                {
                    targetSpeed = 100;
                }
                break;


            default:
                break;
        }


        if (Input.GetKey("s"))
        {
            SaveToFile();
        }
    }

    private float computeThrottle(float targetSpeed)
    {
        controller.GainDerivative = gainDerivative;
        controller.GainIntegral = gainIntegral;
        controller.GainProportional = gainProportional;


        controller.SetPoint = targetSpeed;
        controller.ProcessVariable = m_Car.CurrentSpeed;
        float currentThrottle = (float)controller.ControlVariable(System.TimeSpan.FromSeconds(Time.fixedDeltaTime));

        Debug.Log("Throttle: " + currentThrottle);
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

    #region Save file =======================================================================================
    private string ToCSV()
    {
        var sb = new StringBuilder(scenario.ToString() + " " + targetSpeed.ToString());
        sb.Append('\n').Append("Time,Value,Skidding");
        foreach (var frame in keyFrames)
        {
            sb.Append('\n').Append(frame.Item1.ToString()).Append(',').Append(frame.Item2.ToString()).Append(',').Append(frame.Item3.ToString());
        }

        return sb.ToString();
    }


    public void SaveToFile()
    {
        // Use the CSV generation from before
        var content = ToCSV();

        // The target file path e.g.
#if UNITY_EDITOR
        var folder = Application.streamingAssetsPath;

        if (!Directory.Exists(folder)) Directory.CreateDirectory(folder);
#else
    var folder = Application.persistentDataPath;
#endif

        var filePath = Path.Combine(folder, "export.csv");

        using (var writer = new StreamWriter(filePath, false))
        {
            writer.Write(content);
        }

        // Or just
        //File.WriteAllText(content);

        Debug.Log($"CSV file written to \"{filePath}\"");

#if UNITY_EDITOR
        AssetDatabase.Refresh();
#endif
    }

    #endregion
}
