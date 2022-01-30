using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Text;
using System.IO;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(CarController))]
public class CarTesting : MonoBehaviour
{

    private List<(float, float, bool)> keyFrames = new List<(float, float, bool)>(); //First is time and second is speed, third is "isSkidding"

    private CarController m_Car; // the car controller we want to use

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

        m_Car.Move(0f, 1f, 0f, 0f);
        keyFrames.Add((Time.time, m_Car.CurrentSpeed, m_Car.Skidding));

        if (Input.GetKey("s"))
        {
            SaveToFile();
        }
    }


    private string ToCSV()
    {
        var sb = new StringBuilder("Time,Value");
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
}
