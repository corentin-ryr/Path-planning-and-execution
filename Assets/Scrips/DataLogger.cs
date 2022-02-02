using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;
using UnityEditor;

namespace Logging
{
    public class DataLogger
    {
        List<float[]> keyFrames;

        public List<float[]> KeyFrames { get => keyFrames; set => keyFrames = value; }

        public DataLogger()
        {
            KeyFrames = new List<float[]>();
        }
        public DataLogger(List<float[]> keyFrames)
        {
            KeyFrames = keyFrames;
        }


        private string ToCSV(string header, List<float[]> keyFrames)
        {
            var sb = new StringBuilder(header);
            sb.Append('\n').Append("Time,Value,Skidding");
            foreach (var frame in keyFrames)
            {
                for (int i = 0; i < frame.Length; i++)
                {
                    sb.Append('\n').Append(frame[i].ToString()).Append(',');
                }
            }

            return sb.ToString();
        }


        public void SaveToFile(string header, List<float[]> keyFrames)
        {
            // Use the CSV generation from before
            var content = ToCSV(header, keyFrames);

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
}