using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;
using UnityEditor;

namespace Logging
{
    public class DataLogger
    {
        List<double[]> keyFrames;

        public List<double[]> KeyFrames { get => keyFrames; set => keyFrames = value; }

        public DataLogger()
        {
            KeyFrames = new List<double[]>();
        }
        public DataLogger(List<double[]> keyFrames)
        {
            KeyFrames = keyFrames;
        }


        private string ToCSV(string header)
        {
            var sb = new StringBuilder(header);
            foreach (var frame in keyFrames)
            {
                sb.Append('\n');
                for (int i = 0; i < frame.Length; i++)
                {
                    sb.Append(frame[i].ToString()).Append(',');
                }
            }

            return sb.ToString();
        }


        public void SaveToFile(string header, string filename = "export")
        {
            // Use the CSV generation from before
            var content = ToCSV(header);

            // The target file path e.g.
#if UNITY_EDITOR
            var folder = Application.streamingAssetsPath;

            if (!Directory.Exists(folder)) Directory.CreateDirectory(folder);
#else
    var folder = Application.persistentDataPath;
#endif

            var filePath = Path.Combine(folder, filename + ".csv");

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