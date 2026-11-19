using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Acquire
{
    /// <summary>
    /// Shared installation geometry used by the capture GUI and MATLAB calibration.
    /// Coordinates are [right, forward, up] metres in the ADT7880 radar frame.
    /// </summary>
    internal sealed class CalibrationInstallationConfig
    {
        public double[] CameraOriginInRadarM = new double[] { -0.0093, -0.0063, 0.1000 };
        public double[] LidarOriginInRadarM = new double[] { -0.1000, 0.0000, 0.0000 };
        public double[][] LidarToRadarR = new double[][]
        {
            new double[] { 0, -1, 0 },
            new double[] { 1, 0, 0 },
            new double[] { 0, 0, 1 }
        };
        public string CoordinateFrame = "ADT7880 radar: X right, Y forward, Z up";
        public string UpdatedLocal = "";

        public static string SharedPath
        {
            get
            {
                string baseDirectory = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
                DirectoryInfo directory = new DirectoryInfo(baseDirectory);
                if (directory.Name.Equals("bin", StringComparison.OrdinalIgnoreCase) && directory.Parent != null)
                    return Path.Combine(directory.Parent.FullName, "calibration_installation.json");
                return Path.Combine(baseDirectory, "calibration_installation.json");
            }
        }

        public static CalibrationInstallationConfig Load()
        {
            return Load(SharedPath);
        }

        public static CalibrationInstallationConfig Load(string path)
        {
            CalibrationInstallationConfig result = new CalibrationInstallationConfig();
            try
            {
                if (!File.Exists(path)) return result;
                JsonObject root = MiniJson.Parse(File.ReadAllText(path)) as JsonObject;
                if (root == null) return result;
                result.CameraOriginInRadarM = ReadVector(root.Get("cameraOriginInRadarM", null), result.CameraOriginInRadarM);
                result.LidarOriginInRadarM = ReadVector(root.Get("lidarOriginInRadarM", null), result.LidarOriginInRadarM);
                result.LidarToRadarR = ReadMatrix(root.Get("lidarToRadarR", null), result.LidarToRadarR);
                string frame = root.Get("coordinateFrame", null) as string;
                if (!string.IsNullOrEmpty(frame)) result.CoordinateFrame = frame;
                string updated = root.Get("updatedLocal", null) as string;
                if (!string.IsNullOrEmpty(updated)) result.UpdatedLocal = updated;
            }
            catch
            {
                // Invalid local settings should not prevent sensor capture from opening.
            }
            return result;
        }

        public void Save()
        {
            Save(SharedPath);
        }

        public void Save(string path)
        {
            string directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            UpdatedLocal = DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ss.fff", CultureInfo.InvariantCulture);
            File.WriteAllText(path, MiniJson.Serialize(ToJson()), new System.Text.UTF8Encoding(false));
        }

        public CalibrationInstallationConfig Clone()
        {
            return new CalibrationInstallationConfig
            {
                CameraOriginInRadarM = CopyVector(CameraOriginInRadarM),
                LidarOriginInRadarM = CopyVector(LidarOriginInRadarM),
                LidarToRadarR = CopyMatrix(LidarToRadarR),
                CoordinateFrame = CoordinateFrame,
                UpdatedLocal = UpdatedLocal
            };
        }

        public JsonObject ToJson()
        {
            JsonObject root = new JsonObject();
            root.Add("version", 1);
            root.Add("units", "m");
            root.Add("coordinateFrame", CoordinateFrame);
            root.Add("cameraOriginInRadarM", CameraOriginInRadarM);
            root.Add("lidarOriginInRadarM", LidarOriginInRadarM);
            root.Add("lidarToRadarR", LidarToRadarR);
            root.Add("updatedLocal", UpdatedLocal);
            root.Add("note", "Positions are [right, forward, up] in the ADT7880 radar frame.");
            return root;
        }

        private static double[] ReadVector(object value, double[] fallback)
        {
            List<object> list = value as List<object>;
            if (list == null || list.Count != 3) return fallback;
            double[] result = new double[3];
            for (int i = 0; i < 3; i++)
            {
                if (!(list[i] is double) && !(list[i] is int) && !(list[i] is long)) return fallback;
                result[i] = Convert.ToDouble(list[i], CultureInfo.InvariantCulture);
            }
            return result;
        }

        private static double[][] ReadMatrix(object value, double[][] fallback)
        {
            List<object> rows = value as List<object>;
            if (rows == null || rows.Count != 3) return fallback;
            double[][] result = new double[3][];
            for (int r = 0; r < 3; r++)
            {
                result[r] = ReadVector(rows[r], null);
                if (result[r] == null) return fallback;
            }
            return result;
        }

        private static double[] CopyVector(double[] value)
        {
            return value == null ? null : (double[])value.Clone();
        }

        private static double[][] CopyMatrix(double[][] value)
        {
            if (value == null) return null;
            double[][] result = new double[value.Length][];
            for (int i = 0; i < value.Length; i++) result[i] = CopyVector(value[i]);
            return result;
        }
    }
}
