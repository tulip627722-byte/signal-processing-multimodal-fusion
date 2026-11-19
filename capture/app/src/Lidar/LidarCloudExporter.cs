using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal static class LidarCloudExporter
    {
        public static int ExportTake(string takeDirectory, string outputDirectory, bool merged)
        {
            string inputDirectory = Path.Combine(takeDirectory, "lidar");
            if (!Directory.Exists(inputDirectory))
                throw new DirectoryNotFoundException("take 缺少 lidar 目录：" + takeDirectory);
            string[] files = Directory.GetFiles(inputDirectory, "*.bin");
            Array.Sort(files, StringComparer.OrdinalIgnoreCase);
            if (files.Length == 0) throw new InvalidOperationException("lidar 目录没有原始批次");

            if (string.IsNullOrEmpty(outputDirectory))
                outputDirectory = Path.Combine(takeDirectory, "lidar_cloud");
            Directory.CreateDirectory(outputDirectory);

            List<LidarPoint> all = merged ? new List<LidarPoint>() : null;
            StringBuilder index = new StringBuilder();
            index.Append("batch,first_t_rel_ms,last_t_rel_ms,packets,points,source,ply\r\n");
            for (int i = 0; i < files.Length; i++)
            {
                LidarBatch batch = LidarBinary.Read(files[i]);
                string outputName = Path.GetFileNameWithoutExtension(files[i]) + ".ply";
                string outputPath = Path.Combine(outputDirectory, outputName);
                WritePly(outputPath, batch.Points, batch.Count,
                    "Livox sensor frame: X forward, Y left, Z up; source " + Path.GetFileName(files[i]));
                if (merged) all.AddRange(batch.Points);
                index.Append(batch.Sequence.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(batch.RelMs.ToString("0.000", CultureInfo.InvariantCulture)).Append(',')
                    .Append(batch.EndRelMs.ToString("0.000", CultureInfo.InvariantCulture)).Append(',')
                    .Append(batch.PacketCount.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(batch.Count.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append("lidar/").Append(Path.GetFileName(files[i])).Append(',')
                    .Append(outputName).Append("\r\n");
            }
            File.WriteAllText(Path.Combine(outputDirectory, "index.csv"), index.ToString(), new UTF8Encoding(false));
            if (merged)
                WritePly(Path.Combine(outputDirectory, "merged.ply"), all.ToArray(), all.Count,
                    "Merged Livox sensor-frame cloud; no motion compensation");
            return files.Length;
        }

        public static void WritePly(string path, LidarPoint[] points, int count, string comment)
        {
            if (points == null || count < 0 || count > points.Length)
                throw new ArgumentException("LiDAR 点数组无效");
            StringBuilder header = new StringBuilder();
            header.Append("ply\nformat binary_little_endian 1.0\n");
            header.Append("comment Generated from lossless Livox raw Cartesian records by Acquire.exe\n");
            if (!string.IsNullOrEmpty(comment)) header.Append("comment ").Append(comment.Replace('\r', ' ').Replace('\n', ' ')).Append('\n');
            header.Append("element vertex ").Append(count.ToString(CultureInfo.InvariantCulture)).Append('\n');
            header.Append("property float x\nproperty float y\nproperty float z\nproperty uchar intensity\nend_header\n");
            using (FileStream stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.Read, 1 << 16))
            using (BinaryWriter writer = new BinaryWriter(stream, Encoding.ASCII))
            {
                writer.Write(Encoding.ASCII.GetBytes(header.ToString()));
                for (int i = 0; i < count; i++)
                {
                    writer.Write(points[i].X);
                    writer.Write(points[i].Y);
                    writer.Write(points[i].Z);
                    writer.Write(points[i].Reflectivity);
                }
            }
        }
    }
}
