using System;
using System.IO;
using System.Text;

namespace Acquire
{
    internal static class LidarBinary
    {
        private static readonly byte[] Magic = Encoding.ASCII.GetBytes("LIVOXBIN");

        public static void Write(string path, LidarBatch batch)
        {
            using (FileStream stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.Read))
            using (BinaryWriter writer = new BinaryWriter(stream, Encoding.UTF8))
            {
                writer.Write(Magic);
                writer.Write(1);
                writer.Write(batch.Sequence);
                writer.Write(batch.RelMs);
                writer.Write(batch.DeviceTimestampRaw);
                writer.Write(batch.Count);
                for (int i = 0; i < batch.Count; i++)
                {
                    LidarPoint point = batch.Points[i];
                    writer.Write(point.X);
                    writer.Write(point.Y);
                    writer.Write(point.Z);
                    writer.Write(point.Reflectivity);
                }
            }
        }

        public static LidarBatch Read(string path)
        {
            using (FileStream stream = File.OpenRead(path))
            using (BinaryReader reader = new BinaryReader(stream, Encoding.UTF8))
            {
                byte[] magic = reader.ReadBytes(Magic.Length);
                if (Encoding.ASCII.GetString(magic) != "LIVOXBIN") throw new InvalidDataException("不是 LIVOXBIN 文件");
                int version = reader.ReadInt32();
                if (version != 1) throw new InvalidDataException("不支持的 LIVOXBIN 版本：" + version);
                LidarBatch batch = new LidarBatch();
                batch.Sequence = reader.ReadInt32();
                batch.RelMs = reader.ReadDouble();
                batch.DeviceTimestampRaw = reader.ReadInt64();
                batch.Count = reader.ReadInt32();
                batch.Points = new LidarPoint[batch.Count];
                for (int i = 0; i < batch.Count; i++)
                {
                    batch.Points[i].X = reader.ReadSingle();
                    batch.Points[i].Y = reader.ReadSingle();
                    batch.Points[i].Z = reader.ReadSingle();
                    batch.Points[i].Reflectivity = reader.ReadByte();
                }
                return batch;
            }
        }
    }
}
