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
                writer.Write(3);
                writer.Write(batch.Sequence);
                writer.Write(batch.RelMs);
                writer.Write(batch.EndRelMs);
                writer.Write(batch.HostTimestamp);
                writer.Write(batch.DeviceTimestampRaw);
                writer.Write(batch.TimestampType);
                writer.Write(batch.PacketCount);
                writer.Write(batch.Count);
                LidarPacketInfo[] packets = NormalizePackets(batch);
                writer.Write(packets.Length);
                for (int i = 0; i < packets.Length; i++)
                {
                    writer.Write(packets[i].HostTimestamp);
                    writer.Write(packets[i].DeviceTimestampRaw);
                    writer.Write(packets[i].TimestampType);
                    writer.Write(packets[i].RelMs);
                    writer.Write(packets[i].PointStart);
                    writer.Write(packets[i].PointCount);
                }
                for (int i = 0; i < batch.Count; i++)
                {
                    LidarPoint point = batch.Points[i];
                    writer.Write(point.XMillimeters);
                    writer.Write(point.YMillimeters);
                    writer.Write(point.ZMillimeters);
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
                if (version != 1 && version != 2 && version != 3) throw new InvalidDataException("不支持的 LIVOXBIN 版本：" + version);
                LidarBatch batch = new LidarBatch();
                batch.Sequence = reader.ReadInt32();
                batch.RelMs = reader.ReadDouble();
                if (version >= 2)
                {
                    batch.EndRelMs = reader.ReadDouble();
                    batch.HostTimestamp = reader.ReadInt64();
                    batch.DeviceTimestampRaw = reader.ReadInt64();
                    batch.TimestampType = reader.ReadByte();
                    batch.PacketCount = reader.ReadInt32();
                }
                else
                {
                    batch.EndRelMs = batch.RelMs;
                    batch.DeviceTimestampRaw = reader.ReadInt64();
                    batch.PacketCount = 1;
                }
                batch.Count = reader.ReadInt32();
                if (batch.Count < 0) throw new InvalidDataException("LiDAR 点数无效：" + batch.Count);
                if (version == 3)
                {
                    int storedPacketCount = reader.ReadInt32();
                    if (storedPacketCount < 0) throw new InvalidDataException("LiDAR 包数无效：" + storedPacketCount);
                    batch.Packets = new LidarPacketInfo[storedPacketCount];
                    for (int i = 0; i < storedPacketCount; i++)
                    {
                        batch.Packets[i].HostTimestamp = reader.ReadInt64();
                        batch.Packets[i].DeviceTimestampRaw = reader.ReadInt64();
                        batch.Packets[i].TimestampType = reader.ReadByte();
                        batch.Packets[i].RelMs = reader.ReadDouble();
                        batch.Packets[i].PointStart = reader.ReadInt32();
                        batch.Packets[i].PointCount = reader.ReadInt32();
                    }
                }
                batch.Points = new LidarPoint[batch.Count];
                for (int i = 0; i < batch.Count; i++)
                {
                    if (version == 3)
                    {
                        batch.Points[i] = LidarPoint.FromMillimeters(reader.ReadInt32(), reader.ReadInt32(),
                            reader.ReadInt32(), reader.ReadByte());
                    }
                    else
                    {
                        batch.Points[i] = LidarPoint.FromMeters(reader.ReadSingle(), reader.ReadSingle(),
                            reader.ReadSingle(), reader.ReadByte());
                    }
                }
                if (batch.Packets == null) batch.Packets = NormalizePackets(batch);
                return batch;
            }
        }

        private static LidarPacketInfo[] NormalizePackets(LidarBatch batch)
        {
            if (batch.Packets != null && batch.Packets.Length > 0) return batch.Packets;
            return new[] { new LidarPacketInfo { HostTimestamp = batch.HostTimestamp,
                DeviceTimestampRaw = batch.DeviceTimestampRaw, TimestampType = batch.TimestampType,
                RelMs = batch.RelMs, PointStart = 0, PointCount = batch.Count } };
        }
    }
}
