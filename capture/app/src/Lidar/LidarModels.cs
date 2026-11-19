using System;

namespace Acquire
{
    internal struct LidarPoint
    {
        public int XMillimeters;
        public int YMillimeters;
        public int ZMillimeters;
        public byte Reflectivity;

        public float X { get { return XMillimeters / 1000.0f; } }
        public float Y { get { return YMillimeters / 1000.0f; } }
        public float Z { get { return ZMillimeters / 1000.0f; } }

        public static LidarPoint FromMillimeters(int x, int y, int z, byte reflectivity)
        {
            return new LidarPoint { XMillimeters = x, YMillimeters = y, ZMillimeters = z,
                Reflectivity = reflectivity };
        }

        public static LidarPoint FromMeters(float x, float y, float z, byte reflectivity)
        {
            return FromMillimeters((int)Math.Round(x * 1000.0), (int)Math.Round(y * 1000.0),
                (int)Math.Round(z * 1000.0), reflectivity);
        }
    }

    internal struct LidarPacketInfo
    {
        public long HostTimestamp;
        public long DeviceTimestampRaw;
        public byte TimestampType;
        public double RelMs;
        public int PointStart;
        public int PointCount;
    }

    internal sealed class LidarBatch
    {
        public long DeviceTimestampRaw;
        public byte TimestampType;
        public long HostTimestamp;
        public double RelMs;
        public double EndRelMs;
        public LidarPoint[] Points;
        public LidarPacketInfo[] Packets;
        public int Count;
        public int PacketCount = 1;
        public int Sequence;
    }

    internal sealed class LidarStatus
    {
        public bool Connected;
        public string State = "未启动";
        public string Detail = "";
        public long DroppedBatches;
        public string Serial = "";
        public string Firmware = "";
    }

    internal interface ILidarSource : IDisposable
    {
        string Model { get; }
        string Serial { get; }
        LidarStatus Status { get; }
        void Open();
        LidarBatch ReadBatch(int timeoutMs);
        void Close();
    }
}
