using System;

namespace Acquire
{
    internal struct LidarPoint
    {
        public float X;
        public float Y;
        public float Z;
        public byte Reflectivity;
    }

    internal sealed class LidarBatch
    {
        public long DeviceTimestampRaw;
        public double RelMs;
        public LidarPoint[] Points;
        public int Count;
        public int Sequence;
    }

    internal sealed class LidarStatus
    {
        public bool Connected;
        public string State = "未启动";
        public string Detail = "";
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
