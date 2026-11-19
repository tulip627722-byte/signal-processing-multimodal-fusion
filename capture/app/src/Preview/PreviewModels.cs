using System;
using System.Threading;
using System.Diagnostics;

namespace Acquire
{
    internal sealed class LatestPreview<T> where T : class
    {
        private sealed class Slot
        {
            public T Value;
            public long PublishedTicks;
        }

        private Slot _slot;
        private long _version;
        private long _overwritten;
        private double _lastTakeLatencyMs;

        public void Publish(T value)
        {
            if (value == null) return;
            Slot next = new Slot { Value = value, PublishedTicks = Stopwatch.GetTimestamp() };
            if (Interlocked.Exchange(ref _slot, next) != null)
                Interlocked.Increment(ref _overwritten);
            Interlocked.Increment(ref _version);
        }

        public T Take()
        {
            Slot slot = Interlocked.Exchange(ref _slot, null);
            if (slot != null)
            {
                _lastTakeLatencyMs = (Stopwatch.GetTimestamp() - slot.PublishedTicks) * 1000.0 / Stopwatch.Frequency;
                return slot.Value;
            }
            return null;
        }

        public long Version { get { return Interlocked.Read(ref _version); } }
        public long Overwritten { get { return Interlocked.Read(ref _overwritten); } }
        public double LastTakeLatencyMs { get { return _lastTakeLatencyMs; } }
    }

    internal sealed class CameraPreviewFrame
    {
        public byte[] ColorBgr;
        public int ColorWidth;
        public int ColorHeight;
        public ushort[] Depth;
        public int DepthWidth;
        public int DepthHeight;
        public double DepthScaleMeters;
        public double ColorRelMs = double.NaN;
        public double DepthRelMs = double.NaN;
        public double ValidDepthRatio;
    }

    internal sealed class RadarPreviewFrame
    {
        public double[] RangeMeters;
        public double[] MagnitudeDb;
        public double PeakRangeMeters;
        public double PeakMagnitudeDb;
        public double LeakCutoffMeters;
        public int AveragedChirps;
        public int FrameNumber;
        public double RelMs;
    }

    internal struct RadarCloudPoint
    {
        public float X;
        public float Y;
        public float Z;
        public float MagnitudeDb;
        public float RangeMeters;
        public float VelocityMetersPerSecond;
        public uint Cluster;
    }

    internal sealed class RadarCloudFrame
    {
        public RadarCloudPoint[] Points;
        public int Count;
        public uint Sequence;
        public uint Status;
        public uint BoardTimestampMicroseconds;
        public uint ClusterCount;
        public int Version;
        public double ReceivedAtMs;
    }
}
