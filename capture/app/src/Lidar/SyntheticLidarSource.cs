using System;
using System.Diagnostics;
using System.Threading;

namespace Acquire
{
    internal sealed class SyntheticLidarSource : ILidarSource
    {
        private readonly Func<double> _clock;
        private readonly Stopwatch _pacer = new Stopwatch();
        private readonly Random _random = new Random(70);
        private readonly LidarStatus _status = new LidarStatus();
        private int _sequence;
        private double _nextDueMs;
        private bool _open;

        public SyntheticLidarSource(Func<double> clock) { _clock = clock ?? delegate { return 0; }; }
        public string Model { get { return "Livox MID-70 合成源"; } }
        public string Serial { get { return "MID70-SIM"; } }
        public LidarStatus Status { get { return _status; } }

        public void Open()
        {
            _open = true;
            _pacer.Restart();
            _nextDueMs = 0;
            _status.Connected = true;
            _status.State = "模拟运行";
            _status.Detail = "70.4° 圆形视场";
        }

        public LidarBatch ReadBatch(int timeoutMs)
        {
            if (!_open) return null;
            double now = _pacer.Elapsed.TotalMilliseconds;
            if (now < _nextDueMs)
            {
                int delay = Math.Min(timeoutMs, (int)Math.Ceiling(_nextDueMs - now));
                if (delay > 0) Thread.Sleep(delay);
                if (_pacer.Elapsed.TotalMilliseconds + 0.5 < _nextDueMs) return null;
            }
            _nextDueMs = _pacer.Elapsed.TotalMilliseconds + 50;
            const int count = 2500;
            LidarPoint[] points = new LidarPoint[count];
            double phase = _sequence * 0.045;
            double maxAngle = 35.2 * Math.PI / 180.0;
            for (int i = 0; i < count; i++)
            {
                double radius = Math.Sqrt((i + 0.5) / count);
                double azimuth = i * 2.399963229728653 + phase;
                double angle = radius * maxAngle;
                double distance = 4.5 + 0.55 * Math.Sin(azimuth * 3 + phase) + (_random.NextDouble() - 0.5) * 0.03;
                double lateral = Math.Tan(angle) * distance;
                points[i].X = (float)(Math.Cos(azimuth) * lateral);
                points[i].Y = (float)(Math.Sin(azimuth) * lateral);
                points[i].Z = (float)distance;
                points[i].Reflectivity = (byte)(70 + 170 * radius);
            }
            _sequence++;
            return new LidarBatch
            {
                DeviceTimestampRaw = (long)(_pacer.Elapsed.TotalMilliseconds * 1000000.0),
                RelMs = _clock(),
                Points = points,
                Count = count,
                Sequence = _sequence
            };
        }

        public void Close()
        {
            _open = false;
            _status.Connected = false;
            _status.State = "已停止";
        }

        public void Dispose() { Close(); }
    }
}
