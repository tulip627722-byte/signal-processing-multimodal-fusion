using System;
using System.Diagnostics;
using System.IO;
using System.Threading;

namespace Acquire
{
    internal sealed class ReplayLidarSource : ILidarSource
    {
        private readonly string[] _files;
        private readonly Func<double> _clock;
        private readonly Stopwatch _pacer = new Stopwatch();
        private readonly LidarStatus _status = new LidarStatus();
        private int _index;
        private double _nextDue;

        public ReplayLidarSource(string directory, Func<double> clock)
        {
            _files = Directory.Exists(directory) ? Directory.GetFiles(directory, "*.bin") : new string[0];
            Array.Sort(_files, StringComparer.OrdinalIgnoreCase);
            _clock = clock ?? delegate { return 0; };
        }

        public string Model { get { return "Livox 二进制回放"; } }
        public string Serial { get { return "REPLAY"; } }
        public LidarStatus Status { get { return _status; } }

        public void Open()
        {
            if (_files.Length == 0) throw new InvalidOperationException("回放目录没有 *.bin 点云批次");
            _index = 0;
            _nextDue = 0;
            _pacer.Restart();
            _status.Connected = true;
            _status.State = "回放中";
            _status.Detail = _files.Length + " 个批次";
        }

        public LidarBatch ReadBatch(int timeoutMs)
        {
            if (!_status.Connected || _index >= _files.Length)
            {
                _status.Connected = false;
                _status.State = "回放结束";
                return null;
            }
            double now = _pacer.Elapsed.TotalMilliseconds;
            if (now < _nextDue)
            {
                int delay = Math.Min(timeoutMs, (int)Math.Ceiling(_nextDue - now));
                if (delay > 0) Thread.Sleep(delay);
                if (_pacer.Elapsed.TotalMilliseconds + 0.5 < _nextDue) return null;
            }
            LidarBatch batch = LidarBinary.Read(_files[_index++]);
            batch.RelMs = _clock();
            _nextDue = _pacer.Elapsed.TotalMilliseconds + 50;
            return batch;
        }

        public void Close() { _status.Connected = false; _status.State = "已停止"; }
        public void Dispose() { Close(); }
    }
}
