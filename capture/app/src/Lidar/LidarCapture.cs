using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using System.Threading;

namespace Acquire
{
    internal sealed class LidarCaptureStats
    {
        public long Batches;
        public long Points;
        public long Bytes;
        public double Fps;
        public double FirstRelMs = double.NaN;
        public double LastRelMs = double.NaN;
        public string State = "未启动";
        public string Error = "";
    }

    internal sealed class LidarCapture : IDisposable
    {
        private readonly object _gate = new object();
        private readonly ILidarSource _source;
        private readonly TakeWriter _take;
        private readonly LatestPreview<LidarBatch> _preview;
        private readonly Action<string> _log;
        private Thread _thread;
        private volatile bool _stop;
        private LidarCaptureStats _stats = new LidarCaptureStats();
        private int _outputSequence;

        public LidarCapture(ILidarSource source, TakeWriter take, LatestPreview<LidarBatch> preview, Action<string> log)
        {
            _source = source;
            _take = take;
            _preview = preview;
            _log = log ?? delegate { };
        }

        public void Start()
        {
            _source.Open();
            _take.EnsureLidarDirectory();
            _stop = false;
            lock (_gate) _stats.State = _source.Status.State;
            _thread = new Thread(Loop) { IsBackground = true, Name = "Acquire lidar" };
            _thread.Start();
        }

        private void Loop()
        {
            List<LidarPoint> points = new List<LidarPoint>(10000);
            double firstRelMs = double.NaN;
            long firstDeviceTimestampRaw = 0;
            while (!_stop)
            {
                try
                {
                    LidarBatch batch = _source.ReadBatch(100);
                    if (batch != null)
                    {
                        if (double.IsNaN(firstRelMs))
                        {
                            firstRelMs = batch.RelMs;
                            firstDeviceTimestampRaw = batch.DeviceTimestampRaw;
                        }
                        for (int i = 0; i < batch.Count; i++) points.Add(batch.Points[i]);
                    }
                    double latestRelMs = batch == null ? firstRelMs : batch.RelMs;
                    if (points.Count > 0 && (points.Count >= 10000 || latestRelMs - firstRelMs >= 45 || (_stop && points.Count > 0)))
                    {
                        LidarBatch output = new LidarBatch
                        {
                            Sequence = ++_outputSequence, RelMs = firstRelMs, DeviceTimestampRaw = firstDeviceTimestampRaw,
                            Points = points.ToArray(), Count = points.Count
                        };
                        WriteBatch(output);
                        points.Clear();
                        firstRelMs = double.NaN;
                    }
                }
                catch (Exception ex)
                {
                    lock (_gate) { _stats.Error = ex.Message; _stats.State = "故障"; }
                    _log("LiDAR 采集异常：" + ex.Message);
                    Thread.Sleep(200);
                }
            }
            if (points.Count > 0)
                WriteBatch(new LidarBatch { Sequence = ++_outputSequence, RelMs = firstRelMs, DeviceTimestampRaw = firstDeviceTimestampRaw, Points = points.ToArray(), Count = points.Count });
        }

        private void WriteBatch(LidarBatch batch)
        {
            _preview.Publish(batch);
            string name = batch.Sequence.ToString("D6") + ".bin";
            string path = Path.Combine(_take.LidarDirectory, name);
            LidarBinary.Write(path, batch);
            long bytes = new FileInfo(path).Length;
            _take.AddRow("lidar", batch.Sequence, batch.RelMs, "lidar/" + name);
            lock (_gate)
            {
                if (double.IsNaN(_stats.FirstRelMs)) _stats.FirstRelMs = batch.RelMs;
                _stats.LastRelMs = batch.RelMs;
                _stats.Batches++;
                _stats.Points += batch.Count;
                _stats.Bytes += bytes;
                _stats.State = _source.Status.State;
            }
        }

        public void Stop()
        {
            _stop = true;
            try { _source.Close(); } catch { }
            if (_thread != null && _thread.IsAlive) _thread.Join(3000);
            _thread = null;
        }

        public LidarCaptureStats Snapshot()
        {
            lock (_gate)
            {
                LidarCaptureStats copy = new LidarCaptureStats
                {
                    Batches = _stats.Batches, Points = _stats.Points, Bytes = _stats.Bytes,
                    FirstRelMs = _stats.FirstRelMs, LastRelMs = _stats.LastRelMs,
                    State = _stats.State, Error = _stats.Error
                };
                double span = (copy.LastRelMs - copy.FirstRelMs) / 1000.0;
                copy.Fps = copy.Batches > 1 && span > 0 ? (copy.Batches - 1) / span : 0;
                return copy;
            }
        }

        public void Dispose() { Stop(); try { _source.Dispose(); } catch { } }
    }
}
