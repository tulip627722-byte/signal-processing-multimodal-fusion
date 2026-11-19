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
        public long DroppedBatches;
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
        private readonly Action<LidarBatch, string> _batchWritten;
        private Thread _thread;
        private volatile bool _stop;
        private LidarCaptureStats _stats = new LidarCaptureStats();
        private int _outputSequence;

        public LidarCapture(ILidarSource source, TakeWriter take, LatestPreview<LidarBatch> preview, Action<string> log,
                            Action<LidarBatch, string> batchWritten = null)
        {
            _source = source;
            _take = take;
            _preview = preview;
            _log = log ?? delegate { };
            _batchWritten = batchWritten;
        }

        public void Start()
        {
            Start(false);
        }

        public void Start(bool sourceAlreadyOpen)
        {
            if (!sourceAlreadyOpen) _source.Open();
            _take.EnsureLidarDirectory();
            _stop = false;
            lock (_gate) _stats.State = _source.Status.State;
            _thread = new Thread(Loop) { IsBackground = true, Name = "Acquire lidar" };
            _thread.Start();
        }

        private void Loop()
        {
            List<LidarPoint> points = new List<LidarPoint>(10000);
            List<LidarPacketInfo> packets = new List<LidarPacketInfo>(128);
            List<LidarPoint> previewPoints = new List<LidarPoint>(120000);
            double previewFirstRelMs = double.NaN;
            double previewLastRelMs = double.NaN;
            double firstRelMs = double.NaN;
            long firstDeviceTimestampRaw = 0;
            long firstHostTimestamp = 0;
            byte timestampType = 0;
            int packetCount = 0;
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
                            firstHostTimestamp = batch.HostTimestamp;
                            timestampType = batch.TimestampType;
                        }
                        packetCount += Math.Max(1, batch.PacketCount);
                        int pointOffset = points.Count;
                        if (batch.Packets != null && batch.Packets.Length > 0)
                        {
                            for (int packetIndex = 0; packetIndex < batch.Packets.Length; packetIndex++)
                            {
                                LidarPacketInfo packet = batch.Packets[packetIndex];
                                packet.PointStart += pointOffset;
                                packets.Add(packet);
                            }
                        }
                        else
                        {
                            packets.Add(new LidarPacketInfo { HostTimestamp = batch.HostTimestamp,
                                DeviceTimestampRaw = batch.DeviceTimestampRaw, TimestampType = batch.TimestampType,
                                RelMs = batch.RelMs, PointStart = pointOffset, PointCount = batch.Count });
                        }
                        for (int i = 0; i < batch.Count; i++) points.Add(batch.Points[i]);
                    }
                    double latestRelMs = batch == null ? firstRelMs : batch.RelMs;
                    if (points.Count > 0 && (points.Count >= 10000 || latestRelMs - firstRelMs >= 45 || (_stop && points.Count > 0)))
                    {
                        LidarBatch output = new LidarBatch
                        {
                            Sequence = ++_outputSequence, RelMs = firstRelMs, EndRelMs = latestRelMs,
                            DeviceTimestampRaw = firstDeviceTimestampRaw, TimestampType = timestampType,
                            HostTimestamp = firstHostTimestamp, PacketCount = packetCount,
                            Points = points.ToArray(), Packets = packets.ToArray(), Count = points.Count
                        };
                        WriteBatch(output);
                        AccumulatePreview(output, previewPoints, ref previewFirstRelMs, ref previewLastRelMs);
                        points.Clear();
                        packets.Clear();
                        firstRelMs = double.NaN;
                        packetCount = 0;
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
            {
                LidarBatch output = new LidarBatch { Sequence = ++_outputSequence, RelMs = firstRelMs, EndRelMs = firstRelMs,
                    DeviceTimestampRaw = firstDeviceTimestampRaw, TimestampType = timestampType,
                    HostTimestamp = firstHostTimestamp, PacketCount = packetCount,
                    Points = points.ToArray(), Packets = packets.ToArray(), Count = points.Count };
                WriteBatch(output);
                AccumulatePreview(output, previewPoints, ref previewFirstRelMs, ref previewLastRelMs);
            }
        }

        private void WriteBatch(LidarBatch batch)
        {
            string name = batch.Sequence.ToString("D6") + ".bin";
            string path = Path.Combine(_take.LidarDirectory, name);
            LidarBinary.Write(path, batch);
            long bytes = new FileInfo(path).Length;
            _take.AddRow("lidar", batch.Sequence, batch.RelMs, "lidar/" + name);
            _take.AddLidarBatch(batch, name);
            if (_batchWritten != null) _batchWritten(batch, name);
            lock (_gate)
            {
                if (double.IsNaN(_stats.FirstRelMs)) _stats.FirstRelMs = batch.RelMs;
                _stats.LastRelMs = batch.RelMs;
                _stats.Batches++;
                _stats.Points += batch.Count;
                _stats.Bytes += bytes;
                _stats.DroppedBatches = _source.Status.DroppedBatches;
                _stats.State = _source.Status.State;
            }
        }

        private void AccumulatePreview(LidarBatch batch, List<LidarPoint> points,
            ref double firstRelMs, ref double lastRelMs)
        {
            const double previewFrameMs = 1000.0;
            if (double.IsNaN(firstRelMs)) firstRelMs = batch.RelMs;
            lastRelMs = batch.EndRelMs;
            points.AddRange(batch.Points);
            if (lastRelMs - firstRelMs < previewFrameMs) return;
            _preview.Publish(new LidarBatch
            {
                RelMs = firstRelMs,
                EndRelMs = lastRelMs,
                Points = points.ToArray(),
                Count = points.Count
            });
            points.Clear();
            firstRelMs = double.NaN;
            lastRelMs = double.NaN;
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
                    State = _stats.State, Error = _stats.Error, DroppedBatches = _stats.DroppedBatches
                };
                double span = (copy.LastRelMs - copy.FirstRelMs) / 1000.0;
                copy.Fps = copy.Batches > 1 && span > 0 ? (copy.Batches - 1) / span : 0;
                return copy;
            }
        }

        public void Dispose() { Stop(); try { _source.Dispose(); } catch { } }
    }
}
