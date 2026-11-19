using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Acquire
{
    internal sealed class IdleLidarPreview : IDisposable
    {
        private readonly LatestPreview<LidarBatch> _preview = new LatestPreview<LidarBatch>();
        private readonly Action<string> _log;
        private readonly object _gate = new object();
        private Thread _thread;
        private volatile bool _stop;
        private double _batchRate;
        private long _points;
        private string _status = "未启用";

        public IdleLidarPreview(Action<string> log)
        {
            _log = log ?? delegate { };
        }

        public void Start(string broadcastCode)
        {
            Stop();
            _stop = false;
            SetStatus("正在连接 MID-70…");
            _thread = new Thread(new ThreadStart(delegate { PreviewLoop(broadcastCode ?? ""); }));
            _thread.IsBackground = true;
            _thread.Name = "Acquire idle MID-70 preview";
            _thread.Start();
        }

        public void Stop()
        {
            _stop = true;
            Thread thread = _thread;
            _thread = null;
            if (thread != null && thread.IsAlive)
            {
                try { thread.Join(3000); } catch { }
            }
            lock (_gate)
            {
                _batchRate = 0;
                _points = 0;
                if (_status.IndexOf("失败", StringComparison.Ordinal) < 0) _status = "未启用";
            }
            while (_preview.Take() != null) { }
        }

        public LidarBatch TakeBatch()
        {
            return _preview.Take();
        }

        public void Snapshot(out double batchRate, out long points, out string status)
        {
            lock (_gate)
            {
                batchRate = _batchRate;
                points = _points;
                status = _status;
            }
        }

        private void PreviewLoop(string broadcastCode)
        {
            Stopwatch clock = Stopwatch.StartNew();
            LivoxNativeSource source = null;
            const double previewFrameMs = 1000.0;
            List<LidarPoint> accumulated = new List<LidarPoint>(120000);
            long rawBatches = 0;
            double firstMs = double.NaN;
            double lastMs = double.NaN;
            double publishStartMs = double.NaN;
            try
            {
                source = new LivoxNativeSource(broadcastCode,
                    delegate(long timestamp) { return timestamp * 1000.0 / Stopwatch.Frequency; });
                source.Open();
                while (!_stop)
                {
                    LidarBatch batch = source.ReadBatch(150);
                    if (batch == null) continue;
                    rawBatches++;
                    if (double.IsNaN(firstMs)) firstMs = batch.RelMs;
                    lastMs = batch.RelMs;
                    if (double.IsNaN(publishStartMs)) publishStartMs = batch.RelMs;
                    for (int index = 0; index < batch.Count; index++) accumulated.Add(batch.Points[index]);
                    if (batch.RelMs - publishStartMs >= previewFrameMs)
                    {
                        LidarBatch preview = new LidarBatch
                        {
                            Sequence = (int)rawBatches,
                            RelMs = publishStartMs,
                            EndRelMs = batch.RelMs,
                            Points = accumulated.ToArray(),
                            Count = accumulated.Count,
                            PacketCount = (int)rawBatches
                        };
                        _preview.Publish(preview);
                        accumulated.Clear();
                        publishStartMs = double.NaN;
                    }
                    string identity = string.IsNullOrEmpty(source.Status.Serial) ? "自动发现" : source.Status.Serial;
                    string firmware = string.IsNullOrEmpty(source.Status.Firmware) ? "" : " / FW " + source.Status.Firmware;
                    lock (_gate)
                    {
                        _batchRate = rawBatches > 1 && lastMs > firstMs ? (rawBatches - 1) * 1000.0 / (lastMs - firstMs) : 0;
                        _points += batch.Count;
                        _status = "实时预览 " + identity + firmware + "（不保存）";
                    }
                }
            }
            catch (Exception ex)
            {
                if (!_stop)
                {
                    string message = DynHelper.DescribeException(ex);
                    SetStatus("预览失败：" + message);
                    _log("MID-70 空闲预览失败：" + message);
                }
            }
            finally
            {
                if (source != null) try { source.Dispose(); } catch { }
            }
        }

        private void SetStatus(string status)
        {
            lock (_gate) _status = status;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
