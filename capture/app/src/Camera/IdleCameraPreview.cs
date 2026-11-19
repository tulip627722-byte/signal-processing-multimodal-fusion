using System;
using System.Diagnostics;
using System.Threading;

namespace Acquire
{
    internal sealed class IdleCameraPreview : IDisposable
    {
        private readonly LatestPreview<CameraPreviewFrame> _preview = new LatestPreview<CameraPreviewFrame>();
        private readonly Action<string> _log;
        private readonly object _statusGate = new object();
        private Thread _thread;
        private volatile bool _stop;
        private double _colorFps;
        private double _depthFps;
        private string _status = "未启动";

        public IdleCameraPreview(Action<string> log)
        {
            _log = log ?? delegate { };
        }

        public void Start(CameraOption option)
        {
            Stop();
            if (option == null || !option.Available) return;
            _stop = false;
            SetStatus("正在打开 " + option.Model + "…");
            _thread = new Thread(new ThreadStart(delegate { PreviewLoop(option); }));
            _thread.IsBackground = true;
            _thread.Name = "Acquire idle camera preview";
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
            lock (_statusGate)
            {
                _colorFps = 0;
                _depthFps = 0;
                if (_status.IndexOf("失败", StringComparison.Ordinal) < 0) _status = "已停止";
            }
            while (_preview.Take() != null) { }
        }

        public CameraPreviewFrame TakeFrame()
        {
            return _preview.Take();
        }

        public void Snapshot(out double colorFps, out double depthFps, out string status)
        {
            lock (_statusGate)
            {
                colorFps = _colorFps;
                depthFps = _depthFps;
                status = _status;
            }
        }

        private void PreviewLoop(CameraOption option)
        {
            Stopwatch clock = Stopwatch.StartNew();
            ICameraSource source = null;
            long colorFrames = 0;
            long depthFrames = 0;
            double firstColorMs = double.NaN;
            double firstDepthMs = double.NaN;
            double lastColorMs = double.NaN;
            double lastDepthMs = double.NaN;
            double lastColorPublishMs = double.NegativeInfinity;
            double lastDepthPublishMs = double.NegativeInfinity;
            CameraPreviewFrame latest = new CameraPreviewFrame();
            try
            {
                source = CameraProbe.Create(option, _log, delegate { return clock.Elapsed.TotalMilliseconds; });
                source.Open();
                SetStatus("相机预览中（不保存）");
                _log("空闲预览已启动：" + source.Model + "（不写文件）");
                while (!_stop)
                {
                    CameraFrame frame = source.ReadFrame(100);
                    if (frame == null) continue;
                    if (frame.HasColor)
                    {
                        colorFrames++;
                        if (double.IsNaN(firstColorMs)) firstColorMs = frame.ColorRelMs;
                        lastColorMs = frame.ColorRelMs;
                    }
                    if (frame.HasDepth)
                    {
                        depthFrames++;
                        if (double.IsNaN(firstDepthMs)) firstDepthMs = frame.DepthRelMs;
                        lastDepthMs = frame.DepthRelMs;
                    }
                    bool publishColor = frame.HasColor && frame.ColorRelMs - lastColorPublishMs >= 90;
                    bool publishDepth = frame.HasDepth && frame.DepthRelMs - lastDepthPublishMs >= 90;
                    if (publishColor || publishDepth)
                    {
                        if (publishColor)
                        {
                            CopyColor(frame, latest);
                            lastColorPublishMs = frame.ColorRelMs;
                        }
                        if (publishDepth)
                        {
                            CopyDepth(frame, latest);
                            lastDepthPublishMs = frame.DepthRelMs;
                        }
                        _preview.Publish(latest);
                        latest = CloneReferences(latest);
                    }
                    lock (_statusGate)
                    {
                        _colorFps = Fps(colorFrames, firstColorMs, lastColorMs);
                        _depthFps = Fps(depthFrames, firstDepthMs, lastDepthMs);
                    }
                }
            }
            catch (Exception ex)
            {
                if (!_stop)
                {
                    string message = DynHelper.DescribeException(ex);
                    SetStatus("预览失败：" + message);
                    _log("空闲相机预览失败：" + message);
                }
            }
            finally
            {
                if (source != null)
                {
                    try { source.Dispose(); } catch { }
                }
            }
        }

        private static void CopyColor(CameraFrame frame, CameraPreviewFrame preview)
        {
            int length = frame.ColorWidth * frame.ColorHeight * 3;
            preview.ColorBgr = new byte[length];
            Buffer.BlockCopy(frame.ColorBgr, 0, preview.ColorBgr, 0, length);
            preview.ColorWidth = frame.ColorWidth;
            preview.ColorHeight = frame.ColorHeight;
            preview.ColorRelMs = frame.ColorRelMs;
        }

        private static void CopyDepth(CameraFrame frame, CameraPreviewFrame preview)
        {
            int length = frame.DepthWidth * frame.DepthHeight;
            preview.Depth = new ushort[length];
            Array.Copy(frame.Depth, preview.Depth, length);
            preview.DepthWidth = frame.DepthWidth;
            preview.DepthHeight = frame.DepthHeight;
            preview.DepthScaleMeters = frame.DepthScaleMeters;
            preview.DepthRelMs = frame.DepthRelMs;
            int valid = 0;
            int sampled = 0;
            for (int index = 0; index < length; index += 16)
            {
                sampled++;
                if (preview.Depth[index] > 0) valid++;
            }
            preview.ValidDepthRatio = sampled == 0 ? 0 : valid / (double)sampled;
        }

        private static CameraPreviewFrame CloneReferences(CameraPreviewFrame source)
        {
            return new CameraPreviewFrame
            {
                ColorBgr = source.ColorBgr,
                ColorWidth = source.ColorWidth,
                ColorHeight = source.ColorHeight,
                ColorRelMs = source.ColorRelMs,
                Depth = source.Depth,
                DepthWidth = source.DepthWidth,
                DepthHeight = source.DepthHeight,
                DepthScaleMeters = source.DepthScaleMeters,
                DepthRelMs = source.DepthRelMs,
                ValidDepthRatio = source.ValidDepthRatio
            };
        }

        private static double Fps(long frames, double firstMs, double lastMs)
        {
            if (frames < 2 || double.IsNaN(firstMs) || lastMs <= firstMs) return 0;
            return (frames - 1) * 1000.0 / (lastMs - firstMs);
        }

        private void SetStatus(string status)
        {
            lock (_statusGate) _status = status;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
