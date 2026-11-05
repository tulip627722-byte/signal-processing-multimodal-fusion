using System;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// 复合相机源：深度走 OpenNI2，彩色走 UVC（Media Foundation）。
    ///
    /// 为什么需要它：Astra Pro 的彩色是独立的 UVC 设备（VID_2BC5&amp;PID_0501），
    /// OpenNI2 虽然报告"彩色可用"但一帧都不给（实测 23 种模式全是 0 帧）。
    /// 所以彩色单独用 ds_capture.dll 抓，两路各自打时间戳。
    ///
    /// 对上层（CameraCapture）无感：ReadFrame 返回的 CameraFrame 里
    /// 要么只有彩色、要么只有深度，靠 HasColor/HasDepth 区分。
    /// </summary>
    internal sealed class CompositeCameraSource : ICameraSource, IDepthIntrinsicsProvider
    {
        private readonly ICameraSource _depth;      // OpenNI2（只有深度）
        private readonly DsColorGrabber _color;     // UVC 彩色
        private readonly Action<string> _log;
        private readonly Func<double> _clock;

        private Thread _thread;
        private volatile bool _running;
        private readonly object _gate = new object();
        // 彩色必须排队，不能只存“最新一帧”：
        // ReadFrame 里如果先等深度（最多阻塞 33 ms），这段时间到的彩色会被后者覆盖掉，
        // 实测 20 s 丢 75 帧就是这么来的。
        private readonly System.Collections.Generic.Queue<CameraFrame> _colorQueue =
            new System.Collections.Generic.Queue<CameraFrame>();
        private const int ColorQueueCapacity = 32;
        private long _colorDropped;
        private long _colorFrames;
        private string _colorError = "";
        private bool _opened;

        public CompositeCameraSource(ICameraSource depth, DsColorGrabber color, Action<string> log, Func<double> clock)
        {
            _depth = depth;
            _color = color;
            _log = log;
            _clock = clock;
        }

        public CameraKind Kind { get { return _depth.Kind; } }
        public string Model { get { return _depth.Model + " + UVC 彩色"; } }
        public string Serial { get { return _depth.Serial; } }
        public bool HasColor { get { return _color != null; } }
        public bool HasDepth { get { return _depth.HasDepth; } }
        public double DepthScaleMeters { get { return _depth.DepthScaleMeters; } }
        public long ColorFrames { get { return Interlocked.Read(ref _colorFrames); } }
        public string ColorError { get { return _colorError; } }

        /// <summary>把深度侧的内参原样转发给上层（否则 SessionController 会退回标称 FOV）。</summary>
        public DepthIntrinsics GetDepthIntrinsics()
        {
            IDepthIntrinsicsProvider p = _depth as IDepthIntrinsicsProvider;
            return p == null ? null : p.GetDepthIntrinsics();
        }

        public void Open()
        {
            _depth.Open();
            if (_color != null && !_opened)
            {
                string err;
                if (!_color.Open("Astra", 640, 480, out err))
                {
                    _colorError = err ?? "";
                    if (_log != null) _log("彩色打开失败（" + _colorError + "），本次只有深度");
                }
                else
                {
                    _opened = true;
                    if (_log != null)
                        _log("UVC 彩色已启动：" + _color.DeviceName + " → " + _color.Width + "x" + _color.Height);
                    _running = true;
                    _thread = new Thread(ColorLoop);
                    _thread.IsBackground = true;
                    _thread.Name = "Acquire UVC color";
                    _thread.Start();
                }
            }
        }

        private void ColorLoop()
        {
            while (_running)
            {
                byte[] bgr = null;
                int w = 0, h = 0;
                try
                {
                    bgr = _color.ReadFrame(80);
                    w = _color.Width;
                    h = _color.Height;
                }
                catch (Exception ex)
                {
                    if (_running && _log != null) _log("彩色取帧异常：" + ex.Message);
                    Thread.Sleep(50);
                    continue;
                }
                if (bgr == null) continue;
                double ms = _clock != null ? _clock() : 0;
                CameraFrame cf = new CameraFrame();
                cf.ColorBgr = bgr;
                cf.ColorWidth = w;
                cf.ColorHeight = h;
                cf.ColorRelMs = ms;
                lock (_gate)
                {
                    while (_colorQueue.Count >= ColorQueueCapacity)
                    {
                        _colorQueue.Dequeue();
                        Interlocked.Increment(ref _colorDropped);
                    }
                    _colorQueue.Enqueue(cf);
                }
                Interlocked.Increment(ref _colorFrames);
            }
        }

        public CameraFrame ReadFrame(int timeoutMs)
        {
            // 彩色优先交出去；没货才去等深度
            lock (_gate)
            {
                if (_colorQueue.Count > 0) return _colorQueue.Dequeue();
            }

            CameraFrame df = _depth.ReadFrame(timeoutMs);
            return df;
        }

        public void Close()
        {
            _running = false;
            Thread t = _thread;
            _thread = null;
            if (t != null)
            {
                try { t.Join(500); } catch { }
            }
            if (_color != null)
            {
                try { _color.Dispose(); } catch { }
            }
            try { _depth.Close(); } catch { }
        }

        public void Dispose()
        {
            try { Close(); } catch { }
            try { _depth.Dispose(); } catch { }
        }
    }
}
