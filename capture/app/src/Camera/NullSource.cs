using System;
using System.Diagnostics;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// 合成相机：彩色 640×480 + 深度 640×480（16-bit，单位 1 mm）。
    /// 用于 --self-test 和"没有相机也想验证全链路"的场景。
    /// 按 30 fps 自走，缓冲复用，避免长时间采集时把大对象堆打爆。
    /// </summary>
    internal sealed class NullSource : ICameraSource, IDepthIntrinsicsProvider
    {
        public const int ColorWidth = 640;
        public const int ColorHeight = 480;
        public const int DepthWidth = 640;
        public const int DepthHeight = 480;
        public const double DefaultFps = 30.0;
        public const double DepthScale = 0.001;

        private readonly Func<double> _clock;
        private readonly Stopwatch _pacer = Stopwatch.StartNew();
        private readonly byte[] _color = new byte[ColorWidth * ColorHeight * 3];
        private readonly ushort[] _depth = new ushort[DepthWidth * DepthHeight];
        private readonly CameraFrame _frame = new CameraFrame();
        private readonly double _intervalMs;
        private readonly DepthIntrinsics _intrinsics;
        private long _frameIndex;
        private double _nextDueMs;
        private bool _opened;

        public NullSource(Func<double> clock, double fps)
        {
            _clock = clock ?? delegate { return 0; };
            _intervalMs = 1000.0 / (fps <= 0 ? DefaultFps : fps);
            // 合成相机的内参不是“标定”也不是“设备上报”，单独标 synthetic，避免跟真机数据混淆
            _intrinsics = DepthIntrinsics.FromFov(DepthWidth, DepthHeight,
                DepthIntrinsics.DegreesToRadians(DepthIntrinsics.AstraNominalHFovDeg),
                DepthIntrinsics.DegreesToRadians(DepthIntrinsics.AstraNominalVFovDeg),
                DepthScale, DepthIntrinsics.SourceSynthetic, "合成相机：按 58°/45° 针孔模型");
            _frame.ColorBgr = _color;
            _frame.ColorWidth = ColorWidth;
            _frame.ColorHeight = ColorHeight;
            _frame.Depth = _depth;
            _frame.DepthWidth = DepthWidth;
            _frame.DepthHeight = DepthHeight;
            _frame.DepthScaleMeters = DepthScale;
            _frame.Note = "synthetic";
        }

        public CameraKind Kind { get { return CameraKind.Null; } }
        public string Model { get { return "NullSource"; } }
        public string Serial { get { return "SYNTH-0001"; } }
        public bool HasColor { get { return true; } }
        public bool HasDepth { get { return true; } }
        public double DepthScaleMeters { get { return DepthScale; } }

        public DepthIntrinsics GetDepthIntrinsics()
        {
            return _intrinsics == null ? null : _intrinsics.Clone();
        }

        public void Open()
        {
            if (_opened) return;
            _opened = true;
            _pacer.Restart();
            _nextDueMs = 0;
        }

        public CameraFrame ReadFrame(int timeoutMs)
        {
            if (!_opened) return null;
            double now = _pacer.Elapsed.TotalMilliseconds;
            if (now < _nextDueMs)
            {
                int sleep = (int)Math.Ceiling(_nextDueMs - now);
                if (sleep > timeoutMs) sleep = timeoutMs;
                if (sleep > 0) Thread.Sleep(sleep);
                now = _pacer.Elapsed.TotalMilliseconds;
                if (now < _nextDueMs - 0.5) return null;
            }
            else if (now > _nextDueMs + _intervalMs)
            {
                // 落后超过一帧时重新对齐，而不是立刻补一帧（否则会出现 14 ms 的假短间隔）
                _nextDueMs = now;
            }

            _frameIndex++;
            Fill(_frameIndex);
            _nextDueMs += _intervalMs;

            double rel = _clock();
            _frame.ColorRelMs = rel;
            _frame.DepthRelMs = rel;
            return _frame;
        }

        private void Fill(long index)
        {
            int phase = (int)(index % ColorWidth);
            for (int y = 0; y < ColorHeight; y++)
            {
                int row = y * ColorWidth * 3;
                byte g = (byte)(y * 255 / ColorHeight);
                for (int x = 0; x < ColorWidth; x++)
                {
                    int p = row + x * 3;
                    _color[p] = (byte)((x + phase) & 0xFF);      // B
                    _color[p + 1] = g;                            // G
                    _color[p + 2] = (byte)((255 - x) & 0xFF);     // R
                }
            }

            // 深度：斜面上放一个移动的 1000 mm 方块，方便肉眼确认帧在更新
            int box = 60 + (int)(index * 4 % (DepthWidth - 120));
            for (int y = 0; y < DepthHeight; y++)
            {
                int row = y * DepthWidth;
                for (int x = 0; x < DepthWidth; x++)
                {
                    int inBox = (x >= box && x < box + 60 && y >= DepthHeight / 3 && y < DepthHeight / 3 + 60) ? 1 : 0;
                    int value = inBox == 1 ? 1000 : 500 + (x * 2500 / DepthWidth) + (y * 500 / DepthHeight);
                    _depth[row + x] = (ushort)value;
                }
            }
        }

        public void Close()
        {
            _opened = false;
        }

        public void Dispose()
        {
            Close();
        }
    }
}
