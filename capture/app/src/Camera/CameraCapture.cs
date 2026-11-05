using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace Acquire
{
    internal sealed class CameraCaptureStats
    {
        public long ColorFrames;
        public long DepthFrames;
        public double ColorFps;
        public double DepthFps;
        public double FirstColorRelMs = double.NaN;
        public double LastColorRelMs = double.NaN;
        public double FirstDepthRelMs = double.NaN;
        public double LastDepthRelMs = double.NaN;
        public long ColorBytes;
        public long DepthBytes;
        public int ColorWidth;
        public int ColorHeight;
        public int DepthWidth;
        public int DepthHeight;
        public double DepthScaleMeters;
        public long DroppedFrames;
        public long CloudFrames;
        public long CloudPoints;
        public string LastError;
    }

    /// <summary>
    /// 相机落盘：取帧线程只做"取 + 拷贝 + 打时间戳"，写图线程单独做 PNG 编码。
    /// 这样 GDI+ 编码再慢也不会把取帧节奏拖歪（早期版本曾出现 394 ms 的帧间隔毛刺）。
    /// 队列满时丢弃新帧并计数，绝不阻塞采集。
    /// </summary>
    internal sealed class CameraCapture : IDisposable
    {
        private const int QueueCapacity = 48;
        private const int WriterStopTimeoutMs = 20000;

        private sealed class Pending
        {
            public byte[] Color;
            public int ColorWidth;
            public int ColorHeight;
            public ushort[] Depth;
            public int DepthWidth;
            public int DepthHeight;
            public double DepthScale;
            public double ColorRelMs;
            public double DepthRelMs;
            public bool HasColor;
            public bool HasDepth;
            public int ColorIndex;
            public int DepthIndex;
            public bool Queued;
        }

        private readonly object _statsGate = new object();
        private readonly object _queueGate = new object();
        private readonly CameraCaptureStats _stats = new CameraCaptureStats();
        private readonly ICameraSource _source;
        private readonly TakeWriter _take;
        private readonly Action<string> _log;
        private readonly DepthIntrinsics _intrinsics;
        private readonly bool _saveCloud;
        private readonly int _cloudStride;
        private readonly HashSet<string> _warned = new HashSet<string>();
        private readonly List<Pending> _slots = new List<Pending>();
        private readonly Queue<Pending> _queue = new Queue<Pending>();
        private Thread _reader;
        private Thread[] _writers;
        private volatile bool _stop;
        private volatile bool _running;
        private int _colorIndex;
        private int _depthIndex;
        private long _dropped;
        private bool _warnedBacklog;

        public CameraCapture(ICameraSource source, TakeWriter take, Action<string> log)
            : this(source, take, log, null, false, DepthCloud.DefaultStride)
        {
        }

        public CameraCapture(ICameraSource source, TakeWriter take, Action<string> log,
                             DepthIntrinsics intrinsics, bool saveCloud, int cloudStride)
        {
            _source = source;
            _take = take;
            _log = log ?? delegate { };
            _intrinsics = intrinsics;
            _saveCloud = saveCloud && intrinsics != null && intrinsics.Valid;
            _cloudStride = cloudStride < 1 ? DepthCloud.DefaultStride : cloudStride;
            for (int i = 0; i < QueueCapacity; i++) _slots.Add(new Pending());
        }

        public void Start()
        {
            if (_running) return;
            _stop = false;
            _running = true;
            int workers = Math.Max(2, Math.Min(6, Environment.ProcessorCount - 1));
            _writers = new Thread[workers];
            for (int i = 0; i < workers; i++)
            {
                _writers[i] = new Thread(WriterLoop);
                _writers[i].Name = "Acquire camera writer " + (i + 1);
                _writers[i].IsBackground = true;
                // PNG 压缩很吃 CPU，6 个写手会挤压读帧线程；降低写手优先级
                try { _writers[i].Priority = ThreadPriority.BelowNormal; } catch { }
                _writers[i].Start();
            }
            _reader = new Thread(ReaderLoop);
            _reader.Name = "Acquire camera reader";
            _reader.IsBackground = true;
            // 读帧线程一旦晚 33 ms，设备/驱动那一帧就真丢了，所以给它更高优先级
            try { _reader.Priority = ThreadPriority.AboveNormal; } catch { }
            _reader.Start();
        }

        public void Stop()
        {
            _stop = true;
            Thread reader = _reader;
            if (reader != null && reader.IsAlive) reader.Join(3000);
            lock (_queueGate) Monitor.PulseAll(_queueGate);
            Thread[] writers = _writers;
            if (writers != null)
            {
                foreach (Thread w in writers)
                {
                    if (w != null && w.IsAlive && !w.Join(WriterStopTimeoutMs))
                    {
                        _log("警告：写图线程未在 " + (WriterStopTimeoutMs / 1000) + " 秒内排空");
                        break;
                    }
                }
            }
            _running = false;
            _reader = null;
            _writers = null;
        }

        public CameraCaptureStats Snapshot()
        {
            lock (_statsGate)
            {
                CameraCaptureStats copy = new CameraCaptureStats
                {
                    ColorFrames = _stats.ColorFrames,
                    DepthFrames = _stats.DepthFrames,
                    FirstColorRelMs = _stats.FirstColorRelMs,
                    LastColorRelMs = _stats.LastColorRelMs,
                    FirstDepthRelMs = _stats.FirstDepthRelMs,
                    LastDepthRelMs = _stats.LastDepthRelMs,
                    ColorBytes = _stats.ColorBytes,
                    DepthBytes = _stats.DepthBytes,
                    ColorWidth = _stats.ColorWidth,
                    ColorHeight = _stats.ColorHeight,
                    DepthWidth = _stats.DepthWidth,
                    DepthHeight = _stats.DepthHeight,
                    DepthScaleMeters = _stats.DepthScaleMeters,
                    DroppedFrames = Interlocked.Read(ref _dropped),
                    CloudFrames = _stats.CloudFrames,
                    CloudPoints = _stats.CloudPoints,
                    LastError = _stats.LastError
                };
                copy.ColorFps = Fps(_stats.ColorFrames, _stats.FirstColorRelMs, _stats.LastColorRelMs);
                copy.DepthFps = Fps(_stats.DepthFrames, _stats.FirstDepthRelMs, _stats.LastDepthRelMs);
                return copy;
            }
        }

        private static double Fps(long frames, double first, double last)
        {
            if (frames < 2 || double.IsNaN(first) || double.IsNaN(last)) return 0.0;
            double span = (last - first) / 1000.0;
            if (span <= 0) return 0.0;
            return (frames - 1) / span;
        }

        private void ReaderLoop()
        {
            while (!_stop)
            {
                CameraFrame frame;
                try
                {
                    frame = _source.ReadFrame(100);
                }
                catch (Exception ex)
                {
                    RecordError("取帧失败：" + DynHelper.DescribeException(ex));
                    Thread.Sleep(100);
                    continue;
                }
                if (frame == null) continue;

                Pending slot = null;
                lock (_queueGate)
                {
                    for (int i = 0; i < _slots.Count; i++)
                    {
                        if (!_slots[i].Queued) { slot = _slots[i]; break; }
                    }
                }
                if (slot == null)
                {
                    Interlocked.Increment(ref _dropped);
                    if (!_warnedBacklog)
                    {
                        _warnedBacklog = true;
                        _log("警告：写图队列已满，开始丢帧（磁盘或 PNG 编码跟不上）");
                    }
                    continue;
                }

                try
                {
                    Fill(slot, frame);
                }
                catch (Exception ex)
                {
                    RecordError("拷贝帧失败：" + ex.Message);
                    continue;
                }

                lock (_queueGate)
                {
                    slot.Queued = true;
                    _queue.Enqueue(slot);
                    Monitor.PulseAll(_queueGate);
                }
            }
            lock (_queueGate) Monitor.PulseAll(_queueGate);
        }

        private void Fill(Pending slot, CameraFrame frame)
        {
            slot.HasColor = false;
            slot.HasDepth = false;

            if (frame.HasColor)
            {
                int need = frame.ColorWidth * frame.ColorHeight * 3;
                if (slot.Color == null || slot.Color.Length != need) slot.Color = new byte[need];
                Buffer.BlockCopy(frame.ColorBgr, 0, slot.Color, 0, need);
                slot.ColorWidth = frame.ColorWidth;
                slot.ColorHeight = frame.ColorHeight;
                slot.ColorRelMs = frame.ColorRelMs;
                slot.HasColor = true;
                slot.ColorIndex = Interlocked.Increment(ref _colorIndex);
            }

            if (frame.HasDepth)
            {
                int need = frame.DepthWidth * frame.DepthHeight;
                if (slot.Depth == null || slot.Depth.Length != need) slot.Depth = new ushort[need];
                Array.Copy(frame.Depth, slot.Depth, need);
                slot.DepthWidth = frame.DepthWidth;
                slot.DepthHeight = frame.DepthHeight;
                slot.DepthRelMs = frame.DepthRelMs;
                slot.DepthScale = frame.DepthScaleMeters;
                slot.HasDepth = true;
                slot.DepthIndex = Interlocked.Increment(ref _depthIndex);
            }
        }

        private void WriterLoop()
        {
            while (true)
            {
                Pending slot;
                lock (_queueGate)
                {
                    while (_queue.Count == 0 && !_stop) Monitor.Wait(_queueGate, 100);
                    if (_queue.Count == 0)
                    {
                        if (_stop) return;
                        continue;
                    }
                    slot = _queue.Dequeue();
                }

                try
                {
                    if (slot.HasColor) WriteColor(slot);
                    if (slot.HasDepth) WriteDepth(slot);
                }
                catch (Exception ex)
                {
                    RecordError("写图失败：" + ex.Message);
                }
                finally
                {
                    lock (_queueGate)
                    {
                        slot.Queued = false;
                        Monitor.PulseAll(_queueGate);
                    }
                }
            }
        }

        private void WriteColor(Pending slot)
        {
            string name = slot.ColorIndex.ToString("D6") + ".png";
            string path = Path.Combine(_take.ColorDirectory, name);
            SaveColorReuse(path, slot.Color, slot.ColorWidth, slot.ColorHeight);
            double rel = slot.ColorRelMs;
            long bytes = new FileInfo(path).Length;

            lock (_statsGate)
            {
                if (double.IsNaN(_stats.FirstColorRelMs)) _stats.FirstColorRelMs = rel;
                _stats.LastColorRelMs = rel;
                _stats.ColorFrames++;
                _stats.ColorBytes += bytes;
                _stats.ColorWidth = slot.ColorWidth;
                _stats.ColorHeight = slot.ColorHeight;
            }
            _take.AddRow("color", slot.ColorIndex, rel, "color/" + name);
        }

        private void WriteDepth(Pending slot)
        {
            string name = slot.DepthIndex.ToString("D6") + ".png";
            string path = Path.Combine(_take.DepthDirectory, name);
            Png16.Write(path, slot.Depth, slot.DepthWidth, slot.DepthHeight);
            double rel = slot.DepthRelMs;
            long bytes = new FileInfo(path).Length;

            lock (_statsGate)
            {
                if (double.IsNaN(_stats.FirstDepthRelMs)) _stats.FirstDepthRelMs = rel;
                _stats.LastDepthRelMs = rel;
                _stats.DepthFrames++;
                _stats.DepthBytes += bytes;
                _stats.DepthWidth = slot.DepthWidth;
                _stats.DepthHeight = slot.DepthHeight;
                _stats.DepthScaleMeters = slot.DepthScale;
            }
            _take.AddRow("depth", slot.DepthIndex, rel, "depth/" + name);
            if (_saveCloud) WriteCloud(slot);
        }

        /// <summary>
        /// 采集时可选：按 stride 在像素网格上取点，写 cloud\NNNNNN.ply（与深度帧同号）。
        /// 点云只是派生数据，原始深度图无论如何都会写；这里失败不影响采集，只警告一次。
        /// </summary>
        private void WriteCloud(Pending slot)
        {
            DepthIntrinsics k = _intrinsics;
            if (k == null || !k.Valid)
            {
                WarnOnce("没有可用的深度内参，本次不写点云");
                return;
            }
            if (k.Width != slot.DepthWidth || k.Height != slot.DepthHeight)
            {
                WarnOnce("深度图 " + slot.DepthWidth + "x" + slot.DepthHeight + " 与内参 " +
                         k.Width + "x" + k.Height + " 不符，本次不写点云");
                return;
            }

            int sampled, valid;
            List<CloudPoint> points;
            try
            {
                points = DepthCloud.Build(slot.Depth, slot.DepthWidth, slot.DepthHeight, k, _cloudStride,
                    false, out sampled, out valid);
            }
            catch (Exception ex)
            {
                WarnOnce("点云计算失败：" + ex.Message);
                return;
            }

            string name = slot.DepthIndex.ToString("D6") + ".ply";
            string path = Path.Combine(_take.CloudDirectory, name);
            try
            {
                _take.EnsureCloudDirectory();
                DepthCloud.WritePly(path, points, slot.DepthIndex.ToString("D6"),
                    "capture stride=" + _cloudStride + " intrinsics=" + k.Source);
            }
            catch (Exception ex)
            {
                WarnOnce("点云写盘失败：" + ex.Message);
                return;
            }

            lock (_statsGate)
            {
                _stats.CloudFrames++;
                _stats.CloudPoints += points.Count;
            }
            _take.AddRow("cloud", slot.DepthIndex, slot.DepthRelMs, "cloud/" + name);
        }

        private void WarnOnce(string message)
        {
            lock (_statsGate)
            {
                if (_warned.Contains(message)) return;
                _warned.Add(message);
            }
            _log("警告：" + message);
        }

        private void RecordError(string message)
        {
            lock (_statsGate)
            {
                _stats.LastError = message;
                if (_stats.ColorFrames == 0 && _stats.DepthFrames == 0) _log(message);
            }
        }

        internal static void SaveColorPng(string path, byte[] bgr, int width, int height)
        {
            using (Bitmap bitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb))
            {
                Rectangle rect = new Rectangle(0, 0, width, height);
                BitmapData data = bitmap.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
                try
                {
                    int rowBytes = width * 3;
                    if (data.Stride == rowBytes)
                    {
                        Marshal.Copy(bgr, 0, data.Scan0, rowBytes * height);
                    }
                    else
                    {
                        for (int y = 0; y < height; y++)
                        {
                            Marshal.Copy(bgr, y * rowBytes, IntPtr.Add(data.Scan0, y * data.Stride), rowBytes);
                        }
                    }
                }
                finally
                {
                    bitmap.UnlockBits(data);
                }
                bitmap.Save(path, ImageFormat.Png);
            }
        }

        // 每个写图线程各用一张 Bitmap：共享一张会报“对象当前正在其他地方使用”（GDI+ 线程安全）
        [ThreadStatic] private static Bitmap _tlBitmap;
        [ThreadStatic] private static int _tlWidth;
        [ThreadStatic] private static int _tlHeight;

        /// <summary>写图线程专用：每个线程复用自己那张 Bitmap，避免 30 fps × 900 KB 的大对象堆垃圾。</summary>
        private void SaveColorReuse(string path, byte[] bgr, int width, int height)
        {
            if (_tlBitmap == null || _tlWidth != width || _tlHeight != height)
            {
                if (_tlBitmap != null) _tlBitmap.Dispose();
                _tlBitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
                _tlWidth = width;
                _tlHeight = height;
            }
            BitmapData data = _tlBitmap.LockBits(new Rectangle(0, 0, width, height),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            try
            {
                int rowBytes = width * 3;
                if (data.Stride == rowBytes)
                {
                    Marshal.Copy(bgr, 0, data.Scan0, rowBytes * height);
                }
                else
                {
                    for (int y = 0; y < height; y++)
                        Marshal.Copy(bgr, y * rowBytes, IntPtr.Add(data.Scan0, y * data.Stride), rowBytes);
                }
            }
            finally
            {
                _tlBitmap.UnlockBits(data);
            }
            _tlBitmap.Save(path, ImageFormat.Png);
        }

        public void Dispose()
        {
            Stop();
            if (_tlBitmap != null)
            {
                try { _tlBitmap.Dispose(); } catch { }
                _tlBitmap = null;
            }
            try { _source.Dispose(); } catch { }
        }
    }
}
