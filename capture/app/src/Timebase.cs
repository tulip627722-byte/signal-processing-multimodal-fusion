using System;
using System.Diagnostics;

namespace Acquire
{
    /// <summary>
    /// 采集全程唯一的单调时钟。所有三路时间戳都必须经由这里换算。
    /// 基准：Stopwatch（Windows 上映射到 QPC），起点 = 真正开始记录的那一刻。
    /// t_rel_ms 是唯一权威时间轴；t_utc 由 anchorUtc + t_rel 换算而来，
    /// 因为 DateTime.UtcNow 的精度只有 1~15 ms，不能反过来当基准。
    /// </summary>
    internal sealed class Timebase
    {
        private readonly Stopwatch _sw;
        private DateTime _anchorUtc;

        private Timebase()
        {
            _sw = new Stopwatch();
        }

        public static Timebase StartNew()
        {
            Timebase tb = new Timebase();
            tb.Start();
            return tb;
        }

        public void Start()
        {
            if (_sw.IsRunning) return;
            _anchorUtc = DateTime.UtcNow;
            _sw.Start();
        }

        public void Stop()
        {
            _sw.Stop();
        }

        public bool IsRunning { get { return _sw.IsRunning; } }

        public long Frequency { get { return Stopwatch.Frequency; } }

        public long ElapsedTicks { get { return _sw.ElapsedTicks; } }

        /// <summary>相对记录起点的毫秒数（QPC 精度，约 100 ns 的整数倍）。</summary>
        public double ElapsedMs
        {
            get { return _sw.ElapsedTicks * (1000.0 / Stopwatch.Frequency); }
        }

        public double ElapsedSeconds
        {
            get { return _sw.ElapsedTicks / (double)Stopwatch.Frequency; }
        }

        public DateTime AnchorUtc { get { return _anchorUtc; } }

        /// <summary>把相对毫秒换算成 UTC；三路共用同一个映射，保证对齐。</summary>
        public DateTime UtcFromRelMs(double relMs)
        {
            return _anchorUtc.AddTicks((long)Math.Round(relMs * 10000.0, MidpointRounding.AwayFromZero));
        }

        /// <summary>当前 UTC（同样走 anchor + 单调时钟，不直接读系统时钟）。</summary>
        public DateTime UtcNow
        {
            get { return UtcFromRelMs(ElapsedMs); }
        }
    }
}
