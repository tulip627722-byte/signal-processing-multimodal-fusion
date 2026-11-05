using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// 自检用合成雷达：走真实 UDP 回环，包格式与真机一致
    /// （TinyFrame 头 + 2048 字节 payload + 校验字节，id 全局递增）。
    /// 可以按概率丢包，用来验证丢包统计链路。
    /// </summary>
    internal sealed class SyntheticRadar : IDisposable
    {
        private readonly IPEndPoint _target;
        private readonly int _packetsPerFrame;
        private readonly int _payloadBytes;
        private readonly double _fps;
        private readonly double _lossRate;
        private readonly double _dropFrameRate;
        private readonly int _maxFrames;
        private readonly Random _random = new Random(20261026);
        private Thread _thread;
        private volatile bool _running;
        private long _sent;
        private long _dropped;
        private bool _disposed;

        public SyntheticRadar(string ip, int port, int packetsPerFrame, int payloadBytes, double fps, double lossRate, int maxFrames)
            : this(ip, port, packetsPerFrame, payloadBytes, fps, lossRate, 0, maxFrames)
        {
        }

        public SyntheticRadar(string ip, int port, int packetsPerFrame, int payloadBytes, double fps, double lossRate, double dropFrameRate, int maxFrames)
        {
            _target = new IPEndPoint(IPAddress.Parse(ip), port);
            _packetsPerFrame = packetsPerFrame;
            _payloadBytes = payloadBytes;
            _fps = fps <= 0 ? 1 : fps;
            _lossRate = Math.Max(0, Math.Min(0.5, lossRate));
            _dropFrameRate = Math.Max(0, Math.Min(0.9, dropFrameRate));
            _maxFrames = maxFrames;
        }

        public long Sent { get { return Interlocked.Read(ref _sent); } }
        public long Dropped { get { return Interlocked.Read(ref _dropped); } }

        public static byte PayloadByte(uint id, int index)
        {
            return (byte)((id * 31 + index * 7) & 0xFF);
        }

        public void Start()
        {
            if (_running) return;
            _running = true;
            _thread = new Thread(Loop);
            _thread.Name = "Acquire synthetic radar";
            _thread.IsBackground = true;
            _thread.Start();
        }

        private void Loop()
        {
            UdpClient client = new UdpClient();
            byte[] payload = new byte[_payloadBytes];
            Stopwatch clock = Stopwatch.StartNew();
            uint id = 0;
            try
            {
                int frame = 0;
                double period = 1000.0 / _fps;
                double nextFrameStart = clock.Elapsed.TotalMilliseconds;
                while (_running && (_maxFrames <= 0 || frame < _maxFrames))
                {
                    double now = clock.Elapsed.TotalMilliseconds;
                    if (now < nextFrameStart)
                    {
                        int sleep = (int)Math.Ceiling(nextFrameStart - now);
                        if (sleep > 0) Thread.Sleep(sleep);
                    }
                    else if (now > nextFrameStart + period)
                    {
                        // 落后超过一整帧时重新对齐，避免追赶式突发
                        nextFrameStart = now;
                    }

                    bool dropWholeFrame = _dropFrameRate > 0 && _random.NextDouble() < _dropFrameRate;
                    double frameStart = clock.Elapsed.TotalMilliseconds;
                    for (int i = 0; i < _packetsPerFrame; i++)
                    {
                        if (!_running) break;
                        uint packetId = id;
                        id++;
                        if (dropWholeFrame)
                        {
                            Interlocked.Increment(ref _dropped);
                            continue;
                        }
                        if (_lossRate > 0 && _random.NextDouble() < _lossRate)
                        {
                            Interlocked.Increment(ref _dropped);
                            continue;
                        }
                        for (int k = 0; k < payload.Length; k++) payload[k] = PayloadByte(packetId, k);
                        byte[] datagram = TinyFrameCodec.BuildAdcPacket(packetId, payload);
                        client.Send(datagram, datagram.Length, _target);
                        Interlocked.Increment(ref _sent);

                        // 把 4 MB 分散到帧周期的 60%，模拟固件真实发包节奏，避免瞬时突发
                        if ((i & 63) == 63)
                        {
                            double target = frameStart + period * 0.6 * ((i + 1) / (double)_packetsPerFrame);
                            double t = clock.Elapsed.TotalMilliseconds;
                            if (t < target)
                            {
                                int sleep = (int)Math.Ceiling(target - t);
                                if (sleep > 0) Thread.Sleep(sleep);
                            }
                        }
                    }
                    nextFrameStart = frameStart + period;
                    frame++;
                }
            }
            catch (Exception ex)
            {
                ConsoleIo.WriteLine("合成雷达发送异常：" + ex.Message);
            }
            finally
            {
                try { client.Close(); } catch { }
            }
        }

        public void Stop()
        {
            _running = false;
            Thread thread = _thread;
            if (thread != null && thread.IsAlive) thread.Join(2000);
            _thread = null;
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            Stop();
        }
    }
}
