using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// UDP 收包 + 源地址过滤（复制自 2.0\gui\src\RadarTransport.cs）。
    /// 唯一改动：在 ReceiveFrom 返回后立刻打时间戳（统一时钟 + UTC + 本地时间），
    /// 通过 PacketReceived(packet, stamp) 一起抛出，保证 t_rel_ms 取的是"首包到达"时刻。
    /// 串口/发命令部分本工程用不到，已裁剪。
    /// </summary>
    internal sealed class RadarTransport : IDisposable
    {
        public event Action<string> Log;
        public event Action<TinyFramePacket, RadarStamp> PacketReceived;
        public event Action<bool> UdpStateChanged;

        /// <summary>由 RadarSink 注入：返回相对记录起点的毫秒数。</summary>
        public Func<double> RelativeClock;

        private Socket _udpSocket;
        private Thread _udpThread;
        private volatile bool _udpRunning;
        private string _boardIp;

        private long _rawDatagrams;
        private long _droppedBySourceIp;
        private long _parseFailures;
        private long _otherTypeCount;

        public bool IsUdpOpen { get { return _udpRunning; } }
        public long RawDatagrams { get { return Interlocked.Read(ref _rawDatagrams); } }
        public long DroppedBySourceIp { get { return Interlocked.Read(ref _droppedBySourceIp); } }
        public long ParseFailures { get { return Interlocked.Read(ref _parseFailures); } }
        public long OtherTypeCount { get { return Interlocked.Read(ref _otherTypeCount); } }

        public void NoteOtherType(ushort type)
        {
            long n = Interlocked.Increment(ref _otherTypeCount);
            if (n <= 5 || n % 500 == 0)
                EmitLog("收到非 ADC 帧 type=0x" + type.ToString("X4") + "，累计 " + n + " 个");
        }

        public void OpenUdp(string localIp, int localPort, string boardIp)
        {
            CloseUdp();
            IPAddress bindAddress = string.IsNullOrEmpty(localIp) ? IPAddress.Any : IPAddress.Parse(localIp);
            _boardIp = boardIp;
            Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.ReceiveBufferSize = 64 * 1024 * 1024;
            socket.Bind(new IPEndPoint(bindAddress, localPort));
            socket.ReceiveTimeout = 500;
            _udpSocket = socket;
            _udpRunning = true;
            _udpThread = new Thread(UdpLoop);
            _udpThread.Name = "Acquire UDP receiver";
            _udpThread.IsBackground = true;
            _udpThread.Priority = ThreadPriority.AboveNormal;
            _udpThread.Start();
            EmitLog("UDP 已绑定 " + bindAddress + ":" + localPort +
                    (string.IsNullOrEmpty(boardIp) ? "（不过滤源地址）" : "，只接受来自 " + boardIp + " 的包"));
            if (UdpStateChanged != null) UdpStateChanged(true);
        }

        public void CloseUdp()
        {
            _udpRunning = false;
            Socket socket = _udpSocket;
            _udpSocket = null;
            if (socket != null)
            {
                try { socket.Close(); } catch { }
            }
            if (_udpThread != null && _udpThread.IsAlive)
            {
                _udpThread.Join(1000);
            }
            _udpThread = null;
            if (UdpStateChanged != null) UdpStateChanged(false);
        }

        private void UdpLoop()
        {
            byte[] buffer = new byte[65535];
            while (_udpRunning)
            {
                try
                {
                    EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
                    int count = _udpSocket.ReceiveFrom(buffer, 0, buffer.Length, SocketFlags.None, ref remote);
                    IPEndPoint endpoint = remote as IPEndPoint;

                    // 时间戳尽量贴近"到达"：先读时钟，再做任何解析/判断。
                    RadarStamp stamp;
                    stamp.RelMs = RelativeClock != null ? RelativeClock() : 0;
                    stamp.LocalTime = DateTime.Now;
                    stamp.UtcTime = DateTime.UtcNow;

                    long raw = Interlocked.Increment(ref _rawDatagrams);
                    if (raw == 1)
                    {
                        EmitLog("收到首个 UDP 数据报：来自 " + (endpoint == null ? "?" : endpoint.Address + ":" + endpoint.Port) +
                                "，长度 " + count + (string.IsNullOrEmpty(_boardIp) ? "" : "，期望来源 " + _boardIp));
                    }

                    if (endpoint != null && !string.IsNullOrEmpty(_boardIp) && endpoint.Address.ToString() != _boardIp)
                    {
                        long dropped = Interlocked.Increment(ref _droppedBySourceIp);
                        if (dropped <= 3 || dropped % 2000 == 0)
                        {
                            EmitLog("UDP 源地址不匹配已丢弃 " + dropped + " 个：实际 " + endpoint.Address +
                                    "，而雷达地址填的是 " + _boardIp + "（清空该字段可关闭过滤）");
                        }
                        continue;
                    }

                    TinyFramePacket packet;
                    string error;
                    if (TinyFrameCodec.TryParse(buffer, count, out packet, out error))
                    {
                        if (PacketReceived != null) PacketReceived(packet, stamp);
                    }
                    else
                    {
                        long failed = Interlocked.Increment(ref _parseFailures);
                        if (failed <= 5 || failed % 2000 == 0)
                        {
                            EmitLog("UDP 丢弃非法帧（第 " + failed + " 个）：" + error + "，长度=" + count);
                        }
                    }
                }
                catch (SocketException ex)
                {
                    if (_udpRunning && ex.SocketErrorCode != SocketError.TimedOut)
                    {
                        EmitLog("UDP 接收错误：" + ex.Message);
                    }
                }
                catch (ObjectDisposedException) { }
                catch (Exception ex)
                {
                    if (_udpRunning) EmitLog("UDP 接收异常：" + ex.Message);
                }
            }
        }

        private void EmitLog(string text)
        {
            if (Log != null) Log(DateTime.Now.ToString("HH:mm:ss.fff") + " " + text);
        }

        public void Dispose()
        {
            CloseUdp();
        }
    }
}
