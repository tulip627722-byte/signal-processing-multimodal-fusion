using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Acquire
{
    internal sealed class Ac880PointCloudReceiver : IDisposable
    {
        private const uint Magic = 0x31434450;
        private const uint GoodStatus = 0x44344452;
        private const string DefaultSourceIp = "192.168.221.2";
        private readonly LatestPreview<RadarCloudFrame> _preview = new LatestPreview<RadarCloudFrame>();
        private readonly Action<string> _log;
        private readonly object _gate = new object();
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private readonly Dictionary<uint, FrameSlot> _slots = new Dictionary<uint, FrameSlot>();
        private readonly Queue<FrameStamp> _recentFrames = new Queue<FrameStamp>();
        private Socket _socket;
        private Thread _thread;
        private volatile bool _stop;
        private string _status = "未启动";
        private double _lastPeerMs = double.NegativeInfinity;
        private double _lastFrameMs = double.NegativeInfinity;
        private uint? _lastSequence;
        private double _fps;
        private long _frames;
        private long _missingFrames;
        private long _badFrames;
        private long _invalidDatagrams;
        private long _txErrors;

        private sealed class FrameSlot
        {
            public uint Sequence;
            public uint Status;
            public uint BoardTimestamp;
            public uint ClusterCount;
            public int Version;
            public double CreatedMs;
            public RadarCloudPoint[] Points;
            public bool[] Received;
            public int ReceivedCount;
        }

        private struct FrameStamp
        {
            public uint Sequence;
            public uint BoardTimestamp;
        }

        public Ac880PointCloudReceiver(Action<string> log)
        {
            _log = log ?? delegate { };
        }

        public void Start()
        {
            if (_thread != null) return;
            try
            {
                RadarNetwork.ValidateLocalAddress("192.168.221.100");
            }
            catch (Exception ex)
            {
                SetStatus("AC880 点云不可用：本机网卡未配置 192.168.221.100/24");
                _log("AC880 实时点云未启动：" + ex.Message);
                return;
            }
            _stop = false;
            SetStatus("正在监听 AC880 点云 192.168.221.100:5001");
            _thread = new Thread(ReceiveLoop) { IsBackground = true, Name = "Acquire AC880 point cloud" };
            _thread.Start();
        }

        public RadarCloudFrame TakeFrame()
        {
            return _preview.Take();
        }

        public bool PeerOnline
        {
            get { lock (_gate) return _clock.Elapsed.TotalMilliseconds - _lastPeerMs <= 2000.0; }
        }

        public void Snapshot(out bool online, out double fps, out long frames, out long missing,
            out long bad, out string status)
        {
            lock (_gate)
            {
                double now = _clock.Elapsed.TotalMilliseconds;
                online = now - _lastPeerMs <= 2000.0;
                fps = _fps;
                frames = _frames;
                missing = _missingFrames;
                bad = _badFrames;
                status = online && now - _lastFrameMs > 2500.0
                    ? "AC880 已连接，等待完整点云帧"
                    : _status;
            }
        }

        private void ReceiveLoop()
        {
            try
            {
                Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                socket.ReceiveBufferSize = 4 * 1024 * 1024;
                socket.ReceiveTimeout = 500;
                socket.Bind(new IPEndPoint(IPAddress.Parse("192.168.221.100"), 5001));
                _socket = socket;
                _log("AC880 实时点云：监听 UDP 192.168.221.100:5001，只接受 " + DefaultSourceIp);
                byte[] buffer = new byte[2048];
                while (!_stop)
                {
                    try
                    {
                        EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
                        int count = socket.ReceiveFrom(buffer, 0, buffer.Length, SocketFlags.None, ref remote);
                        IPEndPoint peer = remote as IPEndPoint;
                        if (peer == null || peer.Address.ToString() != DefaultSourceIp) continue;
                        ProcessDatagram(buffer, count, _clock.Elapsed.TotalMilliseconds);
                    }
                    catch (SocketException ex)
                    {
                        if (!_stop && ex.SocketErrorCode != SocketError.TimedOut)
                            SetStatus("AC880 点云接收错误：" + ex.Message);
                    }
                    CleanupExpired(_clock.Elapsed.TotalMilliseconds);
                }
            }
            catch (Exception ex)
            {
                if (!_stop)
                {
                    SetStatus("AC880 点云监听失败：" + ex.Message);
                    _log("AC880 实时点云启动失败：" + ex.Message);
                }
            }
            finally
            {
                Socket socket = _socket;
                _socket = null;
                if (socket != null) try { socket.Close(); } catch { }
            }
        }

        private void ProcessDatagram(byte[] bytes, int length, double nowMs)
        {
            if (length < 36 || ReadUInt32(bytes, 0) != Magic) { NoteInvalid(); return; }
            uint versionWord = ReadUInt32(bytes, 4);
            if (versionWord != 1 && versionWord != 2) { NoteInvalid(); return; }
            int version = (int)versionWord;
            int headerBytes = version == 2 ? 40 : 36;
            int pointWords = version == 2 ? 7 : 6;
            int maxPacketPoints = version == 2 ? 50 : 56;
            if (length < headerBytes) { NoteInvalid(); return; }

            uint sequence = ReadUInt32(bytes, 8);
            uint status = ReadUInt32(bytes, 12);
            uint boardTimestamp = ReadUInt32(bytes, 20);
            uint totalWord = ReadUInt32(bytes, 24);
            uint offsetWord = ReadUInt32(bytes, 28);
            uint countWord = ReadUInt32(bytes, 32);
            uint clusterCount = version == 2 ? ReadUInt32(bytes, 36) : 0;
            lock (_gate) _lastPeerMs = nowMs;

            if (sequence == uint.MaxValue)
            {
                lock (_gate)
                {
                    _txErrors = status;
                    _status = "AC880 心跳正常，等待雷达点云";
                }
                return;
            }

            if (totalWord > 200 || countWord > maxPacketPoints || offsetWord + countWord > totalWord ||
                clusterCount > totalWord || length != headerBytes + (int)countWord * pointWords * 4 ||
                (totalWord > 0 && countWord == 0))
            {
                NoteInvalid();
                return;
            }

            int total = (int)totalWord;
            int offset = (int)offsetWord;
            int count = (int)countWord;
            FrameSlot slot;
            lock (_slots)
            {
                if (!_slots.TryGetValue(sequence, out slot))
                {
                    slot = new FrameSlot
                    {
                        Sequence = sequence, Status = status, BoardTimestamp = boardTimestamp,
                        ClusterCount = clusterCount, Version = version, CreatedMs = nowMs,
                        Points = new RadarCloudPoint[total], Received = new bool[total]
                    };
                    _slots.Add(sequence, slot);
                }
                if (slot.Points.Length != total || slot.Status != status || slot.BoardTimestamp != boardTimestamp ||
                    slot.Version != version || slot.ClusterCount != clusterCount)
                {
                    _slots.Remove(sequence);
                    NoteInvalid();
                    return;
                }

                int payload = headerBytes;
                for (int index = 0; index < count; index++)
                {
                    int row = offset + index;
                    RadarCloudPoint point = new RadarCloudPoint
                    {
                        X = ReadSingle(bytes, payload), Y = ReadSingle(bytes, payload + 4),
                        Z = ReadSingle(bytes, payload + 8), MagnitudeDb = ReadSingle(bytes, payload + 12),
                        RangeMeters = ReadSingle(bytes, payload + 16),
                        VelocityMetersPerSecond = ReadSingle(bytes, payload + 20),
                        Cluster = version == 2 ? ReadUInt32(bytes, payload + 24) : 0
                    };
                    if (!IsFinite(point.X) || !IsFinite(point.Y) || !IsFinite(point.Z) ||
                        !IsFinite(point.MagnitudeDb) || !IsFinite(point.RangeMeters) ||
                        !IsFinite(point.VelocityMetersPerSecond) || point.Cluster > clusterCount)
                    {
                        _slots.Remove(sequence);
                        NoteInvalid();
                        return;
                    }
                    slot.Points[row] = point;
                    if (!slot.Received[row]) { slot.Received[row] = true; slot.ReceivedCount++; }
                    payload += pointWords * 4;
                }
                if (slot.ReceivedCount != total) return;
                _slots.Remove(sequence);
            }
            Publish(slot, nowMs);
        }

        private void Publish(FrameSlot slot, double nowMs)
        {
            lock (_gate)
            {
                if (_lastSequence.HasValue)
                {
                    uint delta = unchecked(slot.Sequence - _lastSequence.Value);
                    if (delta == 0 || delta > 0x80000000U) return;
                    if (delta > 1) _missingFrames += delta - 1;
                }
                _lastSequence = slot.Sequence;
                _lastFrameMs = nowMs;
                _frames++;
                if (slot.Status != GoodStatus) _badFrames++;
                _status = "AC880 实时点云" + (slot.Version == 2 ? "，DBSCAN " + slot.ClusterCount + " 簇" : "") +
                    (_txErrors > 0 ? "，发送错误 " + _txErrors : "");
                _recentFrames.Enqueue(new FrameStamp { Sequence = slot.Sequence, BoardTimestamp = slot.BoardTimestamp });
                while (_recentFrames.Count > 46) _recentFrames.Dequeue();
                if (_recentFrames.Count >= 2)
                {
                    FrameStamp first = _recentFrames.Peek();
                    uint elapsedUs = unchecked(slot.BoardTimestamp - first.BoardTimestamp);
                    uint frameDelta = unchecked(slot.Sequence - first.Sequence);
                    if (elapsedUs > 0 && frameDelta < 0x80000000U) _fps = frameDelta * 1000000.0 / elapsedUs;
                }
            }
            _preview.Publish(new RadarCloudFrame
            {
                Sequence = slot.Sequence, Status = slot.Status,
                BoardTimestampMicroseconds = slot.BoardTimestamp, ClusterCount = slot.ClusterCount,
                Version = slot.Version, Points = slot.Points, Count = slot.Points.Length, ReceivedAtMs = nowMs
            });
        }

        private void CleanupExpired(double nowMs)
        {
            lock (_slots)
            {
                if (_slots.Count == 0) return;
                List<uint> expired = null;
                foreach (KeyValuePair<uint, FrameSlot> pair in _slots)
                {
                    if (nowMs - pair.Value.CreatedMs <= 350.0) continue;
                    if (expired == null) expired = new List<uint>();
                    expired.Add(pair.Key);
                }
                if (expired == null) return;
                foreach (uint sequence in expired) _slots.Remove(sequence);
            }
        }

        private void NoteInvalid() { lock (_gate) _invalidDatagrams++; }
        private void SetStatus(string status) { lock (_gate) _status = status; }

        private static uint ReadUInt32(byte[] bytes, int offset)
        {
            return (uint)(bytes[offset] | bytes[offset + 1] << 8 | bytes[offset + 2] << 16 | bytes[offset + 3] << 24);
        }

        private static float ReadSingle(byte[] bytes, int offset) { return BitConverter.ToSingle(bytes, offset); }
        private static bool IsFinite(float value) { return !float.IsNaN(value) && !float.IsInfinity(value); }

        public void Dispose()
        {
            _stop = true;
            Socket socket = _socket;
            if (socket != null) try { socket.Close(); } catch { }
            Thread thread = _thread;
            if (thread != null && thread.IsAlive) try { thread.Join(2000); } catch { }
            _thread = null;
        }
    }
}
