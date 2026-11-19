using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Acquire
{
    internal sealed class IdleRadarPreview : IDisposable
    {
        private readonly LatestPreview<RadarCloudFrame> _preview = new LatestPreview<RadarCloudFrame>();
        private readonly Action<string> _log;
        private readonly object _gate = new object();
        private RadarTransport _transport;
        private SerialControl _serial;
        private RadarPointCloudProcessor _processor;
        private Stopwatch _clock;
        private string _status = "未启动";
        private long _packets;
        private int _startFrame = -1;
        private int _lastFrame = -1;
        private double _firstPacketMs = double.NaN;
        private double _lastPacketMs = double.NaN;

        public IdleRadarPreview(Action<string> log)
        {
            _log = log ?? delegate { };
        }

        public void Start(string localIp, int localPort, string boardIp)
        {
            Stop();
            try
            {
                localIp = RadarNetwork.ValidateLocalAddress(localIp);
                _clock = Stopwatch.StartNew();
                _processor = new RadarPointCloudProcessor(_preview);
                _transport = new RadarTransport();
                _transport.RelativeClock = delegate { return _clock.Elapsed.TotalMilliseconds; };
                _transport.Log += _log;
                _transport.PacketReceived += HandlePacket;
                _transport.OpenUdp(localIp, localPort, boardIp);
                OpenSerialAndStart();
                SetStatus("雷达空闲预览启动中（不保存）");
            }
            catch (Exception ex)
            {
                SetStatus("雷达空闲预览失败：" + DynHelper.DescribeException(ex));
                _log(_status);
                StopResources(false);
            }
        }

        private void OpenSerialAndStart()
        {
            List<PortCandidate> candidates = SerialControl.AutoCandidates();
            if (candidates.Count == 0) throw new InvalidOperationException("未发现雷达控制串口");
            foreach (PortCandidate candidate in candidates)
            {
                SerialControl serial = new SerialControl(_log);
                string error;
                if (!serial.TryOpen(candidate.Name, 115200, out error))
                {
                    serial.Dispose();
                    continue;
                }
                if (!serial.SendStart())
                {
                    serial.Dispose();
                    continue;
                }
                _serial = serial;
                _log("毫米波空闲预览：已从 " + candidate.Name + " 发送 Start D000（只预览，不保存）");
                return;
            }
            throw new InvalidOperationException("雷达控制串口均无法打开或发送 Start");
        }

        private void HandlePacket(TinyFramePacket packet, RadarStamp stamp)
        {
            if (packet.Type != TinyFrameCodec.TypeAdc || packet.Payload == null ||
                packet.Payload.Length == 0 || packet.Payload.Length % 2048 != 0) return;
            int chirps = packet.Payload.Length / 2048;
            long logicalBase = (long)packet.Id * chirps;
            int frameIndex = (int)(logicalBase / 2048);
            if (_startFrame < 0) _startFrame = frameIndex;
            _lastFrame = frameIndex;
            if (double.IsNaN(_firstPacketMs)) _firstPacketMs = stamp.RelMs;
            _lastPacketMs = stamp.RelMs;
            Interlocked.Add(ref _packets, chirps);
            RadarPointCloudProcessor processor = _processor;
            if (processor == null) return;
            for (int chirp = 0; chirp < chirps; chirp++)
                processor.AddChirp(frameIndex, (int)((logicalBase + chirp) % 2048), packet.Payload,
                    chirp * 2048, 2048, stamp.RelMs);
            SetStatus("ADC 直连 PC 实时点云（不保存）");
        }

        public RadarCloudFrame TakeFrame()
        {
            return _preview.Take();
        }

        public void Snapshot(out double fps, out long packets, out string status)
        {
            lock (_gate)
            {
                double span = (_lastPacketMs - _firstPacketMs) / 1000.0;
                fps = span > 0 && _lastFrame > _startFrame ? (_lastFrame - _startFrame) / span : 0;
                packets = Interlocked.Read(ref _packets);
                status = _status;
            }
        }

        public void Stop()
        {
            StopResources(true);
            while (_preview.Take() != null) { }
        }

        private void StopResources(bool sendPause)
        {
            SerialControl serial = _serial;
            _serial = null;
            if (serial != null)
            {
                try { if (sendPause) serial.SendPause(); } catch { }
                try { serial.Dispose(); } catch { }
            }
            RadarTransport transport = _transport;
            _transport = null;
            if (transport != null) try { transport.Dispose(); } catch { }
            RadarPointCloudProcessor processor = _processor;
            _processor = null;
            if (processor != null) try { processor.Dispose(); } catch { }
            lock (_gate)
            {
                _packets = 0;
                _startFrame = -1;
                _lastFrame = -1;
                _firstPacketMs = double.NaN;
                _lastPacketMs = double.NaN;
                if (_status.IndexOf("失败", StringComparison.Ordinal) < 0) _status = "未启动";
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
