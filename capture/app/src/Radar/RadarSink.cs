using System;
using System.Collections.Generic;
using System.IO;

namespace Acquire
{
    internal sealed class RadarFrameStat
    {
        public int FrameNumber;
        public double FirstRelMs = double.NaN;
        public double LastRelMs = double.NaN;
        public int Received;
        public int Expected;
        public int Missing;

        public bool IsEmpty { get { return Received == 0; } }
        public bool IsComplete { get { return Received == Expected; } }
    }

    internal sealed class RadarSummary
    {
        public long ReceivedPackets;
        public long DuplicatePackets;
        public long InvalidPackets;
        public long LatePackets;
        public long MissingPackets;
        public int FramesSeen;
        public int FramesComplete;
        public int FramesPartial;
        public int FramesEmpty;
        public bool MatWritten;
        public string MatPath;
        public long MatBytes;
        public bool PacketLogRetained;
        public string PacketLogPath;
        public double FirstPacketRelMs = double.NaN;
        public double LastPacketRelMs = double.NaN;
        public int LastFrameNumber;
        public int LastFrameReceived;
        public int LastFrameExpected;

        public double LossRate
        {
            get
            {
                long denom = ReceivedPackets + MissingPackets;
                return denom <= 0 ? 0.0 : MissingPackets / (double)denom;
            }
        }

        /// <summary>
        /// 排除尾帧的丢包率：停止采样的瞬间，最后一帧往往还没收完，
        /// 那部分"缺包"是截断不是丢包，单独拆出来免得误导。
        /// </summary>
        public double LossRateExcludingTail
        {
            get
            {
                long tailMissing = Math.Max(0, LastFrameExpected - LastFrameReceived);
                long missing = Math.Max(0, MissingPackets - tailMissing);
                long denom = ReceivedPackets - LastFrameReceived + missing;
                return denom <= 0 ? 0.0 : missing / (double)denom;
            }
        }
    }

    /// <summary>
    /// 雷达侧落盘：收包 → 去重 → 包日志 → 帧统计 → radar_adc.mat / radar_frames.csv。
    /// 包 ID 全局递增，帧号 = floor(id / packetsPerFrame) + 1（与 compare 脚本一致）。
    /// </summary>
    internal sealed class RadarSink : IDisposable
    {
        private readonly object _gate = new object();
        private readonly TakeWriter _take;
        private readonly Timebase _tb;
        private readonly Action<string> _log;
        private readonly int _packetsPerFrame;
        private readonly int _payloadBytes;
        private readonly RadarTransport _transport = new RadarTransport();
        private readonly Dictionary<int, RadarFrameStat> _frames = new Dictionary<int, RadarFrameStat>();
        private readonly HashSet<uint> _currentFrameIds = new HashSet<uint>();

        private FileStream _logStream;
        private byte[] _record;
        private int _currentFrame = -1;
        private long _received;
        private long _duplicates;
        private long _invalid;
        private long _late;
        private long _missing;
        private long _otherType;
        private int _droppedFrameGaps;
        private double _firstRel = double.NaN;
        private double _lastRel = double.NaN;
        private string _packetLogPath;
        private bool _running;
        private readonly RadarSummary _summary = new RadarSummary();
        private readonly RadarPreviewProcessor _previewProcessor;
        private bool _previewDisposed;

        public RadarSink(TakeWriter take, Timebase tb, int packetsPerFrame, int payloadBytes, Action<string> log)
            : this(take, tb, packetsPerFrame, payloadBytes, log, null)
        {
        }

        public RadarSink(TakeWriter take, Timebase tb, int packetsPerFrame, int payloadBytes, Action<string> log,
                         LatestPreview<RadarPreviewFrame> preview)
        {
            if (packetsPerFrame <= 0) throw new ArgumentOutOfRangeException("packetsPerFrame");
            if (payloadBytes <= 0) throw new ArgumentOutOfRangeException("payloadBytes");
            _take = take;
            _tb = tb;
            _log = log ?? delegate { };
            _packetsPerFrame = packetsPerFrame;
            _payloadBytes = payloadBytes;
            _record = new byte[12 + payloadBytes];
            if (preview != null) _previewProcessor = new RadarPreviewProcessor(preview);
        }

        public bool IsRunning { get { lock (_gate) return _running; } }
        public int PacketsPerFrame { get { return _packetsPerFrame; } }
        public int PayloadBytes { get { return _payloadBytes; } }

        public RadarSummary Summary { get { lock (_gate) return _summary; } }

        /// <summary>采集过程中的实时计数（Summary 只在 Stop() 里才填完整，不能拿来做运行态判断）。</summary>
        public long ReceivedPacketsNow { get { lock (_gate) return _received; } }
        public double LastPacketRelMsNow { get { lock (_gate) return _lastRel; } }

        public RadarSummary LiveSummary()
        {
            lock (_gate)
            {
                RadarSummary live = new RadarSummary();
                live.ReceivedPackets = _received;
                live.DuplicatePackets = _duplicates;
                live.InvalidPackets = _invalid;
                live.LatePackets = _late;
                live.MissingPackets = _missing;
                live.FramesSeen = _frames.Count;
                live.FirstPacketRelMs = _firstRel;
                live.LastPacketRelMs = _lastRel;
                live.LastFrameNumber = _currentFrame + 1;
                return live;
            }
        }

        public void Start(string localIp, int localPort, string boardIp)
        {
            lock (_gate)
            {
                if (_running) throw new InvalidOperationException("雷达已在接收");
                _packetLogPath = Path.Combine(_take.Directory, "radar_adc.packets.part");
                _logStream = new FileStream(_packetLogPath, FileMode.Create, FileAccess.Write, FileShare.Read,
                    8 * 1024 * 1024, FileOptions.SequentialScan);
                _running = true;
            }

            _transport.Log += _log;
            _transport.RelativeClock = delegate { return _tb.ElapsedMs; };
            _transport.PacketReceived += HandlePacket;
            _transport.OpenUdp(localIp, localPort, boardIp);
        }

        private void HandlePacket(TinyFramePacket packet, RadarStamp stamp)
        {
            if (packet.Type != TinyFrameCodec.TypeAdc)
            {
                _transport.NoteOtherType(packet.Type);
                lock (_gate) _otherType++;
                return;
            }
            // 固件现在把多个 chirp 合在一个 UDP 报文里发（默认 4 个 → 8192 字节 payload），
            // 所以这里按 payloadBytes 的整数倍拆分，每个 chirp 仍写一行记录，
            // .mat 的结构与原来完全一致（每帧 2048 行）。
            if (packet.Payload == null || packet.Payload.Length == 0 ||
                (packet.Payload.Length % _payloadBytes) != 0)
            {
                lock (_gate) _invalid++;
                return;
            }

            int chirps = packet.Payload.Length / _payloadBytes;
            // 逻辑 chirp 号 = 报文号 × 每报文 chirp 数；帧内 2048 个 chirp 不变
            long logicalBase = (long)packet.Id * chirps;
            int frameIndex = (int)(logicalBase / _packetsPerFrame);
            lock (_gate)
            {
                if (!_running) return;
                if (frameIndex < _currentFrame)
                {
                    _late++;
                    return;
                }

                if (frameIndex != _currentFrame)
                {
                    // 首次包：把基线定到当前帧号，不能从 0 开始算，
                    // 否则板子之前跑过的历史帧会被当成“丢包”，统计全错。
                    if (_currentFrame < 0)
                    {
                        _log("雷达首包：帧号 " + (frameIndex + 1) + "，从此帧开始记基线（之前的帧不计入丢失）");
                        _currentFrame = frameIndex;
                        _currentFrameIds.Clear();
                    }
                    else
                    {
                        CloseCurrentFrame();
                        // 中间整帧丢失时补记空帧，丢包统计才不会失真
                        int gap = frameIndex - _currentFrame - 1;
                        if (gap > 0)
                        {
                        int recordGap = Math.Min(gap, 1000);
                        for (int i = 1; i <= recordGap; i++)
                        {
                            RadarFrameStat lost = new RadarFrameStat
                            {
                                FrameNumber = _currentFrame + 1 + i,
                                Expected = _packetsPerFrame,
                                Missing = _packetsPerFrame
                            };
                            _frames[lost.FrameNumber] = lost;
                            _missing += _packetsPerFrame;
                        }
                        if (gap > recordGap)
                        {
                            _droppedFrameGaps += gap - recordGap;
                            _missing += (long)_packetsPerFrame * (gap - recordGap);
                        }
                        _log("雷达帧号跳变：从第 " + (_currentFrame + 1) + " 帧直接跳到第 " + (frameIndex + 1) +
                             " 帧，中间丢失 " + gap + " 帧");
                    }
                        _currentFrame = frameIndex;
                        _currentFrameIds.Clear();
                    }
                }

                if (!_currentFrameIds.Add(packet.Id))
                {
                    _duplicates++;
                    return;
                }

                int frameNumber = _currentFrame + 1;
                RadarFrameStat stat;
                if (!_frames.TryGetValue(frameNumber, out stat))
                {
                    stat = new RadarFrameStat { FrameNumber = frameNumber, Expected = _packetsPerFrame };
                    _frames[frameNumber] = stat;
                }
                if (stat.Received == 0)
                {
                    stat.FirstRelMs = stamp.RelMs;
                    if (double.IsNaN(_firstRel)) _firstRel = stamp.RelMs;
                }
                stat.LastRelMs = stamp.RelMs;
                _lastRel = stamp.RelMs;

                long ticks = stamp.LocalTime.Ticks;
                for (int k = 0; k < chirps; k++)
                {
                    if (_previewProcessor != null)
                        _previewProcessor.AddChirp(frameIndex, packet.Payload, k * _payloadBytes, _payloadBytes, stamp.RelMs);
                    Buffer.BlockCopy(BitConverter.GetBytes(ticks), 0, _record, 0, 8);
                    Buffer.BlockCopy(BitConverter.GetBytes((uint)(logicalBase + k)), 0, _record, 8, 4);
                    Buffer.BlockCopy(packet.Payload, k * _payloadBytes, _record, 12, _payloadBytes);
                    _logStream.Write(_record, 0, _record.Length);
                }
                stat.Received += chirps;
                _received += chirps;
            }
        }

        private void CloseCurrentFrame()
        {
            if (_currentFrame < 0) return;
            int frameNumber = _currentFrame + 1;
            RadarFrameStat stat;
            if (_frames.TryGetValue(frameNumber, out stat))
            {
                stat.Missing = Math.Max(0, stat.Expected - stat.Received);
                _missing += stat.Missing;
            }
        }

        /// <summary>停止接收并落盘。返回汇总信息。</summary>
        public RadarSummary Stop()
        {
            try { _transport.Dispose(); } catch { }
            if (_previewProcessor != null && !_previewDisposed)
            {
                _previewProcessor.Dispose();
                _previewDisposed = true;
            }

            List<RadarFrameStat> stats;
            long received;
            lock (_gate)
            {
                if (!_running) return _summary;
                CloseCurrentFrame();
                _running = false;
                if (_logStream != null)
                {
                    try { _logStream.Flush(true); } catch { }
                    _logStream.Dispose();
                    _logStream = null;
                }
                _summary.ReceivedPackets = _received;
                _summary.DuplicatePackets = _duplicates;
                _summary.InvalidPackets = _invalid;
                _summary.LatePackets = _late;
                _summary.MissingPackets = _missing;
                _summary.FirstPacketRelMs = _firstRel;
                _summary.LastPacketRelMs = _lastRel;
                stats = new List<RadarFrameStat>(_frames.Values);
                stats.Sort(delegate(RadarFrameStat a, RadarFrameStat b) { return a.FrameNumber.CompareTo(b.FrameNumber); });
                foreach (RadarFrameStat s in stats)
                {
                    _summary.FramesSeen++;
                    if (s.IsComplete) _summary.FramesComplete++;
                    else if (s.IsEmpty) _summary.FramesEmpty++;
                    else _summary.FramesPartial++;
                }
                _summary.MatPath = Path.Combine(_take.Directory, "radar_adc.mat");
                if (stats.Count > 0)
                {
                    RadarFrameStat last = stats[stats.Count - 1];
                    _summary.LastFrameNumber = last.FrameNumber;
                    _summary.LastFrameReceived = last.Received;
                    _summary.LastFrameExpected = last.Expected;
                }
                received = _received;
            }

            _log("雷达停止：收包 " + received + "，丢包 " + _summary.MissingPackets +
                 "，重复 " + _summary.DuplicatePackets + "，非法 " + _summary.InvalidPackets +
                 "，整帧缺失 " + _summary.FramesEmpty + " 帧" +
                 "｜UDP 原始数据报 " + _transport.RawDatagrams +
                 "，源 IP 不符 " + _transport.DroppedBySourceIp +
                 "，解析失败 " + _transport.ParseFailures);

            foreach (RadarFrameStat s in stats)
            {
                _take.AddRadarFrame(s);
                // 雷达一帧只对应一个 .mat 文件，时间取该帧首包到达时刻
                if (s.Received > 0) _take.AddRow("radar", s.FrameNumber, s.FirstRelMs, "radar_adc.mat");
            }

            // 写 MAT；如果超过 4 GiB，保留包日志作为退路
            try
            {
                if (received > 0)
                {
                    MatWriter.Write(_packetLogPath, _summary.MatPath, _payloadBytes, received, DateTime.Now);
                    _summary.MatWritten = true;
                    _summary.MatBytes = new FileInfo(_summary.MatPath).Length;
                    _log("radar_adc.mat 写入完成：" + _summary.MatBytes + " 字节，" + (received / _packetsPerFrame) + " 帧");
                }
                else
                {
                    _log("警告：本次采集没有收到任何雷达包，未生成 radar_adc.mat");
                }
            }
            catch (Exception ex)
            {
                _summary.MatWritten = false;
                _log("radar_adc.mat 生成失败：" + ex.Message);
            }

            try
            {
                if (File.Exists(_packetLogPath))
                {
                    if (_summary.MatWritten)
                    {
                        File.Delete(_packetLogPath);
                    }
                    else
                    {
                        string retained = Path.Combine(_take.Directory, "radar_adc.packets");
                        if (File.Exists(retained)) File.Delete(retained);
                        File.Move(_packetLogPath, retained);
                        _summary.PacketLogRetained = true;
                        _summary.PacketLogPath = retained;
                        _log("已保留原始包日志：" + Path.GetFileName(retained));
                    }
                }
            }
            catch (Exception ex)
            {
                _log("清理包日志失败：" + ex.Message);
            }

            return _summary;
        }

        public void Dispose()
        {
            try { Stop(); } catch { }
            if (_previewProcessor != null && !_previewDisposed) _previewProcessor.Dispose();
        }
    }
}
