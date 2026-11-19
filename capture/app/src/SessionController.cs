using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Threading;

namespace Acquire
{
    internal enum SessionState
    {
        Idle,
        Countdown,
        Recording,
        Saving
    }

    internal sealed class SessionOptions
    {
        public string OutputRoot;
        public string Label = "";
        public int DelaySeconds;
        public int DurationSeconds = 10;
        public bool RadarEnabled = true;
        public bool RadarSynthetic;
        public bool RadarAc880Mode;
        public string LocalIp = "192.168.220.10";
        public int LocalPort = 8080;
        public string BoardIp = "192.168.220.100";
        /// <summary>UART 控制端口；留空 = 自动挑选。</summary>
        public string SerialPortName = "";
        /// <summary>UART 波特率，老 GUI 默认 115200。</summary>
        public int SerialBaud = 115200;
        public int PacketsPerFrame = 2048;
        public int PayloadBytes = 2048;
        public CameraOption Camera;
        public bool SelfTest;
        /// <summary>采集时同时存一份降采样点云（cloud\*.ply），默认关闭。原始深度图无论如何都写。</summary>
        public bool SaveCloud;
        /// <summary>点云取样步长：每 stride×stride 像素取 1 点。</summary>
        public int CloudStride = DepthCloud.DefaultStride;
        /// <summary>采集提示音：预备结束一声、拍摄结束一声。默认关，GUI 里默认勾选，自检/批处理保持静音。</summary>
        public bool SoundCues;
        public bool VerboseLog = true;
        public bool LidarEnabled;
        public bool FrameAlignmentEnabled;
        public bool LidarSynthetic = true;
        public string LidarBroadcastCode = "";
        public string LidarReplayDirectory = "";
        public string LidarReturnMode = "single";
        public CalibrationInstallationConfig CalibrationInstallation;
    }

    internal sealed class SessionSnapshot
    {
        public SessionState State;
        public int CountdownRemaining;
        public double ElapsedSeconds;
        public double RemainingSeconds;
        public long RadarFrames;
        public long RadarPackets;
        public long RadarMissing;
        public double RadarFps;
        public long ColorFrames;
        public long DepthFrames;
        public double ColorFps;
        public double DepthFps;
        public long CloudFrames;
        public long CloudPoints;
        public string TakeDirectory = "";
        public string Message = "";
        public string Error = "";
        public string RadarHealth = "未启动";
        public int RadarRestarts;
        public long PreviewDropped;
        public double PreviewLatencyMs;
        public long LidarBatches;
        public long LidarPoints;
        public double LidarFps;
        public string LidarHealth = "未启动";
    }

    /// <summary>
    /// 状态机：Idle → Countdown → Recording → Saving → Idle。
    /// 所有采集动作都在后台线程执行，界面只轮询 Snapshot()，避免 UI 线程被收包/落盘拖住。
    /// </summary>
    internal sealed class SessionController : IDisposable
    {
        private readonly object _gate = new object();
        private SessionOptions _options;
        private Thread _worker;
        private volatile SessionState _state = SessionState.Idle;
        private volatile bool _stopRequested;
        private volatile int _countdown;
        private volatile string _message = "";
        private volatile string _error = "";
        private volatile string _takeDirectory = "";
        private Timebase _tb;
        private TakeWriter _take;
        private RadarSink _radar;
        private SerialControl _serial;
        private readonly object _serialGate = new object();
        private List<PortCandidate> _serialCandidates = new List<PortCandidate>();
        private int _serialCandidateIndex;
        private int _serialSendsOnCandidate;
        private string _serialPortUsed = "";
        private int _serialStartSent;
        private int _serialPauseSent;
        private int _radarRestartAttempts;
        private bool _radarEverHealthy;
        private long _lastRadarPacketCount;
        private double _lastRadarProgressAt;
        private double _nextRadarRecoveryAt;
        private volatile string _radarHealth = "未启动";
        private CameraCapture _camera;
        private CameraOption _cameraOption;
        private DepthIntrinsics _intrinsics;
        private string _cameraError = "";
        private string _radarError = "";
        private string _lidarError = "";
        private string _cameraModeNote = "";
        private RadarSummary _radarSummary;
        private SyntheticRadar _syntheticRadar;
        private CameraCaptureStats _cameraStats;
        private DateTime _recordStartUtc;
        private DateTime _recordEndUtc;
        private double _actualSeconds;
        private LidarCapture _lidar;
        private LidarCaptureStats _lidarStats;
        private AlignmentCoordinator _alignment;
        private double _alignmentRadarStartMs = double.NaN;
        private string _lidarSerial = "";
        private string _lidarFirmware = "";
        private bool _cameraRuntimeWarning;
        private bool _radarRuntimeWarning;
        private bool _lidarRuntimeWarning;
        private readonly LatestPreview<CameraPreviewFrame> _cameraPreview = new LatestPreview<CameraPreviewFrame>();
        private readonly LatestPreview<RadarPreviewFrame> _radarPreview = new LatestPreview<RadarPreviewFrame>();
        private readonly LatestPreview<RadarCloudFrame> _radarPointCloudPreview = new LatestPreview<RadarCloudFrame>();
        private readonly LatestPreview<LidarBatch> _lidarPreview = new LatestPreview<LidarBatch>();

        public event Action<string> Log;

        /// <summary>真正进入 Recording 的瞬间（倒计时结束、take 目录已建好、接收已开）。</summary>
        public event Action RecordingStarted;

        public SessionState State { get { return _state; } }
        public bool IsBusy { get { return _state != SessionState.Idle; } }
        public string TakeDirectory { get { return _takeDirectory; } }

        public void Start(SessionOptions options)
        {
            if (options == null) throw new ArgumentNullException("options");
            if (IsBusy) throw new InvalidOperationException("采集正在进行中");
            if (options.DurationSeconds < 1 || options.DurationSeconds > 600)
                throw new ArgumentOutOfRangeException("options.DurationSeconds", "采集时长应在 1~600 秒");
            if (options.DelaySeconds < 0 || options.DelaySeconds > 60)
                throw new ArgumentOutOfRangeException("options.DelaySeconds", "延迟应在 0~60 秒");

            Directory.CreateDirectory(options.OutputRoot);
            _options = options;
            _cameraOption = options.Camera;
            _stopRequested = false;
            _error = "";
            _cameraError = "";
            _radarError = "";
            _lidarError = "";
            _cameraModeNote = "";
            _cameraRuntimeWarning = false;
            _radarRuntimeWarning = false;
            _lidarRuntimeWarning = false;
            _intrinsics = null;
            _takeDirectory = "";
            _radarSummary = null;
            _cameraStats = null;
            _lidarStats = null;
            _alignment = options.FrameAlignmentEnabled ? new AlignmentCoordinator(1000.0, 30.0) : null;
            _alignmentRadarStartMs = double.NaN;
            _lidarSerial = "";
            _lidarFirmware = "";
            _serialCandidates = new List<PortCandidate>();
            _serialCandidateIndex = 0;
            _serialSendsOnCandidate = 0;
            _radarRestartAttempts = 0;
            _radarEverHealthy = false;
            _lastRadarPacketCount = 0;
            _lastRadarProgressAt = 0;
            _nextRadarRecoveryAt = 0;
            _radarHealth = options.RadarEnabled ? "等待启动" : "已关闭";
            _state = SessionState.Countdown;
            _countdown = options.DelaySeconds;
            _message = options.DelaySeconds > 0 ? "倒计时" : "启动中";
            EmitLog("开始任务：延迟 " + options.DelaySeconds + " s，时长 " + options.DurationSeconds + " s，相机=" +
                    (options.Camera == null ? "未启用" : options.Camera.Display));

            _worker = new Thread(Run);
            _worker.Name = "Acquire session";
            _worker.IsBackground = true;
            _worker.Start();
        }

        public void Stop()
        {
            if (_state == SessionState.Idle) return;
            _stopRequested = true;
            EmitLog("收到停止请求");
        }

        private void Run()
        {
            SessionOptions o = _options;
            ICameraSource source = null;
            CuePlayer cues = null;
            try
            {
                // 提示音先建好：波形在构造时就加载进内存，第一次响不会卡在磁盘或解码上
                if (o.SoundCues)
                {
                    try { cues = new CuePlayer(EmitLog); }
                    catch (Exception ex) { EmitLog("提示音初始化失败，本次静音：" + ex.Message); }
                }
                // 先开相机，再做倒计时：倒计时期间一直在抽帧丢弃，
                // 这样雷达一开始录，彩色和深度都已经是热的（不会缺开头几帧）。
                // 相机预热放在时钟启动之前：设备打开耗时不应该计入采集时长
                if (_cameraOption != null)
                {
                    try
                    {
                        source = CameraProbe.Create(_cameraOption, EmitLog, delegate { return _tb == null ? 0 : _tb.ElapsedMs; });
                        source.Open();
                        ValidateCameraFrames(source, _cameraOption);
                        _cameraModeNote = DescribeModes(source);
                        EmitLog("相机首帧验收通过：" + source.Model + " / " + source.Serial +
                                (source.HasDepth ? "（含深度）" : "（深度不可用）"));

                        IDepthIntrinsicsProvider provider = source as IDepthIntrinsicsProvider;
                        if (provider != null) _intrinsics = provider.GetDepthIntrinsics();
                        if (_intrinsics != null && _intrinsics.Valid)
                            EmitLog("深度内参：" + _intrinsics.Describe() + "（" + _intrinsics.Detail + "）");
                        else if (source.HasDepth)
                            EmitLog("警告：该后端没有提供深度内参，只能采集后用标称 FOV 重算");
                    }
                    catch (Exception ex)
                    {
                        _cameraError = DynHelper.DescribeException(ex);
                        if (_cameraError.IndexOf("Failed to set USB interface", StringComparison.OrdinalIgnoreCase) >= 0)
                            _cameraError += "\r\n建议：关闭相机软件，重新插拔 Astra 并使用 USB 3.x 接口。";
                        EmitLog("警告：相机启动失败，本次跳过相机并继续采集：" + _cameraError);
                        try { if (source != null) source.Dispose(); } catch { }
                        source = null;
                    }
                }
                else EmitLog("警告：未启用可用相机，本次继续采集其它传感器");

                // 倒计时：这段时间相机已经在跑，持续抽帧丢弃，把彩色和深度都预热好。
                if (o.DelaySeconds > 0)
                {
                    EmitLog("倒计时 " + o.DelaySeconds + " 秒（相机已开，正在预热，帧丢弃不计入采集）");
                    for (int remaining = o.DelaySeconds; remaining > 0; remaining--)
                    {
                        if (_stopRequested) break;
                        _countdown = remaining;
                        for (int i = 0; i < 10 && !_stopRequested; i++)
                        {
                            try { if (source != null) source.ReadFrame(30); } catch { }
                            Thread.Sleep(70);
                        }
                    }
                    _countdown = 0;
                    if (_stopRequested)
                    {
                        _message = "已取消（倒计时阶段）";
                        EmitLog("倒计时阶段取消，未生成 take");
                        _state = SessionState.Idle;
                        return;
                    }
                }

                _tb = Timebase.StartNew();
                _recordStartUtc = _tb.AnchorUtc;
                string takeName = TakeWriter.MakeTakeName(o.OutputRoot, DateTime.Now, o.Label);
                _take = new TakeWriter(o.OutputRoot, takeName, _tb);
                _takeDirectory = _take.Directory;
                EmitLog("take 目录：" + _take.Directory);

                if (o.RadarEnabled && !o.FrameAlignmentEnabled)
                {
                    try
                    {
                        if (o.RadarAc880Mode)
                        {
                            RadarNetwork.ValidateLocalAddress("192.168.221.100");
                            StartAc880Serial(o);
                            _radarHealth = "AC880 点云采集中";
                            EmitLog("毫米波模式：AC880 实时点云（192.168.221.2 -> 192.168.221.100:5001）");
                        }
                        else
                        {
                            if (!o.RadarSynthetic)
                            {
                                o.LocalIp = RadarNetwork.ValidateLocalAddress(o.LocalIp);
                                EmitLog("雷达网络检查：" + RadarNetwork.Describe(o.LocalIp));
                            }
                            _radar = new RadarSink(_take, _tb, o.PacketsPerFrame, o.PayloadBytes, EmitLog,
                                _radarPreview, _radarPointCloudPreview);
                            _radar.Start(o.LocalIp, o.LocalPort, o.BoardIp);
                            if (o.RadarSynthetic)
                            {
                                _syntheticRadar = new SyntheticRadar(o.LocalIp, o.LocalPort, o.PacketsPerFrame,
                                    o.PayloadBytes, 10, 0, 0, 0, true);
                                _syntheticRadar.Start();
                                _radarHealth = "合成数据运行";
                                EmitLog("毫米波雷达使用 UDP 合成源（演示/测试）");
                            }
                            else StartSerial(o);
                        }
                    }
                    catch (Exception ex)
                    {
                        _radarError = DynHelper.DescribeException(ex);
                        _radarHealth = "不可用";
                        EmitLog("警告：毫米波雷达启动失败，本次跳过雷达并继续采集：" + _radarError);
                        try { if (_syntheticRadar != null) _syntheticRadar.Dispose(); } catch { }
                        _syntheticRadar = null;
                        try { if (_radar != null) _radar.Dispose(); } catch { }
                        _radar = null;
                    }
                }
                else
                {
                    EmitLog("警告：毫米波雷达未启用，本次继续采集其它传感器");
                }

                if (source != null)
                {
                    _camera = new CameraCapture(source, _take, EmitLog, _intrinsics, o.SaveCloud, o.CloudStride, _cameraPreview,
                        delegate(string stream, int frame, double t) { if (_alignment != null) _alignment.AddCamera(stream, frame, t); });
                    _camera.Start();
                    if (o.SaveCloud)
                    {
                        if (_intrinsics == null || !_intrinsics.Valid)
                            EmitLog("警告：没有可用内参，勾选的“同时存点云”本次会被跳过");
                        else
                            EmitLog("点云：每 " + o.CloudStride + "x" + o.CloudStride +
                                    " 像素取 1 点，写入 cloud\\NNNNNN.ply（事后可用 --export-cloud 换 stride 重算）");
                    }
                }

                if (o.LidarEnabled)
                {
                    try
                    {
                        ILidarSource lidarSource;
                        if (!string.IsNullOrEmpty(o.LidarReplayDirectory))
                            lidarSource = new ReplayLidarSource(o.LidarReplayDirectory, delegate { return _tb.ElapsedMs; });
                        else if (o.LidarSynthetic)
                            lidarSource = new SyntheticLidarSource(delegate { return _tb.ElapsedMs; });
                        else
                            lidarSource = new LivoxNativeSource(o.LidarBroadcastCode,
                                delegate(long timestamp) { return _tb.RelMsFromTimestamp(timestamp); });
                        _lidar = new LidarCapture(lidarSource, _take, _lidarPreview, EmitLog,
                            delegate(LidarBatch batch, string file) { if (_alignment != null) _alignment.AddLidar(batch, file); });
                        _lidar.Start();
                        EmitLog("LiDAR 启动：" + lidarSource.Model + "，回波模式 " + o.LidarReturnMode);
                    }
                    catch (Exception ex)
                    {
                        _lidarError = DynHelper.DescribeException(ex);
                        EmitLog("警告：LiDAR 启动失败，本次跳过 LiDAR 并继续采集：" + _lidarError);
                        try { if (_lidar != null) _lidar.Dispose(); } catch { }
                        _lidar = null;
                    }
                }
                else EmitLog("警告：LiDAR 未启用，本次继续采集其它传感器");

                if (o.FrameAlignmentEnabled)
                {
                    EmitLog("三传感器帧对齐：相机/LiDAR 先启动，等待 LiDAR 1000 ms 预缓存后再启动 ADC 雷达");
                    DateTime waitStart = DateTime.UtcNow;
                    while (!_stopRequested)
                    {
                        CameraCaptureStats cameraReady = _camera == null ? null : _camera.Snapshot();
                        LidarCaptureStats lidarReady = _lidar == null ? null : _lidar.Snapshot();
                        bool cameraOk = cameraReady != null && (!o.Camera.HasDepth || cameraReady.DepthFrames > 0);
                        bool lidarOk = lidarReady != null && lidarReady.Batches > 0 &&
                            !double.IsNaN(lidarReady.FirstRelMs) && !double.IsNaN(lidarReady.LastRelMs) &&
                            lidarReady.LastRelMs - lidarReady.FirstRelMs >= 1000.0;
                        if (cameraOk && lidarOk) break;
                        if ((DateTime.UtcNow - waitStart).TotalSeconds >= 10.0)
                            throw new InvalidOperationException("三传感器对齐启动失败：相机深度或 LiDAR 1000 ms 预缓存未就绪");
                        Thread.Sleep(50);
                    }
                    if (o.RadarEnabled)
                    {
                        try
                        {
                            if (!o.RadarSynthetic)
                            {
                                o.LocalIp = RadarNetwork.ValidateLocalAddress(o.LocalIp);
                                EmitLog("雷达网络检查：" + RadarNetwork.Describe(o.LocalIp));
                            }
                            _radar = new RadarSink(_take, _tb, o.PacketsPerFrame, o.PayloadBytes, EmitLog,
                                _radarPreview, _radarPointCloudPreview,
                                delegate(RadarFrameStat stat) { if (_alignment != null) _alignment.AddRadar(stat); });
                            _radar.Start(o.LocalIp, o.LocalPort, o.BoardIp);
                            if (o.RadarSynthetic)
                            {
                                _syntheticRadar = new SyntheticRadar(o.LocalIp, o.LocalPort, o.PacketsPerFrame,
                                    o.PayloadBytes, 10, 0, 0, 0, true);
                                _syntheticRadar.Start();
                                _radarHealth = "合成数据运行";
                            }
                            else StartSerial(o);
                            _alignmentRadarStartMs = _tb.ElapsedMs;
                            EmitLog("ADC 雷达已启动，正式采集计时从雷达启动时刻开始");
                        }
                        catch (Exception ex)
                        {
                            _radarError = DynHelper.DescribeException(ex);
                            throw new InvalidOperationException("ADC 雷达启动失败：" + _radarError, ex);
                        }
                    }
                }

                _state = SessionState.Recording;
                _message = "记录中";
                // 倒计时结束、彩色/深度/雷达都已经在进数据，这一声才是"真正开始"
                if (cues != null) cues.Play(CueKind.Start);
                Action started = RecordingStarted;
                if (started != null) { try { started(); } catch { } }

                bool cameraRecordingConfirmed = false;
                while (!_stopRequested)
                {
                    double elapsed = o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs)
                        ? (_tb.ElapsedMs - _alignmentRadarStartMs) / 1000.0 : _tb.ElapsedSeconds;
                    if (elapsed >= o.DurationSeconds) break;
                    if (_radar != null && !o.RadarSynthetic && !o.RadarAc880Mode) MonitorRadar(o, elapsed);
                    MonitorOptionalStreams(elapsed);
                    if (!cameraRecordingConfirmed && _camera != null)
                    {
                        CameraCaptureStats cameraNow = _camera.Snapshot();
                        bool colorOk = !o.Camera.HasColor || cameraNow.ColorFrames > 0;
                        bool depthOk = !o.Camera.HasDepth || cameraNow.DepthFrames > 0;
                        if (colorOk && depthOk)
                        {
                            cameraRecordingConfirmed = true;
                            EmitLog("相机落盘确认：彩色 " + cameraNow.ColorFrames + " 帧，深度 " + cameraNow.DepthFrames + " 帧");
                        }
                        else if (elapsed >= 3.0)
                        {
                            cameraRecordingConfirmed = true;
                            _cameraRuntimeWarning = true;
                            EmitLog("警告：相机进入记录后 3 秒仍未完整落盘（彩色 " + cameraNow.ColorFrames +
                                    " 帧，深度 " + cameraNow.DepthFrames + " 帧），其它传感器继续采集");
                        }
                    }
                    Thread.Sleep(25);
                }

                _state = SessionState.Saving;
                _message = "正在落盘";
                if (cues != null) cues.Play(CueKind.End);
                // 先让板子停下来，再停 UDP
                StopSerial();
                if (_syntheticRadar != null) _syntheticRadar.Stop();
                // 时长与结束时刻在停止任何采集器之前就固定，避免把 MAT 写盘耗时算进采集时长
                _actualSeconds = o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs)
                    ? (_tb.ElapsedMs - _alignmentRadarStartMs) / 1000.0 : _tb.ElapsedSeconds;
                _recordEndUtc = _tb.UtcFromRelMs((o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs) ? _alignmentRadarStartMs : 0) + _actualSeconds * 1000.0);
                EmitLog("停止采集，开始落盘");

                if (_camera != null)
                {
                    _camera.Stop();
                    _cameraStats = _camera.Snapshot();
                }
                if (_lidar != null)
                {
                    _lidar.Stop();
                    _lidarStats = _lidar.Snapshot();
                }
                if (_radar != null)
                {
                    _radarSummary = _radar.Stop();
                }

                WriteOutputs(o);
                _message = "完成：" + Path.GetFileName(_takeDirectory);
                EmitLog("采集完成：" + _takeDirectory + "（实测 " + _actualSeconds.ToString("0.000") + " s）");
            }
            catch (Exception ex)
            {
                _error = DynHelper.DescribeException(ex);
                EmitLog("采集异常：" + _error);
                _message = "失败：" + _error;
                try { StopSerial(); } catch { }
                try { if (_syntheticRadar != null) _syntheticRadar.Stop(); } catch { }
                TryFinalizeOnError(o);
            }
            finally
            {
                if (cues != null)
                {
                    // 收尾音还在响，别急着掐断；释放交给后台线程
                    cues.DisposeSoon(800);
                    cues = null;
                }
                if (_serial != null)
                {
                    try { _serial.Dispose(); } catch { }
                    _serial = null;
                }
                if (_syntheticRadar != null)
                {
                    try { _syntheticRadar.Dispose(); } catch { }
                    _syntheticRadar = null;
                }
                if (_lidar != null)
                {
                    try { _lidar.Dispose(); } catch { }
                    _lidar = null;
                }
                if (_camera != null)
                {
                    try { _camera.Dispose(); } catch { }
                    _camera = null;
                }
                if (_radar != null)
                {
                    try { _radar.Dispose(); } catch { }
                    _radar = null;
                }
                if (source != null)
                {
                    try { source.Dispose(); } catch { }
                }
                _state = SessionState.Idle;
                _countdown = 0;
            }
        }

        private void ValidateCameraFrames(ICameraSource source, CameraOption expected)
        {
            bool needColor = expected != null && (expected.HasColor || expected.Kind == CameraKind.OpenNi2);
            bool needDepth = expected != null && expected.HasDepth;
            bool gotColor = false;
            bool gotDepth = false;
            DateTime deadline = DateTime.UtcNow.AddSeconds(3);

            EmitLog("相机首帧验收：最多等待 3 秒，要求" +
                    (needColor ? "彩色" : "") + (needColor && needDepth ? "+" : "") + (needDepth ? "深度" : "任意有效帧"));
            while (!_stopRequested && DateTime.UtcNow < deadline)
            {
                CameraFrame frame = source.ReadFrame(150);
                if (frame == null) continue;
                if (frame.HasColor) gotColor = true;
                if (frame.HasDepth) gotDepth = true;
                if ((needColor || needDepth) ? (!needColor || gotColor) && (!needDepth || gotDepth) : gotColor || gotDepth)
                    return;
            }

            string missing = !gotColor && !gotDepth ? "彩色和深度均为 0 帧"
                : (!gotColor ? "彩色为 0 帧" : "深度为 0 帧");
            throw new InvalidOperationException(
                "相机首帧验收失败（" + missing + "）。请关闭 OrbbecViewer/NiViewer/其它采集程序，" +
                "重新插拔相机并优先连接电脑的 USB 3.x 接口；若仍失败，请重装 Orbbec/OpenNI2 驱动。");
        }

        /// <summary>
        /// 枚举 UART 候选并发 Start。自动模式保留全部端口，后续由看门狗逐个尝试，
        /// 以实际收到 UDP 数据作为“雷达启动成功”的唯一依据。
        /// </summary>
        private void StartSerial(SessionOptions o)
        {
            _serialCandidates = new List<PortCandidate>();
            if (!string.IsNullOrEmpty(o.SerialPortName))
                _serialCandidates.Add(new PortCandidate(o.SerialPortName, "用户指定"));
            else
                _serialCandidates = SerialControl.AutoCandidates();

            if (_serialCandidates.Count == 0)
            {
                EmitLog("警告：没有可用串口，雷达不会启动（UDP 会一直收不到包）");
                _radarHealth = "故障：未发现串口";
                return;
            }
            EmitLog("串口自动探测到 " + _serialCandidates.Count + " 个候选，将以雷达 UDP 回包确认正确端口");
            _serialCandidateIndex = 0;
            OpenCurrentSerial(o, false);
            _nextRadarRecoveryAt = 1.0;
        }

        private void StartAc880Serial(SessionOptions o)
        {
            List<PortCandidate> candidates = string.IsNullOrEmpty(o.SerialPortName)
                ? SerialControl.AutoCandidates()
                : new List<PortCandidate> { new PortCandidate(o.SerialPortName, "AC880 雷达控制串口") };
            if (candidates.Count == 0) throw new InvalidOperationException("AC880 模式未发现雷达控制串口（README 默认 COM3）");
            foreach (PortCandidate candidate in candidates)
            {
                SerialControl serial = new SerialControl(EmitLog);
                string error;
                if (!serial.TryOpen(candidate.Name, o.SerialBaud, out error))
                {
                    serial.Dispose();
                    EmitLog("AC880 模式串口 " + candidate.Name + " 打开失败：" + error);
                    continue;
                }
                _serial = serial;
                _serialPortUsed = candidate.Name;
                if (_serial.SendStop()) _serialPauseSent++;
                EmitLog("AC880 启动前已发 Stop D001，等待 750 ms 清理上次会话");
                Thread.Sleep(750);
                if (!_serial.SendStart()) throw new InvalidOperationException("AC880 模式无法向 " + candidate.Name + " 发送 Start D000");
                _serialStartSent++;
                EmitLog("AC880 模式已发 Start D000，等待实时点云");
                return;
            }
            throw new InvalidOperationException("AC880 模式无法打开任何雷达控制串口");
        }

        private void MonitorRadar(SessionOptions o, double elapsed)
        {
            long packets = _radar.ReceivedPacketsNow;
            if (packets > _lastRadarPacketCount)
            {
                bool recovered = !_radarEverHealthy || _radarHealth != "运行正常";
                _lastRadarPacketCount = packets;
                _lastRadarProgressAt = elapsed;
                _radarEverHealthy = true;
                _radarHealth = "运行正常";
                _message = "记录中（雷达正常）";
                _serialSendsOnCandidate = 0;
                _nextRadarRecoveryAt = elapsed + 1.0;
                if (recovered) EmitLog("雷达启动/恢复成功：已收到 " + packets + " 个包，串口 " + _serialPortUsed);
                return;
            }

            double silentSeconds = elapsed - _lastRadarProgressAt;
            if (!_radarRuntimeWarning && ((!_radarEverHealthy && elapsed >= 3.0) ||
                (_radarEverHealthy && silentSeconds >= 3.0)))
            {
                _radarRuntimeWarning = true;
                EmitLog("警告：毫米波雷达超过 3 秒没有有效 ADC 数据，其它传感器继续采集");
            }
            double threshold = _radarEverHealthy ? 2.0 : 1.0;
            if (silentSeconds < threshold || elapsed < _nextRadarRecoveryAt) return;

            _radarRestartAttempts++;
            _radarHealth = _radarEverHealthy ? "断流，自动重启中" : "未回包，自动检测中";
            _message = "记录中（" + _radarHealth + "）";

            bool switchPort = _serial == null || !_serial.IsOpen || _serialSendsOnCandidate >= 2;
            if (switchPort)
            {
                if (_serialCandidates.Count > 1)
                    _serialCandidateIndex = (_serialCandidateIndex + 1) % _serialCandidates.Count;
                OpenCurrentSerial(o, true);
            }
            else
            {
                EmitLog("雷达 " + silentSeconds.ToString("0.0") + " 秒无数据，自动重发 Start（第 " + _radarRestartAttempts + " 次恢复）");
                if (_serial.SendPause()) _serialPauseSent++;
                Thread.Sleep(80);
                if (_serial.SendStart()) _serialStartSent++;
                _serialSendsOnCandidate++;
            }
            _nextRadarRecoveryAt = elapsed + 1.0;
        }

        private void MonitorOptionalStreams(double elapsed)
        {
            if (_camera != null && !_cameraRuntimeWarning)
            {
                CameraCaptureStats camera = _camera.Snapshot();
                bool colorDisconnected = _cameraOption != null && _cameraOption.HasColor && camera.ColorFrames > 0 &&
                    elapsed * 1000.0 - camera.LastColorRelMs >= 3000.0;
                bool depthDisconnected = _cameraOption != null && _cameraOption.HasDepth && camera.DepthFrames > 0 &&
                    elapsed * 1000.0 - camera.LastDepthRelMs >= 3000.0;
                if (colorDisconnected || depthDisconnected || !string.IsNullOrEmpty(camera.LastError))
                {
                    _cameraRuntimeWarning = true;
                    EmitLog("警告：相机" + (colorDisconnected || depthDisconnected ? "超过 3 秒没有新帧" : "采集故障：" + camera.LastError) +
                            "，其它传感器继续采集");
                }
            }
            if (_lidar != null && !_lidarRuntimeWarning)
            {
                LidarCaptureStats lidar = _lidar.Snapshot();
                bool noData = elapsed >= 3.0 && lidar.Batches == 0;
                bool disconnected = lidar.Batches > 0 && elapsed * 1000.0 - lidar.LastRelMs >= 3000.0;
                if (noData || disconnected || !string.IsNullOrEmpty(lidar.Error))
                {
                    _lidarRuntimeWarning = true;
                    EmitLog("警告：LiDAR " + (noData ? "3 秒内没有点云" : (disconnected ? "超过 3 秒没有新点云" : "采集故障：" + lidar.Error)) +
                            "，其它传感器继续采集");
                }
            }
        }

        private void OpenCurrentSerial(SessionOptions o, bool recovery)
        {
            lock (_serialGate)
            {
                if (_serial != null) { try { _serial.Close(); } catch { } }
                _serial = new SerialControl(EmitLog);
                _serialSendsOnCandidate = 0;
                int checkedPorts = 0;
                while (checkedPorts < _serialCandidates.Count)
                {
                    PortCandidate candidate = _serialCandidates[_serialCandidateIndex];
                    string error;
                    if (_serial.TryOpen(candidate.Name, o.SerialBaud, out error))
                    {
                        _serialPortUsed = candidate.Name;
                        if (_serial.SendStart()) _serialStartSent++;
                        _serialSendsOnCandidate = 1;
                        EmitLog((recovery ? "雷达自动恢复" : "雷达启动") + "：尝试串口 " + candidate);
                        return;
                    }
                    EmitLog("串口 " + candidate.Name + " 打开失败：" + error);
                    _serialCandidateIndex = (_serialCandidateIndex + 1) % _serialCandidates.Count;
                    checkedPorts++;
                }
                _radarHealth = "故障：串口均无法打开";
            }
        }

        private void StopSerial()
        {
            SerialControl serial = _serial;
            if (serial != null && serial.IsOpen)
            {
                if (_options != null && _options.RadarAc880Mode)
                {
                    if (serial.SendStop()) _serialPauseSent++;
                    EmitLog("AC880 模式已发 Stop D001；下一轮独立采集前需按 README 重启 AC880");
                }
                else if (serial.SendPause()) _serialPauseSent++;
            }
            if (serial != null) serial.Close();
            _radarHealth = "已停止";
        }

        private void TryFinalizeOnError(SessionOptions o)        {
            try
            {
                if (_camera != null) { _camera.Stop(); _cameraStats = _camera.Snapshot(); }
            }
            catch { }
            try
            {
                if (_radar != null) _radarSummary = _radar.Stop();
            }
            catch { }
            try
            {
                if (_lidar != null) { _lidar.Stop(); _lidarStats = _lidar.Snapshot(); }
            }
            catch { }
            try
            {
                if (_take != null && _tb != null)
                {
                    _actualSeconds = o != null && o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs)
                        ? (_tb.ElapsedMs - _alignmentRadarStartMs) / 1000.0 : _tb.ElapsedSeconds;
                    _recordEndUtc = _tb.UtcFromRelMs((o != null && o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs) ? _alignmentRadarStartMs : 0) + _actualSeconds * 1000.0);
                    WriteOutputs(o);
                }
            }
            catch (Exception ex)
            {
                EmitLog("异常收尾也失败：" + ex.Message);
            }
        }

        private void WriteOutputs(SessionOptions o)
        {
            RadarSummary radar = _radarSummary;
            CameraCaptureStats cam = _cameraStats;
            double duration = _actualSeconds <= 0 ? 0 : _actualSeconds;

            double depthScale = cam != null && cam.DepthScaleMeters > 0 ? cam.DepthScaleMeters
                : (o.Camera != null && o.Camera.HasDepth ? 0.001 : 0);
            if (cam != null && cam.DepthFrames > 0 && depthScale > 0) _take.WriteDepthScale(depthScale);

            JsonObject meta = new JsonObject();
            meta.Add("format", "ACQUIRE_TAKE_V1");
            meta.Add("takeId", Path.GetFileName(_take.Directory));
            meta.Add("createdLocal", DateTime.Now.ToString("yyyy-MM-ddTHH:mm:ss.fff", CultureInfo.InvariantCulture));
            meta.Add("selfTest", o.SelfTest);
            meta.Add("delaySeconds", o.DelaySeconds);
            meta.Add("requestedDurationSeconds", o.DurationSeconds);
            meta.Add("actualDurationSeconds", Math.Round(duration, 3));
            meta.Add("recordStartUtc", _recordStartUtc.ToString("o", CultureInfo.InvariantCulture));
            meta.Add("recordEndUtc", _recordEndUtc.ToString("o", CultureInfo.InvariantCulture));
            JsonObject alignment = new JsonObject();
            alignment.Add("enabled", o.FrameAlignmentEnabled);
            alignment.Add("master", o.FrameAlignmentEnabled ? "ADT7880 radar frame center" : null);
            alignment.Add("lidarWindowMs", o.FrameAlignmentEnabled ? 1000 : 0);
            alignment.Add("maxCameraDeltaMs", o.FrameAlignmentEnabled ? 30 : 0);
            alignment.Add("radarStartPrebufferMs", o.FrameAlignmentEnabled && !double.IsNaN(_alignmentRadarStartMs) ? Math.Round(_alignmentRadarStartMs, 3) : 0);
            alignment.Add("frames", _alignment == null ? 0 : _alignment.Count);
            alignment.Add("validFrames", _alignment == null ? 0 : _alignment.ValidCount);
            alignment.Add("file", _alignment == null ? null : "aligned_frames.csv");
            meta.Add("alignment", alignment);
            if (o.CalibrationInstallation != null)
                meta.Add("calibrationInstallation", o.CalibrationInstallation.ToJson());

            JsonObject timebase = new JsonObject();
            timebase.Add("clock", "Stopwatch/QueryPerformanceCounter");
            timebase.Add("frequencyHz", _tb.Frequency);
            timebase.Add("origin", "recording start");
            timebase.Add("tRelResolutionMs", 1000.0 / _tb.Frequency);
            timebase.Add("utcAnchorPrecisionNote", "DateTime.UtcNow 锚点精度约 1~15 ms；对齐以 t_rel_ms 为准");
            meta.Add("timebase", timebase);

            JsonObject radarObj = new JsonObject();
            bool radarAvailable = radar != null && radar.ReceivedPackets > 0;
            radarObj.Add("requested", o.RadarEnabled);
            radarObj.Add("enabled", radarAvailable);
            radarObj.Add("available", radarAvailable);
            radarObj.Add("source", !o.RadarEnabled ? null : (o.RadarSynthetic ? "synthetic-udp" : "ADT7880"));
            radarObj.Add("startupError", string.IsNullOrEmpty(_radarError) ? null : (object)_radarError);
            radarObj.Add("packetsPerFrame", o.PacketsPerFrame);
            radarObj.Add("payloadBytes", o.PayloadBytes);
            if (radar != null)
            {
                radarObj.Add("frames", radar.FramesSeen);
                radarObj.Add("completeFrames", radar.FramesComplete);
                radarObj.Add("partialFrames", radar.FramesPartial);
                radarObj.Add("missingFrames", radar.FramesEmpty);
                radarObj.Add("receivedPackets", radar.ReceivedPackets);
                radarObj.Add("duplicatePackets", radar.DuplicatePackets);
                radarObj.Add("invalidPackets", radar.InvalidPackets);
                radarObj.Add("latePackets", radar.LatePackets);
                radarObj.Add("missingPackets", radar.MissingPackets);
                radarObj.Add("lossRate", Math.Round(radar.LossRate, 6));
                radarObj.Add("lossRateExcludingTail", Math.Round(radar.LossRateExcludingTail, 6));
                radarObj.Add("lastFrameNumber", radar.LastFrameNumber);
                radarObj.Add("lastFrameReceived", radar.LastFrameReceived);
                radarObj.Add("lastFrameExpected", radar.LastFrameExpected);
                radarObj.Add("fps", duration > 0 ? Math.Round(radar.FramesSeen / duration, 3) : 0);
                radarObj.Add("adcRows", radar.ReceivedPackets);
                radarObj.Add("adcCols", 4);
                radarObj.Add("matFile", radar.MatWritten ? "radar_adc.mat" : null);
                radarObj.Add("matBytes", radar.MatBytes);
                radarObj.Add("packetLogFile", radar.PacketLogRetained ? Path.GetFileName(radar.PacketLogPath) : null);
            }
            else
            {
                radarObj.Add("frames", 0);
                radarObj.Add("receivedPackets", 0);
                radarObj.Add("missingPackets", 0);
                radarObj.Add("lossRate", 0);
                radarObj.Add("matFile", null);
            }
            meta.Add("radar", radarObj);

            LidarCaptureStats lidar = _lidarStats;
            JsonObject lidarObj = new JsonObject();
            bool lidarAvailable = lidar != null && lidar.Batches > 0;
            lidarObj.Add("requested", o.LidarEnabled);
            lidarObj.Add("enabled", lidarAvailable);
            lidarObj.Add("available", lidarAvailable);
            lidarObj.Add("model", !o.LidarEnabled ? null : (!string.IsNullOrEmpty(o.LidarReplayDirectory) ? "Livox binary replay" : (o.LidarSynthetic ? "Livox MID-70 synthetic" : "Livox MID-70")));
            lidarObj.Add("broadcastCode", string.IsNullOrEmpty(o.LidarBroadcastCode) ? null : (object)o.LidarBroadcastCode);
            lidarObj.Add("actualBroadcastCode", string.IsNullOrEmpty(_lidarSerial) ? null : (object)_lidarSerial);
            lidarObj.Add("firmware", string.IsNullOrEmpty(_lidarFirmware) ? null : (object)_lidarFirmware);
            lidarObj.Add("returnMode", o.LidarReturnMode);
            lidarObj.Add("source", !o.LidarEnabled ? null : (!string.IsNullOrEmpty(o.LidarReplayDirectory) ? "replay" : (o.LidarSynthetic ? "synthetic" : "Livox-SDK")));
            lidarObj.Add("batches", lidar == null ? 0 : lidar.Batches);
            lidarObj.Add("points", lidar == null ? 0 : lidar.Points);
            lidarObj.Add("fps", lidar == null ? 0 : Math.Round(lidar.Fps, 3));
            lidarObj.Add("bytes", lidar == null ? 0 : lidar.Bytes);
            lidarObj.Add("dir", lidar != null && lidar.Batches > 0 ? "lidar" : null);
            lidarObj.Add("format", "LIVOXBIN_V3: lossless LivoxRawPoint XYZ int32 millimetres + reflectivity uint8, per-packet device timestamp/type and host QPC/t_rel; V1/V2 replay compatible");
            lidarObj.Add("coordinateFrame", "Livox sensor frame: X forward, Y left, Z up");
            lidarObj.Add("timeSync", o.LidarSynthetic || !string.IsNullOrEmpty(o.LidarReplayDirectory) ? "shared host Stopwatch" : "native callback QPC plus device timestamp/type; PTP/PPS pending hardware validation");
            lidarObj.Add("droppedNativeBatches", lidar == null ? 0 : lidar.DroppedBatches);
            string lidarRuntimeError = lidar == null ? "" : lidar.Error;
            lidarObj.Add("startupError", string.IsNullOrEmpty(_lidarError) ? null : (object)_lidarError);
            lidarObj.Add("error", string.IsNullOrEmpty(lidarRuntimeError) ? null : (object)lidarRuntimeError);
            meta.Add("lidar", lidarObj);

        JsonObject serialObj = new JsonObject();
        serialObj.Add("requested", o.RadarEnabled && !o.RadarSynthetic);
        serialObj.Add("enabled", radarAvailable && !o.RadarSynthetic);
        serialObj.Add("port", _serialPortUsed.Length > 0 ? _serialPortUsed : null);
        serialObj.Add("baudRate", o.SerialBaud);
        serialObj.Add("startSent", _serialStartSent);
        serialObj.Add("pauseSent", _serialPauseSent);
        serialObj.Add("recoveryAttempts", _radarRestartAttempts);
        serialObj.Add("note", "UART 只发控制帧（Start 0xD000 / Pause 0xD002），ADC 数据走 UDP");
        meta.Add("serial", serialObj);

            JsonObject color = new JsonObject();
            color.Add("available", cam != null && cam.ColorFrames > 0);
            color.Add("frames", cam == null ? 0 : cam.ColorFrames);
            color.Add("fps", cam == null ? 0 : Math.Round(cam.ColorFps, 3));
            color.Add("width", cam == null ? 0 : cam.ColorWidth);
            color.Add("height", cam == null ? 0 : cam.ColorHeight);
            color.Add("format", "png24");
            meta.Add("color", color);

            JsonObject depth = new JsonObject();
            bool depthAvailable = cam != null && cam.DepthFrames > 0;
            depth.Add("available", depthAvailable);
            depth.Add("frames", cam == null ? 0 : cam.DepthFrames);
            depth.Add("fps", cam == null ? 0 : Math.Round(cam.DepthFps, 3));
            depth.Add("width", cam == null ? 0 : cam.DepthWidth);
            depth.Add("height", cam == null ? 0 : cam.DepthHeight);
            depth.Add("format", depthAvailable ? "png16" : null);
            depth.Add("depthScaleMetersPerUnit", depthAvailable ? depthScale : 0);
            depth.Add("reason", depthAvailable ? "" : (o.Camera != null && !o.Camera.HasDepth ? "该相机无深度流" : "未收到深度帧"));

            JsonObject cloud = new JsonObject();
            bool cloudWritten = cam != null && cam.CloudFrames > 0;
            cloud.Add("enabled", o.SaveCloud);
            cloud.Add("stride", o.CloudStride);
            cloud.Add("frames", cam == null ? 0 : cam.CloudFrames);
            cloud.Add("points", cam == null ? 0 : cam.CloudPoints);
            cloud.Add("dir", cloudWritten ? "cloud" : null);
            cloud.Add("format", "ply-binary-little-endian");
            cloud.Add("frame", "camera");
            cloud.Add("note", cloudWritten ? "" : (!o.SaveCloud ? "未启用" : "未写出（无可用内参或写入失败，见 warnings）"));
            depth.Add("cloud", cloud);
            meta.Add("depth", depth);

            JsonObject camera = new JsonObject();
            bool cameraRequested = o.Camera != null;
            bool cameraAvailable = cam != null && (cam.ColorFrames > 0 || cam.DepthFrames > 0);
            camera.Add("requested", cameraRequested);
            camera.Add("available", cameraAvailable);
            camera.Add("kind", o.Camera == null ? "None" : o.Camera.Kind.ToString());
            camera.Add("model", o.Camera == null ? "" : o.Camera.Model);
            camera.Add("serial", o.Camera == null ? "" : o.Camera.Serial);
            camera.Add("display", o.Camera == null ? "" : o.Camera.Display);
            camera.Add("backend", o.Camera == null ? "" : o.Camera.Backend);
            camera.Add("activeModes", _cameraModeNote);
            camera.Add("droppedFrames", cam == null ? (object)0 : (object)cam.DroppedFrames);
            camera.Add("startupError", _cameraError.Length > 0 ? (object)_cameraError : null);
            camera.Add("runtimeError", cam != null && !string.IsNullOrEmpty(cam.LastError) ? (object)cam.LastError : null);
            camera.Add("note", !cameraRequested ? "本次未启用相机" : (_cameraError.Length > 0 ? _cameraError : (o.SelfTest ? "self-test 使用合成相机" : "")));

            // 内参：优先用后端实际给到的那一档；没有就按标称值兜底，并写明来源
            DepthIntrinsics intrinsics = _intrinsics;
            if (intrinsics == null || !intrinsics.Valid)
            {
                if (depthAvailable && cam.DepthWidth > 0 && cam.DepthHeight > 0)
                {
                    intrinsics = DepthIntrinsics.Nominal(cam.DepthWidth, cam.DepthHeight,
                        depthScale > 0 ? depthScale : 0.001, DepthIntrinsics.SourceFallback);
                    intrinsics.Detail = "后端未提供 FOV，按标称 58°/45° 推算";
                }
                else
                {
                    intrinsics = null;
                }
            }
            else if (depthScale > 0 && Math.Abs(intrinsics.DepthScale - depthScale) > 1e-12)
            {
                intrinsics = intrinsics.Clone();
                intrinsics.DepthScale = depthScale;
            }
            camera.Add("depthIntrinsics", intrinsics == null ? null : (object)intrinsics.ToJson());
            meta.Add("camera", camera);

            List<object> warnings = new List<object>();
            if (intrinsics != null && intrinsics.Valid && intrinsics.Source != DepthIntrinsics.SourceCalibration)
                warnings.Add("深度内参非标定值（source=" + intrinsics.Source +
                             "），主点按图像中心假设、无畸变模型；2 m 处横向误差可达 2~3 cm。" +
                             "要精确请在 exe 同目录放 calib.json");;
            if (!o.RadarEnabled) warnings.Add("毫米波雷达未启用");
            else if (!radarAvailable)
                warnings.Add("雷达未收到任何数据包，radar_adc.mat 未生成");
            if (_radarError.Length > 0) warnings.Add("雷达启动失败：" + _radarError);
            if (_radarRuntimeWarning) warnings.Add("毫米波雷达记录期间超过 3 秒没有有效 ADC 数据，其它传感器已继续采集");
            if (!cameraRequested) warnings.Add("RGB-D 相机未启用");
            if (_cameraError.Length > 0) warnings.Add("相机启动失败：" + _cameraError);
            if (_cameraRuntimeWarning) warnings.Add("相机记录开始后 3 秒内未完整落盘，其它传感器已继续采集");
            if (cam != null && !string.IsNullOrEmpty(cam.LastError)) warnings.Add("相机采集错误：" + cam.LastError);
            if (o.Camera != null && !o.Camera.HasDepth) warnings.Add("相机没有深度流，depth 目录为空");
            if (radar != null && radar.PacketLogRetained)
                warnings.Add("radar_adc.mat 超过 MAT v5 4 GiB 上限，已保留原始包日志 " + Path.GetFileName(radar.PacketLogPath));
            if (o.SaveCloud && !cloudWritten)
                warnings.Add("勾选了“同时存点云”，但没有写出任何 cloud\\*.ply（无可用内参或写入失败）");
            if (cam != null && cam.DroppedFrames > 0)
                warnings.Add("写图队列丢帧 " + cam.DroppedFrames + " 帧（落盘跟不上，可检查磁盘/杀毒扫描）");
            if (!o.LidarEnabled) warnings.Add("LiDAR 未启用");
            else if (!lidarAvailable)
                warnings.Add("MID-70 真机未采到点云；需要 LivoxBridge.dll、官方 Livox-SDK 与设备联调");
            if (_lidarError.Length > 0) warnings.Add("LiDAR 启动失败：" + _lidarError);
            if (_lidarRuntimeWarning) warnings.Add("LiDAR 记录期间没有点云或发生断流，其它传感器已继续采集");
            if (!string.IsNullOrEmpty(lidarRuntimeError)) warnings.Add("LiDAR 采集错误：" + lidarRuntimeError);
            if (_error.Length > 0) warnings.Add("会话异常：" + _error);
            meta.Add("warnings", warnings);
            meta.Add("error", _error.Length > 0 ? _error : null);
            bool realCamera = o.Camera != null && o.Camera.Kind != CameraKind.Null;
            bool validForCalibration = radarAvailable && lidarAvailable && cameraAvailable &&
                cam.ColorFrames > 0 && cam.DepthFrames > 0 && realCamera && !o.RadarSynthetic &&
                !o.LidarSynthetic && string.IsNullOrEmpty(o.LidarReplayDirectory) && _error.Length == 0 &&
                string.IsNullOrEmpty(_radarError) && string.IsNullOrEmpty(_lidarError) &&
                string.IsNullOrEmpty(_cameraError) && string.IsNullOrEmpty(lidarRuntimeError);
            meta.Add("validForCalibration", validForCalibration);

            if (_alignment != null) _alignment.Write(_take.Directory);
            _take.WriteAll(meta);
            EmitLog("manifest / meta 已写入，共 " + _take.RowCount + " 行");
        }

        private static string DescribeModes(ICameraSource source)
        {
            ICameraModeReporter reporter = source as ICameraModeReporter;
            if (reporter == null) return "";
            try { return reporter.DescribeActiveModes() ?? ""; }
            catch { return ""; }
        }

        public SessionSnapshot Snapshot()
        {
            SessionSnapshot s = new SessionSnapshot();
            s.State = _state;
            s.CountdownRemaining = _countdown;
            s.Message = _message;
            s.Error = _error;
            s.RadarHealth = _radarHealth;
            s.RadarRestarts = _radarRestartAttempts;
            s.TakeDirectory = _takeDirectory;

            Timebase tb = _tb;
            if (tb != null && (s.State == SessionState.Recording || s.State == SessionState.Saving))
            {
                s.ElapsedSeconds = tb.ElapsedSeconds;
                int duration = _options == null ? 0 : _options.DurationSeconds;
                s.RemainingSeconds = Math.Max(0, duration - s.ElapsedSeconds);
            }

            RadarSink radar = _radar;
            if (radar != null)
            {
                RadarSummary summary = radar.LiveSummary();
                s.RadarFrames = summary.FramesSeen;
                s.RadarPackets = summary.ReceivedPackets;
                s.RadarMissing = summary.MissingPackets;
                if (s.ElapsedSeconds > 0) s.RadarFps = summary.FramesSeen / s.ElapsedSeconds;
            }
            else if (_radarSummary != null)
            {
                s.RadarFrames = _radarSummary.FramesSeen;
                s.RadarPackets = _radarSummary.ReceivedPackets;
                s.RadarMissing = _radarSummary.MissingPackets;
            }

            CameraCapture camera = _camera;
            if (camera != null)
            {
                CameraCaptureStats stats = camera.Snapshot();
                s.ColorFrames = stats.ColorFrames;
                s.DepthFrames = stats.DepthFrames;
                s.ColorFps = stats.ColorFps;
                s.DepthFps = stats.DepthFps;
                s.CloudFrames = stats.CloudFrames;
                s.CloudPoints = stats.CloudPoints;
            }
            LidarCapture lidarCapture = _lidar;
            LidarCaptureStats lidarNow = lidarCapture == null ? _lidarStats : lidarCapture.Snapshot();
            if (lidarNow != null)
            {
                s.LidarBatches = lidarNow.Batches;
                s.LidarPoints = lidarNow.Points;
                s.LidarFps = lidarNow.Fps;
                s.LidarHealth = lidarNow.State;
            }
            s.PreviewDropped = _cameraPreview.Overwritten + _radarPreview.Overwritten + _lidarPreview.Overwritten;
            s.PreviewLatencyMs = Math.Max(_cameraPreview.LastTakeLatencyMs,
                Math.Max(_radarPreview.LastTakeLatencyMs, _lidarPreview.LastTakeLatencyMs));
            return s;
        }

        public CameraPreviewFrame TakeCameraPreview() { return _cameraPreview.Take(); }
        public RadarPreviewFrame TakeRadarPreview() { return _radarPreview.Take(); }
        public RadarCloudFrame TakeRadarPointCloudPreview() { return _radarPointCloudPreview.Take(); }
        public LidarBatch TakeLidarPreview() { return _lidarPreview.Take(); }

        private void EmitLog(string text)
        {
            Action<string> handler = Log;
            if (handler == null) return;
            try
            {
                handler(DateTime.Now.ToString("HH:mm:ss.fff") + " " + text);
            }
            catch { }
        }

        public void Dispose()
        {
            Stop();
            Thread worker = _worker;
            if (worker != null && worker.IsAlive) worker.Join(15000);
        }
    }
}
