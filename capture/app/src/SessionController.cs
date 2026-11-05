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
        private Thread _serialThread;
        private volatile bool _serialRetryStop;
        private string _serialPortUsed = "";
        private int _serialStartSent;
        private int _serialPauseSent;
        private CameraCapture _camera;
        private CameraOption _cameraOption;
        private DepthIntrinsics _intrinsics;
        private string _cameraError = "";
        private string _cameraModeNote = "";
        private RadarSummary _radarSummary;
        private CameraCaptureStats _cameraStats;
        private DateTime _recordStartUtc;
        private DateTime _recordEndUtc;
        private double _actualSeconds;

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
            if (options.Camera == null) throw new ArgumentNullException("options.Camera");
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
            _cameraModeNote = "";
            _intrinsics = null;
            _takeDirectory = "";
            _radarSummary = null;
            _cameraStats = null;
            _state = SessionState.Countdown;
            _countdown = options.DelaySeconds;
            _message = options.DelaySeconds > 0 ? "倒计时" : "启动中";
            EmitLog("开始任务：延迟 " + options.DelaySeconds + " s，时长 " + options.DurationSeconds + " s，相机=" + options.Camera.Display);

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
                source = CameraProbe.Create(_cameraOption, EmitLog, delegate { return _tb == null ? 0 : _tb.ElapsedMs; });
                try
                {
                    source.Open();
                    _cameraModeNote = DescribeModes(source);
                    EmitLog("相机已打开：" + source.Model + " / " + source.Serial +
                            (source.HasDepth ? "（含深度）" : "（深度不可用）"));

                    IDepthIntrinsicsProvider provider = source as IDepthIntrinsicsProvider;
                    if (provider != null) _intrinsics = provider.GetDepthIntrinsics();
                    if (_intrinsics != null && _intrinsics.Valid)
                    {
                        EmitLog("深度内参：" + _intrinsics.Describe() + "（" + _intrinsics.Detail + "）");
                    }
                    else if (source.HasDepth)
                    {
                        EmitLog("警告：该后端没有提供深度内参，只能采集后用标称 FOV 重算");
                    }
                }
                catch (Exception ex)
                {
                    _cameraError = DynHelper.DescribeException(ex);
                    EmitLog("相机启动失败：" + _cameraError);
                    try { source.Dispose(); } catch { }
                    source = null;
                    if (!o.RadarEnabled)
                        throw new InvalidOperationException("相机不可用且雷达未启用，无法采集。" + _cameraError);
                }

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

                if (o.RadarEnabled)
                {
                    _radar = new RadarSink(_take, _tb, o.PacketsPerFrame, o.PayloadBytes, EmitLog);
                    _radar.Start(o.LocalIp, o.LocalPort, o.BoardIp);
                    StartSerial(o);
                }
                else
                {
                    EmitLog("雷达未启用，本次只录相机");
                }

                if (source != null)
                {
                    _camera = new CameraCapture(source, _take, EmitLog, _intrinsics, o.SaveCloud, o.CloudStride);
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

                _state = SessionState.Recording;
                _message = "记录中";
                // 倒计时结束、彩色/深度/雷达都已经在进数据，这一声才是"真正开始"
                if (cues != null) cues.Play(CueKind.Start);
                Action started = RecordingStarted;
                if (started != null) { try { started(); } catch { } }

                bool warnedSilence = false;
                while (!_stopRequested)
                {
                    double elapsed = _tb.ElapsedSeconds;
                    if (elapsed >= o.DurationSeconds) break;

                    if (!warnedSilence && _radar != null && elapsed > 3.0 && _radar.ReceivedPacketsNow == 0)
                    {
                        warnedSilence = true;
                        EmitLog("警告：已记录 3 秒仍未收到雷达包，请检查雷达 IP/端口与网线");
                        _message = "记录中（雷达无数据）";
                    }
                    Thread.Sleep(25);
                }

                _state = SessionState.Saving;
                _message = "正在落盘";
                if (cues != null) cues.Play(CueKind.End);
                // 先让板子停下来，再停 UDP
                StopSerial();
                // 时长与结束时刻在停止任何采集器之前就固定，避免把 MAT 写盘耗时算进采集时长
                _actualSeconds = _tb.ElapsedSeconds;
                _recordEndUtc = _tb.UtcFromRelMs(_actualSeconds * 1000.0);
                EmitLog("停止采集，开始落盘");

                if (_camera != null)
                {
                    _camera.Stop();
                    _cameraStats = _camera.Snapshot();
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
                if (source != null)
                {
                    try { source.Dispose(); } catch { }
                }
                _state = SessionState.Idle;
                _countdown = 0;
            }
        }

        /// <summary>
        /// 打开 UART 并发 Start。固件需要这条命令才开始往 UDP 发数据，
        /// 所以带一个 500 ms 的重发循环，收到第一个包就停。
        /// </summary>
        private void StartSerial(SessionOptions o)
        {
            _serial = new SerialControl(EmitLog);

            string port = o.SerialPortName;
            string reason;
            if (string.IsNullOrEmpty(port))
            {
                port = SerialControl.AutoDetect(out reason);
                EmitLog("串口自动选择：" + reason);
            }
            if (string.IsNullOrEmpty(port))
            {
                EmitLog("警告：没有可用串口，雷达不会启动（UDP 会一直收不到包）");
                return;
            }

            string err;
            if (!_serial.TryOpen(port, o.SerialBaud, out err))
            {
                EmitLog("警告：串口 " + port + " 打开失败（" + err + "），雷达不会启动");
                return;
            }

            _serialPortUsed = port;
            _serial.SendStart();
            _serialStartSent = _serial.StartSent;

            _serialRetryStop = false;
            _serialThread = new Thread(delegate()
            {
                const int maxAttempts = 20;      // 20 x 500ms = 10s
                for (int i = 1; i <= maxAttempts; i++)
                {
                    for (int w = 0; w < 5 && !_serialRetryStop; w++) Thread.Sleep(100);
                    if (_serialRetryStop) return;
                    RadarSink radar = _radar;
                    if (radar != null && radar.ReceivedPacketsNow > 0)
                    {
                        EmitLog("雷达已回包（" + radar.ReceivedPacketsNow + " 包），停止重发 Start");
                        return;
                    }
                    SerialControl serial = _serial;
                    if (serial == null || !serial.IsOpen) return;
                    serial.SendStart();
                    _serialStartSent = serial.StartSent;
                }
                EmitLog("警告：已重发 Start " + maxAttempts + " 次仍未收到雷达包，请检查串口选择/波特率/网线");
            });
            _serialThread.Name = "Acquire UART start";
            _serialThread.IsBackground = true;
            _serialThread.Start();
        }

        private void StopSerial()
        {
            _serialRetryStop = true;
            SerialControl serial = _serial;
            if (serial != null && serial.IsOpen)
            {
                serial.SendPause();
                _serialPauseSent = serial.PauseSent;
            }
            if (serial != null) serial.Close();
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
                if (_take != null && _tb != null)
                {
                    _actualSeconds = _tb.ElapsedSeconds;
                    _recordEndUtc = _tb.UtcFromRelMs(_actualSeconds * 1000.0);
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

            JsonObject timebase = new JsonObject();
            timebase.Add("clock", "Stopwatch/QueryPerformanceCounter");
            timebase.Add("frequencyHz", _tb.Frequency);
            timebase.Add("origin", "recording start");
            timebase.Add("tRelResolutionMs", 1000.0 / _tb.Frequency);
            timebase.Add("utcAnchorPrecisionNote", "DateTime.UtcNow 锚点精度约 1~15 ms；对齐以 t_rel_ms 为准");
            meta.Add("timebase", timebase);

            JsonObject radarObj = new JsonObject();
            radarObj.Add("enabled", o.RadarEnabled);
            radarObj.Add("packetsPerFrame", o.PacketsPerFrame);
            radarObj.Add("payloadBytes", o.PayloadBytes);
            if (radar != null && o.RadarEnabled)
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

        JsonObject serialObj = new JsonObject();
        serialObj.Add("enabled", o.RadarEnabled);
        serialObj.Add("port", _serialPortUsed.Length > 0 ? _serialPortUsed : null);
        serialObj.Add("baudRate", o.SerialBaud);
        serialObj.Add("startSent", _serialStartSent);
        serialObj.Add("pauseSent", _serialPauseSent);
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
            camera.Add("kind", o.Camera == null ? "None" : o.Camera.Kind.ToString());
            camera.Add("model", o.Camera == null ? "" : o.Camera.Model);
            camera.Add("serial", o.Camera == null ? "" : o.Camera.Serial);
            camera.Add("display", o.Camera == null ? "" : o.Camera.Display);
            camera.Add("backend", o.Camera == null ? "" : o.Camera.Backend);
            camera.Add("activeModes", _cameraModeNote);
            camera.Add("droppedFrames", cam == null ? (object)0 : (object)cam.DroppedFrames);
            camera.Add("note", _cameraError.Length > 0 ? _cameraError : (o.SelfTest ? "self-test 使用合成相机" : ""));

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
            if (radar != null && o.RadarEnabled && radar.ReceivedPackets == 0)
                warnings.Add("雷达未收到任何数据包，radar_adc.mat 未生成");
            if (_cameraError.Length > 0) warnings.Add("相机启动失败：" + _cameraError);
            if (o.Camera != null && !o.Camera.HasDepth) warnings.Add("相机没有深度流，depth 目录为空");
            if (radar != null && radar.PacketLogRetained)
                warnings.Add("radar_adc.mat 超过 MAT v5 4 GiB 上限，已保留原始包日志 " + Path.GetFileName(radar.PacketLogPath));
            if (o.SaveCloud && !cloudWritten)
                warnings.Add("勾选了“同时存点云”，但没有写出任何 cloud\\*.ply（无可用内参或写入失败）");
            if (cam != null && cam.DroppedFrames > 0)
                warnings.Add("写图队列丢帧 " + cam.DroppedFrames + " 帧（落盘跟不上，可检查磁盘/杀毒扫描）");
            if (_error.Length > 0) warnings.Add("会话异常：" + _error);
            meta.Add("warnings", warnings);
            meta.Add("error", _error.Length > 0 ? _error : null);

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
            return s;
        }

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
