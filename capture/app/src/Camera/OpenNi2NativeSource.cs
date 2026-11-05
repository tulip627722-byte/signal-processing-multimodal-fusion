using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// Orbbec Astra Pro 深度/彩色采集，走 OpenNI2.dll 的原生 C API（GetProcAddress 绑符号）。
    ///
    /// Astra Pro 的彩色是独立 UVC 通道、深度是结构光，两路**没有硬件同步**：
    /// 这里用 oniWaitForAnyStream 等“任意一路到帧”，谁到了就只取谁、各自打时间戳，
    /// 绝不假设彩色和深度属于同一时刻。下游 manifest 本来就是按流分开记 t_rel_ms 的。
    ///
    /// 时间戳一律取项目统一的 Stopwatch（注入的 clock），不使用 OniFrame.Timestamp，
    /// 保证和雷达、另一路相机在同一根时间轴上。
    /// </summary>
    internal sealed class OpenNi2NativeSource : ICameraSource, ICameraModeReporter, IDepthIntrinsicsProvider
    {
        private readonly CameraOption _option;
        private readonly Action<string> _log;
        private readonly Func<double> _clock;
        private readonly CameraFrame _frame = new CameraFrame();

        private OpenNi2NativeApi _api;
        private IntPtr _device = IntPtr.Zero;
        private IntPtr _depthStream = IntPtr.Zero;
        private IntPtr _colorStream = IntPtr.Zero;
        private OniVideoMode _depthMode;
        private OniVideoMode _colorMode;

        private bool _hasColor;
        private bool _hasDepth;
        private bool _opened;
        private double _depthScale = 0.001;
        private double? _hfovRadians;
        private double? _vfovRadians;
        private DepthIntrinsics _intrinsics;
        private string _model = "";
        private string _serial = "";
        private string _activeModes = "";

        private ushort[] _depthFrame;
        private byte[] _colorFrame;
        private byte[] _scratch;
        private int _depthFailures;
        private int _colorFailures;
        private bool _loggedDepthStats;

        public OpenNi2NativeSource(CameraOption option, Action<string> log, Func<double> clock)
        {
            _option = option;
            _log = log ?? delegate { };
            _clock = clock ?? delegate { return 0; };
            _model = option == null ? "" : option.Model;
            _serial = option == null ? "" : option.Serial;
        }

        public CameraKind Kind { get { return CameraKind.OpenNi2; } }
        public string Model { get { return _model; } }
        public string Serial { get { return _serial; } }
        public bool HasColor { get { return _hasColor; } }
        public bool HasDepth { get { return _hasDepth; } }
        public double DepthScaleMeters { get { return _depthScale; } }

        public DepthIntrinsics GetDepthIntrinsics()
        {
            return _intrinsics;
        }

        public string DescribeActiveModes()
        {
            return _activeModes;
        }

        // ------------------------------------------------------------ 探测

        /// <summary>枚举设备与能力。不打开流，因此不会占用相机；但会短暂打开设备句柄读传感器信息。</summary>
        public static List<CameraOption> Probe(OpenNi2Locator locator, Action<string> log, out string error,
                                              out bool initialized)
        {
            error = null;
            initialized = false;
            List<CameraOption> options = new List<CameraOption>();
            OpenNi2NativeApi api = null;
            try
            {
                api = OpenNi2NativeApi.Open(locator.NativeDll);
                initialized = true;
                List<OniDeviceInfo> devices = api.ListDevices();
                if (devices.Count == 0)
                {
                    error = "OpenNI2 已初始化（" + locator.Root + "），但没有枚举到设备。" +
                            "请确认相机已插好（建议 USB 3.0 口）、没被 NiViewer 等程序占用。";
                    return options;
                }

                foreach (OniDeviceInfo info in devices)
                {
                    CameraOption option = new CameraOption();
                    option.Kind = CameraKind.OpenNi2;
                    option.Backend = "native";
                    option.Address = info.Uri;
                    option.Model = string.IsNullOrEmpty(info.Name) ? "Orbbec 设备" : info.Name;
                    option.Serial = info.SerialNumber;
                    option.Display = "Orbbec " + option.Model +
                                     (string.IsNullOrEmpty(option.Serial) ? "" : "  SN:" + option.Serial);
                    option.Capabilities = new List<string>();

                    bool hasDepth = false;
                    bool hasColor = false;
                    IntPtr device = IntPtr.Zero;
                    try
                    {
                        device = api.OpenDevice(info.Uri);
                        List<OniVideoMode> depthModes = api.GetSupportedModes(device, Oni2.SensorDepth);
                        List<OniVideoMode> colorModes = api.GetSupportedModes(device, Oni2.SensorColor);
                        hasDepth = depthModes.Count > 0;
                        hasColor = colorModes.Count > 0;
                        AddCapabilities(option.Capabilities, "深度", depthModes);
                        AddCapabilities(option.Capabilities, "彩色", colorModes);
                        string preview = IntrinsicsPreview(api, device, depthModes);
                        if (!string.IsNullOrEmpty(preview) && (hasDepth || depthModes.Count > 0)) option.Capabilities.Add(preview);

                        OniDeviceInfo detailed = api.GetDeviceInfo(device, info);
                        if (!string.IsNullOrEmpty(detailed.SerialNumber)) option.Serial = detailed.SerialNumber;
                        if (!string.IsNullOrEmpty(detailed.Name)) option.Model = detailed.Name;
                        option.Display = "Orbbec " + option.Model +
                                         (string.IsNullOrEmpty(option.Serial) ? "" : "  SN:" + option.Serial);
                    }
                    catch (Exception ex)
                    {
                        option.Capabilities.Add("读取能力失败：" + DynHelper.DescribeException(ex));
                    }
                    finally
                    {
                        if (device != IntPtr.Zero) { try { api.DeviceClose(device); } catch { } }
                    }

                    option.HasDepth = hasDepth;
                    option.HasColor = hasColor;
                    option.Available = hasDepth || hasColor;
                    option.Reason = hasDepth && hasColor ? "彩色 + 深度"
                        : hasDepth ? "只有深度，彩色不可用"
                        : hasColor ? "只有彩色，深度不可用"
                        : "设备已连上但两路传感器都读不到";

                    if (log != null)
                        log("OpenNI2 探测到设备：" + option.Model + " / " + option.Serial +
                            "（" + (hasDepth ? "深度" : "无深度") + "，" + (hasColor ? "彩色" : "无彩色") + "）");
                    options.Add(option);
                }
            }
            catch (Exception ex)
            {
                error = DynHelper.DescribeException(ex);
            }
            finally
            {
                if (api != null) api.Dispose();
            }
            return options;
        }

        /// <summary>
        /// 探测阶段预览内参：临时建一个深度流（不 StartStream）、套上 640x480 深度模式，读 FOV 后立刻销毁。
        /// 读不到 FOV 也不报错，直接说明将以标称值兜底——这一步只是让 --list-cameras 能告诉你取到哪一档。
        /// </summary>
        private static string IntrinsicsPreview(OpenNi2NativeApi api, IntPtr device, List<OniVideoMode> depthModes)
        {
            if (depthModes.Count == 0) return null;
            OniVideoMode preferred = depthModes[0];
            foreach (OniVideoMode mode in depthModes)
            {
                if (mode.ResolutionX == 640 && mode.ResolutionY == 480 && mode.PixelFormat == Oni2.PixelDepth1Mm)
                {
                    preferred = mode;
                    break;
                }
            }

            IntPtr stream = IntPtr.Zero;
            try
            {
                stream = api.CreateStream(device, Oni2.SensorDepth);
                api.TrySetVideoMode(stream, preferred.PixelFormat, preferred.ResolutionX, preferred.ResolutionY, preferred.Fps);
                OniVideoMode actual = api.GetVideoMode(stream);
                if (actual.ResolutionX <= 0 || actual.ResolutionY <= 0) actual = preferred;

                float hfov, vfov;
                bool gotH = api.TryGetFloatProperty(stream, Oni2.StreamPropertyHorizontalFov, out hfov);
                bool gotV = api.TryGetFloatProperty(stream, Oni2.StreamPropertyVerticalFov, out vfov);
                bool fovOk = gotH && gotV && hfov > 0.05f && hfov < 3.0f && vfov > 0.05f && vfov < 3.0f;

                DepthIntrinsics k = DepthIntrinsics.Resolve(
                    actual.ResolutionX, actual.ResolutionY, Oni2.DepthScaleFor(actual.PixelFormat),
                    fovOk ? (double?)hfov : null, fovOk ? (double?)vfov : null,
                    DepthIntrinsics.CalibrationPath, null);
                return "深度内参预览（" + actual.ResolutionX + "x" + actual.ResolutionY + "）：" + k.Describe();
            }
            catch
            {
                return null;
            }
            finally
            {
                if (stream != IntPtr.Zero && api.StreamDestroy != null)
                {
                    try { api.StreamDestroy(stream); } catch { }
                }
            }
        }

        private static void AddCapabilities(List<string> target, string label, List<OniVideoMode> modes)
        {
            if (modes.Count == 0)
            {
                target.Add(label + "：无");
                return;
            }
            StringBuilder sb = new StringBuilder(label + "：");
            int limit = Math.Min(modes.Count, 8);
            for (int i = 0; i < limit; i++)
            {
                OniVideoMode mode = modes[i];
                if (i > 0) sb.Append("；");
                sb.Append(mode.ResolutionX).Append('x').Append(mode.ResolutionY)
                  .Append('@').Append(mode.Fps).Append(' ')
                  .Append(Oni2.FormatName(mode.PixelFormat));
            }
            if (modes.Count > limit) sb.Append("；…共 ").Append(modes.Count).Append(" 种");
            target.Add(sb.ToString());
        }

        // ------------------------------------------------------------ 打开

        public void Open()
        {
            if (_opened) return;

            OpenNi2Locator locator = OpenNi2Locator.Locate();
            if (!locator.NativeFound) throw new InvalidOperationException(locator.DescribeMissing());
            string loadError = locator.LoadNative();
            if (loadError != null) throw new InvalidOperationException(loadError);
            locator.EnsureDriverRepository(_log);

            _api = OpenNi2NativeApi.Open(locator.NativeDll);
            _device = OpenDeviceByAddress(locator);
            OniDeviceInfo info = _api.GetDeviceInfo(_device, new OniDeviceInfo());
            if (!string.IsNullOrEmpty(info.Name)) _model = info.Name;
            if (!string.IsNullOrEmpty(info.SerialNumber)) _serial = info.SerialNumber;

            string errors = "";
            try
            {
                StartDepth();
            }
            catch (Exception ex)
            {
                _hasDepth = false;
                errors += "深度流启动失败：" + DynHelper.DescribeException(ex) + "；";
            }
            try
            {
                StartColor();
            }
            catch (Exception ex)
            {
                _hasColor = false;
                errors += "彩色流启动失败：" + DynHelper.DescribeException(ex) + "；";
            }

            if (!_hasDepth && !_hasColor)
                throw new InvalidOperationException("设备已打开（" + _model + "），但深度和彩色都没起来。" + errors);

            _opened = true;
            StringBuilder modes = new StringBuilder();
            if (_hasDepth) modes.Append("深度 ").Append(Describe(_depthMode));
            if (_hasDepth && _hasColor) modes.Append("；");
            if (_hasColor) modes.Append("彩色 ").Append(Describe(_colorMode));
            _activeModes = modes.ToString();

            _log("OpenNI2 已启动：" + _model + " / " + _serial + "｜" + _activeModes +
                 (_hasDepth ? "｜depth_scale=" + _depthScale.ToString("0.######") + " m/单位" : "") +
                 "｜彩色与深度独立取帧，无硬件同步");
            if (_intrinsics != null && _intrinsics.Valid)
                _log("深度内参：" + _intrinsics.Describe());
            if (errors.Length > 0) _log(errors);
        }

        private IntPtr OpenDeviceByAddress(OpenNi2Locator locator)
        {
            string address = _option == null ? "" : _option.Address;
            if (!string.IsNullOrEmpty(address))
            {
                try { return _api.OpenDevice(address); }
                catch (Exception ex)
                {
                    _log("按 URI 打开设备失败（" + address + "）：" + DynHelper.DescribeException(ex) + "，改试第一台设备");
                }
            }

            List<OniDeviceInfo> devices = _api.ListDevices();
            if (devices.Count == 0)
                throw new InvalidOperationException("没有枚举到 OpenNI2 设备（" + locator.Root + "）。" +
                                                   "请确认相机已插好、没被其它程序占用。");
            return _api.OpenDevice(devices[0].Uri);
        }

        private void StartDepth()
        {
            if (!_api.HasSensor(_device, Oni2.SensorDepth))
            {
                _hasDepth = false;
                throw new InvalidOperationException("该设备没有深度传感器");
            }
            _depthStream = _api.CreateStream(_device, Oni2.SensorDepth);

            // 优先 1 mm 单位：直接就是毫米，后续不用换算
            bool set = _api.TrySetVideoMode(_depthStream, Oni2.PixelDepth1Mm, 640, 480, 30);
            if (!set) set = _api.TrySetVideoMode(_depthStream, Oni2.PixelDepth100Um, 640, 480, 30);
            if (!set) _log("深度 640x480@30 设置失败，改用设备默认模式");

            _depthMode = _api.GetVideoMode(_depthStream);
            if (_depthMode.ResolutionX <= 0 || _depthMode.Fps <= 0)
                throw new InvalidOperationException("读不到深度流的实际模式，驱动可能没配对");
            if (!Oni2.IsDepthFormat(_depthMode.PixelFormat))
                throw new InvalidOperationException(
                    "深度像素格式是 " + Oni2.FormatName(_depthMode.PixelFormat) +
                    "，不是 1 mm / 100 µm 深度值。请在 orbbec.ini 的 [Depth] 里设 OutputFormat=100");

            _depthScale = Oni2.DepthScaleFor(_depthMode.PixelFormat);
            _api.StartStream(_depthStream);
            ReadFov();
            _intrinsics = DepthIntrinsics.Resolve(_depthMode.ResolutionX, _depthMode.ResolutionY, _depthScale,
                _hfovRadians, _vfovRadians, DepthIntrinsics.CalibrationPath, _log);
            _hasDepth = true;
        }

        /// <summary>
        /// 读深度流的水平/垂直视场角（弧度）。Astra Pro 的 orbbec 驱动一般会给，
        /// 但也有驱动返回 NOT_SUPPORTED；那就留空，让 Resolve 退到标称值。
        /// </summary>
        private void ReadFov()
        {
            float hfov, vfov;
            bool gotH = _api.TryGetFloatProperty(_depthStream, Oni2.StreamPropertyHorizontalFov, out hfov);
            bool gotV = _api.TryGetFloatProperty(_depthStream, Oni2.StreamPropertyVerticalFov, out vfov);
            if (gotH && gotV && hfov > 0.05f && hfov < 3.0f && vfov > 0.05f && vfov < 3.0f)
            {
                _hfovRadians = hfov;
                _vfovRadians = vfov;
                return;
            }
            _log("深度流没有上报有效 FOV（hfov=" + hfov.ToString("0.####") + " rad, vfov=" +
                 vfov.ToString("0.####") + " rad），内参改用 calib.json 或标称 58°/45°");
        }

        private void StartColor()
        {
            if (!_api.HasSensor(_device, Oni2.SensorColor))
            {
                _hasColor = false;
                throw new InvalidOperationException("该设备没有彩色传感器（Astra Pro 的 RGB 走 UVC，需驱动支持）");
            }
            _colorStream = _api.CreateStream(_device, Oni2.SensorColor);

            bool set = _api.TrySetVideoMode(_colorStream, Oni2.PixelRgb888, 640, 480, 30);
            if (!set) set = _api.TrySetVideoMode(_colorStream, Oni2.PixelYuv422, 640, 480, 30);
            if (!set) _log("彩色 640x480@30 设置失败，改用设备默认模式");

            _colorMode = _api.GetVideoMode(_colorStream);
            if (_colorMode.ResolutionX <= 0 || _colorMode.Fps <= 0)
                throw new InvalidOperationException("读不到彩色流的实际模式，驱动可能没配对");
            if (!Oni2.IsColorFormat(_colorMode.PixelFormat))
                throw new InvalidOperationException(
                    "彩色像素格式是 " + Oni2.FormatName(_colorMode.PixelFormat) +
                    "，本程序不做 JPEG 解码。请在 orbbec.ini 的 [Image] 里设 OutputFormat=200（RGB888）");

            _api.StartStream(_colorStream);
            _hasColor = true;
        }

        private static string Describe(OniVideoMode mode)
        {
            return mode.ResolutionX + "x" + mode.ResolutionY + "@" + mode.Fps + " " + Oni2.FormatName(mode.PixelFormat);
        }

        // ------------------------------------------------------------ 取帧

        public CameraFrame ReadFrame(int timeoutMs)
        {
            if (!_opened) return null;

            IntPtr[] handles = BuildHandles();
            if (handles.Length == 0) return null;

            int index;
            int status = _api.WaitForAny(handles, out index, Math.Max(1, timeoutMs));
            if (status != Oni2.StatusOk || index < 0 || index >= handles.Length) return null;

            IntPtr stream = handles[index];
            bool isDepth = stream == _depthStream;

            OniFrame frame;
            IntPtr framePointer;
            if (!_api.ReadFrame(stream, out frame, out framePointer))
            {
                RecordFailure(isDepth, "读帧失败");
                return null;
            }

            try
            {
                if (frame.Width <= 0 || frame.Height <= 0 || frame.Width > 8192 || frame.Height > 8192 || frame.Data == IntPtr.Zero)
                {
                    RecordFailure(isDepth, "帧头异常（width/height/data 不合法），可能是 OpenNI2 版本 ABI 不匹配");
                    return null;
                }
                int bytesPerPixel = 2;
                if (!isDepth)
                {
                    int format = frame.VideoMode.PixelFormat;
                    if (!Oni2.IsColorFormat(format)) format = _colorMode.PixelFormat;
                    bytesPerPixel = format == Oni2.PixelRgb888 ? 3 : 2;
                }
                int minimum = frame.Width * bytesPerPixel * frame.Height;
                if (frame.DataSize > 0 && frame.DataSize < minimum)
                {
                    RecordFailure(isDepth, "帧数据长度不足（dataSize=" + frame.DataSize + " < " + minimum +
                                          "），可能是像素格式与预期不符");
                    return null;
                }
                if (isDepth) CopyDepth(frame);
                else CopyColor(frame);
            }
            finally
            {
                _api.ReleaseFrame(framePointer);
            }

            double rel = _clock();
            if (isDepth)
            {
                _depthFailures = 0;
                _frame.DepthRelMs = rel;
            }
            else
            {
                _colorFailures = 0;
                _frame.ColorRelMs = rel;
            }
            return _frame;
        }

        private IntPtr[] BuildHandles()
        {
            if (_hasDepth && _hasColor) return new IntPtr[] { _depthStream, _colorStream };
            if (_hasDepth) return new IntPtr[] { _depthStream };
            if (_hasColor) return new IntPtr[] { _colorStream };
            return new IntPtr[0];
        }

        private void CopyDepth(OniFrame frame)
        {
            int width = frame.Width;
            int height = frame.Height;
            int rowBytes = width * 2;
            int stride = SafeStride(frame.Stride, rowBytes, height, frame.DataSize);
            int total = stride * height;

            EnsureScratch(total);
            Marshal.Copy(frame.Data, _scratch, 0, total);

            if (_depthFrame == null || _depthFrame.Length != width * height) _depthFrame = new ushort[width * height];
            for (int y = 0; y < height; y++)
                Buffer.BlockCopy(_scratch, y * stride, _depthFrame, y * rowBytes, rowBytes);

            _frame.ColorBgr = null;
            _frame.Depth = _depthFrame;
            _frame.DepthWidth = width;
            _frame.DepthHeight = height;
            _frame.DepthScaleMeters = _depthScale;
            _frame.Note = "depth";

            if (!_loggedDepthStats)
            {
                _loggedDepthStats = true;
                int nonzero = 0;
                int min = int.MaxValue;
                int max = 0;
                for (int i = 0; i < _depthFrame.Length; i++)
                {
                    int value = _depthFrame[i];
                    if (value == 0) continue;
                    nonzero++;
                    if (value < min) min = value;
                    if (value > max) max = value;
                }
                int pixelCount = width * height;
                if (nonzero == 0)
                    _log("深度首帧：全部为 0（" + width + "x" + height +
                         "）。请检查镜头遮挡、物体距离（Astra 有效约 0.5~8 m）与红外发射器是否工作");
                else
                    _log("深度首帧：非零 " + (100.0 * nonzero / pixelCount).ToString("0.0") + "%，范围 " +
                         (min * _depthScale * 1000).ToString("0") + "~" + (max * _depthScale * 1000).ToString("0") + " mm");
            }
        }

        private void CopyColor(OniFrame frame)
        {
            int width = frame.Width;
            int height = frame.Height;
            int pixelFormat = frame.VideoMode.PixelFormat;
            if (!Oni2.IsColorFormat(pixelFormat)) pixelFormat = _colorMode.PixelFormat;
            int bytesPerPixel = pixelFormat == Oni2.PixelRgb888 ? 3 : 2;
            int stride = SafeStride(frame.Stride, width * bytesPerPixel, height, frame.DataSize);
            int total = stride * height;

            EnsureScratch(total);
            Marshal.Copy(frame.Data, _scratch, 0, total);

            int need = width * height * 3;
            if (_colorFrame == null || _colorFrame.Length != need) _colorFrame = new byte[need];

            if (pixelFormat == Oni2.PixelRgb888)
            {
                // RGB888 → BGR（GDI+ 的 24bppRgb 实际是 BGR 排列）
                for (int y = 0; y < height; y++)
                {
                    int src = y * stride;
                    int dst = y * width * 3;
                    for (int x = 0; x < width; x++)
                    {
                        int s = src + x * 3;
                        int d = dst + x * 3;
                        _colorFrame[d] = _scratch[s + 2];
                        _colorFrame[d + 1] = _scratch[s + 1];
                        _colorFrame[d + 2] = _scratch[s];
                    }
                }
            }
            else
            {
                // YUV422 打包格式（Astra Pro 的 UVC 通道常给这个）：每 4 字节 2 像素
                bool uyvy = pixelFormat == Oni2.PixelYuv422;
                for (int y = 0; y < height; y++)
                {
                    int src = y * stride;
                    int dst = y * width * 3;
                    for (int x = 0; x + 1 < width; x += 2)
                    {
                        int s = src + x * 2;
                        int u, v, y1, y2;
                        if (uyvy)
                        {
                            u = _scratch[s]; y1 = _scratch[s + 1]; v = _scratch[s + 2]; y2 = _scratch[s + 3];
                        }
                        else
                        {
                            y1 = _scratch[s]; u = _scratch[s + 1]; y2 = _scratch[s + 2]; v = _scratch[s + 3];
                        }
                        WriteBgr(_colorFrame, dst + x * 3, y1, u, v);
                        WriteBgr(_colorFrame, dst + (x + 1) * 3, y2, u, v);
                    }
                }
            }

            _frame.Depth = null;
            _frame.ColorBgr = _colorFrame;
            _frame.ColorWidth = width;
            _frame.ColorHeight = height;
            _frame.Note = "color";
        }

        /// <summary>行跨距保护：驱动给的 stride 不可信时退到紧凑布局，避免越界读。</summary>
        private static int SafeStride(int stride, int rowBytes, int height, int dataSize)
        {
            if (stride < rowBytes) stride = rowBytes;
            if (stride > rowBytes * 4 + 4096) stride = rowBytes;   // 明显不合理的跨距直接不信
            if (dataSize > 0 && (long)stride * height > dataSize) stride = rowBytes;
            return stride;
        }

        private static void WriteBgr(byte[] target, int offset, int y, int u, int v)
        {
            int c = y;
            int d = u - 128;
            int e = v - 128;
            int r = c + (91881 * e >> 16);
            int g = c - (22554 * d >> 16) - (46802 * e >> 16);
            int b = c + (116130 * d >> 16);
            target[offset] = Clamp(b);
            target[offset + 1] = Clamp(g);
            target[offset + 2] = Clamp(r);
        }

        private static byte Clamp(int value)
        {
            if (value < 0) return 0;
            if (value > 255) return 255;
            return (byte)value;
        }

        private void EnsureScratch(int size)
        {
            if (_scratch == null || _scratch.Length < size) _scratch = new byte[size];
        }

        private void RecordFailure(bool depth, string message)
        {
            int count = depth ? ++_depthFailures : ++_colorFailures;
            if (count == 1) _log((depth ? "深度" : "彩色") + message);
            if (count < 30) return;

            // 连续失败太多：把这一路停掉，避免主循环一直被坏流拖住
            if (depth && _hasDepth)
            {
                _hasDepth = false;
                _log("深度流连续 30 次读帧失败，已停用深度（彩色不受影响）");
            }
            else if (!depth && _hasColor)
            {
                _hasColor = false;
                _log("彩色流连续 30 次读帧失败，已停用彩色（深度不受影响）");
            }
        }

        // ------------------------------------------------------------ 关闭

        public void Close()
        {
            _opened = false;
            StopStream(ref _depthStream);
            StopStream(ref _colorStream);
            if (_device != IntPtr.Zero)
            {
                try { _api.DeviceClose(_device); } catch { }
                _device = IntPtr.Zero;
            }
            if (_api != null)
            {
                _api.Dispose();
                _api = null;
            }
            _hasColor = false;
            _hasDepth = false;
            _intrinsics = null;
        }

        private void StopStream(ref IntPtr stream)
        {
            if (stream == IntPtr.Zero || _api == null) return;
            try { if (_api.StreamStop != null) _api.StreamStop(stream); } catch { }
            try { if (_api.StreamDestroy != null) _api.StreamDestroy(stream); } catch { }
            stream = IntPtr.Zero;
        }

        public void Dispose()
        {
            try { Close(); } catch { }
        }
    }
}
