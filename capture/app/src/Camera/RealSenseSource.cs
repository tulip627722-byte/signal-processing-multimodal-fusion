using System;
using System.Collections.Generic;
using System.Linq.Expressions;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// Intel RealSense 适配器：全程反射加载 Intel.RealSense.dll，编译期零硬依赖。
    ///
    /// 兼容 librealsense2 的 C# 绑定：Context.QueryDevices → Device.Info(CameraInfo)
    /// → Device.QuerySensors → Sensor.Open / Start(VideoStreamer, callback)。
    /// 找不到 SDK / 没有设备 / SDK 版本 API 不同，都会以明确原因报出来，
    /// 绝不让界面以为"相机正常但没数据"。
    ///
    /// 注意：本机无 RealSense 硬件，此路径未做真机验证，接真机前请先看 docs\说明.md。
    /// </summary>
    internal sealed class RealSenseSource : ICameraSource
    {
        private readonly CameraOption _option;
        private readonly Action<string> _log;
        private readonly Func<double> _clock;
        private readonly object _gate = new object();
        private readonly CameraFrame _frame = new CameraFrame();
        private readonly List<object> _sensors = new List<object>();
        private readonly List<object> _streamers = new List<object>();
        private readonly List<Delegate> _callbacks = new List<Delegate>();

        private Assembly _asm;
        private object _context;
        private object _device;
        private byte[] _colorPending;
        private byte[] _colorRead;
        private ushort[] _depthPending;
        private ushort[] _depthRead;
        private int _colorWidth;
        private int _colorHeight;
        private int _depthWidth;
        private int _depthHeight;
        private bool _colorDirty;
        private bool _depthDirty;
        private double _colorRel;
        private double _depthRel;
        private double _depthScale = 0.001;
        private bool _hasColor;
        private bool _hasDepth;
        private bool _opened;
        private int _callbackErrors;

        public RealSenseSource(CameraOption option, Action<string> log, Func<double> clock)
        {
            _option = option;
            _log = log ?? delegate { };
            _clock = clock ?? delegate { return 0; };
            _hasColor = option.HasColor;
            _hasDepth = option.HasDepth;
        }

        public CameraKind Kind { get { return CameraKind.RealSense; } }
        public string Model { get { return _option.Model; } }
        public string Serial { get { return _option.Serial; } }
        public bool HasColor { get { return _hasColor; } }
        public bool HasDepth { get { return _hasDepth; } }
        public double DepthScaleMeters { get { return _depthScale; } }

        // ---------------------------------------------------------------- 探测

        public static List<CameraOption> Probe(Action<string> log, out string reason)
        {
            bool sdkLoaded;
            return Probe(log, out reason, out sdkLoaded);
        }

        public static List<CameraOption> Probe(Action<string> log, out string reason, out bool sdkLoaded)
        {
            reason = null;
            sdkLoaded = false;
            List<CameraOption> found = new List<CameraOption>();
            Assembly asm = LoadAssembly(out reason);
            if (asm == null) return found;
            sdkLoaded = true;

            object context = null;
            try
            {
                Type ctxType = asm.GetType("Intel.RealSense.Context", false);
                if (ctxType == null)
                {
                    sdkLoaded = false;
                    reason = "Intel.RealSense.dll 里没有 Context 类型（不是 librealsense2 的 C# 绑定？）";
                    return found;
                }
                context = Activator.CreateInstance(ctxType);
                object deviceList = DynHelper.CallFirst(context, new[] { "QueryDevices" });
                int index = 0;
                foreach (object device in DynHelper.Enumerate(deviceList))
                {
                    string model = ReadDeviceInfo(asm, device, "Name");
                    string serial = ReadDeviceInfo(asm, device, "SerialNumber");
                    if (string.IsNullOrEmpty(model)) model = "未知型号";
                    bool hasColor, hasDepth;
                    ReadSensorCapability(asm, device, out hasColor, out hasDepth);
                    found.Add(new CameraOption
                    {
                        Kind = CameraKind.RealSense,
                        Display = "RealSense " + model + (string.IsNullOrEmpty(serial) ? "" : "  SN:" + serial),
                        Model = model,
                        Serial = serial,
                        HasColor = hasColor,
                        HasDepth = hasDepth,
                        Available = true,
                        Reason = hasDepth ? "彩色 + 深度" : "只有彩色，深度不可用"
                    });
                    if (log != null) log("RealSense 探测到设备：" + model + " / " + serial);
                    index++;
                }
                if (found.Count == 0 && reason == null)
                    reason = "SDK 已加载，但没有枚举到设备（USB 未插好，或被其他程序占用）";
            }
            catch (Exception ex)
            {
                reason = "RealSense 探测失败：" + DynHelper.DescribeException(ex);
            }
            finally
            {
                DisposeQuietly(context);
            }
            return found;
        }

        private static Assembly LoadAssembly(out string reason)
        {
            reason = null;
            try
            {
                Assembly loaded = Assembly.Load("Intel.RealSense");
                if (loaded != null) return loaded;
            }
            catch { }

            List<string> candidates = new List<string>();
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            candidates.Add(System.IO.Path.Combine(baseDir, "Intel.RealSense.dll"));
            candidates.Add(System.IO.Path.Combine(baseDir, "realsense", "Intel.RealSense.dll"));
            string path = Environment.GetEnvironmentVariable("PATH");
            if (!string.IsNullOrEmpty(path))
            {
                foreach (string dir in path.Split(';'))
                {
                    if (string.IsNullOrEmpty(dir)) continue;
                    candidates.Add(System.IO.Path.Combine(dir, "Intel.RealSense.dll"));
                }
            }
            candidates.Add(@"C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\Intel.RealSense.dll");
            candidates.Add(@"C:\Program Files\Intel RealSense SDK 2.0\bin\x64\Intel.RealSense.dll");

            foreach (string candidate in candidates)
            {
                try
                {
                    if (!System.IO.File.Exists(candidate)) continue;
                    Assembly asm = Assembly.LoadFrom(candidate);
                    if (asm.GetType("Intel.RealSense.Context", false) != null) return asm;
                }
                catch (BadImageFormatException)
                {
                    reason = "找到的 Intel.RealSense.dll 位数不对（需要 x64）：" + candidate;
                }
                catch (Exception ex)
                {
                    reason = "加载 " + candidate + " 失败：" + DynHelper.DescribeException(ex);
                }
            }
            if (reason == null)
                reason = "未找到 Intel.RealSense.dll。请安装 Intel RealSense SDK 2.0（x64），或把 Intel.RealSense.dll 与 realsense2.dll 放到 exe 同目录。";
            return null;
        }

        private static string ReadDeviceInfo(Assembly asm, object device, string member)
        {
            try
            {
                object infoEnum = DynHelper.EnumValue(asm, "Intel.RealSense.CameraInfo", member);
                object value = DynHelper.CallFirst(device, new[] { "Info", "GetInfo" }, infoEnum);
                return value == null ? null : DynHelper.Format(value);
            }
            catch
            {
                object value = DynHelper.GetOptional(device, member);
                return value == null ? null : DynHelper.Format(value);
            }
        }

        private static void ReadSensorCapability(Assembly asm, object device, out bool hasColor, out bool hasDepth)
        {
            hasColor = false;
            hasDepth = false;
            List<string> names = new List<string>();
            try
            {
                object sensors = DynHelper.CallFirst(device, new[] { "QuerySensors" });
                if (sensors == null) sensors = DynHelper.GetOptional(device, "Sensors");
                foreach (object sensor in DynHelper.Enumerate(sensors))
                {
                    string name = ReadSensorName(asm, sensor);
                    names.Add(name ?? "");
                }
            }
            catch
            {
                return;
            }

            foreach (string raw in names)
            {
                string name = (raw ?? "").ToLowerInvariant();
                if (name.IndexOf("rgb") >= 0 || name.IndexOf("color") >= 0) hasColor = true;
                if (name.IndexOf("stereo") >= 0 || name.IndexOf("depth") >= 0) hasDepth = true;
            }
            if (!hasColor && !hasDepth)
            {
                // 读不到传感器名时按常见双流设备兜底
                hasColor = names.Count >= 1;
                hasDepth = names.Count >= 1;
            }
        }

        private static string ReadSensorName(Assembly asm, object sensor)
        {
            try
            {
                object infoEnum = DynHelper.EnumValue(asm, "Intel.RealSense.SensorInfo", "Name");
                object value = DynHelper.CallFirst(sensor, new[] { "Info", "GetInfo" }, infoEnum);
                return value == null ? null : DynHelper.Format(value);
            }
            catch
            {
                object value = DynHelper.GetOptional(sensor, "Name");
                return value == null ? null : DynHelper.Format(value);
            }
        }

        // ---------------------------------------------------------------- 启动

        public void Open()
        {
            if (_opened) return;
            string reason;
            _asm = LoadAssembly(out reason);
            if (_asm == null) throw new InvalidOperationException(reason);

            Type ctxType = _asm.GetType("Intel.RealSense.Context", true);
            _context = Activator.CreateInstance(ctxType);
            _device = FindDevice();

            string errors = "";
            bool colorOk = false;
            bool depthOk = false;
            try
            {
                StartColor();
                colorOk = true;
            }
            catch (Exception ex)
            {
                errors += "彩色流启动失败：" + DynHelper.DescribeException(ex) + "。";
            }
            try
            {
                StartDepth();
                depthOk = true;
            }
            catch (Exception ex)
            {
                errors += "深度流启动失败：" + DynHelper.DescribeException(ex) + "。";
            }
            if (!colorOk && !depthOk)
                throw new InvalidOperationException("RealSense 设备已识别，但两路都没起来。" + errors);

            _hasColor = colorOk;
            _hasDepth = depthOk;
            _opened = true;
            _log("RealSense 已启动：彩色=" + (colorOk ? "可用" : "不可用") +
                 "，深度=" + (depthOk ? "可用" : "不可用") +
                 (depthOk ? "（scale=" + _depthScale.ToString("0.000000") + " m/单位）" : ""));
            if (errors.Length > 0) _log(errors);
        }

        private object FindDevice()
        {
            object deviceList = DynHelper.CallFirst(_context, new[] { "QueryDevices" });
            List<object> devices = new List<object>();
            foreach (object device in DynHelper.Enumerate(deviceList)) devices.Add(device);
            if (devices.Count == 0) throw new InvalidOperationException("没有枚举到 RealSense 设备");

            if (!string.IsNullOrEmpty(_option.Serial))
            {
                foreach (object device in devices)
                {
                    string serial = ReadDeviceInfo(_asm, device, "SerialNumber");
                    if (string.Equals(serial, _option.Serial, StringComparison.Ordinal)) return device;
                }
            }
            return devices[0];
        }

        private List<object> GetSensors()
        {
            object sensors = DynHelper.CallFirst(_device, new[] { "QuerySensors" });
            if (sensors == null) sensors = DynHelper.GetOptional(_device, "Sensors");
            List<object> list = new List<object>();
            foreach (object sensor in DynHelper.Enumerate(sensors)) list.Add(sensor);
            return list;
        }

        private void StartColor()
        {
            object sensor = FindSensor(true);
            Type vsType = RequireType("Intel.RealSense.VideoStreamer");
            object streamer = DynHelper.New(vsType,
                DynHelper.EnumValue(_asm, "Intel.RealSense.Stream", "Color"),
                640, 480,
                DynHelper.EnumValue(_asm, "Intel.RealSense.Format", "Rgb8"),
                30);
            StartSensor(sensor, streamer, false);
        }

        private void StartDepth()
        {
            object sensor = FindSensor(false);
            Type vsType = RequireType("Intel.RealSense.VideoStreamer");
            object streamer = DynHelper.New(vsType,
                DynHelper.EnumValue(_asm, "Intel.RealSense.Stream", "Depth"),
                640, 480,
                DynHelper.EnumValue(_asm, "Intel.RealSense.Format", "Z16"),
                30);
            StartSensor(sensor, streamer, true);
        }

        private object FindSensor(bool color)
        {
            List<object> sensors = GetSensors();
            if (sensors.Count == 0) throw new InvalidOperationException("设备没有可用传感器");
            foreach (object sensor in sensors)
            {
                string name = (ReadSensorName(_asm, sensor) ?? "").ToLowerInvariant();
                bool isColor = name.IndexOf("rgb") >= 0 || name.IndexOf("color") >= 0;
                bool isDepth = name.IndexOf("stereo") >= 0 || name.IndexOf("depth") >= 0;
                if (color && isColor) return sensor;
                if (!color && isDepth) return sensor;
            }
            // 名字读不到时按 D435 常见顺序兜底：0=深度，1=彩色
            if (sensors.Count >= 2) return sensors[color ? 1 : 0];
            if (sensors.Count == 1 && color) return sensors[0];
            throw new InvalidOperationException(color ? "找不到彩色传感器" : "找不到深度传感器（该型号可能只有彩色）");
        }

        private void StartSensor(object sensor, object streamer, bool depth)
        {
            DynHelper.CallFirst(sensor, new[] { "Open" });

            MethodInfo start = null;
            Type delegateType = null;
            Type frameType = null;
            foreach (MethodInfo method in sensor.GetType().GetMethods(BindingFlags.Public | BindingFlags.Instance))
            {
                if (!string.Equals(method.Name, "Start", StringComparison.Ordinal)) continue;
                ParameterInfo[] ps = method.GetParameters();
                if (ps.Length != 2) continue;
                if (!ps[0].ParameterType.IsInstanceOfType(streamer)) continue;
                if (!typeof(Delegate).IsAssignableFrom(ps[1].ParameterType)) continue;
                MethodInfo invoke = ps[1].ParameterType.GetMethod("Invoke");
                if (invoke == null) continue;
                ParameterInfo[] invokeParams = invoke.GetParameters();
                if (invokeParams.Length != 1) continue;
                start = method;
                delegateType = ps[1].ParameterType;
                frameType = invokeParams[0].ParameterType;
                break;
            }
            if (start == null)
                throw new MissingMethodException("当前 SDK 的 Sensor 不支持 Start(VideoStreamer, callback) 形式，请把 Intel.RealSense.dll 版本告知开发者");

            Delegate callback = BuildCallback(delegateType, frameType, depth);
            start.Invoke(sensor, new object[] { streamer, callback });
            _callbacks.Add(callback);
            _streamers.Add(streamer);
            _sensors.Add(sensor);
        }

        private Delegate BuildCallback(Type delegateType, Type frameType, bool depth)
        {
            ParameterExpression parameter = Expression.Parameter(frameType, "frame");
            MethodInfo handler = typeof(RealSenseSource).GetMethod(
                depth ? "OnDepthFrameObject" : "OnColorFrameObject",
                BindingFlags.Instance | BindingFlags.NonPublic);
            Expression body = Expression.Call(
                Expression.Constant(this),
                handler,
                Expression.Convert(parameter, typeof(object)));
            return Expression.Lambda(delegateType, body, parameter).Compile();
        }

        private void OnColorFrameObject(object frame)
        {
            try
            {
                IngestColor(frame);
            }
            catch (Exception ex)
            {
                if (Interlocked.Increment(ref _callbackErrors) <= 5) _log("RealSense 彩色帧处理失败：" + ex.Message);
            }
            finally
            {
                DisposeQuietly(frame);
            }
        }

        private void OnDepthFrameObject(object frame)
        {
            try
            {
                IngestDepth(frame);
            }
            catch (Exception ex)
            {
                if (Interlocked.Increment(ref _callbackErrors) <= 5) _log("RealSense 深度帧处理失败：" + ex.Message);
            }
            finally
            {
                DisposeQuietly(frame);
            }
        }

        private void IngestColor(object frame)
        {
            int width = Convert.ToInt32(DynHelper.Get(frame, "Width"));
            int height = Convert.ToInt32(DynHelper.Get(frame, "Height"));
            IntPtr data = ToIntPtr(DynHelper.Get(frame, "Data"));
            if (data == IntPtr.Zero || width <= 0 || height <= 0) return;
            int stride = ReadIntOptional(frame, "Stride", width * 3);

            bool swap = true;
            object profile = DynHelper.GetOptional(frame, "Profile");
            object format = profile == null ? null : DynHelper.GetOptional(profile, "Format");
            string formatName = format == null ? "" : format.ToString();
            if (formatName.IndexOf("Bgr", StringComparison.OrdinalIgnoreCase) >= 0) swap = false;
            if (formatName.IndexOf("Rgb", StringComparison.OrdinalIgnoreCase) >= 0) swap = true;

            int rowBytes = width * 3;
            byte[] temp = new byte[Math.Max(rowBytes, stride) * height];
            Marshal.Copy(data, temp, 0, temp.Length);
            double rel = _clock();

            lock (_gate)
            {
                int need = rowBytes * height;
                if (_colorPending == null || _colorPending.Length != need) _colorPending = new byte[need];
                for (int y = 0; y < height; y++)
                {
                    int src = y * stride;
                    int dst = y * rowBytes;
                    if (swap)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            _colorPending[dst + x * 3] = temp[src + x * 3 + 2];
                            _colorPending[dst + x * 3 + 1] = temp[src + x * 3 + 1];
                            _colorPending[dst + x * 3 + 2] = temp[src + x * 3];
                        }
                    }
                    else
                    {
                        Buffer.BlockCopy(temp, src, _colorPending, dst, rowBytes);
                    }
                }
                _colorWidth = width;
                _colorHeight = height;
                _colorRel = rel;
                _colorDirty = true;
                Monitor.PulseAll(_gate);
            }
        }

        private void IngestDepth(object frame)
        {
            int width = Convert.ToInt32(DynHelper.Get(frame, "Width"));
            int height = Convert.ToInt32(DynHelper.Get(frame, "Height"));
            IntPtr data = ToIntPtr(DynHelper.Get(frame, "Data"));
            if (data == IntPtr.Zero || width <= 0 || height <= 0) return;
            int stride = ReadIntOptional(frame, "Stride", width * 2);

            object scale = DynHelper.GetOptional(frame, "DepthScale");
            if (scale == null) scale = DynHelper.CallFirst(frame, new[] { "GetDepthScale" });
            if (scale != null)
            {
                double parsed = Convert.ToDouble(scale);
                if (parsed > 0) _depthScale = parsed;
            }

            byte[] temp = new byte[Math.Max(width * 2, stride) * height];
            Marshal.Copy(data, temp, 0, temp.Length);
            double rel = _clock();

            lock (_gate)
            {
                int need = width * height;
                if (_depthPending == null || _depthPending.Length != need) _depthPending = new ushort[need];
                for (int y = 0; y < height; y++)
                {
                    Buffer.BlockCopy(temp, y * stride, _depthPending, y * width * 2, width * 2);
                }
                _depthWidth = width;
                _depthHeight = height;
                _depthRel = rel;
                _depthDirty = true;
                Monitor.PulseAll(_gate);
            }
        }

        private static int ReadIntOptional(object target, string property, int fallback)
        {
            object value = DynHelper.GetOptional(target, property);
            if (value == null) return fallback;
            try { return Convert.ToInt32(value); }
            catch { return fallback; }
        }

        private static IntPtr ToIntPtr(object value)
        {
            if (value is IntPtr) return (IntPtr)value;
            if (value == null) return IntPtr.Zero;
            return new IntPtr(Convert.ToInt64(value));
        }

        // ---------------------------------------------------------------- 取帧

        public CameraFrame ReadFrame(int timeoutMs)
        {
            lock (_gate)
            {
                if (!_colorDirty && !_depthDirty)
                    Monitor.Wait(_gate, Math.Max(1, timeoutMs));
                if (!_colorDirty && !_depthDirty) return null;

                _frame.ColorBgr = null;
                _frame.Depth = null;

                if (_colorDirty)
                {
                    int need = _colorWidth * _colorHeight * 3;
                    if (_colorRead == null || _colorRead.Length != need) _colorRead = new byte[need];
                    Buffer.BlockCopy(_colorPending, 0, _colorRead, 0, need);
                    _frame.ColorBgr = _colorRead;
                    _frame.ColorWidth = _colorWidth;
                    _frame.ColorHeight = _colorHeight;
                    _frame.ColorRelMs = _colorRel;
                    _colorDirty = false;
                }
                if (_depthDirty)
                {
                    int need = _depthWidth * _depthHeight;
                    if (_depthRead == null || _depthRead.Length != need) _depthRead = new ushort[need];
                    Array.Copy(_depthPending, _depthRead, need);
                    _frame.Depth = _depthRead;
                    _frame.DepthWidth = _depthWidth;
                    _frame.DepthHeight = _depthHeight;
                    _frame.DepthRelMs = _depthRel;
                    _frame.DepthScaleMeters = _depthScale;
                    _depthDirty = false;
                }
                return _frame;
            }
        }

        public void Close()
        {
            _opened = false;
            foreach (object sensor in _sensors)
            {
                DynHelper.CallFirst(sensor, new[] { "Stop" });
                DynHelper.CallFirst(sensor, new[] { "Close" });
                DisposeQuietly(sensor);
            }
            _sensors.Clear();
            foreach (object streamer in _streamers) DisposeQuietly(streamer);
            _streamers.Clear();
            _callbacks.Clear();
            DisposeQuietly(_device);
            _device = null;
            DisposeQuietly(_context);
            _context = null;
        }

        public void Dispose()
        {
            try { Close(); } catch { }
        }

        private static void DisposeQuietly(object target)
        {
            if (target == null) return;
            try { DynHelper.CallFirst(target, new[] { "Dispose" }); } catch { }
        }

        private Type RequireType(string name)
        {
            Type type = _asm.GetType(name, false);
            if (type == null) throw new MissingMemberException("Intel.RealSense.dll 缺少类型 " + name + "（SDK 版本不兼容）");
            return type;
        }

        // ---------------------------------------------------------------- 诊断

        /// <summary>给界面/自检用：报告 SDK 与设备探测细节。</summary>
        public static string DescribeProbe(Action<string> log)
        {
            string reason;
            List<CameraOption> options = Probe(log, out reason);
            if (options.Count == 0) return reason;
            return "探测到 " + options.Count + " 台 RealSense 设备";
        }
    }
}
