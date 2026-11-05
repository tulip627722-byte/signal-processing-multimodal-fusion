using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;

namespace Acquire
{
    /// <summary>
    /// Orbbec Astra Pro 适配器（托管包装路线）：运行时反射加载 OpenNI2.Net.dll，
    /// 再经由它 P/Invoke OpenNI2.dll。编译期对这两个 DLL 零引用，csc 只认 .NET Framework 自带程序集。
    ///
    /// 现实情况：官方 OpenNI SDK 2.3.0.86 的 Windows 包里并不带 OpenNI2.Net.dll（只有 OpenNI2.dll +
    /// OpenNI2\Drivers），所以这条路线只有在用户自己补上托管包装时才会走到；补不上时
    /// CameraProbe 会切到 OpenNi2NativeSource（直接绑 C API）。
    ///
    /// 另一件必须说清楚的事：不同来源的托管包装在类型名/方法名上并不完全一致
    /// （Context / VideoStream / VideoFrameRef 大体一致，读取帧既可能是 ReadFrame(ref frame)，
    /// 也可能要靠 NewFrame 事件）。这里按 OpenNI 2 官方 .NET 包装的形状绑定，并对常见别名做兜底；
    /// 若实际包装形状不同，会在打开阶段给出明确原因，而不是安静地没有数据。
    /// </summary>
    internal sealed class OpenNi2ManagedSource : ICameraSource, ICameraModeReporter
    {
        private sealed class StreamContext
        {
            public object Stream;
            public MethodInfo ReadMethod;
            public Type FrameType;
            public bool IsDepth;
        }

        private readonly CameraOption _option;
        private readonly Action<string> _log;
        private readonly Func<double> _clock;
        private readonly object _gate = new object();
        private readonly CameraFrame _frame = new CameraFrame();

        private Assembly _asm;
        private object _context;
        private object _device;
        private StreamContext _depthCtx;
        private StreamContext _colorCtx;
        private Thread _depthThread;
        private Thread _colorThread;
        private volatile bool _stop;

        private byte[] _colorPending;
        private byte[] _colorRead;
        private ushort[] _depthPending;
        private ushort[] _depthRead;
        private byte[] _scratch;
        private int _colorWidth;
        private int _colorHeight;
        private int _depthWidth;
        private int _depthHeight;
        private string _colorFormat = "";
        private string _depthFormat = "";
        private bool _colorDirty;
        private bool _depthDirty;
        private double _colorRel;
        private double _depthRel;
        private double _depthScale = 0.001;
        private bool _hasColor;
        private bool _hasDepth;
        private bool _opened;
        private string _model = "";
        private string _serial = "";
        private string _activeModes = "";
        private int _depthErrors;
        private int _colorErrors;

        public OpenNi2ManagedSource(CameraOption option, Action<string> log, Func<double> clock)
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
        public string DescribeActiveModes() { return _activeModes; }

        // ------------------------------------------------------------ 反射基础设施

        private static Type FindType(Assembly asm, string name)
        {
            Type[] types;
            try { types = asm.GetTypes(); }
            catch (ReflectionTypeLoadException ex) { types = ex.Types; }
            catch { return null; }

            Type fallback = null;
            foreach (Type type in types)
            {
                if (type == null || type.Name != name) continue;
                string ns = type.Namespace ?? "";
                if (ns == "OpenNI2" || ns == "OpenNI" || ns == "OpenNI2.Net") return type;
                if (fallback == null && ns.StartsWith("OpenNI", StringComparison.OrdinalIgnoreCase)) fallback = type;
            }
            return fallback;
        }

        private static MethodInfo FindMethod(Type type, string name, int parameterCount)
        {
            if (type == null) return null;
            foreach (MethodInfo method in type.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static))
            {
                if (!string.Equals(method.Name, name, StringComparison.Ordinal)) continue;
                if (method.GetParameters().Length == parameterCount) return method;
            }
            return null;
        }

        private static object EnumMember(Type enumType, string[] candidates)
        {
            if (enumType == null || !enumType.IsEnum) return null;
            string[] names = Enum.GetNames(enumType);
            foreach (string candidate in candidates)
            {
                foreach (string name in names)
                {
                    if (string.Equals(name, candidate, StringComparison.OrdinalIgnoreCase))
                        return Enum.Parse(enumType, name, true);
                }
            }
            // Orbbec 的包装里常有 DEPTH_SENSOR 之类的别名
            foreach (string candidate in candidates)
            {
                foreach (string name in names)
                {
                    if (name.IndexOf(candidate, StringComparison.OrdinalIgnoreCase) >= 0)
                        return Enum.Parse(enumType, name, true);
                }
            }
            return null;
        }

        private static void TrySetProperty(object target, string[] names, object value)
        {
            if (target == null || value == null) return;
            Type type = target.GetType();
            foreach (string name in names)
            {
                PropertyInfo property = type.GetProperty(name, BindingFlags.Public | BindingFlags.Instance);
                if (property == null || !property.CanWrite || property.PropertyType.IsInstanceOfType(value) == false) continue;
                try
                {
                    property.SetValue(target, value, null);
                    return;
                }
                catch { }
            }
        }

        private static object ReadMember(object target, string[] names)
        {
            if (target == null) return null;
            foreach (string name in names)
            {
                object value = DynHelper.GetOptional(target, name);
                if (value != null) return value;
            }
            return null;
        }

        private static bool IsSuccess(object status)
        {
            if (status == null) return true;
            try
            {
                if (status is bool) return (bool)status;
                int code = Convert.ToInt32(status);
                return code == 0;
            }
            catch { return true; }
        }

        // ------------------------------------------------------------ 探测

        public static List<CameraOption> Probe(OpenNi2Locator locator, Action<string> log, out string error,
                                              out bool initialized)
        {
            error = null;
            initialized = false;
            List<CameraOption> options = new List<CameraOption>();
            object context = null;
            try
            {
                string preload = locator.LoadNative();
                if (preload != null)
                {
                    error = preload;
                    return options;
                }
                Assembly asm = Assembly.LoadFrom(locator.ManagedDll);
                Type contextType = FindType(asm, "Context");
                if (contextType == null)
                {
                    error = locator.ManagedDll + " 里找不到 Context 类型（不是 OpenNI 2 的 .NET 包装？）";
                    return options;
                }
                context = Activator.CreateInstance(contextType);
                DynHelper.CallFirst(context, new[] { "Init", "Initialize" });
                initialized = true;

                List<object> devices = EnumerateDevices(asm, context);
                if (devices.Count == 0)
                {
                    error = "OpenNI2 托管包装已加载，但没有枚举到设备（相机没插好，或被别的程序占用）。";
                    return options;
                }

                foreach (object device in devices)
                {
                    CameraOption option = new CameraOption();
                    option.Kind = CameraKind.OpenNi2;
                    option.Backend = "managed";
                    option.Capabilities = new List<string>();

                    object info = DynHelper.CallFirst(device, new[] { "GetDeviceInfo" });
                    if (info == null) info = ReadMember(device, new[] { "DeviceInfo", "Info" });
                    option.Model = Text(ReadMember(info, new[] { "Name", "Vendor" }));
                    option.Serial = Text(ReadMember(info, new[] { "SerialNumber", "Serial", "Uri" }));
                    option.Address = Text(ReadMember(info, new[] { "Uri", "Address" }));
                    if (string.IsNullOrEmpty(option.Model)) option.Model = "Orbbec 设备";
                    option.Display = "Orbbec " + option.Model +
                                     (string.IsNullOrEmpty(option.Serial) ? "" : "  SN:" + option.Serial);

                    bool hasDepth;
                    bool hasColor;
                    Caps(asm, device, Oni2.SensorDepth, "深度", option.Capabilities, out hasDepth);
                    Caps(asm, device, Oni2.SensorColor, "彩色", option.Capabilities, out hasColor);

                    option.HasDepth = hasDepth;
                    option.HasColor = hasColor;
                    option.Available = hasDepth || hasColor;
                    option.Reason = hasDepth && hasColor ? "彩色 + 深度"
                        : hasDepth ? "只有深度，彩色不可用"
                        : hasColor ? "只有彩色，深度不可用"
                        : "设备已连上但两路传感器都读不到";
                    options.Add(option);
                    if (log != null) log("OpenNI2（托管包装）探测到设备：" + option.Model + " / " + option.Serial);
                }
            }
            catch (Exception ex)
            {
                error = "OpenNI2 托管包装探测失败：" + DynHelper.DescribeException(ex);
            }
            finally
            {
                if (context != null) DynHelper.CallFirst(context, new[] { "Dispose", "Shutdown", "Release" });
            }
            return options;
        }

        private static List<object> EnumerateDevices(Assembly asm, object context)
        {
            List<object> devices = new List<object>();

            object list = DynHelper.CallFirst(context, new[] { "EnumerateDevices", "QueryDevices" });
            if (list != null)
            {
                Type deviceType = FindType(asm, "Device");
                foreach (object item in DynHelper.Enumerate(list))
                {
                    if (item == null) continue;
                    if (deviceType != null && deviceType.IsInstanceOfType(item))
                    {
                        devices.Add(item);
                        continue;
                    }
                    // 拿到的是 DeviceInfo：按 Uri 打开
                    string uri = Text(ReadMember(item, new[] { "Uri", "Address" }));
                    object device = null;
                    if (!string.IsNullOrEmpty(uri))
                        device = DynHelper.CallFirst(context, new[] { "OpenDevice", "OpenDeviceByUri", "Open" }, uri);
                    if (device == null) device = DynHelper.CallFirst(context, new[] { "OpenDevice", "Open", "OpenAny" }, item);
                    if (device != null) devices.Add(device);
                }
                if (devices.Count > 0) return devices;
            }

            object any = DynHelper.CallFirst(context, new[] { "OpenAny" });
            if (any != null) devices.Add(any);
            return devices;
        }

        private static void Caps(Assembly asm, object device, int sensorType, string label,
                                 List<string> target, out bool present)
        {
            present = false;
            try
            {
                object sensor = DynHelper.CallFirst(device, new[] { "GetSensorInfo", "QuerySensorInfo" },
                    EnumMember(FindType(asm, "SensorType"), new[] { SensorName(sensorType) }));
                if (sensor == null)
                {
                    target.Add(label + "：未知（包装未提供传感器查询）");
                    return;
                }
                present = true;
                object modes = ReadMember(sensor, new[] { "SupportedVideoModes", "VideoModes" });
                List<string> parts = new List<string>();
                foreach (object mode in DynHelper.Enumerate(modes))
                {
                    if (parts.Count >= 8) break;
                    object x = ReadMember(mode, new[] { "ResolutionX", "Width" });
                    object y = ReadMember(mode, new[] { "ResolutionY", "Height" });
                    object fps = ReadMember(mode, new[] { "Fps", "FrameRate" });
                    object format = ReadMember(mode, new[] { "PixelFormat", "Format" });
                    parts.Add(Text(x) + "x" + Text(y) + "@" + Text(fps) + " " + Text(format));
                }
                target.Add(parts.Count == 0 ? label + "：有（模式未列出）" : label + "：" + string.Join("；", parts.ToArray()));
            }
            catch (Exception ex)
            {
                target.Add(label + "：查询失败（" + ex.Message + "）");
            }
        }

        private static string SensorName(int sensorType)
        {
            return sensorType == Oni2.SensorDepth ? "DEPTH" : "COLOR";
        }

        private static string Text(object value)
        {
            if (value == null) return "";
            return Convert.ToString(value);
        }

        // ------------------------------------------------------------ 打开

        public void Open()
        {
            if (_opened) return;
            OpenNi2Locator locator = OpenNi2Locator.Locate();
            if (!locator.ManagedFound)
                throw new InvalidOperationException("未找到 OpenNI2.Net.dll，无法使用托管包装路线");
            string preload = locator.LoadNative();
            if (preload != null) throw new InvalidOperationException(preload);
            locator.EnsureDriverRepository(_log);

            _asm = Assembly.LoadFrom(locator.ManagedDll);
            Type contextType = FindType(_asm, "Context");
            if (contextType == null) throw new InvalidOperationException(locator.ManagedDll + " 里找不到 Context 类型");
            Type deviceType = FindType(_asm, "Device");
            if (deviceType == null) throw new InvalidOperationException(locator.ManagedDll + " 里找不到 Device 类型");

            _context = Activator.CreateInstance(contextType);
            DynHelper.CallFirst(_context, new[] { "Init", "Initialize" });

            string uri = _option == null ? "" : _option.Address;
            if (!string.IsNullOrEmpty(uri))
            {
                _device = DynHelper.CallFirst(_context, new[] { "OpenDevice", "OpenDeviceByUri", "Open" }, uri);
                if (_device == null) _device = DynHelper.CallFirst(_context, new[] { "OpenAny" });
            }
            else
            {
                _device = DynHelper.CallFirst(_context, new[] { "OpenAny" });
            }
            if (_device == null) throw new InvalidOperationException("托管包装打不开任何 OpenNI2 设备");

            object info = DynHelper.CallFirst(_device, new[] { "GetDeviceInfo" });
            if (info == null) info = ReadMember(_device, new[] { "DeviceInfo", "Info" });
            string name = Text(ReadMember(info, new[] { "Name" }));
            string serial = Text(ReadMember(info, new[] { "SerialNumber", "Serial" }));
            if (!string.IsNullOrEmpty(name)) _model = name;
            if (!string.IsNullOrEmpty(serial)) _serial = serial;

            string errors = "";
            try { StartStream(Oni2.SensorDepth, true); }
            catch (Exception ex) { _hasDepth = false; errors += "深度流启动失败：" + DynHelper.DescribeException(ex) + "；"; }
            try { StartStream(Oni2.SensorColor, false); }
            catch (Exception ex) { _hasColor = false; errors += "彩色流启动失败：" + DynHelper.DescribeException(ex) + "；"; }

            if (!_hasDepth && !_hasColor)
                throw new InvalidOperationException("托管包装已连上设备，但两路都没起来。" + errors);

            _stop = false;
            if (_hasDepth) { _depthThread = StartThread("Acquire OpenNI2 depth", _depthCtx); }
            if (_hasColor) { _colorThread = StartThread("Acquire OpenNI2 color", _colorCtx); }

            _opened = true;
            _activeModes = (_hasDepth ? "深度 " + _depthFormat : "") +
                           (_hasDepth && _hasColor ? "；" : "") +
                           (_hasColor ? "彩色 " + _colorFormat : "");
            _log("OpenNI2（托管包装）已启动：" + _model + " / " + _serial + "｜" + _activeModes +
                 "｜彩色与深度独立取帧，无硬件同步");
            if (errors.Length > 0) _log(errors);
        }

        private Thread StartThread(string name, StreamContext context)
        {
            Thread thread = new Thread(Loop);
            thread.Name = name;
            thread.IsBackground = true;
            thread.Start(context);
            return thread;
        }

        private void StartStream(int sensorType, bool depth)
        {
            Type streamType = FindType(_asm, "VideoStream");
            if (streamType == null) throw new InvalidOperationException("包装里找不到 VideoStream 类型");

            object stream = Activator.CreateInstance(streamType);
            object sensor = EnumMember(FindType(_asm, "SensorType"), new[] { SensorName(sensorType) });
            if (sensor == null) throw new InvalidOperationException("包装里找不到 SensorType." + SensorName(sensorType));

            object created = DynHelper.CallFirst(stream, new[] { "Create", "Open" }, _device, sensor);
            if (created is bool && !(bool)created)
                throw new InvalidOperationException("VideoStream.Create 返回 false（该设备可能没有这一路）");

            ApplyVideoMode(stream, sensorType == Oni2.SensorDepth);
            object started = DynHelper.CallFirst(stream, new[] { "Start", "StartStream" });
            if (started is bool && !(bool)started) throw new InvalidOperationException("VideoStream.Start 返回 false");

            Type frameType = FindType(_asm, "VideoFrameRef");
            if (frameType == null) frameType = FindType(_asm, "VideoFrame");
            if (frameType == null) throw new InvalidOperationException("包装里找不到 VideoFrameRef 类型");
            MethodInfo read = FindMethod(streamType, "ReadFrame", 1);
            if (read == null) throw new InvalidOperationException("包装的 VideoStream 没有 ReadFrame(ref frame) 方法，当前形状不受支持");

            StreamContext ctx = new StreamContext();
            ctx.Stream = stream;
            ctx.ReadMethod = read;
            ctx.FrameType = frameType;
            ctx.IsDepth = depth;
            ConfigureStream(ctx);

            if (depth) _depthCtx = ctx; else _colorCtx = ctx;
        }

        private void ConfigureStream(StreamContext ctx)
        {
            object mode = ReadMember(ctx.Stream, new[] { "VideoMode", "CurrentVideoMode" });
            string text = "";
            if (mode != null)
            {
                object x = ReadMember(mode, new[] { "ResolutionX", "Width" });
                object y = ReadMember(mode, new[] { "ResolutionY", "Height" });
                object fps = ReadMember(mode, new[] { "Fps", "FrameRate" });
                object format = ReadMember(mode, new[] { "PixelFormat", "Format" });
                text = Text(x) + "x" + Text(y) + "@" + Text(fps) + " " + Text(format);
                if (ctx.IsDepth)
                {
                    string name = Text(format);
                    if (name.IndexOf("100Um", StringComparison.OrdinalIgnoreCase) >= 0) _depthScale = 0.0001;
                    else if (name.IndexOf("1Mm", StringComparison.OrdinalIgnoreCase) >= 0) _depthScale = 0.001;
                }
            }
            if (ctx.IsDepth) _depthFormat = text; else _colorFormat = text;
        }

        private void ApplyVideoMode(object stream, bool depth)
        {
            Type modeType = FindType(_asm, "VideoMode");
            Type pixelType = FindType(_asm, "PixelFormat");
            if (modeType == null || pixelType == null) return;

            object mode;
            try { mode = Activator.CreateInstance(modeType); }
            catch { return; }

            object pixel = depth
                ? EnumMember(pixelType, new[] { "Depth1Mm", "DEPTH_1_MM", "Depth100Um", "DEPTH_100_UM" })
                : EnumMember(pixelType, new[] { "RGB888", "Rgb888", "RGB_888" });
            if (pixel == null) return;

            TrySetProperty(mode, new[] { "ResolutionX", "Width" }, 640);
            TrySetProperty(mode, new[] { "ResolutionY", "Height" }, 480);
            TrySetProperty(mode, new[] { "Fps", "FrameRate" }, 30);
            TrySetProperty(mode, new[] { "PixelFormat", "Format" }, pixel);

            PropertyInfo property = stream.GetType().GetProperty("VideoMode", BindingFlags.Public | BindingFlags.Instance);
            if (property != null && property.CanWrite)
            {
                try
                {
                    property.SetValue(stream, mode, null);
                    return;
                }
                catch { }
            }
            MethodInfo setter = FindMethod(stream.GetType(), "SetVideoMode", 1);
            if (setter != null)
            {
                try { setter.Invoke(stream, new object[] { mode }); } catch { }
            }
            // 设不上就用设备默认模式，实际值稍后从 VideoMode 读回来
        }

        // ------------------------------------------------------------ 取帧线程

        private void Loop(object state)
        {
            StreamContext ctx = (StreamContext)state;
            while (!_stop)
            {
                object frameObject = null;
                try
                {
                    frameObject = Activator.CreateInstance(ctx.FrameType);
                    object[] args = new object[] { frameObject };
                    object status = ctx.ReadMethod.Invoke(ctx.Stream, args);
                    if (args[0] != null) frameObject = args[0];
                    if (!IsSuccess(status))
                    {
                        RecordError(ctx.IsDepth, "ReadFrame 返回非 OK：" + Text(status));
                        Thread.Sleep(2);
                        continue;
                    }
                    Ingest(frameObject, ctx.IsDepth);
                }
                catch (Exception ex)
                {
                    RecordError(ctx.IsDepth, DynHelper.DescribeException(ex));
                    Thread.Sleep(50);
                }
            }
        }

        private void Ingest(object frameObject, bool depth)
        {
            object dataValue = ReadMember(frameObject, new[] { "Data", "DataPtr", "Buffer" });
            IntPtr data = ToIntPtr(dataValue);
            int width = ToInt(ReadMember(frameObject, new[] { "Width" }));
            int height = ToInt(ReadMember(frameObject, new[] { "Height" }));
            if (data == IntPtr.Zero || width <= 0 || height <= 0) return;

            int stride = ToInt(ReadMember(frameObject, new[] { "StrideInBytes", "Stride" }));
            int dataSize = ToInt(ReadMember(frameObject, new[] { "DataSize", "Bytes" }));
            double rel = _clock();

            if (depth)
            {
                int rowBytes = width * 2;
                if (stride < rowBytes) stride = rowBytes;
                if (dataSize > 0 && (long)stride * height > dataSize) stride = rowBytes;
                EnsureScratch(stride * height);
                Marshal.Copy(data, _scratch, 0, stride * height);
                lock (_gate)
                {
                    if (_depthPending == null || _depthPending.Length != width * height) _depthPending = new ushort[width * height];
                    for (int y = 0; y < height; y++)
                        Buffer.BlockCopy(_scratch, y * stride, _depthPending, y * rowBytes, rowBytes);
                    _depthWidth = width;
                    _depthHeight = height;
                    _depthRel = rel;
                    _depthDirty = true;
                    _depthErrors = 0;
                    Monitor.PulseAll(_gate);
                }
                return;
            }

            int bytesPerPixel = 3;
            object format = ReadMember(frameObject, new[] { "PixelFormat", "Format" });
            if (format != null && Text(format).IndexOf("RGB", StringComparison.OrdinalIgnoreCase) < 0) bytesPerPixel = 2;
            int colorRow = width * bytesPerPixel;
            if (stride < colorRow) stride = colorRow;
            if (dataSize > 0 && (long)stride * height > dataSize) stride = colorRow;
            EnsureScratch(stride * height);
            Marshal.Copy(data, _scratch, 0, stride * height);

            lock (_gate)
            {
                int need = width * height * 3;
                if (_colorPending == null || _colorPending.Length != need) _colorPending = new byte[need];
                if (bytesPerPixel == 3)
                {
                    for (int y = 0; y < height; y++)
                    {
                        int src = y * stride;
                        int dst = y * width * 3;
                        for (int x = 0; x < width; x++)
                        {
                            int s = src + x * 3;
                            int d = dst + x * 3;
                            _colorPending[d] = _scratch[s + 2];
                            _colorPending[d + 1] = _scratch[s + 1];
                            _colorPending[d + 2] = _scratch[s];
                        }
                    }
                }
                else
                {
                    for (int y = 0; y < height; y++)
                    {
                        int src = y * stride;
                        int dst = y * width * 3;
                        for (int x = 0; x + 1 < width; x += 2)
                        {
                            int s = src + x * 2;
                            int u = _scratch[s], y1 = _scratch[s + 1], v = _scratch[s + 2], y2 = _scratch[s + 3];
                            WriteBgr(_colorPending, dst + x * 3, y1, u, v);
                            WriteBgr(_colorPending, dst + (x + 1) * 3, y2, u, v);
                        }
                    }
                }
                _colorWidth = width;
                _colorHeight = height;
                _colorRel = rel;
                _colorDirty = true;
                _colorErrors = 0;
                Monitor.PulseAll(_gate);
            }
        }

        private static void WriteBgr(byte[] target, int offset, int y, int u, int v)
        {
            int d = u - 128;
            int e = v - 128;
            int r = y + (91881 * e >> 16);
            int g = y - (22554 * d >> 16) - (46802 * e >> 16);
            int b = y + (116130 * d >> 16);
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

        private void RecordError(bool depth, string message)
        {
            int count = depth ? ++_depthErrors : ++_colorErrors;
            if (count == 3)
                _log((depth ? "深度" : "彩色") + "取帧连续失败：" + message);
            if (count < 60) return;
            if (depth) _hasDepth = false; else _hasColor = false;
            _log((depth ? "深度" : "彩色") + "流连续失败 60 次，已停用该路");
        }

        private static int ToInt(object value)
        {
            if (value == null) return 0;
            try { return Convert.ToInt32(value); }
            catch { return 0; }
        }

        private static IntPtr ToIntPtr(object value)
        {
            if (value is IntPtr) return (IntPtr)value;
            if (value == null) return IntPtr.Zero;
            try { return new IntPtr(Convert.ToInt64(value)); }
            catch { return IntPtr.Zero; }
        }

        // ------------------------------------------------------------ 对外取帧

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

        // ------------------------------------------------------------ 关闭

        public void Close()
        {
            _stop = true;
            _opened = false;
            JoinQuietly(_depthThread);
            JoinQuietly(_colorThread);
            _depthThread = null;
            _colorThread = null;
            StopStream(_depthCtx);
            StopStream(_colorCtx);
            _depthCtx = null;
            _colorCtx = null;
            if (_device != null)
            {
                DynHelper.CallFirst(_device, new[] { "Close", "Dispose", "Release" });
                _device = null;
            }
            if (_context != null)
            {
                DynHelper.CallFirst(_context, new[] { "Dispose", "Shutdown", "Release" });
                _context = null;
            }
            _hasColor = false;
            _hasDepth = false;
        }

        private static void JoinQuietly(Thread thread)
        {
            if (thread == null || !thread.IsAlive) return;
            try { thread.Join(2000); } catch { }
        }

        private void StopStream(StreamContext ctx)
        {
            if (ctx == null || ctx.Stream == null) return;
            DynHelper.CallFirst(ctx.Stream, new[] { "Stop", "StopStream" });
            DynHelper.CallFirst(ctx.Stream, new[] { "Destroy", "Dispose", "Release", "Close" });
            ctx.Stream = null;
        }

        public void Dispose()
        {
            try { Close(); } catch { }
        }
    }
}
