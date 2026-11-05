using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Acquire
{
    /// <summary>OpenNI 2 原生 ABI 的最小镜像，字段与 OniCTypes.h / OniEnums.h 一一对应。</summary>
    internal static class Oni2
    {
        public const int StatusOk = 0;

        public const int SensorIr = 1;
        public const int SensorColor = 2;
        public const int SensorDepth = 3;

        public const int PixelDepth1Mm = 100;
        public const int PixelDepth100Um = 101;
        public const int PixelShift92 = 102;
        public const int PixelShift93 = 103;
        public const int PixelRgb888 = 200;
        public const int PixelYuv422 = 201;
        public const int PixelGray8 = 202;
        public const int PixelGray16 = 203;
        public const int PixelYuyv = 205;

        public const int StreamPropertyVideoMode = 3;
        /// <summary>Orbbec OpenNI2 扩展：读取设备 EEPROM 中的双目标定参数。</summary>
        public const int DevicePropertyCameraParams = 14;
        /// <summary>ONI_STREAM_PROPERTY_HORIZONTAL_FOV，float，弧度（OpenNI 2 的 OniCProperties.h）。</summary>
        public const int StreamPropertyHorizontalFov = 1;
        /// <summary>ONI_STREAM_PROPERTY_VERTICAL_FOV，float，弧度。</summary>
        public const int StreamPropertyVerticalFov = 2;
        public const int MaxStr = 256;

        public static string FormatName(int pixelFormat)
        {
            switch (pixelFormat)
            {
                case PixelDepth1Mm: return "Depth1Mm";
                case PixelDepth100Um: return "Depth100Um";
                case PixelShift92: return "Shift_9_2";
                case PixelShift93: return "Shift_9_3";
                case PixelRgb888: return "RGB888";
                case PixelYuv422: return "YUV422";
                case PixelGray8: return "Gray8";
                case PixelGray16: return "Gray16";
                case PixelYuyv: return "YUYV";
                default: return "format" + pixelFormat;
            }
        }

        public static bool IsDepthFormat(int pixelFormat)
        {
            return pixelFormat == PixelDepth1Mm || pixelFormat == PixelDepth100Um;
        }

        public static bool IsColorFormat(int pixelFormat)
        {
            return pixelFormat == PixelRgb888 || pixelFormat == PixelYuv422 || pixelFormat == PixelYuyv;
        }

        public static double DepthScaleFor(int pixelFormat)
        {
            if (pixelFormat == PixelDepth1Mm) return 0.001;      // 1 单位 = 1 mm
            if (pixelFormat == PixelDepth100Um) return 0.0001;   // 1 单位 = 100 µm
            return 0;
        }
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct OniDeviceInfo
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Oni2.MaxStr)]
        public string Uri;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Oni2.MaxStr)]
        public string Vendor;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Oni2.MaxStr)]
        public string Name;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Oni2.MaxStr)]
        public string SerialNumber;
        public ushort UsbVendorId;
        public ushort UsbProductId;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct OniVideoMode
    {
        public int PixelFormat;
        public int ResolutionX;
        public int ResolutionY;
        public int Fps;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct OniSensorInfo
    {
        public int SensorType;
        public int NumSupportedVideoModes;
        public IntPtr SupportedVideoModes;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct OniFrame
    {
        public int DataSize;
        public IntPtr Data;
        public int SensorType;
        public ulong Timestamp;
        public int FrameIndex;
        public int Width;
        public int Height;
        public OniVideoMode VideoMode;
        public int CroppingEnabled;
        public int CropOriginX;
        public int CropOriginY;
        public int Stride;
    }

    /// <summary>
    /// Orbbec 官方 OniCTypes.h 中的 OBCameraParams。字段均为 float，合计 120 字节。
    /// left 是深度/IR，right 是 RGB；r2l 表示 RGB 到深度的外参。
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct ObCameraParams
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] public float[] LeftIntrinsics;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)] public float[] RightIntrinsics;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)] public float[] RightToLeftRotation;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public float[] RightToLeftTranslation;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public float[] LeftDistortion;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)] public float[] RightDistortion;
    }

    /// <summary>
    /// 直接绑 OpenNI2.dll 的 C API。函数地址用 GetProcAddress 取，不做静态 DllImport，
    /// 这样编译期对 OpenNI2 零依赖，运行期也能精确报出是哪个符号/哪一步失败。
    /// 之所以留这条通道：官方 OpenNI SDK 2.3 的 Windows 包里只有 OpenNI2.dll + OpenNI2\Drivers，
    /// 并不含托管包装 OpenNI2.Net.dll（已对 2.3.0.86 实包核对）。
    /// </summary>
    internal sealed class OpenNi2NativeApi : IDisposable
    {
        internal delegate int InitializeFn(int apiVersion);
        internal delegate void ShutdownFn();
        internal delegate int GetDeviceListFn(out IntPtr devices, out int count);
        internal delegate int ReleaseDeviceListFn(IntPtr devices);
        internal delegate int DeviceOpenFn([MarshalAs(UnmanagedType.LPStr)] string uri, out IntPtr device);
        internal delegate int DeviceCloseFn(IntPtr device);
        internal delegate IntPtr DeviceGetSensorInfoFn(IntPtr device, int sensorType);
        internal delegate int DeviceGetInfoFn(IntPtr device, out OniDeviceInfo info);
        internal delegate int DeviceGetPropertyFn(IntPtr device, int propertyId, IntPtr data, ref int dataSize);
        internal delegate int DeviceCreateStreamFn(IntPtr device, int sensorType, out IntPtr stream);
        internal delegate void StreamDestroyFn(IntPtr stream);
        internal delegate int StreamStartFn(IntPtr stream);
        internal delegate void StreamStopFn(IntPtr stream);
        internal delegate int StreamReadFrameFn(IntPtr stream, out IntPtr frame);
        internal delegate int StreamSetPropertyFn(IntPtr stream, int propertyId, ref OniVideoMode mode, int dataSize);
        internal delegate int StreamGetPropertyFn(IntPtr stream, int propertyId, out OniVideoMode mode, ref int dataSize);
        internal delegate int StreamGetPropertyFloatFn(IntPtr stream, int propertyId, out float value, ref int dataSize);
        internal delegate int WaitForAnyStreamFn(IntPtr[] streams, int streamCount, out int streamIndex, int timeout);
        internal delegate void FrameReleaseFn(IntPtr frame);
        internal delegate IntPtr GetExtendedErrorFn();

        private IntPtr _module;
        private bool _initialized;

        public InitializeFn Initialize;
        public ShutdownFn Shutdown;
        public GetDeviceListFn GetDeviceList;
        public ReleaseDeviceListFn ReleaseDeviceList;
        public DeviceOpenFn DeviceOpen;
        public DeviceCloseFn DeviceClose;
        public DeviceGetSensorInfoFn DeviceGetSensorInfo;
        public DeviceGetInfoFn DeviceGetInfo;
        public DeviceGetPropertyFn DeviceGetProperty;
        public DeviceCreateStreamFn DeviceCreateStream;
        public StreamDestroyFn StreamDestroy;
        public StreamStartFn StreamStart;
        public StreamStopFn StreamStop;
        public StreamReadFrameFn StreamReadFrame;
        public StreamSetPropertyFn StreamSetProperty;
        public StreamGetPropertyFn StreamGetProperty;
        public StreamGetPropertyFloatFn StreamGetPropertyFloat;
        public WaitForAnyStreamFn WaitForAnyStream;
        public FrameReleaseFn FrameRelease;
        public GetExtendedErrorFn GetExtendedError;

        /// <summary>初始化 OpenNI2。失败一律抛 InvalidOperationException，消息可直接给用户看。</summary>
        public static OpenNi2NativeApi Open(string nativeDllPath)
        {
            OpenNi2NativeApi api = new OpenNi2NativeApi();
            api.Bind(nativeDllPath);
            api.InitializeRuntime();
            return api;
        }

        private void Bind(string nativeDllPath)
        {
            string loadError;
            _module = OpenNi2Loader.Load(nativeDllPath, out loadError);
            if (_module == IntPtr.Zero) throw new InvalidOperationException(loadError);

            Initialize = (InitializeFn)BindSymbol<InitializeFn>("oniInitialize", true);
            Shutdown = (ShutdownFn)BindSymbol<ShutdownFn>("oniShutdown", false);
            GetDeviceList = (GetDeviceListFn)BindSymbol<GetDeviceListFn>("oniGetDeviceList", true);
            ReleaseDeviceList = (ReleaseDeviceListFn)BindSymbol<ReleaseDeviceListFn>("oniReleaseDeviceList", false);
            DeviceOpen = (DeviceOpenFn)BindSymbol<DeviceOpenFn>("oniDeviceOpen", true);
            DeviceClose = (DeviceCloseFn)BindSymbol<DeviceCloseFn>("oniDeviceClose", false);
            DeviceGetSensorInfo = (DeviceGetSensorInfoFn)BindSymbol<DeviceGetSensorInfoFn>("oniDeviceGetSensorInfo", true);
            DeviceGetInfo = (DeviceGetInfoFn)BindSymbol<DeviceGetInfoFn>("oniDeviceGetInfo", true);
            DeviceGetProperty = (DeviceGetPropertyFn)BindSymbol<DeviceGetPropertyFn>("oniDeviceGetProperty", true);
            DeviceCreateStream = (DeviceCreateStreamFn)BindSymbol<DeviceCreateStreamFn>("oniDeviceCreateStream", true);
            StreamDestroy = (StreamDestroyFn)BindSymbol<StreamDestroyFn>("oniStreamDestroy", false);
            StreamStart = (StreamStartFn)BindSymbol<StreamStartFn>("oniStreamStart", true);
            StreamStop = (StreamStopFn)BindSymbol<StreamStopFn>("oniStreamStop", false);
            StreamReadFrame = (StreamReadFrameFn)BindSymbol<StreamReadFrameFn>("oniStreamReadFrame", true);
            StreamSetProperty = (StreamSetPropertyFn)BindSymbol<StreamSetPropertyFn>("oniStreamSetProperty", true);
            StreamGetProperty = (StreamGetPropertyFn)BindSymbol<StreamGetPropertyFn>("oniStreamGetProperty", true);
            StreamGetPropertyFloat = (StreamGetPropertyFloatFn)BindSymbol<StreamGetPropertyFloatFn>("oniStreamGetProperty", false);
            WaitForAnyStream = (WaitForAnyStreamFn)BindSymbol<WaitForAnyStreamFn>("oniWaitForAnyStream", true);
            FrameRelease = (FrameReleaseFn)BindSymbol<FrameReleaseFn>("oniFrameRelease", false);
            GetExtendedError = (GetExtendedErrorFn)BindSymbol<GetExtendedErrorFn>("oniGetExtendedError", false);
        }

        private Delegate BindSymbol<T>(string name, bool required)
        {
            IntPtr address = OpenNi2Locator.GetProcAddress(_module, name);
            if (address == IntPtr.Zero)
            {
                if (required)
                    throw new InvalidOperationException("OpenNI2.dll 缺少导出函数 " + name +
                                                        "（版本不兼容，建议用 OpenNI 2.3 x64）");
                return null;
            }
            return Marshal.GetDelegateForFunctionPointer(address, typeof(T));
        }

        private void InitializeRuntime()
        {
            int[] candidates = new int[] { 2003, 2002, 2001 };
            int lastStatus = -1;
            foreach (int version in candidates)
            {
                int status = Initialize(version);
                if (status == Oni2.StatusOk)
                {
                    _initialized = true;
                    return;
                }
                lastStatus = status;
            }
            throw new InvalidOperationException(
                "OpenNI2 初始化失败（oniInitialize 返回 " + lastStatus + "）。" +
                "请确认 Drivers 目录完整（orbbec.dll / orbbec.ini），并用官方 NiViewer.exe 先确认能出深度。" +
                DescribeError());
        }

        public string DescribeError()
        {
            if (GetExtendedError == null) return "";
            try
            {
                IntPtr text = GetExtendedError();
                if (text == IntPtr.Zero) return "";
                string message = Marshal.PtrToStringAnsi(text);
                return string.IsNullOrEmpty(message) ? "" : "（OpenNI2：" + message + "）";
            }
            catch { return ""; }
        }

        // ------------------------------------------------------------ 设备

        public List<OniDeviceInfo> ListDevices()
        {
            List<OniDeviceInfo> devices = new List<OniDeviceInfo>();
            IntPtr array;
            int count;
            int status = GetDeviceList(out array, out count);
            if (status != Oni2.StatusOk || array == IntPtr.Zero || count <= 0) return devices;
            try
            {
                int size = Marshal.SizeOf(typeof(OniDeviceInfo));
                for (int i = 0; i < count; i++)
                {
                    IntPtr item = new IntPtr(array.ToInt64() + i * size);
                    devices.Add((OniDeviceInfo)Marshal.PtrToStructure(item, typeof(OniDeviceInfo)));
                }
            }
            finally
            {
                if (ReleaseDeviceList != null) ReleaseDeviceList(array);
            }
            return devices;
        }

        public IntPtr OpenDevice(string uri)
        {
            IntPtr device;
            int status = DeviceOpen(uri, out device);
            if (status != Oni2.StatusOk || device == IntPtr.Zero)
                throw new InvalidOperationException("打开 OpenNI2 设备失败（" + uri + "），状态 " + status + DescribeError());
            return device;
        }

        public OniDeviceInfo GetDeviceInfo(IntPtr device, OniDeviceInfo fallback)
        {
            OniDeviceInfo info;
            int status = DeviceGetInfo(device, out info);
            if (status != Oni2.StatusOk) return fallback;
            if (string.IsNullOrEmpty(info.Name) && string.IsNullOrEmpty(info.Uri)) return fallback;
            if (string.IsNullOrEmpty(info.SerialNumber)) info.SerialNumber = fallback.SerialNumber;
            if (string.IsNullOrEmpty(info.Name)) info.Name = fallback.Name;
            return info;
        }

        /// <summary>读某个传感器的全部可选模式；传感器不存在返回空表。</summary>
        public List<OniVideoMode> GetSupportedModes(IntPtr device, int sensorType)
        {
            List<OniVideoMode> modes = new List<OniVideoMode>();
            IntPtr pointer = DeviceGetSensorInfo(device, sensorType);
            if (pointer == IntPtr.Zero) return modes;
            OniSensorInfo info = (OniSensorInfo)Marshal.PtrToStructure(pointer, typeof(OniSensorInfo));
            if (info.NumSupportedVideoModes <= 0 || info.SupportedVideoModes == IntPtr.Zero) return modes;
            int size = Marshal.SizeOf(typeof(OniVideoMode));
            for (int i = 0; i < info.NumSupportedVideoModes; i++)
            {
                IntPtr item = new IntPtr(info.SupportedVideoModes.ToInt64() + i * size);
                modes.Add((OniVideoMode)Marshal.PtrToStructure(item, typeof(OniVideoMode)));
            }
            return modes;
        }

        public bool HasSensor(IntPtr device, int sensorType)
        {
            return DeviceGetSensorInfo(device, sensorType) != IntPtr.Zero;
        }

        /// <summary>读取 Orbbec 出厂双目标定。失败时返回驱动状态和扩展错误，不使用标称值兜底。</summary>
        public ObCameraParams GetFactoryCameraParams(IntPtr device)
        {
            int expectedSize = Marshal.SizeOf(typeof(ObCameraParams));
            IntPtr buffer = Marshal.AllocHGlobal(expectedSize);
            try
            {
                for (int i = 0; i < expectedSize; i++) Marshal.WriteByte(buffer, i, 0);
                int actualSize = expectedSize;
                int status = DeviceGetProperty(device, Oni2.DevicePropertyCameraParams, buffer, ref actualSize);
                if (status != Oni2.StatusOk)
                    throw new InvalidOperationException("读取 Orbbec 出厂标定失败，状态 " + status + DescribeError());
                if (actualSize != expectedSize)
                    throw new InvalidOperationException("Orbbec 出厂标定数据长度异常：应为 " + expectedSize +
                                                        " 字节，实际 " + actualSize + " 字节");
                return (ObCameraParams)Marshal.PtrToStructure(buffer, typeof(ObCameraParams));
            }
            finally
            {
                Marshal.FreeHGlobal(buffer);
            }
        }

        // ------------------------------------------------------------ 流

        public IntPtr CreateStream(IntPtr device, int sensorType)
        {
            IntPtr stream;
            int status = DeviceCreateStream(device, sensorType, out stream);
            if (status != Oni2.StatusOk || stream == IntPtr.Zero)
                throw new InvalidOperationException("创建 OpenNI2 流失败（sensorType=" + sensorType + "），状态 " +
                                                    status + DescribeError());
            return stream;
        }

        public bool TrySetVideoMode(IntPtr stream, int pixelFormat, int width, int height, int fps)
        {
            OniVideoMode mode = new OniVideoMode();
            mode.PixelFormat = pixelFormat;
            mode.ResolutionX = width;
            mode.ResolutionY = height;
            mode.Fps = fps;
            try
            {
                return StreamSetProperty(stream, Oni2.StreamPropertyVideoMode, ref mode,
                                         Marshal.SizeOf(typeof(OniVideoMode))) == Oni2.StatusOk;
            }
            catch { return false; }
        }

        public OniVideoMode GetVideoMode(IntPtr stream)
        {
            OniVideoMode mode = new OniVideoMode();
            int size = Marshal.SizeOf(typeof(OniVideoMode));
            try
            {
                int status = StreamGetProperty(stream, Oni2.StreamPropertyVideoMode, out mode, ref size);
                if (status != Oni2.StatusOk) return new OniVideoMode();
            }
            catch { return new OniVideoMode(); }
            return mode;
        }

        /// <summary>
        /// 读 float 类型流属性（FOV 用）。驱动不支持时返回 false，调用方降级到标称值，不当成错误。
        /// </summary>
        public bool TryGetFloatProperty(IntPtr stream, int propertyId, out float value)
        {
            value = 0;
            if (StreamGetPropertyFloat == null) return false;
            int size = 4;
            try
            {
                float result;
                if (StreamGetPropertyFloat(stream, propertyId, out result, ref size) != Oni2.StatusOk) return false;
                value = result;
                return true;
            }
            catch { return false; }
        }

        public void StartStream(IntPtr stream)
        {
            int status = StreamStart(stream);
            if (status != Oni2.StatusOk)
                throw new InvalidOperationException("启动 OpenNI2 流失败，状态 " + status + DescribeError());
        }

        public bool ReadFrame(IntPtr stream, out OniFrame frame, out IntPtr framePointer)
        {
            frame = new OniFrame();
            IntPtr pointer;
            int status = StreamReadFrame(stream, out pointer);
            framePointer = pointer;
            if (status != Oni2.StatusOk || pointer == IntPtr.Zero) return false;
            frame = (OniFrame)Marshal.PtrToStructure(pointer, typeof(OniFrame));
            return true;
        }

        public void ReleaseFrame(IntPtr framePointer)
        {
            if (framePointer != IntPtr.Zero && FrameRelease != null) FrameRelease(framePointer);
        }

        public int WaitForAny(IntPtr[] streams, out int index, int timeoutMs)
        {
            int status = WaitForAnyStream(streams, streams.Length, out index, timeoutMs);
            if (status != Oni2.StatusOk) index = -1;
            return status;
        }

        public void Dispose()
        {
            if (_initialized)
            {
                _initialized = false;
                try { if (Shutdown != null) Shutdown(); } catch { }
            }
            _module = IntPtr.Zero;
        }
    }

    /// <summary>加载原生 OpenNI2.dll，并把 Win32 错误码翻译成用户能动手的原因。</summary>
    internal static class OpenNi2Loader
    {
        private const uint LoadWithAlteredSearchPath = 0x00000008;

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
        private static extern IntPtr LoadLibraryExW(string fileName, IntPtr file, uint flags);

        public static IntPtr Load(string path, out string error)
        {
            error = null;
            if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
            {
                error = "找不到 OpenNI2.dll：" + path;
                return IntPtr.Zero;
            }
            IntPtr module = LoadLibraryExW(path, IntPtr.Zero, LoadWithAlteredSearchPath);
            if (module != IntPtr.Zero) return module;

            int code = Marshal.GetLastWin32Error();
            error = Describe(path, code);
            return IntPtr.Zero;
        }

        private static string Describe(string path, int code)
        {
            string head = "OpenNI2.dll 加载失败（" + path + "），Win32 错误 " + code + "：";
            switch (code)
            {
                case 126:
                    return head + "缺少依赖 DLL。最常见的是 VC++ 2013 x64 运行库（MSVCR120.dll）；" +
                           "装 vcredist_x64 2013 后重试。也可能这份 OpenNI2.dll 旁边缺了配套文件。";
                case 193:
                    return head + "位数不匹配，需要 x64 版 OpenNI2.dll（当前程序是 x64）。";
                case 5:
                    return head + "拒绝访问，可能被杀毒软件拦截，或需要管理员权限。";
                case 87:
                    return head + "参数错误，路径可能过长或含异常字符。";
                default:
                    return head + "请确认 OpenNI2.dll 是 x64 且依赖完整。";
            }
        }
    }
}
