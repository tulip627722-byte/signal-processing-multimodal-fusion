using System;
using System.Runtime.InteropServices;

namespace Acquire
{
    internal sealed class LivoxNativeSource : ILidarSource
    {
        [StructLayout(LayoutKind.Sequential)]
        private struct NativePoint
        {
            public float X;
            public float Y;
            public float Z;
            public byte Reflectivity;
        }

        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern IntPtr livox_create(string broadcastCode);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int livox_start(IntPtr handle);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int livox_poll(IntPtr handle, [Out] NativePoint[] points, int capacity,
            out long deviceTimestampRaw, int timeoutMs);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void livox_stop(IntPtr handle);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void livox_destroy(IntPtr handle);

        private readonly string _broadcastCode;
        private readonly Func<double> _clock;
        private readonly LidarStatus _status = new LidarStatus();
        private readonly NativePoint[] _buffer = new NativePoint[20000];
        private IntPtr _handle;
        private int _sequence;

        public LivoxNativeSource(string broadcastCode, Func<double> clock)
        {
            _broadcastCode = broadcastCode ?? "";
            _clock = clock ?? delegate { return 0; };
        }

        public string Model { get { return "Livox MID-70"; } }
        public string Serial { get { return _broadcastCode; } }
        public LidarStatus Status { get { return _status; } }

        public void Open()
        {
            try
            {
                _handle = livox_create(_broadcastCode);
                if (_handle == IntPtr.Zero || livox_start(_handle) != 0)
                    throw new InvalidOperationException("LivoxBridge 启动失败");
                _status.Connected = true;
                _status.State = "真机运行";
                _status.Detail = _broadcastCode;
            }
            catch (DllNotFoundException)
            {
                throw new InvalidOperationException("缺少 LivoxBridge.dll；请按 native/LivoxBridge/README.md 构建并部署 x64 DLL");
            }
        }

        public LidarBatch ReadBatch(int timeoutMs)
        {
            long timestamp;
            int count = livox_poll(_handle, _buffer, _buffer.Length, out timestamp, timeoutMs);
            if (count < 0) throw new InvalidOperationException("LivoxBridge 读取失败，错误码 " + count);
            if (count == 0) return null;
            LidarPoint[] points = new LidarPoint[count];
            for (int i = 0; i < count; i++)
            {
                points[i].X = _buffer[i].X;
                points[i].Y = _buffer[i].Y;
                points[i].Z = _buffer[i].Z;
                points[i].Reflectivity = _buffer[i].Reflectivity;
            }
            _sequence++;
            return new LidarBatch { DeviceTimestampRaw = timestamp, RelMs = _clock(), Points = points, Count = count, Sequence = _sequence };
        }

        public void Close()
        {
            if (_handle != IntPtr.Zero) livox_stop(_handle);
            _status.Connected = false;
            _status.State = "已停止";
        }

        public void Dispose()
        {
            Close();
            if (_handle != IntPtr.Zero) { livox_destroy(_handle); _handle = IntPtr.Zero; }
        }
    }
}
