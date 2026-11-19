using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Acquire
{
    internal sealed class LivoxNativeSource : ILidarSource
    {
        [StructLayout(LayoutKind.Sequential)]
        private struct NativePoint
        {
            public int XMillimeters;
            public int YMillimeters;
            public int ZMillimeters;
            public byte Reflectivity;
        }

        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern IntPtr livox_create(string broadcastCode);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int livox_start(IntPtr handle);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int livox_poll(IntPtr handle, [Out] NativePoint[] points, int capacity,
            out long deviceTimestampRaw, out byte timestampType, out long hostTimestamp,
            out long droppedBatches, int timeoutMs);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void livox_stop(IntPtr handle);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void livox_destroy(IntPtr handle);
        [DllImport("LivoxBridge.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int livox_get_device_info(IntPtr handle, byte[] broadcastCode, int codeCapacity,
            byte[] firmwareVersion, int firmwareCapacity);

        private readonly string _broadcastCode;
        private readonly Func<long, double> _timestampConverter;
        private readonly LidarStatus _status = new LidarStatus();
        private readonly NativePoint[] _buffer = new NativePoint[20000];
        private IntPtr _handle;
        private int _sequence;

        public LivoxNativeSource(string broadcastCode, Func<long, double> timestampConverter)
        {
            _broadcastCode = broadcastCode ?? "";
            _timestampConverter = timestampConverter ?? delegate { return 0; };
        }

        public string Model { get { return "Livox MID-70"; } }
        public string Serial { get { return string.IsNullOrEmpty(_status.Serial) ? _broadcastCode : _status.Serial; } }
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
            byte timestampType;
            long hostTimestamp;
            long droppedBatches;
            int count = livox_poll(_handle, _buffer, _buffer.Length, out timestamp, out timestampType,
                out hostTimestamp, out droppedBatches, timeoutMs);
            if (count < 0) throw new InvalidOperationException("LivoxBridge 读取失败，错误码 " + count);
            if (count == 0) return null;
            LidarPoint[] points = new LidarPoint[count];
            for (int i = 0; i < count; i++)
            {
                points[i] = LidarPoint.FromMillimeters(_buffer[i].XMillimeters,
                    _buffer[i].YMillimeters, _buffer[i].ZMillimeters, _buffer[i].Reflectivity);
            }
            _sequence++;
            double relMs = _timestampConverter(hostTimestamp);
            _status.DroppedBatches = droppedBatches;
            if (string.IsNullOrEmpty(_status.Serial)) ReadDeviceInfo();
            LidarPacketInfo packet = new LidarPacketInfo { DeviceTimestampRaw = timestamp,
                TimestampType = timestampType, HostTimestamp = hostTimestamp, RelMs = relMs,
                PointStart = 0, PointCount = count };
            return new LidarBatch { DeviceTimestampRaw = timestamp, TimestampType = timestampType,
                HostTimestamp = hostTimestamp, RelMs = relMs, EndRelMs = relMs, Points = points,
                Packets = new[] { packet }, Count = count, PacketCount = 1, Sequence = _sequence };
        }

        private void ReadDeviceInfo()
        {
            byte[] code = new byte[32];
            byte[] firmware = new byte[4];
            if (livox_get_device_info(_handle, code, code.Length, firmware, firmware.Length) != 0) return;
            int length = Array.IndexOf(code, (byte)0);
            if (length < 0) length = code.Length;
            _status.Serial = Encoding.ASCII.GetString(code, 0, length);
            _status.Firmware = firmware[0] + "." + firmware[1].ToString("D2") + "." + firmware[2] + "." + firmware[3];
            _status.Detail = _status.Serial + " / firmware " + _status.Firmware;
            if (firmware[0] < 10 || (firmware[0] == 10 && firmware[1] < 4))
                throw new InvalidOperationException("MID-70 固件 " + _status.Firmware + " 低于 Livox-SDK v2.3.1 支持的最低版本 10.04.0");
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
