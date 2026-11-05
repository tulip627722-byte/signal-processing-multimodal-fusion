using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// DirectShow 抓 UVC 彩色（native\ds_capture.dll）。
    /// Astra Pro 的 RGB 是独立 UVC 设备，OpenNI2 拿不到；MF 在本机枚举不到设备，
    /// DirectShow 走 ksproxy 可行（厂家 OrbbecViewer 用的就是这条路径）。
    /// </summary>
    internal sealed class DsColorGrabber : IDisposable
    {
        private const string Dll = "ds_capture.dll";
        private IntPtr _handle = IntPtr.Zero;

        public int Width { get; private set; }
        public int Height { get; private set; }
        public string DeviceName { get; private set; }
        public int FrameBytes { get { return Width * Height * 3; } }

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        private static extern int ds_enum(StringBuilder names, int maxChars, int maxDevices);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        private static extern IntPtr ds_open(string namePart, int w, int h, byte[] err, int errLen);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern int ds_grab(IntPtr handle, byte[] bgr, int bufLen, int timeoutMs);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        private static extern int ds_info(IntPtr handle, out int w, out int h, StringBuilder name, int nameLen);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern void ds_close(IntPtr handle);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        private static extern int ds_serial(IntPtr handle);

        /// <summary>已收到的帧计数（诊断用，不拷贝内存）。</summary>
        public int Serial
        {
            get { return _handle == IntPtr.Zero ? -1 : ds_serial(_handle); }
        }

        /// <summary>列出所有 UVC 视频设备名。</summary>
        public static List<string> ListDevices()
        {
            List<string> list = new List<string>();
            try
            {
                StringBuilder sb = new StringBuilder(4096);
                int n = ds_enum(sb, sb.Capacity, 16);
                if (n <= 0) return list;
                foreach (string s in sb.ToString().Split('\n'))
                {
                    string t = s.Trim();
                    if (t.Length > 0) list.Add(t);
                }
            }
            catch (DllNotFoundException)
            {
                list.Add("（缺 ds_capture.dll）");
            }
            return list;
        }

        public bool Open(string namePart, int width, int height, out string error)
        {
            error = null;
            byte[] err = new byte[512];
            try
            {
                _handle = ds_open(namePart, width, height, err, err.Length);
            }
            catch (DllNotFoundException)
            {
                error = "找不到 ds_capture.dll，请把它放到 exe 同目录";
                return false;
            }
            catch (BadImageFormatException)
            {
                error = "ds_capture.dll 位数不对（需要 x64）";
                return false;
            }
            if (_handle == IntPtr.Zero)
            {
                error = DecodeError(err);
                return false;
            }
            int w, h;
            StringBuilder name = new StringBuilder(256);
            ds_info(_handle, out w, out h, name, name.Capacity);
            Width = w; Height = h; DeviceName = name.ToString();
            return true;
        }

        /// <summary>阻塞取一帧 BGR24（自上而下）。超时返回 null。</summary>
        public byte[] ReadFrame(int timeoutMs)
        {
            if (_handle == IntPtr.Zero) return null;
            int need = FrameBytes;
            if (need <= 0) return null;
            byte[] buf = new byte[need];
            int got = ds_grab(_handle, buf, need, timeoutMs);
            return got == need ? buf : null;
        }

        public void Dispose()
        {
            if (_handle != IntPtr.Zero)
            {
                try { ds_close(_handle); } catch { }
                _handle = IntPtr.Zero;
            }
        }

        private static string DecodeError(byte[] err)
        {
            if (err == null) return "";
            int len = Array.IndexOf(err, (byte)0);
            if (len < 0) len = err.Length;
            return Encoding.UTF8.GetString(err, 0, len);
        }
    }
}
