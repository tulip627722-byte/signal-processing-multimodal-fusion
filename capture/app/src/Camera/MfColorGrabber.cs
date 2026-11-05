using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Acquire
{
    /// <summary>
    /// 用 Media Foundation 抓 UVC 彩色摄像头（Astra Pro 的 RGB 走独立 UVC 设备，
    /// OpenNI2 拿不到，只能从这里取）。
    ///
    /// 只用这几样：MFStartup / MFEnumDeviceSources / MFCreateDeviceSource /
    /// MFCreateSourceReaderFromMediaSource / ReadSample / IMFMediaBuffer。
    /// 输出统一转成 24 位 BGR（自下而上翻成自上而下），和程序的 CameraFrame 约定一致。
    /// </summary>
    internal sealed class MfColorGrabber : IDisposable
    {
        private IMFSourceReader _reader;
        private IMFMediaSource _source;
        private int _width;
        private int _height;
        private bool _disposed;
        private bool _mfStarted;

        public int Width { get { return _width; } }
        public int Height { get { return _height; } }
        public string DeviceName { get; private set; }

        // ------------------------------------------------------------ 设备枚举

        /// <summary>列出所有视频采集设备（UVC）的友好名。</summary>
        public static List<string> ListDevices(out string error)
        {
            error = null;
            List<string> names = new List<string>();
            bool started = false;
            int startupHr = -1;
            try
            {
                int hr = MFStartup(MF_VERSION, MFSTARTUP_FULL);
                startupHr = hr;
                if (hr < 0) { error = "MFStartup 失败 0x" + hr.ToString("X8"); return names; }
                started = true;

                IMFAttributes attrs = CreateAttributes();
                SetGuid(attrs, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
                IntPtr arrayPtr;
                int count;
                int cnt = -1; attrs.GetCount(out cnt);
                hr = MFEnumDeviceSources(attrs, out arrayPtr, out count);
                if (hr < 0) { error = "MFEnumDeviceSources 失败 0x" + hr.ToString("X8") + "（属性数=" + cnt + "，MFStartup=" + startupHr.ToString("X8") + "）"; return names; }
                try
                {
                    for (int i = 0; i < count; i++)
                    {
                        IntPtr p = Marshal.ReadIntPtr(arrayPtr, i * IntPtr.Size);
                        if (p == IntPtr.Zero) continue;
                        object obj = Marshal.GetObjectForIUnknown(p);
                        IMFActivate activate = obj as IMFActivate;
                        Marshal.Release(p);
                        if (activate == null) continue;
                        string name = GetString(activate, MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME);
                        names.Add(string.IsNullOrEmpty(name) ? ("视频设备 #" + (i + 1)) : name);
                        Marshal.ReleaseComObject(activate);
                    }
                }
                finally
                {
                    if (arrayPtr != IntPtr.Zero) Marshal.FreeCoTaskMem(arrayPtr);
                }
                Marshal.ReleaseComObject(attrs);
            }
            catch (Exception ex)
            {
                error = ex.Message;
            }
            finally
            {
                if (started) { try { MFShutdown(); } catch { } }
            }
            return names;
        }

        // ------------------------------------------------------------ 打开

        /// <summary>按名字片段挑设备并打开；nameContains 为空时取第一台。</summary>
        public bool Open(string nameContains, int width, int height, int fps, Action<string> log)
        {
            if (_disposed) throw new ObjectDisposedException("MfColorGrabber");

            int hr = MFStartup(MF_VERSION, MFSTARTUP_FULL);
            if (hr < 0) { if (log != null) log("MFStartup 失败 0x" + hr.ToString("X8")); return false; }
            _mfStarted = true;

            IMFAttributes attrs = CreateAttributes();
            SetGuid(attrs, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);

            IntPtr arrayPtr;
            int count;
            hr = MFEnumDeviceSources(attrs, out arrayPtr, out count);
            if (hr < 0) { if (log != null) log("枚举设备失败 0x" + hr.ToString("X8")); return false; }

            IMFActivate chosen = null;
            try
            {
                for (int i = 0; i < count; i++)
                {
                    IntPtr p = Marshal.ReadIntPtr(arrayPtr, i * IntPtr.Size);
                    if (p == IntPtr.Zero) continue;
                    IMFActivate activate = Marshal.GetObjectForIUnknown(p) as IMFActivate;
                    Marshal.Release(p);
                    if (activate == null) continue;
                    string name = GetString(activate, MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME);
                    bool match = string.IsNullOrEmpty(nameContains) ||
                                 (name != null && name.IndexOf(nameContains, StringComparison.OrdinalIgnoreCase) >= 0);
                    if (match && chosen == null)
                    {
                        chosen = activate;
                        DeviceName = name;
                        continue;
                    }
                    Marshal.ReleaseComObject(activate);
                }
            }
            finally
            {
                if (arrayPtr != IntPtr.Zero) Marshal.FreeCoTaskMem(arrayPtr);
            }

            if (chosen == null)
            {
                if (log != null) log("没有匹配的 UVC 视频设备（找的是：" + (nameContains ?? "任意") + "）");
                return false;
            }

            object srcObj;
            hr = chosen.ActivateObject(typeof(IMFMediaSource).GUID, out srcObj);
            Marshal.ReleaseComObject(chosen);
            if (hr < 0) { if (log != null) log("激活设备失败 0x" + hr.ToString("X8")); return false; }
            _source = srcObj as IMFMediaSource;
            if (_source == null) { if (log != null) log("设备不是 IMFMediaSource"); return false; }

            hr = MFCreateSourceReaderFromMediaSource(_source, attrs, out _reader);
            if (hr < 0) { if (log != null) log("创建 SourceReader 失败 0x" + hr.ToString("X8")); return false; }

            IMFMediaType type = CreateMediaType();
            IMFAttributes ta = (IMFAttributes)type;
            SetGuid(ta, MF_MT_MAJOR_TYPE, MFMediaType_Video);
            SetGuid(ta, MF_MT_SUBTYPE, MFVideoFormat_RGB32);
            SetU32(ta, MF_MT_INTERLACE_MODE, 2 /* Progressive */);
            SetSize(ta, width, height);
            SetFrameRate(ta, fps, 1);

            hr = _reader.SetCurrentMediaType(FirstVideoStream, IntPtr.Zero, type);
            Marshal.ReleaseComObject(type);
            if (hr < 0)
            {
                if (log != null) log("设置 RGB32 " + width + "x" + height + "@" + fps + " 失败 0x" + hr.ToString("X8") + "，改用设备默认格式");
            }

            IMFMediaType current = null;
            hr = _reader.GetCurrentMediaType(FirstVideoStream, out current);
            if (hr >= 0 && current != null)
            {
                IMFAttributes ca = (IMFAttributes)current;
                long size = GetU64(ca, MF_MT_FRAME_SIZE, -1);
                if (size >= 0)
                {
                    _width = (int)(size >> 32);
                    _height = (int)(size & 0xFFFFFFFF);
                }
                Guid sub = GetGuid(ca, MF_MT_SUBTYPE);
                if (sub != Guid.Empty && sub != MFVideoFormat_RGB32)
                {
                    if (log != null) log("警告：当前格式不是 RGB32（" + sub.ToString() + "），像素转换可能不正确");
                }
                Marshal.ReleaseComObject(current);
            }
            if (_width <= 0 || _height <= 0) { _width = width; _height = height; }

            Marshal.ReleaseComObject(attrs);
            if (log != null) log("UVC 彩色已打开：" + DeviceName + " → " + _width + "x" + _height);
            return true;
        }

        /// <summary>阻塞读一帧，返回 BGR24（自上而下）。超时返回 null。</summary>
        public byte[] ReadFrame(int timeoutMs)
        {
            if (_reader == null) return null;
            int flags, streamIndex;
            long timestamp;
            IMFSample sample;
            int hr = _reader.ReadSample(FirstVideoStream, 0, out streamIndex, out flags, out timestamp, out sample);
            if (hr < 0 || sample == null) return null;
            try
            {
                IMFMediaBuffer buffer;
                hr = sample.ConvertToContiguousBuffer(out buffer);
                if (hr < 0 || buffer == null) return null;
                try
                {
                    IntPtr p;
                    int maxLen, curLen;
                    hr = buffer.Lock(out p, out maxLen, out curLen);
                    if (hr < 0) return null;
                    try
                    {
                        int w = _width, h = _height;
                        int stride = w * 4;
                        int need = stride * h;
                        if (curLen < need) return null;
                        byte[] bgr = new byte[w * h * 3];
                        for (int y = 0; y < h; y++)
                        {
                            // MF 的 RGB32 是自下而上，这里翻成自上而下
                            IntPtr rowSrc = IntPtr.Add(p, (h - 1 - y) * stride);
                            int o = y * w * 3;
                            for (int x = 0; x < w; x++)
                            {
                                bgr[o] = Marshal.ReadByte(rowSrc, x * 4 + 0);      // B
                                bgr[o + 1] = Marshal.ReadByte(rowSrc, x * 4 + 1);  // G
                                bgr[o + 2] = Marshal.ReadByte(rowSrc, x * 4 + 2);  // R
                                o += 3;
                            }
                        }
                        return bgr;
                    }
                    finally { buffer.Unlock(); }
                }
                finally { Marshal.ReleaseComObject(buffer); }
            }
            finally { Marshal.ReleaseComObject(sample); }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (_reader != null) { try { Marshal.ReleaseComObject(_reader); } catch { } _reader = null; }
            if (_source != null) { try { _source.Shutdown(); } catch { } try { Marshal.ReleaseComObject(_source); } catch { } _source = null; }
            if (_mfStarted) { try { MFShutdown(); } catch { } _mfStarted = false; }
        }

        // ------------------------------------------------------------ 小工具

        private const int FirstVideoStream = 0;
        private static readonly Guid MFVideoFormat_RGB32 = new Guid("00000016-0000-0010-8000-00AA00389B71");

        private static IMFAttributes CreateAttributes()
        {
            IMFAttributes attrs;
            int hr = MFCreateAttributes(out attrs, 4);
            if (hr < 0) throw new COMException("MFCreateAttributes 失败", hr);
            return attrs;
        }

        private static IMFMediaType CreateMediaType()
        {
            IMFMediaType type;
            int hr = MFCreateMediaType(out type);
            if (hr < 0) throw new COMException("MFCreateMediaType 失败", hr);
            return type;
        }

        private static void SetSize(IMFAttributes attrs, int w, int h)
        {
            long packed = ((long)w << 32) | (uint)h;
            SetU64(attrs, MF_MT_FRAME_SIZE, packed);
        }

        private static void SetFrameRate(IMFAttributes attrs, int num, int den)
        {
            long packed = ((long)num << 32) | (uint)den;
            SetU64(attrs, MF_MT_FRAME_RATE, packed);
        }

        private static void SetU64(IMFAttributes a, Guid key, long value)
        {
            Guid k = key;
            a.SetUINT64(ref k, value);
        }

        private static string GetString(IMFActivate attrs, Guid key)
        {
            Guid k = key;
            IntPtr p;
            int len;
            if (attrs.GetAllocatedString(ref k, out p, out len) < 0 || p == IntPtr.Zero) return null;
            try { return Marshal.PtrToStringUni(p); }
            finally { Marshal.FreeCoTaskMem(p); }
        }

        private static void SetGuid(IMFAttributes a, Guid key, Guid value)
        {
            Guid k = key, v = value;
            a.SetGUID(ref k, ref v);
        }

        private static void SetU32(IMFAttributes a, Guid key, uint value)
        {
            Guid k = key;
            a.SetUINT32(ref k, value);
        }

        private static long GetU64(IMFAttributes a, Guid key, long fallback)
        {
            Guid k = key;
            long v;
            return a.GetUINT64(ref k, out v) < 0 ? fallback : v;
        }

        private static Guid GetGuid(IMFAttributes a, Guid key)
        {
            Guid k = key, v = Guid.Empty;
            return a.GetGUID(ref k, out v) < 0 ? Guid.Empty : v;
        }

        // ------------------------------------------------------------ P/Invoke

        private const uint MF_VERSION = 0x00020070;
        private const uint MFSTARTUP_FULL = 0;

        private static readonly Guid MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE = new Guid("c60ac5fe-252a-478f-a0ef-bc8fa5f7cad6");
        private static readonly Guid MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID = new Guid("8ac3587a-4ae7-42d8-99e0-0a6013eef90f");
        private static readonly Guid MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME = new Guid("60d0e559-52f8-4fa2-bbce-acdb8c772500");
        private static readonly Guid MF_MT_MAJOR_TYPE = new Guid("48eba18e-f8c9-4687-bf11-0a74c9f96a8f");
        private static readonly Guid MF_MT_SUBTYPE = new Guid("f7e34c9a-42e8-4714-b74b-cb29d72c35e5");
        private static readonly Guid MF_MT_FRAME_SIZE = new Guid("1652c33d-d6b2-4012-b834-72030849a37d");
        private static readonly Guid MF_MT_FRAME_RATE = new Guid("c459a2e8-3d2c-4e44-b132-fee5156c7bb0");
        private static readonly Guid MF_MT_INTERLACE_MODE = new Guid("e2724bb8-e676-4806-b4b2-a8d6efb44ccd");
        private static readonly Guid MFMediaType_Video = new Guid("73646976-0000-0010-8000-00AA00389B71");

        [DllImport("mfplat.dll", ExactSpelling = true)]
        private static extern int MFStartup(uint version, uint flags);
        [DllImport("mfplat.dll", ExactSpelling = true)]
        private static extern int MFShutdown();
        [DllImport("mfplat.dll", ExactSpelling = true)]
        private static extern int MFCreateAttributes(out IMFAttributes attributes, int initialSize);
        [DllImport("mfplat.dll", ExactSpelling = true)]
        private static extern int MFCreateMediaType(out IMFMediaType type);
        [DllImport("mfreadwrite.dll", ExactSpelling = true)]
        private static extern int MFCreateSourceReaderFromMediaSource(IMFMediaSource source, IMFAttributes attributes, out IMFSourceReader reader);
        [DllImport("mf.dll", ExactSpelling = true)]
        private static extern int MFEnumDeviceSources(IMFAttributes attributes, out IntPtr devices, out int count);

        // ------------------------------------------------------------ COM 接口

        [ComImport, Guid("2cd2d921-c447-44a7-a13c-4adabfc247e3"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFAttributes
        {
            int GetItem(ref Guid key, IntPtr value);
            int GetItemType(ref Guid key, out int type);
            int CompareItem(ref Guid key, IntPtr value, out int result);
            int Compare(IMFAttributes theirs, int matchType, out int result);
            int GetUINT32(ref Guid key, out uint value);
            int GetUINT64(ref Guid key, out long value);
            int GetDouble(ref Guid key, out double value);
            int GetGUID(ref Guid key, out Guid value);
            int GetStringLength(ref Guid key, out int length);
            int GetString(ref Guid key, [MarshalAs(UnmanagedType.LPWStr)] System.Text.StringBuilder value, int size, out int length);
            int GetAllocatedString(ref Guid key, out IntPtr value, out int length);
            int GetBlobSize(ref Guid key, out int size);
            int GetBlob(ref Guid key, IntPtr buffer, int bufferSize, out int blobSize);
            int GetAllocatedBlob(ref Guid key, out IntPtr buffer, out int size);
            int GetUnknown(ref Guid key, ref Guid riid, out IntPtr ppv);
            int SetItem(ref Guid key, IntPtr value);
            int DeleteItem(ref Guid key);
            int DeleteAllItems();
            int SetUINT32(ref Guid key, uint value);
            int SetUINT64(ref Guid key, long value);
            int SetDouble(ref Guid key, double value);
            int SetGUID(ref Guid key, ref Guid value);
            int SetString(ref Guid key, [MarshalAs(UnmanagedType.LPWStr)] string value);
            int SetBlob(ref Guid key, IntPtr buffer, int size);
            int SetUnknown(ref Guid key, [MarshalAs(UnmanagedType.IUnknown)] object unknown);
            int LockStore();
            int UnlockStore();
            int GetCount(out int count);
            int GetItemByIndex(int index, out Guid key, IntPtr value);
            int CopyAllItems(IMFAttributes dest);
        }

        [ComImport, Guid("44ae0fa8-ea31-4109-8d2e-4cae4997c555"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFMediaType { }

        [ComImport, Guid("279a808d-aec7-40c8-9c6b-a6b492c78a66"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFMediaSource
        {
            // IMFMediaEventGenerator
            int GetEvent(int flags, out IntPtr evt);
            int BeginGetEvent(IntPtr callback, IntPtr state);
            int EndGetEvent(IntPtr evt, out IntPtr result);
            int QueueEvent(IntPtr evt);
            // IMFMediaSource
            int CreatePresentationDescriptor(out IntPtr descriptor);
            int Start(IntPtr descriptor, IntPtr format);
            int Stop();
            int Pause();
            int Shutdown();
        }

        [ComImport, Guid("7fee9e9a-4a89-47a6-899c-b6a53a70fb67"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFActivate
        {
            // IMFAttributes（只声明到 SetGUID 之前的读取方法，vtable 顺序必须完整）
            int GetItem(ref Guid key, IntPtr value);
            int GetItemType(ref Guid key, out int type);
            int CompareItem(ref Guid key, IntPtr value, out int result);
            int Compare(IntPtr theirs, int matchType, out int result);
            int GetUINT32(ref Guid key, out uint value);
            int GetUINT64(ref Guid key, out long value);
            int GetDouble(ref Guid key, out double value);
            int GetGUID(ref Guid key, out Guid value);
            int GetStringLength(ref Guid key, out int length);
            int GetString(ref Guid key, [MarshalAs(UnmanagedType.LPWStr)] System.Text.StringBuilder value, int size, out int length);
            int GetAllocatedString(ref Guid key, out IntPtr value, out int length);
            int GetBlobSize(ref Guid key, out int size);
            int GetBlob(ref Guid key, IntPtr buffer, int bufferSize, out int blobSize);
            int GetAllocatedBlob(ref Guid key, out IntPtr buffer, out int size);
            int GetUnknown(ref Guid key, ref Guid riid, out IntPtr ppv);
            int SetItem(ref Guid key, IntPtr value);
            int DeleteItem(ref Guid key);
            int DeleteAllItems();
            int SetUINT32(ref Guid key, uint value);
            int SetUINT64(ref Guid key, long value);
            int SetDouble(ref Guid key, double value);
            int SetGUID(ref Guid key, ref Guid value);
            int SetString(ref Guid key, [MarshalAs(UnmanagedType.LPWStr)] string value);
            int SetBlob(ref Guid key, IntPtr buffer, int size);
            int SetUnknown(ref Guid key, [MarshalAs(UnmanagedType.IUnknown)] object unknown);
            int LockStore();
            int UnlockStore();
            int GetCount(out int count);
            int GetItemByIndex(int index, out Guid key, IntPtr value);
            int CopyAllItems(IntPtr dest);
            // IMFActivate
            int ActivateObject(Guid riid, out object ppv);
            int ShutdownObject();
            int DetachObject();
        }

        [ComImport, Guid("70ae66f2-c809-4e4f-8915-bdcb406b7993"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFSourceReader
        {
            int GetStreamSelection(int index, out int selected);
            int SetStreamSelection(int index, int selected);
            int GetNativeMediaType(int index, int typeIndex, out IMFMediaType type);
            int GetCurrentMediaType(int index, out IMFMediaType type);
            int SetCurrentMediaType(int index, IntPtr reserved, IMFMediaType type);
            int SetCurrentPosition(ref Guid format, long position);
            int ReadSample(int index, int flags, out int actualIndex, out int streamFlags, out long timestamp, out IMFSample sample);
            int Flush(int index);
            int GetServiceForStream(int index, ref Guid service, ref Guid riid, out IntPtr ppv);
            int GetPresentationAttribute(int index, ref Guid attribute, IntPtr value);
        }

        [ComImport, Guid("c40a00f2-b93a-4d80-ae8c-5a1c634f58e4"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFSample
        {
            int GetSampleFlags(out int flags);
            int SetSampleFlags(int flags);
            int GetSampleTime(out long time);
            int SetSampleTime(long time);
            int GetSampleDuration(out long duration);
            int SetSampleDuration(long duration);
            int GetBufferCount(out int count);
            int GetBufferByIndex(int index, out IMFMediaBuffer buffer);
            int ConvertToContiguousBuffer(out IMFMediaBuffer buffer);
            int AddBuffer(IMFMediaBuffer buffer);
            int RemoveBufferByIndex(int index);
            int RemoveAllBuffers();
            int GetTotalLength(out int length);
            int CopyToBuffer(IMFMediaBuffer buffer);
        }

        [ComImport, Guid("045fa593-8799-42b8-bc8d-8968c6453507"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
        private interface IMFMediaBuffer
        {
            int Lock(out IntPtr buffer, out int maxLength, out int currentLength);
            int Unlock();
            int GetCurrentLength(out int length);
            int SetCurrentLength(int length);
            int GetMaxLength(out int length);
        }
    }
}
