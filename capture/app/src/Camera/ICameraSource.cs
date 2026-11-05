using System;
using System.Collections.Generic;

namespace Acquire
{
    internal enum CameraKind
    {
        Null,
        RealSense,
        OpenCv,
        OpenNi2
    }

    /// <summary>一帧相机数据。缓冲由 IDataSource 复用，消费者只能读、保存完立即释放引用。</summary>
    internal sealed class CameraFrame
    {
        public byte[] ColorBgr;
        public int ColorWidth;
        public int ColorHeight;
        public ushort[] Depth;
        public int DepthWidth;
        public int DepthHeight;
        public double DepthScaleMeters;
        /// <summary>彩色/深度各自的到达时刻（相对统一时钟），两路可分开对齐。</summary>
        public double ColorRelMs;
        public double DepthRelMs;
        public bool HasColor { get { return ColorBgr != null; } }
        public bool HasDepth { get { return Depth != null; } }
        public string Note;
    }

    /// <summary>探测结果：界面据此列出可选设备，或明确告知缺什么。</summary>
    internal sealed class CameraOption
    {
        public CameraKind Kind;
        public string Display;
        public string Model;
        public string Serial;
        public bool HasColor;
        public bool HasDepth;
        public bool Available;
        public string Reason;

        /// <summary>OpenNI2 用哪套绑定打开：native（OpenNI2.dll 的 C API）或 managed（OpenNI2.Net.dll 反射）。</summary>
        public string Backend = "";

        /// <summary>OpenNI2 设备 URI（oniDeviceOpen 用）；其它后端留空。</summary>
        public string Address = "";

        /// <summary>探测到的能力描述，如“深度 640x480@30 Depth1Mm”。--list-cameras 用。</summary>
        public List<string> Capabilities = new List<string>();

        public override string ToString()
        {
            return Display + (Available ? "" : "（不可用）");
        }
    }

    internal interface ICameraSource : IDisposable
    {
        CameraKind Kind { get; }
        string Model { get; }
        string Serial { get; }
        bool HasColor { get; }
        bool HasDepth { get; }
        double DepthScaleMeters { get; }
        void Open();
        /// <summary>阻塞取一帧，超时返回 null。返回的缓冲在下一次调用前有效。</summary>
        CameraFrame ReadFrame(int timeoutMs);
        void Close();
    }
}
