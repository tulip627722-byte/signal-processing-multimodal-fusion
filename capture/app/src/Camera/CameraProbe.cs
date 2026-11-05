using System;
using System.Collections.Generic;

namespace Acquire
{
    /// <summary>
    /// 相机探测与实例化。界面只认 CameraOption，不关心后面是哪家 SDK。
    /// 探测链顺序：RealSense → OpenNI2(Orbbec) → 合成帧 Null。
    /// 每一级失败都必须给出“缺什么、下一步做什么”，不允许静默变成没数据。
    /// </summary>
    internal static class CameraProbe
    {
        public static CameraProbeResult ProbeAll(Action<string> log)
        {
            CameraProbeResult result = new CameraProbeResult();

            // ---------------------------------------------------------- RealSense
            string realsenseReason;
            bool realsenseLoaded;
            List<CameraOption> realsense = RealSenseSource.Probe(log, out realsenseReason, out realsenseLoaded);
            CameraBackendStatus realsenseStatus = new CameraBackendStatus();
            realsenseStatus.Name = "Intel RealSense（librealsense2）";
            realsenseStatus.Ready = realsenseLoaded;
            realsenseStatus.DeviceFound = realsense.Count > 0;
            realsenseStatus.Detail = realsense.Count > 0
                ? "SDK 已加载，枚举到 " + realsense.Count + " 台设备"
                : (realsenseReason ?? "SDK 已加载，但没有枚举到设备");
            result.Backends.Add(realsenseStatus);

            if (realsense.Count > 0)
            {
                result.Options.AddRange(realsense);
                if (log != null) log("RealSense：" + realsense.Count + " 台可用");
            }
            else
            {
                result.Options.Add(new CameraOption
                {
                    Kind = CameraKind.RealSense,
                    Display = "Intel RealSense（未检测到）",
                    Model = "",
                    Serial = "",
                    HasColor = false,
                    HasDepth = false,
                    Available = false,
                    Reason = realsenseStatus.Detail
                });
                if (log != null) log("RealSense 不可用：" + realsenseStatus.Detail);
            }

            // ---------------------------------------------------------- OpenNI2
            List<CameraOption> openni;
            CameraBackendStatus openniStatus = OpenNi2Probe.Run(log, out openni);
            result.Backends.Add(openniStatus);
            if (openni.Count > 0)
            {
                result.Options.AddRange(openni);
                if (log != null) log("OpenNI2：" + openni.Count + " 台可用");
            }
            else
            {
                result.Options.Add(new CameraOption
                {
                    Kind = CameraKind.OpenNi2,
                    Display = "Orbbec OpenNI2（未检测到）",
                    Model = "",
                    Serial = "",
                    HasColor = false,
                    HasDepth = false,
                    Available = false,
                    Reason = OpenNi2Probe.ShortReason(openniStatus)
                });
            }

            // ---------------------------------------------------------- 合成帧
            CameraBackendStatus nullStatus = new CameraBackendStatus();
            nullStatus.Name = "合成帧 NullSource";
            nullStatus.Ready = true;
            nullStatus.DeviceFound = true;
            nullStatus.Detail = "无需硬件，用于 --self-test 与全链路验证";
            result.Backends.Add(nullStatus);

            result.Options.Add(new CameraOption
            {
                Kind = CameraKind.Null,
                Display = "合成帧 NullSource（彩色 + 深度，测试用）",
                Model = "NullSource",
                Serial = "SYNTH-0001",
                HasColor = true,
                HasDepth = true,
                Available = true,
                Reason = "无需硬件"
            });

            return result;
        }

        public static List<CameraOption> Probe(Action<string> log)
        {
            return ProbeAll(log).Options;
        }

        public static ICameraSource Create(CameraOption option, Action<string> log, Func<double> clock)
        {
            if (option == null) throw new ArgumentNullException("option");
            switch (option.Kind)
            {
                case CameraKind.Null:
                    return new NullSource(clock, NullSource.DefaultFps);
                case CameraKind.RealSense:
                    if (!option.Available) throw new InvalidOperationException(option.Reason);
                    return new RealSenseSource(option, log, clock);
                case CameraKind.OpenNi2:
                    if (!option.Available) throw new InvalidOperationException(option.Reason);
                    {
                        ICameraSource depthOnly = new OpenNi2Source(option, log, clock);
                        // Astra Pro 的彩色不在 OpenNI2 里（实测 23 种彩色模式全部 0 帧），
                        // 单独用 UVC（Media Foundation）抓，合成一个源给上层用。
                        bool needUvc = option.Model == null ||
                                       option.Model.IndexOf("Astra", StringComparison.OrdinalIgnoreCase) >= 0;
                        if (!needUvc) return depthOnly;
                        return new CompositeCameraSource(depthOnly, new DsColorGrabber(), log, clock);
                    }
                case CameraKind.OpenCv:
                    return new OpenCvSource(option, log, clock);
                default:
                    throw new NotSupportedException("未知相机类型：" + option.Kind);
            }
        }

        /// <summary>给界面用的一句话结论。</summary>
        public static string Summarize(List<CameraOption> options)
        {
            if (options == null || options.Count == 0) return "没有可用相机";
            foreach (CameraOption o in options)
            {
                if (o.Kind == CameraKind.RealSense && o.Available)
                    return "RealSense 已就绪：" + o.Model + (o.HasDepth ? "（彩色+深度）" : "（仅彩色，深度不可用）");
            }
            foreach (CameraOption o in options)
            {
                if (o.Kind == CameraKind.OpenNi2 && o.Available)
                    return "Orbbec OpenNI2 已就绪：" + o.Model + (o.HasDepth ? "（彩色+深度）" : "（仅彩色，深度不可用）");
            }
            return "未检测到可用真相机（RealSense / OpenNI2 都未就绪，原因见下拉框）；合成帧可用";
        }
    }

    /// <summary>
    /// 通用 USB 相机占位实现。当前不在探测链里：Astra Pro 的深度必须走 OpenNI2，
    /// 彩色虽然也是 UVC，但由 Orbbec 驱动统一接管更省事。留着是为了以后接纯 UVC 相机。
    /// 没有 OpenCV 绑定就不假装能取流：Open() 直接抛出带说明的异常。
    /// </summary>
    internal sealed class OpenCvSource : ICameraSource
    {
        public const string UnavailableReason =
            "本版本未内置 OpenCV 绑定（需要 OpenCvSharp/opencv_csharp 与对应 x64 原生 DLL）。" +
            "若相机是普通 USB 摄像头，请先确认型号，再扩展 OpenCvSource。";

        private readonly CameraOption _option;

        public OpenCvSource(CameraOption option, Action<string> log, Func<double> clock)
        {
            _option = option;
        }

        public CameraKind Kind { get { return CameraKind.OpenCv; } }
        public string Model { get { return _option == null ? "" : _option.Model; } }
        public string Serial { get { return _option == null ? "" : _option.Serial; } }
        public bool HasColor { get { return false; } }
        public bool HasDepth { get { return false; } }
        public double DepthScaleMeters { get { return 0; } }

        public void Open()
        {
            throw new NotSupportedException(UnavailableReason);
        }

        public CameraFrame ReadFrame(int timeoutMs) { return null; }

        public void Close() { }

        public void Dispose() { }
    }
}
