using System;
using System.Collections.Generic;

namespace Acquire
{
    /// <summary>一个相机后端（RealSense / OpenNI2 / 合成帧）的就绪状态，--list-cameras 直接打印。</summary>
    internal sealed class CameraBackendStatus
    {
        public string Name = "";
        /// <summary>SDK/运行库是否就绪（不代表一定插了相机）。</summary>
        public bool Ready;
        public bool DeviceFound;
        public string Detail = "";
        /// <summary>补充信息，例如搜索过的目录、生成过的 ini。</summary>
        public List<string> Notes = new List<string>();
    }

    internal sealed class CameraProbeResult
    {
        public readonly List<CameraBackendStatus> Backends = new List<CameraBackendStatus>();
        public readonly List<CameraOption> Options = new List<CameraOption>();
    }

    /// <summary>
    /// OpenNI2 后端的探测编排：定位 redist → 预加载原生库 → 优先托管包装（反射），
    /// 不行就退回原生 C API。任何一步失败都留下可操作的原因。
    /// </summary>
    internal static class OpenNi2Probe
    {
        public const string BackendName = "Orbbec / OpenNI2（Astra Pro 等）";

        public static CameraBackendStatus Run(Action<string> log, out List<CameraOption> options)
        {
            options = new List<CameraOption>();
            CameraBackendStatus status = new CameraBackendStatus();
            status.Name = BackendName;

            OpenNi2Locator locator = OpenNi2Locator.Locate();
            foreach (string line in locator.Searched) status.Notes.Add(line);

            if (!locator.NativeFound)
            {
                status.Ready = false;
                status.Detail = locator.DescribeMissing();
                if (log != null) log("OpenNI2 不可用：" + status.Detail);
                return status;
            }

            string loadError = locator.LoadNative();
            if (loadError != null)
            {
                status.Ready = false;
                status.Detail = loadError;
                if (log != null) log("OpenNI2 不可用：" + loadError);
                return status;
            }
            locator.EnsureDriverRepository(log);
            if (!locator.DriversFound)
                status.Notes.Add("未找到驱动目录：" + locator.Root + "\\OpenNI2\\Drivers");

            status.Ready = true;
            status.Detail = "OpenNI2.dll：" + locator.NativeDll;

            string managedError = null;
            bool managedReady = false;
            if (locator.ManagedFound)
            {
                List<CameraOption> managed = OpenNi2ManagedSource.Probe(locator, log, out managedError, out managedReady);
                if (managed.Count > 0)
                {
                    options = managed;
                    status.Ready = managedReady;
                    status.DeviceFound = true;
                    status.Detail += "；托管包装：" + locator.ManagedDll;
                    return status;
                }
                if (managedError != null && log != null) log("OpenNI2 托管包装探测未成功：" + managedError);
            }

            string nativeError;
            bool nativeReady;
            List<CameraOption> native = OpenNi2NativeSource.Probe(locator, log, out nativeError, out nativeReady);
            status.Ready = nativeReady;
            if (native.Count > 0)
            {
                options = native;
                status.DeviceFound = true;
                status.Detail += "；绑定方式：原生 C API";
                if (managedError != null) status.Notes.Add("托管包装不可用：" + managedError);
                return status;
            }

            status.DeviceFound = false;
            string reason = nativeError;
            if (string.IsNullOrEmpty(reason)) reason = managedError;
            if (string.IsNullOrEmpty(reason)) reason = "OpenNI2 已初始化，但没有枚举到设备";
            status.Detail = reason;
            if (log != null) log("OpenNI2 未找到相机：" + reason);
            return status;
        }

        /// <summary>把“缺少什么”压成一行，供界面文本框使用（太长会截断，完整信息看 --list-cameras）。</summary>
        public static string ShortReason(CameraBackendStatus status)
        {
            if (status == null) return "";
            string text = status.Ready && !status.DeviceFound
                ? "SDK 已就绪，但没有检测到相机：" + status.Detail
                : status.Detail;
            if (text != null && text.Length > 160) text = text.Substring(0, 160) + "…（完整原因见 --list-cameras）";
            return text;
        }
    }
}
