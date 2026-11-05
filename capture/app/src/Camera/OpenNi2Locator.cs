using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// 定位 Orbbec/OpenNI2 的运行时文件。三处候选，按顺序命中即用：
    ///   1. 环境变量 OPENNI2_REDIST
    ///   2. exe 同目录
    ///   3. C:\Program Files\Orbbec\OpenNI2\Redist  /  C:\Program Files\OpenNI2\Redist
    ///
    /// 说明（对着 Orbbec OpenNI SDK 2.3.0.86 的实包核过）：
    ///   - OpenNI 2.3 的 OpenNI2.dll 只会去“自己所在目录”读 OpenNI.ini，默认驱动仓库是
    ///     &lt;dir&gt;\OpenNI2\Drivers。所以官方 Redist 的布局是 OpenNI2.dll + OpenNI2\Drivers\。
    ///   - 任务书里写的“exe 同目录 + Drivers\”是扁平布局，这里用一份 OpenNI.ini
    ///     （Repository=Drivers）把它补齐；已存在 OpenNI.ini 时不改动，只给出提示。
    ///   - OpenNI2.dll 静态依赖 MSVCR120.dll（VC++ 2013 x64 运行库），缺失时 LoadLibrary
    ///     直接报 126。这条在实际机器上很常见，必须单独说清楚。
    /// </summary>
    internal sealed class OpenNi2Locator
    {
        public const string MissingHint =
            "未找到 OpenNI2：请安装 Orbbec OpenNI SDK 2.3（x64），" +
            "或把 OpenNI2.dll + OpenNI2.Net.dll + Drivers 目录放到 exe 同目录，" +
            "或设置环境变量 OPENNI2_REDIST";

        private readonly List<string> _searched = new List<string>();
        private IntPtr _nativeHandle;

        public string Root = "";
        public string NativeDll = "";
        public string ManagedDll = "";
        public string DriversDir = "";

        public bool NativeFound { get { return NativeDll.Length > 0; } }
        public bool ManagedFound { get { return ManagedDll.Length > 0; } }
        public bool DriversFound { get { return DriversDir.Length > 0; } }

        public IList<string> Searched { get { return _searched; } }

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Ansi)]
        internal static extern IntPtr GetProcAddress(IntPtr module, string procName);

        // ------------------------------------------------------------ 探测

        public static OpenNi2Locator Locate()
        {
            OpenNi2Locator loc = new OpenNi2Locator();
            List<string> roots = new List<string>();

            string env = Environment.GetEnvironmentVariable("OPENNI2_REDIST");
            if (!string.IsNullOrEmpty(env))
            {
                env = env.Trim().Trim('"');
                if (env.EndsWith(".dll", StringComparison.OrdinalIgnoreCase)) env = Path.GetDirectoryName(env);
                if (!string.IsNullOrEmpty(env) && Directory.Exists(env)) roots.Add(env);
                loc._searched.Add("环境变量 OPENNI2_REDIST = " + env);
            }
            else
            {
                loc._searched.Add("环境变量 OPENNI2_REDIST = （未设置）");
            }

            string exeDir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
            roots.Add(exeDir);
            roots.Add(@"C:\Program Files\Orbbec\OpenNI2\Redist");
            roots.Add(@"C:\Program Files\OpenNI2\Redist");
            // 少数分发包会把 Redist 再套一层
            roots.Add(Path.Combine(exeDir, "OpenNI2", "Redist"));

            foreach (string root in roots)
            {
                if (string.IsNullOrEmpty(root) || !Directory.Exists(root))
                {
                    if (!string.IsNullOrEmpty(root)) loc._searched.Add(root + "（目录不存在）");
                    continue;
                }
                string native = Path.Combine(root, "OpenNI2.dll");
                string managed = Path.Combine(root, "OpenNI2.Net.dll");
                bool hasNative = File.Exists(native);
                bool hasManaged = File.Exists(managed);
                loc._searched.Add(root + "（OpenNI2.dll=" + (hasNative ? "有" : "无") +
                                  "，OpenNI2.Net.dll=" + (hasManaged ? "有" : "无") + "）");

                if (loc.NativeDll.Length == 0 && hasNative)
                {
                    loc.NativeDll = native;
                    loc.Root = root;
                }
                if (loc.ManagedDll.Length == 0 && hasManaged) loc.ManagedDll = managed;
            }

            // 托管包装可能单独放在 exe 目录，而原生库来自安装路径
            if (loc.ManagedDll.Length == 0)
            {
                string candidate = Path.Combine(exeDir, "OpenNI2.Net.dll");
                if (File.Exists(candidate)) loc.ManagedDll = candidate;
            }

            if (loc.NativeDll.Length == 0 && loc.ManagedDll.Length > 0)
                loc.Root = Path.GetDirectoryName(loc.ManagedDll);

            if (loc.Root.Length > 0)
            {
                string official = Path.Combine(loc.Root, "OpenNI2", "Drivers");
                if (Directory.Exists(official)) loc.DriversDir = official;
                else
                {
                    string flat = Path.Combine(loc.Root, "Drivers");
                    if (Directory.Exists(flat)) loc.DriversDir = flat;
                }
            }
            return loc;
        }

        public string DescribeMissing()
        {
            if (!NativeFound && !ManagedFound) return MissingHint;
            if (!NativeFound)
                return MissingHint + "（当前只找到 OpenNI2.Net.dll，但托管包装离不开原生 OpenNI2.dll）";
            if (!DriversFound)
                return "已找到 OpenNI2.dll，但找不到驱动目录。请确认 Drivers 在 " + Root +
                       "\\OpenNI2\\Drivers（官方布局）或 " + Root + "\\Drivers（扁平布局）";
            return MissingHint;
        }

        // ------------------------------------------------------------ 加载

        /// <summary>把原生库预加载进进程，成功返回 null，失败返回带诊断的原因。</summary>
        public string LoadNative()
        {
            if (!NativeFound) return DescribeMissing();
            if (_nativeHandle != IntPtr.Zero) return null;

            string error;
            _nativeHandle = OpenNi2Loader.Load(NativeDll, out error);
            if (_nativeHandle == IntPtr.Zero) return error;

            // 让同进程里 OpenNI2.Net.dll 的 P/Invoke("OpenNI2.dll") 命中同一份已加载模块
            try { Environment.SetEnvironmentVariable("OPENNI2_REDIST", Root); }
            catch { }
            return null;
        }

        /// <summary>
        /// 扁平布局（exe 同目录直接放 Drivers\）需要一份 OpenNI.ini 指路。
        /// 只在 OpenNI.ini 不存在时新建；已存在则不动用户文件，只提示。
        /// </summary>
        public void EnsureDriverRepository(Action<string> log)
        {
            if (!DriversFound) return;
            string official = Path.Combine(Root, "OpenNI2", "Drivers");
            if (Directory.Exists(official)) return;

            string ini = Path.Combine(Root, "OpenNI.ini");
            if (File.Exists(ini))
            {
                if (log != null)
                    log("提示：驱动在 " + DriversDir + "，但 " + ini +
                        " 已存在且未指向它；若驱动加载失败，请把 Drivers 移到 " + official + " 或在该 ini 里设 Repository=Drivers");
                return;
            }
            try
            {
                File.WriteAllText(ini, "[Drivers]\r\nRepository=Drivers\r\n", new UTF8Encoding(false));
                if (log != null) log("已生成 " + ini + "（让 OpenNI2 从 " + DriversDir + " 加载驱动）");
            }
            catch (Exception ex)
            {
                if (log != null)
                    log("提示：无法写入 " + ini + "（" + ex.Message + "），若驱动加载失败请把 Drivers 移到 " + official);
            }
        }
    }
}
