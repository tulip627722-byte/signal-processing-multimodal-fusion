using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace Acquire
{
    internal static class Program
    {
        [DllImport("winmm.dll")]
        private static extern uint timeBeginPeriod(uint milliseconds);

        [STAThread]
        private static int Main(string[] args)
        {
            try { timeBeginPeriod(1); } catch { }

            if (HasArg(args, "--self-test"))
            {
                ConsoleIo.Ensure();
                return SelfTestRunner.Run(SelfTestOptions.Parse(args));
            }

            if (HasArg(args, "--preview-stress-test"))
            {
                int seconds = SelfTestOptions.ParseInt(ValueOf(args, "--seconds"), 1800);
                return PreviewStressTest.Run(seconds);
            }

            string verify = ValueOf(args, "--verify-take");
            if (verify != null)
            {
                ConsoleIo.Ensure();
                return VerifyTake(verify);
            }

            string exportTake = ValueOf(args, "--export-cloud");
            if (exportTake != null)
            {
                ConsoleIo.Ensure();
                return ExportCloud(args, exportTake);
            }

            if (HasArg(args, "--dump-factory-calibration"))
            {
                ConsoleIo.Ensure();
                string output = ValueOf(args, "--out");
                if (!string.IsNullOrEmpty(output) && output.StartsWith("--", StringComparison.Ordinal)) output = null;
                return OrbbecFactoryCalibration.Dump(output);
            }

            if (HasArg(args, "--capture"))
            {
                ConsoleIo.Ensure();
                return Capture(args);
            }

            if (HasArg(args, "--help") || HasArg(args, "-h"))
            {
                ConsoleIo.Ensure();
                PrintUsage();
                return 0;
            }

            if (HasArg(args, "--list-cameras"))
            {
                ConsoleIo.Ensure();
                return ListCameras();
            }

            if (HasArg(args, "--align"))
            {
                ConsoleIo.Ensure();
                return RunAlign(args);
            }

            if (HasArg(args, "--probe-color"))
            {
                ConsoleIo.Ensure();
                return ProbeColor();
            }

            if (HasArg(args, "--probe-uvc"))
            {
                ConsoleIo.Ensure();
                try { return ProbeUvc(args); } catch (Exception ex) { ConsoleIo.WriteLine("UVC 探针异常：" + ex.GetType().Name + " : " + ex.Message); ConsoleIo.WriteLine(ex.StackTrace); return 3; }
            }

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            string demoScreenshot = ValueOf(args, "--demo-screenshot");
            bool demoMode = HasArg(args, "--demo") || !string.IsNullOrEmpty(demoScreenshot);
            MainForm form = new MainForm(demoMode);
            if (!string.IsNullOrEmpty(demoScreenshot))
            {
                System.Windows.Forms.Timer screenshotTimer = new System.Windows.Forms.Timer { Interval = 7000 };
                screenshotTimer.Tick += delegate
                {
                    screenshotTimer.Stop();
                    form.SaveScreenshot(Path.GetFullPath(demoScreenshot));
                    form.Close();
                };
                form.Shown += delegate { screenshotTimer.Start(); };
            }
            if (HasArg(args, "--ui-smoke-test"))
            {
                System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer { Interval = 1500 };
                timer.Tick += delegate
                {
                    timer.Stop();
                    form.Close();
                };
                form.Shown += delegate { timer.Start(); };
            }
            Application.Run(form);
            return 0;
        }

        /// <summary>
        /// --align：扫描 takes 目录，把还没对齐的 take 全部生成 align.csv。
        /// 不传 --out 时用默认 takes 根目录；加 --force 则重建已有的对齐表。
        /// </summary>
        private static int RunAlign(string[] args)
        {
            string root = ValueOf(args, "--out");
            if (string.IsNullOrEmpty(root) || root.StartsWith("--", StringComparison.Ordinal))
                root = DefaultTakeRoot();
            bool force = HasArg(args, "--force");

            ConsoleIo.WriteLine("对齐目录：" + root);
            if (!Directory.Exists(root))
            {
                ConsoleIo.WriteLine("目录不存在");
                return 2;
            }

            if (force)
            {
                // 强制重建：把已有的 align.csv 删掉，让它重新进待办
                string[] dirs = Directory.GetDirectories(root);
                int removed = 0;
                foreach (string dir in dirs)
                {
                    string p = Path.Combine(dir, TakeAligner.AlignFileName);
                    if (File.Exists(p)) { File.Delete(p); removed++; }
                }
                ConsoleIo.WriteLine("已清除 " + removed + " 份旧对齐表，将全部重建");
            }

            List<string> pending = TakeAligner.FindPending(root);
            if (pending.Count == 0)
            {
                ConsoleIo.WriteLine("没有待对齐的 take");
                return 0;
            }
            ConsoleIo.WriteLine("待对齐：" + pending.Count + " 个");

            List<AlignResult> results = TakeAligner.AlignAll(root,
                delegate(string line) { ConsoleIo.WriteLine(line); },
                delegate(int done, int total)
                {
                    ConsoleIo.WriteLine("  进度 " + done + "/" + total);
                },
                null);

            ConsoleIo.WriteLine("完成：" + results.Count + " 个 take");
            foreach (AlignResult r in results)
            {
                ConsoleIo.WriteLine("  " + Path.GetFileName(r.TakeDirectory) +
                    "｜雷达 " + r.RadarFrames + " 帧｜深度残差中位 " + (double.IsNaN(r.DepthDtMedian) ? "—" : r.DepthDtMedian.ToString("0.0")) + " ms" +
                    "｜彩色 " + r.ColorFrames + " 帧");
            }
            return 0;
        }

        /// <summary>
        /// 诊断用：把相机报的每一种彩色模式都试一遍，看哪种真能出帧。
        /// Astra Pro 会报 20 多种模式，但实际能用的可能只有一两种。
        /// </summary>
        private static int ProbeColor()
        {
            OpenNi2Locator locator = OpenNi2Locator.Locate();
            if (!locator.NativeFound)
            {
                ConsoleIo.WriteLine(locator.DescribeMissing());
                return 2;
            }
            string loadError = locator.LoadNative();
            if (!string.IsNullOrEmpty(loadError))
            {
                ConsoleIo.WriteLine(loadError);
                return 2;
            }
            locator.EnsureDriverRepository(delegate(string s) { ConsoleIo.WriteLine(s); });

            OpenNi2NativeApi api = OpenNi2NativeApi.Open(locator.NativeDll);
            if (api == null)
            {
                ConsoleIo.WriteLine("OpenNI2 初始化失败");
                return 2;
            }
            try
            {
                List<OniDeviceInfo> devices = api.ListDevices();
                if (devices.Count == 0) { ConsoleIo.WriteLine("没有设备"); return 2; }
                IntPtr device = api.OpenDevice(devices[0].Uri);
                if (device == IntPtr.Zero) { ConsoleIo.WriteLine("打不开设备"); return 2; }

                if (!api.HasSensor(device, Oni2.SensorColor))
                {
                    ConsoleIo.WriteLine("设备没有彩色传感器");
                    return 2;
                }
                List<OniVideoMode> modes = api.GetSupportedModes(device, Oni2.SensorColor);
                ConsoleIo.WriteLine("彩色模式共 " + modes.Count + " 种，逐个试 1.5 秒：");

                foreach (OniVideoMode mode in modes)
                {
                    IntPtr stream = api.CreateStream(device, Oni2.SensorColor);
                    if (stream == IntPtr.Zero) { ConsoleIo.WriteLine("  创建流失败"); continue; }
                    bool set = api.TrySetVideoMode(stream, mode.PixelFormat, mode.ResolutionX, mode.ResolutionY, mode.Fps);
                    OniVideoMode actual = api.GetVideoMode(stream);
                    string label = mode.ResolutionX + "x" + mode.ResolutionY + "@" + mode.Fps +
                                   " " + Oni2.FormatName(mode.PixelFormat);
                    if (!set)
                    {
                        ConsoleIo.WriteLine("  " + label + " —— 设置失败，跳过");
                        continue;
                    }
                    api.StartStream(stream);
                    int frames = 0; long bytes = 0; int firstW = 0, firstH = 0, firstFmt = 0;
                    DateTime deadline = DateTime.UtcNow.AddMilliseconds(1500);
                    while (DateTime.UtcNow < deadline)
                    {
                        int idx;
                        int st = api.WaitForAny(new IntPtr[] { stream }, out idx, 200);
                        if (st != Oni2.StatusOk) continue;
                        OniFrame frame; IntPtr fp;
                        if (!api.ReadFrame(stream, out frame, out fp)) continue;
                        if (frames == 0) { firstW = frame.Width; firstH = frame.Height; firstFmt = frame.VideoMode.PixelFormat; }
                        bytes += frame.DataSize;
                        frames++;
                        api.ReleaseFrame(fp);
                    }
                    ConsoleIo.WriteLine("  " + label + " —— 收到 " + frames + " 帧" +
                        (frames > 0 ? ("，首帧 " + firstW + "x" + firstH + " " + Oni2.FormatName(firstFmt) + "，共 " + (bytes / 1024) + " KB") : ""));
                }
                return 0;
            }
            finally
            {
                api.Dispose();
            }
        }

        /// <summary>
        /// 用 Media Foundation 抓 UVC 彩色（Astra Pro 的 RGB 不在 OpenNI2 里）。
        /// --probe-uvc [--name Astra] [--width 640 --height 480 --fps 30] [--seconds 3]
        /// </summary>
                private static int ProbeUvc(string[] args)
        {
            List<string> devices = DsColorGrabber.ListDevices();
            ConsoleIo.WriteLine("UVC 视频设备共 " + devices.Count + " 台");
            foreach (string d in devices) ConsoleIo.WriteLine("  " + d);
            if (devices.Count == 0) return 2;

            string name = ValueOf(args, "--name");
            if (string.IsNullOrEmpty(name) || name.StartsWith("--", StringComparison.Ordinal)) name = "Astra";
            int width = IntArg(args, "--width", 640);
            int height = IntArg(args, "--height", 480);
            int seconds = IntArg(args, "--seconds", 3);

            using (DsColorGrabber grabber = new DsColorGrabber())
            {
                string err;
                if (!grabber.Open(name, width, height, out err))
                {
                    ConsoleIo.WriteLine("打开失败：" + err);
                    return 2;
                }
                ConsoleIo.WriteLine("已打开：" + grabber.DeviceName + " → " + grabber.Width + "x" + grabber.Height);

                // 阶段 1：只看帧计数，不拷贝内存——用来区分“回调没数据”和“memcpy 越界”
                int s0 = grabber.Serial;
                System.Threading.Thread.Sleep(1000);
                int s1 = grabber.Serial;
                ConsoleIo.WriteLine("serial 计数：" + s0 + " → " + s1 + "（1 秒内 " + (s1 - s0) + " 帧）");

                System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
                DateTime deadline = DateTime.UtcNow.AddSeconds(Math.Max(1, seconds));
                int frames = 0;
                byte[] first = null;
                while (DateTime.UtcNow < deadline)
                {
                    byte[] bgr = grabber.ReadFrame(300);
                    if (bgr == null) continue;
                    if (first == null) first = bgr;
                    frames++;
                }
                double sec = sw.Elapsed.TotalSeconds;
                ConsoleIo.WriteLine("收到 " + frames + " 帧，" + (frames / Math.Max(sec, 0.001)).ToString("0.0") + " fps");

                if (first != null)
                {
                    string outPath = System.IO.Path.Combine(
                        System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location),
                        "uvc_probe.png");
                    using (System.Drawing.Bitmap bmp = new System.Drawing.Bitmap(grabber.Width, grabber.Height,
                               System.Drawing.Imaging.PixelFormat.Format24bppRgb))
                    {
                        System.Drawing.Rectangle rect = new System.Drawing.Rectangle(0, 0, grabber.Width, grabber.Height);
                        System.Drawing.Imaging.BitmapData data = bmp.LockBits(rect,
                            System.Drawing.Imaging.ImageLockMode.WriteOnly, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
                        for (int y = 0; y < grabber.Height; y++)
                            Marshal.Copy(first, y * grabber.Width * 3, IntPtr.Add(data.Scan0, y * data.Stride), grabber.Width * 3);
                        bmp.UnlockBits(data);
                        bmp.Save(outPath, System.Drawing.Imaging.ImageFormat.Png);
                    }
                    ConsoleIo.WriteLine("首帧已存：" + outPath);
                }
                else
                {
                    ConsoleIo.WriteLine("一帧都没拿到");
                }
            }
            return 0;
        }

private static int IntArg(string[] args, string name, int fallback)
        {
            string v = ValueOf(args, name);
            int n;
            if (!string.IsNullOrEmpty(v) && !v.StartsWith("--", StringComparison.Ordinal) && int.TryParse(v, out n))
                return n;
            return fallback;
        }

        private static void PrintUsage()
        {            ConsoleIo.WriteLine("Acquire.exe　　雷达 ADC + 彩色 + 深度采集，深度点云（离线/采集时）");
            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("  （无参数）                    打开界面");
            ConsoleIo.WriteLine("  --self-test [--delay 1] [--duration 3] [--fps 2] [--loss r] [--drop-frames r] [--out 目录]");
            ConsoleIo.WriteLine("  --list-cameras                列出相机后端与设备能力（含深度内参预览）");
            ConsoleIo.WriteLine("  --dump-factory-calibration [--out 文件]  从 Orbbec EEPROM 读取 RGB/深度出厂标定");
            ConsoleIo.WriteLine("  --align [--out 目录] [--force]  把 takes 里还没对齐的 take 生成 align.csv（雷达帧↔深度/彩色帧）");
            ConsoleIo.WriteLine("  --probe-color                 逐个试相机的彩色模式，看哪种真能出帧（诊断用）");
            ConsoleIo.WriteLine("  --probe-uvc [--name Astra] [--seconds 3]  用 Media Foundation 抓 UVC 彩色并自检（诊断用）");
            ConsoleIo.WriteLine("  --verify-take <take目录>      校验已有 take");
            ConsoleIo.WriteLine("  --export-cloud <take目录> [--stride 4] [--frame 120] [--radar-frame] [--csv] [--out 目录]");
            ConsoleIo.WriteLine("                                用 depth\\*.png + meta.json 内参生成点云");
            ConsoleIo.WriteLine("  --capture [--camera 1|null] [--delay 1] [--duration 3] [--label 文本] [--out 目录] [--no-radar] [--cues]");
            ConsoleIo.WriteLine("                                不开界面采一次（验证真机内参档位用）；--cues 开提示音");
            ConsoleIo.WriteLine("  --ui-smoke-test               起界面 1.5 s 后自动关闭（构建自测用）");
            ConsoleIo.WriteLine("  --demo                        启动四路合成数据演示");
            ConsoleIo.WriteLine("  --demo-screenshot <png>       运行四路合成采集并保存界面截图");
            ConsoleIo.WriteLine("  --preview-stress-test [--seconds 1800]  四路最新帧缓冲压力测试");
        }

        /// <summary>
        /// --list-cameras：只探测、不打开采集。没有相机也要正常退出（0），把缺什么说清楚。
        /// </summary>
        private static int ListCameras()
        {
            ConsoleIo.WriteLine("Acquire 相机探测 " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            ConsoleIo.WriteLine(new string('=', 62));

            CameraProbeResult result = CameraProbe.ProbeAll(delegate(string line) { });

            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("[后端]");
            foreach (CameraBackendStatus status in result.Backends)
            {
                ConsoleIo.WriteLine("  " + (status.Ready ? "[就绪]   " : "[未就绪] ") + status.Name);
                if (!string.IsNullOrEmpty(status.Detail)) ConsoleIo.WriteLine("            " + status.Detail);
                if (!status.Ready)
                {
                    foreach (string note in status.Notes) ConsoleIo.WriteLine("            - " + note);
                }
            }

            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("[相机]");
            int real = 0;
            foreach (CameraOption option in result.Options)
            {
                if (option.Kind == CameraKind.Null) continue;
                real++;
                if (!option.Available)
                {
                    ConsoleIo.WriteLine("  " + option.Display);
                    ConsoleIo.WriteLine("            原因：" + option.Reason);
                    continue;
                }
                ConsoleIo.WriteLine("  " + option.Model);
                ConsoleIo.WriteLine("            序列号：" + (string.IsNullOrEmpty(option.Serial) ? "（未提供）" : option.Serial));
                ConsoleIo.WriteLine("            后端：" + DescribeBackend(option));
                ConsoleIo.WriteLine("            彩色：" + (option.HasColor ? "可用" : "不可用") +
                                    "　深度：" + (option.HasDepth ? "可用" : "不可用"));
                foreach (string capability in option.Capabilities)
                    ConsoleIo.WriteLine("            " + capability);
            }
            if (real == 0)
                ConsoleIo.WriteLine("  （没有检测到相机）");

            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("结论：" + CameraProbe.Summarize(result.Options));

            bool anyReal = false;
            foreach (CameraOption option in result.Options)
            {
                if (option.Kind != CameraKind.Null && option.Available) anyReal = true;
            }
            if (!anyReal)
            {
                ConsoleIo.WriteLine("");
                ConsoleIo.WriteLine("下一步：");
                ConsoleIo.WriteLine("  - Astra Pro 需要 Orbbec OpenNI SDK 2.3（x64）：https://www.orbbec.com/developers/openni-sdk/");
                ConsoleIo.WriteLine("  - 解包后把 Redist 目录（OpenNI2.dll + OpenNI2\\Drivers）放到 exe 同目录，" +
                                    "或设 OPENNI2_REDIST 指向它，或装到 C:\\Program Files\\Orbbec\\OpenNI2\\Redist");
                ConsoleIo.WriteLine("  - 先用 SDK 自带的 NiViewer.exe 确认深度能出，再回来跑 --list-cameras");
                ConsoleIo.WriteLine("  - 若提示 OpenNI2.dll 加载失败（126），装 VC++ 2013 x64 运行库（msvcr120.dll）");
            }
            return 0;
        }

        private static string DescribeBackend(CameraOption option)
        {
            if (option.Kind == CameraKind.OpenNi2)
                return string.Equals(option.Backend, "native", StringComparison.OrdinalIgnoreCase)
                    ? "OpenNI2 原生 C API（OpenNI2.dll）"
                    : "OpenNI2 托管包装（OpenNI2.Net.dll）";
            if (option.Kind == CameraKind.RealSense) return "Intel RealSense（Intel.RealSense.dll）";
            if (option.Kind == CameraKind.Null) return "合成帧";
            return option.Kind.ToString();
        }

        private static void PrintExportCloudUsage()
        {
            ConsoleIo.WriteLine("用法：Acquire.exe --export-cloud <take目录> [--stride 4] [--frame 120] [--radar-frame] [--csv] [--out 目录]");
            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("  --stride N     每 N×N 像素取 1 点（默认 4，范围 1~64）");
            ConsoleIo.WriteLine("  --frame N      只导出第 N 帧（默认导出全部；N 与 depth\\NNNNNN.png 的编号一致）");
            ConsoleIo.WriteLine("  --radar-frame  额外输出雷达坐标系版本到 cloud_radar\\（Xr=Xc, Yr=Zc, Zr=-Yc）");
            ConsoleIo.WriteLine("  --csv          输出 X,Y,Z 文本而不是二进制 PLY");
            ConsoleIo.WriteLine("  --out 目录     输出目录（默认 <take>\\cloud）");
        }

        /// <summary>
        /// --export-cloud：读 take 里的 depth\*.png + meta.json 内参，离线生成 cloud\*.ply（或 --csv 的 X,Y,Z）。
        /// 退出码：0 成功；1 运行失败；2 参数/结构错误。
        /// </summary>
        private static int ExportCloud(string[] args, string takeDirectory)
        {
            if (string.IsNullOrEmpty(takeDirectory) || takeDirectory.StartsWith("--", StringComparison.Ordinal))
            {
                ConsoleIo.WriteLine("缺少 take 目录。");
                PrintExportCloudUsage();
                return 2;
            }

            string error;
            int stride;
            if (!TryIntArg(args, "--stride", DepthCloud.DefaultStride, out stride, out error))
            {
                ConsoleIo.WriteLine(error);
                PrintExportCloudUsage();
                return 2;
            }
            int frame;
            if (!TryIntArg(args, "--frame", 0, out frame, out error))
            {
                ConsoleIo.WriteLine(error);
                PrintExportCloudUsage();
                return 2;
            }
            if (frame < 0)
            {
                ConsoleIo.WriteLine("--frame 必须 >= 1（当前 " + frame + "）");
                return 2;
            }

            CloudExportOptions options = new CloudExportOptions();
            options.TakeDirectory = takeDirectory;
            options.Stride = stride;
            options.Frame = frame;
            options.RadarFrame = HasArg(args, "--radar-frame");
            options.Csv = HasArg(args, "--csv");
            string outDirectory = ValueOf(args, "--out");
            if (!string.IsNullOrEmpty(outDirectory) && !outDirectory.StartsWith("--", StringComparison.Ordinal))
                options.OutputDirectory = outDirectory;
            options.Log = delegate(string line) { ConsoleIo.WriteLine(line); };

            int code = CloudExporter.Run(options);
            if (code != 0) ConsoleIo.WriteLine("导出失败（退出码 " + code + "）");
            return code;
        }

        /// <summary>
        /// --capture：命令行跑一次采集，用来在真机上确认内参取到了哪一档、深度帧率是否正常。
        /// 相机序号对应 --list-cameras 里 [相机] 段可用设备的顺序（1 起），或写 null 用合成帧。
        /// </summary>
        private static int Capture(string[] args)
        {
            ConsoleIo.WriteLine("Acquire 命令行采集 " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            ConsoleIo.WriteLine(new string('=', 62));

            List<CameraOption> options = CameraProbe.Probe(delegate(string line) { ConsoleIo.WriteLine("  " + line); });
            List<CameraOption> real = new List<CameraOption>();
            CameraOption synthetic = null;
            foreach (CameraOption option in options)
            {
                if (option.Kind == CameraKind.Null) synthetic = option;
                else if (option.Available) real.Add(option);
            }

            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("[可用相机]");
            for (int i = 0; i < real.Count; i++)
            {
                CameraOption option = real[i];
                ConsoleIo.WriteLine("  " + (i + 1) + ") " + option.Model + "（" + option.Display + "）　彩色=" +
                                    (option.HasColor ? "有" : "无") + " 深度=" + (option.HasDepth ? "有" : "无"));
            }
            if (real.Count == 0) ConsoleIo.WriteLine("  （没有真实相机；可用 --camera null 跑合成帧）");

            string cameraArg = ValueOf(args, "--camera");
            bool cameraArgIsFlag = !string.IsNullOrEmpty(cameraArg) && cameraArg.StartsWith("--", StringComparison.Ordinal);
            CameraOption camera = null;
            if (string.IsNullOrEmpty(cameraArg) || cameraArgIsFlag)
            {
                camera = real.Count > 0 ? real[0] : synthetic;
                if (camera != null) ConsoleIo.WriteLine("未指定 --camera，使用：" + camera.Model);
            }
            else if (string.Equals(cameraArg, "null", StringComparison.OrdinalIgnoreCase) ||
                     string.Equals(cameraArg, "synthetic", StringComparison.OrdinalIgnoreCase))
            {
                camera = synthetic;
            }
            else
            {
                int index;
                if (!int.TryParse(cameraArg, NumberStyles.Integer, CultureInfo.InvariantCulture, out index) ||
                    index < 1 || index > real.Count)
                {
                    ConsoleIo.WriteLine("--camera " + cameraArg + " 无效：可用序号 1~" + real.Count + "，或写 null 用合成帧");
                    return 2;
                }
                camera = real[index - 1];
            }
            if (camera == null)
            {
                ConsoleIo.WriteLine("没有可用相机，无法采集");
                return 1;
            }

            string error;
            int delay;
            int duration;
            if (!TryIntArg(args, "--delay", 1, out delay, out error) || delay < 0 || delay > 60)
            {
                ConsoleIo.WriteLine(string.IsNullOrEmpty(error) ? "--delay 必须在 0~60 秒" : error);
                return 2;
            }
            if (!TryIntArg(args, "--duration", 3, out duration, out error) || duration < 1 || duration > 600)
            {
                ConsoleIo.WriteLine(string.IsNullOrEmpty(error) ? "--duration 必须在 1~600 秒" : error);
                return 2;
            }

            string outRoot = ValueOf(args, "--out");
            if (string.IsNullOrEmpty(outRoot) || outRoot.StartsWith("--", StringComparison.Ordinal))
                outRoot = DefaultTakeRoot();
            string label = ValueOf(args, "--label");
            if (!string.IsNullOrEmpty(label) && label.StartsWith("--", StringComparison.Ordinal)) label = null;

            SessionOptions session = new SessionOptions();
            session.OutputRoot = outRoot;
            session.Label = string.IsNullOrEmpty(label) ? "cli" : label;
            session.DelaySeconds = delay;
            session.DurationSeconds = duration;
            session.RadarEnabled = !HasArg(args, "--no-radar");
            session.SaveCloud = HasArg(args, "--cloud");
            session.SoundCues = HasArg(args, "--cues");
            string serial = ValueOf(args, "--serial");
            if (!string.IsNullOrEmpty(serial) && !serial.StartsWith("--", StringComparison.Ordinal))
                session.SerialPortName = serial;
            string baudText = ValueOf(args, "--baud");
            int baud;
            if (!string.IsNullOrEmpty(baudText) && int.TryParse(baudText, out baud) && baud > 0)
                session.SerialBaud = baud;
            session.Camera = camera;
            session.SelfTest = camera.Kind == CameraKind.Null;

            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("开始采集：相机=" + camera.Display + "，延迟 " + delay + " s，时长 " + duration + " s，雷达=" +
                                (session.RadarEnabled ? "启用" : "关闭") + "，点云=" + (session.SaveCloud ? "存" : "不存"));

            string takeDirectory;
            string sessionError;
            using (SessionController controller = new SessionController())
            {
                controller.Log += delegate(string line) { ConsoleIo.WriteLine(line); };
                try
                {
                    controller.Start(session);
                }
                catch (Exception ex)
                {
                    ConsoleIo.WriteLine("启动失败：" + ex.Message);
                    return 2;
                }

                DateTime deadline = DateTime.UtcNow.AddSeconds(delay + duration + 180);
                while (controller.IsBusy && DateTime.UtcNow < deadline) Thread.Sleep(100);
                if (controller.IsBusy)
                {
                    ConsoleIo.WriteLine("会话超时未结束，正在停止");
                    controller.Stop();
                    Thread.Sleep(3000);
                }
                takeDirectory = controller.TakeDirectory;
                sessionError = controller.Snapshot().Error;
            }

            if (!string.IsNullOrEmpty(sessionError))
            {
                ConsoleIo.WriteLine("采集失败：" + sessionError);
                return 1;
            }
            if (string.IsNullOrEmpty(takeDirectory))
            {
                ConsoleIo.WriteLine("没有生成 take 目录");
                return 1;
            }
            PrintTakeSummary(takeDirectory);
            return 0;
        }

        private static void PrintTakeSummary(string takeDirectory)
        {
            ConsoleIo.WriteLine("");
            ConsoleIo.WriteLine("[结果] take：" + takeDirectory);
            string metaPath = Path.Combine(takeDirectory, "meta.json");
            if (!File.Exists(metaPath))
            {
                ConsoleIo.WriteLine("       缺少 meta.json");
                return;
            }
            try
            {
                JsonObject meta = MiniJson.Parse(File.ReadAllText(metaPath, Encoding.UTF8)) as JsonObject;
                JsonObject camera = meta.Get("camera", null) as JsonObject;
                JsonObject depth = meta.Get("depth", null) as JsonObject;
                JsonObject cloud = depth == null ? null : depth.Get("cloud", null) as JsonObject;
                JsonObject intrinsics = camera == null ? null : camera.Get("depthIntrinsics", null) as JsonObject;

                ConsoleIo.WriteLine("       深度帧：" + FormatNumber(GetNumber(depth, "frames", 0)) +
                                    "　深度 fps：" + FormatNumber(GetNumber(depth, "fps", 0)) +
                                    "　点云帧：" + FormatNumber(GetNumber(cloud, "frames", 0)) +
                                    "　点数：" + FormatNumber(GetNumber(cloud, "points", 0)));
                if (intrinsics == null)
                {
                    ConsoleIo.WriteLine("       内参：无（该 take 没有深度内参）");
                    return;
                }
                ConsoleIo.WriteLine("       内参：fx=" + FormatNumber(GetNumber(intrinsics, "fx", 0)) +
                                    " fy=" + FormatNumber(GetNumber(intrinsics, "fy", 0)) +
                                    " cx=" + FormatNumber(GetNumber(intrinsics, "cx", 0)) +
                                    " cy=" + FormatNumber(GetNumber(intrinsics, "cy", 0)) +
                                    " " + FormatNumber(GetNumber(intrinsics, "width", 0)) + "x" +
                                    FormatNumber(GetNumber(intrinsics, "height", 0)) +
                                    " depthScale=" + FormatNumber(GetNumber(intrinsics, "depthScale", 0)) +
                                    " source=" + (intrinsics.Get("source", null) as string));
                string note = intrinsics.Get("note", null) as string;
                if (!string.IsNullOrEmpty(note)) ConsoleIo.WriteLine("       来源：" + note);
            }
            catch (Exception ex)
            {
                ConsoleIo.WriteLine("       读取 meta.json 失败：" + ex.Message);
            }
        }

        private static string FormatNumber(double value)
        {
            return value.ToString("0.####", CultureInfo.InvariantCulture);
        }

        private static string DefaultTakeRoot()
        {
            string exeDir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
            try
            {
                DirectoryInfo bin = new DirectoryInfo(exeDir);
                if (bin.Parent != null && string.Equals(bin.Name, "bin", StringComparison.OrdinalIgnoreCase))
                    return Path.Combine(bin.Parent.FullName, "takes");
            }
            catch { }
            return Path.Combine(exeDir, "takes");
        }

        private static bool TryIntArg(string[] args, string name, int fallback, out int value, out string error)
        {
            value = fallback;
            error = null;
            string text = ValueOf(args, name);
            if (text == null) return true;
            if (text.StartsWith("--", StringComparison.Ordinal))
            {
                error = name + " 缺少数值";
                return false;
            }
            if (!int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value))
            {
                error = name + " 必须是整数，收到\"" + text + "\"";
                return false;
            }
            return true;
        }

        private static int VerifyTake(string directory)
        {
            if (!Directory.Exists(directory))
            {
                ConsoleIo.WriteLine("目录不存在：" + directory);
                return 2;
            }
            string metaPath = Path.Combine(directory, "meta.json");
            if (!File.Exists(metaPath))
            {
                ConsoleIo.WriteLine("缺少 meta.json");
                return 2;
            }

            JsonObject meta = MiniJson.Parse(File.ReadAllText(metaPath, Encoding.UTF8)) as JsonObject;
            JsonObject radar = meta == null ? null : meta.Get("radar", null) as JsonObject;
            int packets = (int)GetNumber(radar, "packetsPerFrame", 2048);
            int payload = (int)GetNumber(radar, "payloadBytes", 2048);
            int delay = (int)GetNumber(meta, "delaySeconds", 0);
            int duration = (int)GetNumber(meta, "requestedDurationSeconds", 0);
            bool selfTest = meta != null && meta.Get("selfTest", null) is bool && (bool)meta.Get("selfTest", false);

            TakeValidation validation = TakeValidator.Validate(directory, packets, payload, selfTest, delay, duration);
            ConsoleIo.Write(validation.Report());
            return validation.Ok ? 0 : 1;
        }

        private static double GetNumber(JsonObject obj, string key, double fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            if (value == null) return fallback;
            try { return Convert.ToDouble(value, CultureInfo.InvariantCulture); }
            catch { return fallback; }
        }

        private static bool HasArg(string[] args, string name)
        {
            foreach (string a in args)
            {
                if (string.Equals(a, name, StringComparison.OrdinalIgnoreCase)) return true;
            }
            return false;
        }

        private static string ValueOf(string[] args, string name)
        {
            for (int i = 0; i < args.Length; i++)
            {
                if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
                    return i + 1 < args.Length ? args[i + 1] : "";
            }
            return null;
        }
    }
}
