using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace Acquire
{
    internal sealed class SelfTestOptions
    {
        public int Delay = 1;
        public int Duration = 3;
        public double Fps = 2;
        public double Loss;
        public double DropFrames;
        public string OutRoot;
        public int Packets = 2048;
        public int Payload = 2048;

        public static SelfTestOptions Parse(string[] args)
        {
            SelfTestOptions o = new SelfTestOptions();
            for (int i = 0; i < args.Length; i++)
            {
                string a = args[i];
                switch (a)
                {
                    case "--delay": o.Delay = ParseInt(Next(args, ref i), o.Delay); break;
                    case "--duration": o.Duration = ParseInt(Next(args, ref i), o.Duration); break;
                    case "--fps": o.Fps = ParseDouble(Next(args, ref i), o.Fps); break;
                    case "--loss": o.Loss = ParseDouble(Next(args, ref i), o.Loss); break;
                    case "--drop-frames": o.DropFrames = ParseDouble(Next(args, ref i), o.DropFrames); break;
                    case "--out": o.OutRoot = Next(args, ref i); break;
                    case "--packets": o.Packets = ParseInt(Next(args, ref i), o.Packets); break;
                    case "--payload": o.Payload = ParseInt(Next(args, ref i), o.Payload); break;
                }
            }
            if (o.Delay < 0) o.Delay = 0;
            if (o.Delay > 60) o.Delay = 60;
            if (o.Duration < 1) o.Duration = 1;
            if (o.Duration > 600) o.Duration = 600;
            if (o.Fps <= 0) o.Fps = 2;
            return o;
        }

        internal static string Next(string[] args, ref int i)
        {
            if (i + 1 >= args.Length) return null;
            i++;
            return args[i];
        }

        internal static int ParseInt(string text, int fallback)
        {
            int value;
            return int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value) ? value : fallback;
        }

        internal static double ParseDouble(string text, double fallback)
        {
            double value;
            return double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ? value : fallback;
        }
    }

    /// <summary>
    /// --self-test：无雷达、无真相机也能跑通全链路。
    /// 合成雷达走真实 UDP 回环，相机用 NullSource，会话走真实状态机（含倒计时/自动停止/落盘），
    /// 最后用 TakeValidator 把结构、时间轴、MAT、PNG 位深全部核对一遍。
    /// </summary>
    internal static class SelfTestRunner
    {
        public static int Run(SelfTestOptions options)
        {
            ConsoleIo.Ensure();
            StringBuilder journal = new StringBuilder();
            object journalGate = new object();

            Action<string> log = delegate(string line)
            {
                lock (journalGate) journal.AppendLine(line);
                ConsoleIo.WriteLine(line);
            };

            string root = options.OutRoot;
            if (string.IsNullOrEmpty(root)) root = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "selftest");
            root = Path.GetFullPath(root);
            Directory.CreateDirectory(root);

            string takeDir = null;
            try
            {
                log("=== Acquire --self-test ===");
                log("时间：" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
                log("exe：" + System.Windows.Forms.Application.ExecutablePath);
                log("输出根目录：" + root);
                log("参数：delay=" + options.Delay + " s，duration=" + options.Duration + " s，fps=" +
                    options.Fps.ToString("0.###") + "，packets/frame=" + options.Packets +
                    "，payload=" + options.Payload + " B，模拟丢包=" + options.Loss.ToString("0.####") +
                    "，整帧丢弃率=" + options.DropFrames.ToString("0.###"));

                log("");
                log("-- 相机探测 --");
                List<CameraOption> cameras = CameraProbe.Probe(log);
                CameraOption synthetic = null;
                foreach (CameraOption option in cameras)
                {
                    log("  " + option.Display + " | 可用=" + option.Available +
                        " | 彩色=" + option.HasColor + " 深度=" + option.HasDepth +
                        " | " + option.Reason);
                    if (option.Kind == CameraKind.Null && synthetic == null) synthetic = option;
                }
                if (synthetic == null)
                {
                    log("FAIL：没有 NullSource，无法进行无硬件自检");
                    return 1;
                }

                int port = FindFreeUdpPort();
                int plannedFrames = (int)Math.Floor(options.Duration * options.Fps);
                if (plannedFrames < 1) plannedFrames = 1;
                log("");
                log("-- 启动合成雷达 --");
                log("  127.0.0.1:" + port + "，" + options.Packets + " 包/帧 × " + options.Payload +
                    " B，" + options.Fps + " fps，计划 " + plannedFrames + " 帧，整帧丢弃率 " + options.DropFrames.ToString("0.####"));

                using (SyntheticRadar radar = new SyntheticRadar("127.0.0.1", port, options.Packets,
                           options.Payload, options.Fps, options.Loss, options.DropFrames, plannedFrames))
                {
                    SessionOptions session = new SessionOptions
                    {
                        OutputRoot = root,
                        Label = "selftest",
                        DelaySeconds = options.Delay,
                        DurationSeconds = options.Duration,
                        RadarEnabled = true,
                        LocalIp = "127.0.0.1",
                        LocalPort = port,
                        BoardIp = "",
                        PacketsPerFrame = options.Packets,
                        PayloadBytes = options.Payload,
                        Camera = synthetic,
                        SelfTest = true,
                        SaveCloud = true
                    };

                    using (SessionController controller = new SessionController())
                    {
                        controller.Log += log;
                        controller.RecordingStarted += delegate
                        {
                            log("  会话进入 Recording，合成雷达开始发送");
                            radar.Start();
                        };

                        DateTime startedAt = DateTime.UtcNow;
                        controller.Start(session);

                        DateTime deadline = startedAt.AddSeconds(options.Delay + options.Duration + 120);
                        while (controller.IsBusy && DateTime.UtcNow < deadline) Thread.Sleep(100);
                        if (controller.IsBusy)
                        {
                            log("FAIL：会话超时未结束，强制停止");
                            controller.Stop();
                            Thread.Sleep(2000);
                            return 1;
                        }
                        radar.Stop();
                        takeDir = controller.TakeDirectory;
                        log("  合成雷达：发送 " + radar.Sent + " 包，主动丢弃 " + radar.Dropped + " 包");
                    }
                }

                if (string.IsNullOrEmpty(takeDir))
                {
                    log("FAIL：没有生成 take 目录");
                    return 1;
                }

                log("");
                log("-- 校验 take 目录 --");
                log("  " + takeDir);
                TakeValidation validation = TakeValidator.Validate(takeDir, options.Packets, options.Payload,
                    true, options.Delay, options.Duration);
                foreach (string line in validation.Passed) log("  PASS  " + line);
                foreach (string line in validation.Notes) log("  INFO  " + line);
                foreach (string line in validation.Failed) log("  FAIL  " + line);

                SelfTestRunnerCloud.VerifyClouds(log, takeDir, validation);

                log("");
                log(validation.Ok ? "SELF-TEST PASSED" : "SELF-TEST FAILED（" + validation.Failed.Count + " 项）");

                string reportPath = Path.Combine(takeDir, "selftest-report.txt");
                File.WriteAllText(reportPath, journal.ToString(), new UTF8Encoding(false));
                log("报告：" + reportPath);
                File.WriteAllText(Path.Combine(root, "selftest-report.txt"), journal.ToString(), new UTF8Encoding(false));

                return validation.Ok ? 0 : 1;
            }
            catch (Exception ex)
            {
                log("SELF-TEST ERROR: " + ex);
                ConsoleIo.WriteLine("SELF-TEST ERROR: " + ex.Message);
                try
                {
                    Directory.CreateDirectory(root);
                    File.WriteAllText(Path.Combine(root, "selftest-report.txt"), journal.ToString(), new UTF8Encoding(false));
                }
                catch { }
                return 1;
            }
        }

        private static int FindFreeUdpPort()
        {
            using (UdpClient probe = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
            {
                return ((IPEndPoint)probe.Client.LocalEndPoint).Port;
            }
        }
    }
}
