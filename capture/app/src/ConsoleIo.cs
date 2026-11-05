using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// winexe 默认不连控制台，但 --self-test 需要把过程打出来。
    /// 优先级：已被重定向（管道/文件）→ 直接用；父进程有控制台 → Attach；
    /// 都没有 → 新开一个控制台窗口。所有输出同时由调用方写日志文件兜底。
    /// </summary>
    internal static class ConsoleIo
    {
        private const int StdOutputHandle = -11;
        private const uint FileTypeUnknown = 0;
        private const uint FileTypeDisk = 1;
        private const uint FileTypeChar = 2;
        private const uint FileTypePipe = 3;

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool AttachConsole(int processId);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool AllocConsole();

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr GetStdHandle(int handle);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern uint GetFileType(IntPtr handle);

        private static readonly object Gate = new object();
        private static bool _ready;

        public static void Ensure()
        {
            lock (Gate)
            {
                if (_ready) return;
                _ready = true;
                try
                {
                    IntPtr handle = GetStdHandle(StdOutputHandle);
                    uint type = GetFileType(handle) & 0xFF;
                    if (type == FileTypePipe || type == FileTypeDisk || type == FileTypeChar)
                    {
                        Rebind();
                        return;
                    }
                    if (!AttachConsole(-1) && !AllocConsole())
                    {
                        return;
                    }
                    Rebind();
                }
                catch
                {
                    // 控制台不可用时静默；日志文件仍然存在
                }
            }
        }

        private static void Rebind()
        {
            try
            {
                Stream stdout = Console.OpenStandardOutput();
                StreamWriter writer = new StreamWriter(stdout, new UTF8Encoding(false));
                writer.AutoFlush = true;
                Console.SetOut(writer);
            }
            catch { }
        }

        public static void WriteLine(string text)
        {
            Ensure();
            try
            {
                Console.WriteLine(text);
            }
            catch { }
        }

        public static void Write(string text)
        {
            Ensure();
            try
            {
                Console.Write(text);
            }
            catch { }
        }
    }
}
