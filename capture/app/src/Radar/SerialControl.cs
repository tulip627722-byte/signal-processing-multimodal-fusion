using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Management;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// UART 控制通道。
    ///
    /// 板子的行为是：收到 UART 的 Start 命令才开始往 UDP 发 ADC；只绑 UDP 是收不到任何东西的。
    /// 帧格式与老 GUI（2.0\gui\src\TinyFrame.cs）完全一致：12 字节头，无 payload。
    ///   01 | id(4B BE) | len(4B BE)=0 | type(2B BE) | checksum(1B, ~XOR)
    /// 命令字：Start=0xD000，Pause=0xD002。
    /// </summary>
    internal sealed class SerialControl : IDisposable
    {
        private readonly Action<string> _log;
        private SerialPort _port;

        public SerialControl(Action<string> log)
        {
            _log = log;
        }

        public bool IsOpen { get { return _port != null && _port.IsOpen; } }
        public string PortName { get { return _port == null ? "" : _port.PortName; } }
        public int BaudRate { get; private set; }
        public int StartSent { get; private set; }
        public int PauseSent { get; private set; }

        // ---------------------------------------------------------------- 端口发现

        private static readonly string[] PreferredHints =
        {
            "USB-SERIAL", "USB SERIAL", "FTDI", "CP210", "CP2102", "SILICON LABS",
            "CH340", "CH341", "PROLIFIC", "PL2303", "USB 串行", "USB串行"
        };

        /// <summary>列出所有串口，并尽力给出描述（拿不到描述就只用端口名）。</summary>
        public static List<PortCandidate> ListPorts()
        {
            List<PortCandidate> list = new List<PortCandidate>();
            string[] names;
            try { names = SerialPort.GetPortNames(); }
            catch { names = new string[0]; }

            Dictionary<string, string> desc = DescribePorts();
            foreach (string name in names)
            {
                string d;
                if (!desc.TryGetValue(name, out d)) d = "";
                list.Add(new PortCandidate(name, d));
            }
            list.Sort(delegate(PortCandidate a, PortCandidate b) { return string.CompareOrdinal(a.Name, b.Name); });
            return list;
        }

        private static Dictionary<string, string> DescribePorts()
        {
            Dictionary<string, string> map = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            try
            {
                using (ManagementObjectSearcher searcher =
                       new ManagementObjectSearcher("SELECT Name, Caption FROM Win32_PnPEntity"))
                {
                    foreach (ManagementObject mo in searcher.Get())
                    {
                        string name = mo["Name"] as string;
                        if (string.IsNullOrEmpty(name)) continue;
                        int open = name.LastIndexOf("(COM", StringComparison.OrdinalIgnoreCase);
                        if (open < 0) continue;
                        int close = name.IndexOf(')', open);
                        if (close < 0) continue;
                        string com = name.Substring(open + 1, close - open - 1);   // COM5
                        if (!map.ContainsKey(com)) map[com] = name;
                    }
                }
            }
            catch
            {
                // WMI 不可用就用端口名兜底，不影响功能
            }
            return map;
        }

        /// <summary>自动挑一个串口：只有一个就直接用；多个时优先 USB 串口芯片。</summary>
        public static string AutoDetect(out string reason)
        {
            List<PortCandidate> ports = ListPorts();
            if (ports.Count == 0)
            {
                reason = "系统里没有任何串口（SerialPort.GetPortNames() 返回空）";
                return null;
            }
            if (ports.Count == 1)
            {
                reason = "只有一个串口，直接使用：" + ports[0].Name;
                return ports[0].Name;
            }

            foreach (PortCandidate p in ports)
            {
                string text = (p.Name + " " + p.Description).ToUpperInvariant();
                foreach (string hint in PreferredHints)
                {
                    if (text.Contains(hint.ToUpperInvariant()))
                    {
                        reason = "按芯片特征选中 " + p.Name + "（" + p.Description + "）";
                        return p.Name;
                    }
                }
            }

            StringBuilder sb = new StringBuilder();
            foreach (PortCandidate p in ports) sb.Append(p.Name).Append(' ');
            reason = "有多个串口但都不像 USB 串口：" + sb.ToString().Trim() +
                     "；请用 --serial COMx 指定";
            return null;
        }

        /// <summary>自动模式下返回全部候选，USB 串口优先；不能只猜一个，否则换电脑后很容易选错。</summary>
        public static List<PortCandidate> AutoCandidates()
        {
            List<PortCandidate> ports = ListPorts();
            ports.Sort(delegate(PortCandidate a, PortCandidate b)
            {
                int preferred = IsPreferred(b).CompareTo(IsPreferred(a));
                return preferred != 0 ? preferred : string.CompareOrdinal(a.Name, b.Name);
            });
            return ports;
        }

        private static bool IsPreferred(PortCandidate port)
        {
            string text = (port.Name + " " + port.Description).ToUpperInvariant();
            foreach (string hint in PreferredHints)
            {
                if (text.Contains(hint.ToUpperInvariant())) return true;
            }
            return false;
        }

        // ---------------------------------------------------------------- 打开 / 发送

        public bool TryOpen(string portName, int baudRate, out string error)
        {
            error = null;
            try
            {
                Close();
                SerialPort serial = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
                serial.ReadTimeout = 500;
                serial.WriteTimeout = 500;
                serial.DtrEnable = true;
                serial.RtsEnable = true;
                serial.Open();
                _port = serial;
                BaudRate = baudRate;
                if (_log != null) _log("UART 已打开 " + portName + " @" + baudRate + " 8N1");
                return true;
            }
            catch (Exception ex)
            {
                error = ex.GetType().Name + ": " + ex.Message;
                return false;
            }
        }

        public bool SendStart()
        {
            bool sent = SendCommand(0, TinyFrameCodec.TypeStart);
            if (sent) StartSent++;
            return sent;
        }

        public bool SendPause()
        {
            bool sent = SendCommand(2, TinyFrameCodec.TypePause);
            if (sent) PauseSent++;
            return sent;
        }

        public bool SendCommand(uint id, ushort type)
        {
            SerialPort serial = _port;
            if (serial == null || !serial.IsOpen) return false;
            byte[] frame = TinyFrameCodec.BuildCommand(id, type);
            try
            {
                serial.Write(frame, 0, frame.Length);
                if (_log != null) _log("UART TX " + ToHex(frame));
                return true;
            }
            catch (Exception ex)
            {
                if (_log != null) _log("UART 发送失败：" + ex.Message);
                return false;
            }
        }

        private static string ToHex(byte[] data)
        {
            StringBuilder sb = new StringBuilder(data.Length * 3);
            for (int i = 0; i < data.Length; i++)
            {
                if (i > 0) sb.Append(' ');
                sb.Append(data[i].ToString("X2"));
            }
            return sb.ToString();
        }

        public void Close()
        {
            SerialPort serial = _port;
            _port = null;
            if (serial == null) return;
            try { if (serial.IsOpen) serial.Close(); } catch { }
            try { serial.Dispose(); } catch { }
        }

        public void Dispose() { Close(); }
    }

    internal sealed class PortCandidate
    {
        public readonly string Name;
        public readonly string Description;

        public PortCandidate(string name, string description)
        {
            Name = name;
            Description = description == null ? "" : description;
        }

        public override string ToString()
        {
            return string.IsNullOrEmpty(Description) ? Name : Name + "  (" + Description + ")";
        }
    }
}
