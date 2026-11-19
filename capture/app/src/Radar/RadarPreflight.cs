using System;
using System.Collections.Generic;
using System.Threading;

namespace Acquire
{
    internal static class RadarPreflight
    {
        public static string Verify(SessionOptions options, Action<string> log)
        {
            ManualResetEvent received = new ManualResetEvent(false);
            using (RadarTransport transport = new RadarTransport())
            {
                transport.RelativeClock = delegate { return 0; };
                transport.Log += log;
                transport.PacketReceived += delegate(TinyFramePacket packet, RadarStamp stamp)
                {
                    if (packet.Type == TinyFrameCodec.TypeAdc && packet.Payload != null &&
                        packet.Payload.Length > 0 && packet.Payload.Length % options.PayloadBytes == 0)
                        received.Set();
                };
                transport.OpenUdp(options.LocalIp, options.LocalPort, options.BoardIp);

                if (received.WaitOne(500))
                {
                    log("毫米波预检通过：板卡已在发送有效 ADC 包");
                    return options.SerialPortName ?? "";
                }

                List<PortCandidate> candidates = string.IsNullOrEmpty(options.SerialPortName)
                    ? SerialControl.AutoCandidates()
                    : new List<PortCandidate> { new PortCandidate(options.SerialPortName, "用户指定") };
                if (candidates.Count == 0)
                    throw new InvalidOperationException("毫米波预检失败：没有可用控制串口，也没有收到正在发送的 ADC 包。");

                foreach (PortCandidate candidate in candidates)
                {
                    using (SerialControl serial = new SerialControl(log))
                    {
                        string error;
                        if (!serial.TryOpen(candidate.Name, options.SerialBaud, out error))
                        {
                            log("毫米波预检：串口 " + candidate.Name + " 打开失败：" + error);
                            continue;
                        }
                        received.Reset();
                        serial.SendPause();
                        Thread.Sleep(80);
                        serial.SendStart();
                        if (received.WaitOne(2500))
                        {
                            serial.SendPause();
                            log("毫米波预检通过：有效 ADC 包来自串口候选 " + candidate.Name);
                            return candidate.Name;
                        }
                        serial.SendPause();
                    }
                }
            }
            throw new InvalidOperationException("毫米波预检失败：已尝试所有串口，但 2.5 秒内没有收到有效 ADC 包。");
        }
    }
}
