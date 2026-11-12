using System;
using System.Collections.Generic;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace Acquire
{
    internal static class RadarNetwork
    {
        public static string ValidateLocalAddress(string requested)
        {
            if (string.IsNullOrWhiteSpace(requested) || requested == "0.0.0.0") return "";

            IPAddress address;
            if (!IPAddress.TryParse(requested, out address) || address.AddressFamily != AddressFamily.InterNetwork)
                throw new InvalidOperationException("本机 IP 格式无效：" + requested);
            if (IPAddress.IsLoopback(address)) return requested;

            List<string> local = ListLocalIpv4();
            foreach (string item in local)
            {
                if (string.Equals(item, requested, StringComparison.OrdinalIgnoreCase)) return requested;
            }

            string found = local.Count == 0 ? "未发现可用 IPv4 地址" : string.Join("、", local.ToArray());
            throw new InvalidOperationException(
                "当前电脑没有配置雷达要求的本机 IP " + requested + "。\r\n" +
                "当前 IPv4：" + found + "。\r\n" +
                "请给连接雷达的有线网卡添加静态地址 " + requested + "（建议掩码 255.255.255.0），然后重试。" +
                "雷达固件会固定把数据发到该地址，程序无法用其他电脑 IP 代替。");
        }

        public static string Describe(string requested)
        {
            string bind = ValidateLocalAddress(requested);
            return bind.Length == 0 ? "自动绑定全部本机网卡" : "已确认本机网卡地址 " + bind;
        }

        private static List<string> ListLocalIpv4()
        {
            List<string> result = new List<string>();
            try
            {
                foreach (NetworkInterface nic in NetworkInterface.GetAllNetworkInterfaces())
                {
                    if (nic.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;
                    foreach (UnicastIPAddressInformation info in nic.GetIPProperties().UnicastAddresses)
                    {
                        if (info.Address.AddressFamily != AddressFamily.InterNetwork) continue;
                        string value = info.Address.ToString();
                        if (!result.Contains(value)) result.Add(value);
                    }
                }
            }
            catch { }

            if (result.Count == 0)
            {
                try
                {
                    foreach (IPAddress address in Dns.GetHostAddresses(Dns.GetHostName()))
                    {
                        if (address.AddressFamily == AddressFamily.InterNetwork && !IPAddress.IsLoopback(address))
                            result.Add(address.ToString());
                    }
                }
                catch { }
            }
            return result;
        }
    }
}
