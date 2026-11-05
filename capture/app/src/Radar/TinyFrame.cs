using System;
using System.Text;

namespace Acquire
{
    internal sealed class TinyFramePacket
    {
        public uint Id;
        public uint Length;
        public ushort Type;
        public byte[] Payload;
    }

    /// <summary>
    /// TinyFrame 协议编解码（复制自 2.0\gui\src\TinyFrame.cs，行为保持一致）。
    /// 包头 12 字节：SOF(0x01) + id(BE u32) + payloadLen(BE u32) + type(BE u16) + checksum；
    /// payload 非空时尾部再跟一个 payload 校验字节。
    /// </summary>
    internal static class TinyFrameCodec
    {
        public const byte Sof = 0x01;
        public const ushort TypeAdc = 0x0B00;
        public const ushort TypeStart = 0xD000;
        public const ushort TypeStop = 0xD001;
        public const ushort TypePause = 0xD002;
        public const int HeaderSize = 12;

        public static byte[] BuildCommand(uint id, ushort type)
        {
            byte[] frame = new byte[HeaderSize];
            frame[0] = Sof;
            WriteUInt32BigEndian(frame, 1, id);
            WriteUInt32BigEndian(frame, 5, 0);
            WriteUInt16BigEndian(frame, 9, type);
            frame[11] = ComputeChecksum(frame, 0, 11);
            return frame;
        }

        public static byte[] BuildAdcPacket(uint id, byte[] payload)
        {
            byte[] frame = new byte[HeaderSize + payload.Length + 1];
            frame[0] = Sof;
            WriteUInt32BigEndian(frame, 1, id);
            WriteUInt32BigEndian(frame, 5, (uint)payload.Length);
            WriteUInt16BigEndian(frame, 9, TypeAdc);
            frame[11] = ComputeChecksum(frame, 0, 11);
            Buffer.BlockCopy(payload, 0, frame, HeaderSize, payload.Length);
            frame[frame.Length - 1] = ComputeChecksum(frame, HeaderSize, payload.Length);
            return frame;
        }

        public static bool TryParse(byte[] buffer, int count, out TinyFramePacket packet, out string error)
        {
            packet = null;
            error = null;
            if (count < HeaderSize)
            {
                error = "frame shorter than TinyFrame header";
                return false;
            }
            if (buffer[0] != Sof)
            {
                error = "invalid SOF";
                return false;
            }
            if (ComputeChecksum(buffer, 0, 11) != buffer[11])
            {
                error = "header checksum mismatch";
                return false;
            }

            uint payloadLength = ReadUInt32BigEndian(buffer, 5);
            long totalLength = HeaderSize + (long)payloadLength + (payloadLength > 0 ? 1 : 0);
            if (totalLength != count)
            {
                error = "length mismatch";
                return false;
            }

            if (payloadLength > 0)
            {
                byte expected = ComputeChecksum(buffer, HeaderSize, checked((int)payloadLength));
                if (expected != buffer[count - 1])
                {
                    error = "payload checksum mismatch";
                    return false;
                }
            }

            byte[] payload = new byte[payloadLength];
            if (payloadLength > 0)
            {
                Buffer.BlockCopy(buffer, HeaderSize, payload, 0, checked((int)payloadLength));
            }

            packet = new TinyFramePacket
            {
                Id = ReadUInt32BigEndian(buffer, 1),
                Length = payloadLength,
                Type = ReadUInt16BigEndian(buffer, 9),
                Payload = payload
            };
            return true;
        }

        public static int GetExpectedFrameLength(byte[] buffer, int count)
        {
            if (count < HeaderSize || buffer[0] != Sof) return -1;
            uint length = ReadUInt32BigEndian(buffer, 5);
            long total = HeaderSize + (long)length + (length > 0 ? 1 : 0);
            return total > int.MaxValue ? -1 : (int)total;
        }

        private static byte ComputeChecksum(byte[] data, int offset, int count)
        {
            byte value = 0;
            for (int i = 0; i < count; i++) value ^= data[offset + i];
            return (byte)~value;
        }

        public static uint ReadUInt32BigEndian(byte[] data, int offset)
        {
            return ((uint)data[offset] << 24) | ((uint)data[offset + 1] << 16) |
                   ((uint)data[offset + 2] << 8) | data[offset + 3];
        }

        private static ushort ReadUInt16BigEndian(byte[] data, int offset)
        {
            return (ushort)(((ushort)data[offset] << 8) | data[offset + 1]);
        }

        private static void WriteUInt32BigEndian(byte[] data, int offset, uint value)
        {
            data[offset] = (byte)(value >> 24);
            data[offset + 1] = (byte)(value >> 16);
            data[offset + 2] = (byte)(value >> 8);
            data[offset + 3] = (byte)value;
        }

        private static void WriteUInt16BigEndian(byte[] data, int offset, ushort value)
        {
            data[offset] = (byte)(value >> 8);
            data[offset + 1] = (byte)value;
        }
    }

    /// <summary>
    /// 收包瞬间打的时间戳。三个字段在同一处、同一顺序采集，避免跨线程重算造成偏移。
    /// </summary>
    internal struct RadarStamp
    {
        public double RelMs;
        public DateTime LocalTime;
        public DateTime UtcTime;
    }
}
