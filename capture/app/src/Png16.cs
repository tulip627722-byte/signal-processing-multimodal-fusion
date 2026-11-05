using System;
using System.IO;
using System.IO.Compression;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// 最小 16-bit 灰度 PNG 编码器。
    /// GDI+ 只支持 8 bpp 索引/24 bpp，无法直出 16-bit 深度图，
    /// 所以这里按 PNG 规范手写：IHDR(bitdepth=16, colortype=0) + zlib(IDAT) + IEND。
    /// </summary>
    internal static class Png16
    {
        private static readonly byte[] Signature = new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 };

        public static void Write(string path, ushort[] depth, int width, int height)
        {
            if (depth == null) throw new ArgumentNullException("depth");
            if (width <= 0 || height <= 0) throw new ArgumentException("尺寸非法");
            if (depth.Length < (long)width * height) throw new ArgumentException("深度缓冲小于尺寸");

            // 原始扫描线：每行 1 字节 filter(0) + width * 2 字节大端样本
            int rowBytes = width * 2;
            byte[] raw = new byte[height * (rowBytes + 1)];
            int offset = 0;
            for (int y = 0; y < height; y++)
            {
                raw[offset++] = 0;
                int src = y * width;
                for (int x = 0; x < width; x++)
                {
                    ushort v = depth[src + x];
                    raw[offset++] = (byte)(v >> 8);
                    raw[offset++] = (byte)(v & 0xFF);
                }
            }

            byte[] compressed;
            using (MemoryStream buffer = new MemoryStream())
            {
                // .NET Framework 的 DeflateStream 输出 raw deflate，需要自己补 zlib 头与 adler32
                buffer.WriteByte(0x78);
                buffer.WriteByte(0x9C);
                using (DeflateStream deflate = new DeflateStream(buffer, CompressionMode.Compress, true))
                {
                    deflate.Write(raw, 0, raw.Length);
                }
                uint adler = Adler32(raw);
                buffer.WriteByte((byte)(adler >> 24));
                buffer.WriteByte((byte)(adler >> 16));
                buffer.WriteByte((byte)(adler >> 8));
                buffer.WriteByte((byte)adler);
                compressed = buffer.ToArray();
            }

            using (FileStream fs = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.Read))
            {
                fs.Write(Signature, 0, Signature.Length);

                byte[] ihdr = new byte[13];
                WriteBE32(ihdr, 0, (uint)width);
                WriteBE32(ihdr, 4, (uint)height);
                ihdr[8] = 16;  // bit depth
                ihdr[9] = 0;   // color type: grayscale
                ihdr[10] = 0;  // compression
                ihdr[11] = 0;  // filter
                ihdr[12] = 0;  // interlace
                WriteChunk(fs, "IHDR", ihdr);
                WriteChunk(fs, "IDAT", compressed);
                WriteChunk(fs, "IEND", new byte[0]);
            }
        }

        private static void WriteChunk(Stream stream, string type, byte[] data)
        {
            byte[] header = new byte[8];
            WriteBE32(header, 0, (uint)data.Length);
            byte[] typeBytes = Encoding.ASCII.GetBytes(type);
            Buffer.BlockCopy(typeBytes, 0, header, 4, 4);
            stream.Write(header, 0, header.Length);
            if (data.Length > 0) stream.Write(data, 0, data.Length);

            uint crc = Crc32(0xFFFFFFFFu, header, 4, 4);
            crc = Crc32(crc, data, 0, data.Length) ^ 0xFFFFFFFFu;
            byte[] crcBytes = new byte[4];
            WriteBE32(crcBytes, 0, crc);
            stream.Write(crcBytes, 0, 4);
        }

        private static void WriteBE32(byte[] buffer, int offset, uint value)
        {
            buffer[offset] = (byte)(value >> 24);
            buffer[offset + 1] = (byte)(value >> 16);
            buffer[offset + 2] = (byte)(value >> 8);
            buffer[offset + 3] = (byte)value;
        }

        private static uint Adler32(byte[] data)
        {
            const uint mod = 65521;
            uint a = 1, b = 0;
            for (int i = 0; i < data.Length; i++)
            {
                a = (a + data[i]) % mod;
                b = (b + a) % mod;
            }
            return (b << 16) | a;
        }

        private static uint[] _crcTable;

        private static uint Crc32(uint crc, byte[] data, int offset, int count)
        {
            if (_crcTable == null)
            {
                uint[] table = new uint[256];
                for (uint n = 0; n < 256; n++)
                {
                    uint c = n;
                    for (int k = 0; k < 8; k++)
                        c = ((c & 1) != 0) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
                    table[n] = c;
                }
                _crcTable = table;
            }
            for (int i = 0; i < count; i++)
                crc = _crcTable[(crc ^ data[offset + i]) & 0xFF] ^ (crc >> 8);
            return crc;
        }

        /// <summary>自检用：读 PNG 头，确认是 16-bit 灰度图。</summary>
        public static bool InspectDepthPng(string path, out int width, out int height, out string error)
        {
            width = 0;
            height = 0;
            error = null;
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                byte[] header = new byte[33];
                if (fs.Read(header, 0, header.Length) != header.Length)
                {
                    error = "文件过短";
                    return false;
                }
                for (int i = 0; i < Signature.Length; i++)
                {
                    if (header[i] != Signature[i])
                    {
                        error = "PNG 签名不正确";
                        return false;
                    }
                }
                if (Encoding.ASCII.GetString(header, 12, 4) != "IHDR")
                {
                    error = "首个块不是 IHDR";
                    return false;
                }
                width = (int)ReadBE32(header, 16);
                height = (int)ReadBE32(header, 20);
                if (header[24] != 16)
                {
                    error = "位深不是 16（实际 " + header[24] + "）";
                    return false;
                }
                if (header[25] != 0)
                {
                    error = "颜色类型不是灰度（实际 " + header[25] + "）";
                    return false;
                }
                return true;
            }
        }

        private static uint ReadBE32(byte[] buffer, int offset)
        {
            return ((uint)buffer[offset] << 24) | ((uint)buffer[offset + 1] << 16) |
                   ((uint)buffer[offset + 2] << 8) | buffer[offset + 3];
        }
    }
}
