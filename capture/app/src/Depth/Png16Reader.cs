using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;

namespace Acquire
{
    internal sealed class DepthImage
    {
        public int Width;
        public int Height;
        public ushort[] Data;
        /// <summary>非致命提示，例如“PNG 是 8-bit，只做了 ×257 放大”。</summary>
        public string Warning = "";
    }

    /// <summary>
    /// 16-bit 灰度 PNG 读取器。
    ///
    /// 为什么不用 GDI+：GDI+ 的 Bitmap 不认 16 bpp 灰度（PixelFormat.Format16bppGrayScale 只存在于枚举里，
    /// 实际解码会抛“不支持”），而深度图必须是 16-bit 原始值。所以这里自己解 PNG：
    /// 签名 → IHDR → 拼 IDAT → zlib（DeflateStream 解 raw deflate，跳过 2 字节 zlib 头）
    /// → 逐行反滤波（0..4 全支持）→ 大端 16-bit 样本转 ushort。
    ///
    /// 只支持非隔行、灰度（colorType 0）与灰度+alpha（colorType 4）。
    /// 写端是 Png16.cs（filter 恒为 0），读端支持全部滤波类型是为了能读别的工具导出的深度图。
    /// </summary>
    internal static class Png16Reader
    {
        private static readonly byte[] Signature = new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 };

        public static DepthImage Read(string path)
        {
            if (!File.Exists(path)) throw new FileNotFoundException("文件不存在：" + path, path);

            byte[] file = File.ReadAllBytes(path);
            if (file.Length < 8 + 25 + 12)
                throw new InvalidDataException("文件过短，不是完整 PNG：" + path);
            for (int i = 0; i < Signature.Length; i++)
            {
                if (file[i] != Signature[i]) throw new InvalidDataException("PNG 签名不正确：" + path);
            }

            int width = 0, height = 0, bitDepth = 0, colorType = -1, interlace = 0;
            MemoryStream idat = new MemoryStream();
            int offset = 8;
            bool sawHeader = false;
            bool sawEnd = false;

            while (offset + 8 <= file.Length)
            {
                int length = ReadBE32(file, offset);
                string type = System.Text.Encoding.ASCII.GetString(file, offset + 4, 4);
                int dataStart = offset + 8;
                if (length < 0 || dataStart + length + 4 > file.Length)
                    throw new InvalidDataException("PNG 块 " + type + " 越界，文件可能被截断：" + path);

                if (type == "IHDR")
                {
                    width = ReadBE32(file, dataStart);
                    height = ReadBE32(file, dataStart + 4);
                    bitDepth = file[dataStart + 8];
                    colorType = file[dataStart + 9];
                    int compression = file[dataStart + 10];
                    int filter = file[dataStart + 11];
                    interlace = file[dataStart + 12];
                    sawHeader = true;
                    if (compression != 0) throw new InvalidDataException("PNG 压缩方法不是 0：" + path);
                    if (filter != 0) throw new InvalidDataException("PNG 滤波方法不是 0：" + path);
                    if (interlace != 0) throw new InvalidDataException("不支持隔行 PNG（Adam7），请重新导出：" + path);
                }
                else if (type == "IDAT")
                {
                    idat.Write(file, dataStart, length);
                }
                else if (type == "IEND")
                {
                    sawEnd = true;
                    break;
                }

                offset = dataStart + length + 4;
            }

            if (!sawHeader) throw new InvalidDataException("缺少 IHDR 块：" + path);
            if (!sawEnd) throw new InvalidDataException("缺少 IEND 块（文件可能不完整）：" + path);
            if (width <= 0 || height <= 0 || (long)width * height > 1L << 28)
                throw new InvalidDataException("PNG 尺寸异常：" + width + "x" + height + "（" + path + "）");

            int channels;
            if (colorType == 0) channels = 1;
            else if (colorType == 4) channels = 2;
            else throw new InvalidDataException(
                "PNG 颜色类型是 " + colorType + "，这不是灰度深度图（期望 0 或 4）：" + path);

            if (bitDepth != 16 && bitDepth != 8)
                throw new InvalidDataException("PNG 位深是 " + bitDepth + "，只支持 8 / 16：" + path);

            int bytesPerSample = bitDepth / 8;
            int rowBytes = width * channels * bytesPerSample;
            byte[] raw = Inflate(idat.ToArray(), rowBytes * height + height);
            byte[] pixels = new byte[rowBytes * height];
            Unfilter(raw, pixels, width, height, rowBytes, bytesPerSample * channels);

            DepthImage image = new DepthImage();
            image.Width = width;
            image.Height = height;
            image.Data = new ushort[width * height];
            if (bitDepth == 16)
            {
                for (int i = 0, src = 0; i < image.Data.Length; i++, src += 2 * channels)
                    image.Data[i] = (ushort)((pixels[src] << 8) | pixels[src + 1]);
            }
            else
            {
                // 8-bit 灰度：×257 映射到 0~65535。深度单位会失真，只作为“能读但值不可信”的兜底。
                for (int i = 0, src = 0; i < image.Data.Length; i++, src += channels)
                    image.Data[i] = (ushort)(pixels[src] * 257);
                image.Warning = "PNG 是 8-bit 灰度，值已放大 ×257；深度单位不可信";
            }
            return image;
        }

        private static byte[] Inflate(byte[] zlib, int expected)
        {
            if (zlib.Length < 6) throw new InvalidDataException("IDAT 数据过短，无法解压");
            // zlib 流 = 2 字节头 + raw deflate + 4 字节 adler32
            byte[] compressed = new byte[zlib.Length - 6];
            Buffer.BlockCopy(zlib, 2, compressed, 0, compressed.Length);

            byte[] result;
            try
            {
                result = Deflate(compressed);
            }
            catch
            {
                // 有的实现会带尾随字节，去掉尾巴再试一次
                byte[] full = new byte[zlib.Length - 2];
                Buffer.BlockCopy(zlib, 2, full, 0, full.Length);
                result = Deflate(full);
            }
            if (result.Length < expected)
                throw new InvalidDataException("PNG 像素数据不完整：解出 " + result.Length + " 字节，期望至少 " + expected);
            return result;
        }

        private static byte[] Deflate(byte[] data)
        {
            using (MemoryStream input = new MemoryStream(data, false))
            using (DeflateStream inflater = new DeflateStream(input, CompressionMode.Decompress))
            using (MemoryStream output = new MemoryStream())
            {
                byte[] buffer = new byte[65536];
                while (true)
                {
                    int read = inflater.Read(buffer, 0, buffer.Length);
                    if (read <= 0) break;
                    output.Write(buffer, 0, read);
                }
                return output.ToArray();
            }
        }

        private static void Unfilter(byte[] raw, byte[] output, int width, int height, int rowBytes, int bytesPerPixel)
        {
            int stride = rowBytes + 1;
            if (raw.Length < stride * height)
                throw new InvalidDataException("PNG 扫描线长度不足");

            for (int y = 0; y < height; y++)
            {
                int filter = raw[y * stride];
                int src = y * stride + 1;
                int dst = y * rowBytes;
                int up = dst - rowBytes;
                for (int x = 0; x < rowBytes; x++)
                {
                    int a = x >= bytesPerPixel ? output[dst + x - bytesPerPixel] : 0;
                    int b = y > 0 ? output[up + x] : 0;
                    int c = (y > 0 && x >= bytesPerPixel) ? output[up + x - bytesPerPixel] : 0;
                    int value = raw[src + x];
                    switch (filter)
                    {
                        case 0: break;
                        case 1: value += a; break;
                        case 2: value += b; break;
                        case 3: value += (a + b) >> 1; break;
                        case 4: value += Paeth(a, b, c); break;
                        default: throw new InvalidDataException("PNG 行滤波类型非法：" + filter + "（行 " + y + "）");
                    }
                    output[dst + x] = (byte)value;
                }
            }
        }

        private static int Paeth(int a, int b, int c)
        {
            int p = a + b - c;
            int pa = Math.Abs(p - a);
            int pb = Math.Abs(p - b);
            int pc = Math.Abs(p - c);
            if (pa <= pb && pa <= pc) return a;
            if (pb <= pc) return b;
            return c;
        }

        private static int ReadBE32(byte[] buffer, int offset)
        {
            return (buffer[offset] << 24) | (buffer[offset + 1] << 16) |
                   (buffer[offset + 2] << 8) | buffer[offset + 3];
        }

        /// <summary>列出目录下的深度图（只认 16-bit 灰度 PNG；本工程写端就是 PNG，不再另做 .bin 分支）</summary>
        public static List<string> ListDepthFiles(string directory)
        {
            List<string> files = new List<string>();
            if (!Directory.Exists(directory)) return files;
            files.AddRange(Directory.GetFiles(directory, "*.png"));
            files.Sort(StringComparer.Ordinal);
            return files;
        }
    }
}
