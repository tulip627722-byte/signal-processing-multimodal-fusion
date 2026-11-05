using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal sealed class MatInspection
    {
        public long Rows;
        public int Cols;
        public bool IsCell;
        public string VarName;
        public string Description;
        public int PayloadBytes;
        public ushort Type;
        public string FirstStamp;
        public string LastStamp;
        public uint FirstId;
        public uint LastId;
        public long ElementCount;
        public byte[] FirstPayloadHead;
        public byte[] LastPayloadHead;
        public long FileBytes;
    }

    /// <summary>
    /// 自检用的 MAT v5 结构校验器：不依赖 MATLAB，也不依赖第三方库。
    /// 逐元素走完整个 cell，任何长度/类/维度不一致都会抛异常，
    /// 相当于把 load(...,'ADCData') 之后会用到的东西都核对一遍。
    /// </summary>
    internal static class MatVerifier
    {
        private const uint miInt8 = 1;
        private const uint miUInt8 = 2;
        private const uint miUInt16 = 4;
        private const uint miInt32 = 5;
        private const uint miUInt32 = 6;
        private const uint miUtf8 = 16;
        private const uint miMatrix = 14;

        private static bool _littleEndian = true;

        public static MatInspection Verify(string path, int expectedPayloadBytes, ushort expectedType)
        {
            MatInspection result = new MatInspection();
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read,
                       1024 * 1024, FileOptions.SequentialScan))
            using (BinaryReader br = new BinaryReader(fs, Encoding.ASCII))
            {
                result.FileBytes = fs.Length;
                if (fs.Length < 256) throw new InvalidDataException("MAT 文件过短：" + fs.Length + " 字节");

                byte[] header = br.ReadBytes(128);
                string magic = Encoding.ASCII.GetString(header, 0, Math.Min(116, header.Length)).TrimEnd(' ', '\0');
                if (!magic.StartsWith("MATLAB 5.0 MAT-file", StringComparison.Ordinal))
                    throw new InvalidDataException("不是 MATLAB 5 格式文件，头信息：" + magic);
                if (header[126] != 'I' || header[127] != 'M')
                    throw new InvalidDataException("字节序标记不是 IM");
                _littleEndian = true; // 本工程与现有 GUI 均按小端写；IM 即小端
                result.Description = magic;

                uint type;
                uint bytes;
                ReadTag(br, out type, out bytes);
                if (type != miMatrix) throw new InvalidDataException("顶层元素不是 miMATRIX，实际 type=" + type);
                long outerEnd = fs.Position + bytes;
                if (outerEnd > fs.Length) throw new InvalidDataException("顶层元素长度越界");

                uint flags;
                ReadSmallTag(br, miUInt32, 8, out flags);
                result.IsCell = ((flags & 0xFF) == 1);

                int rows, cols;
                ReadDimensions(br, out rows, out cols);
                result.Rows = rows;
                result.Cols = cols;

                ReadName(br, out result.VarName);

                if (!result.IsCell) throw new InvalidDataException("ADCData 不是 cell（array flags=" + flags + "）");
                if (cols != 4) throw new InvalidDataException("ADCData 列数应为 4，实际 " + cols);

                long index = 0;
                uint previousId = 0;
                bool hasPreviousId = false;
                byte[] head = new byte[16];

                while (fs.Position < outerEnd)
                {
                    long before = fs.Position;
                    int col = (int)(index / rows);
                    int row = (int)(index % rows);
                    if (col > 3) throw new InvalidDataException("cell 元素多于 4 列");

                    long elemEnd;
                    uint cls, elemRows, elemCols;
                    ReadNestedHeader(br, out elemEnd, out cls, out elemRows, out elemCols);

                    if (col == 0)
                    {
                        if (cls != 4) throw new InvalidDataException("第 1 列应为 char，实际 class=" + cls);
                        string value = ReadCharPayload(br, (int)elemRows, (int)elemCols);
                        DateTime parsed;
                        if (!DateTime.TryParseExact(value, "yyyyMMddHHmmss.fff", CultureInfo.InvariantCulture,
                                DateTimeStyles.None, out parsed))
                            throw new InvalidDataException("时间戳无法按 yyyyMMddHHmmss.fff 解析：" + value);
                        if (row == 0) result.FirstStamp = value;
                        if (row == rows - 1) result.LastStamp = value;
                    }
                    else if (col == 1)
                    {
                        if (cls != 13) throw new InvalidDataException("第 2 列应为 uint32，实际 class=" + cls);
                        uint id = ReadScalarUInt32(br);
                        if (hasPreviousId && id <= previousId)
                            throw new InvalidDataException("packetId 非递增：" + previousId + " → " + id + "（行 " + (row + 1) + "）");
                        previousId = id;
                        hasPreviousId = true;
                        if (row == 0) result.FirstId = id;
                        if (row == rows - 1) result.LastId = id;
                    }
                    else if (col == 2)
                    {
                        if (cls != 11) throw new InvalidDataException("第 3 列应为 uint16，实际 class=" + cls);
                        ushort value = ReadScalarUInt16(br);
                        if (value != expectedType)
                            throw new InvalidDataException("type 应为 0x" + expectedType.ToString("X4") + "，实际 0x" + value.ToString("X4"));
                        result.Type = value;
                    }
                    else
                    {
                        if (cls != 9) throw new InvalidDataException("第 4 列应为 uint8，实际 class=" + cls);
                        if (elemRows != 1 || elemCols != (uint)expectedPayloadBytes)
                            throw new InvalidDataException("payload 应为 1×" + expectedPayloadBytes +
                                                           "，实际 " + elemRows + "×" + elemCols);
                        int available = Math.Min(16, expectedPayloadBytes);
                        byte[] data = ReadBytesPayload(br, available, expectedPayloadBytes);
                        if (row == 0) { result.FirstPayloadHead = data; result.PayloadBytes = (int)elemCols; }
                        if (row == rows - 1) result.LastPayloadHead = data;
                    }

                    if (fs.Position != elemEnd)
                        throw new InvalidDataException("cell 元素 " + index + " 长度不一致（期望结束于 " + elemEnd + "，实际 " + fs.Position + "）");
                    if (fs.Position <= before) throw new InvalidDataException("解析未推进，文件损坏");
                    index++;
                }

                if (fs.Position != outerEnd) throw new InvalidDataException("cell 尾部越界");
                if (outerEnd != fs.Length) throw new InvalidDataException("文件在 cell 之后还有多余数据");
                if (index != (long)rows * cols)
                    throw new InvalidDataException("cell 元素数应为 " + ((long)rows * cols) + "，实际 " + index);

                result.ElementCount = index;
                return result;
            }
        }

        private static void ReadTag(BinaryReader br, out uint type, out uint bytes)
        {
            type = ReadUInt32Ordered(br);
            uint rawBytes = ReadUInt32Ordered(br);
            if (rawBytes > int.MaxValue) throw new InvalidDataException("元素长度异常：" + rawBytes);
            bytes = rawBytes;
            if (type == 0 && bytes == 0) throw new InvalidDataException("遇到空 tag");
        }

        private static void ReadSmallTag(BinaryReader br, uint expectedType, uint expectedBytes, out uint firstWord)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != expectedType) throw new InvalidDataException("tag 类型应为 " + expectedType + "，实际 " + type);
            if (bytes != expectedBytes) throw new InvalidDataException("tag 长度应为 " + expectedBytes + "，实际 " + bytes);
            firstWord = ReadUInt32Ordered(br);
            for (int i = 4; i < bytes; i++) br.ReadByte();
        }

        private static void ReadDimensions(BinaryReader br, out int rows, out int cols)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miInt32) throw new InvalidDataException("维度 tag 类型应为 miINT32");
            if (bytes != 8) throw new InvalidDataException("维度长度应为 8，实际 " + bytes);
            rows = (int)ReadUInt32Ordered(br);
            cols = (int)ReadUInt32Ordered(br);
            if (rows < 0 || cols < 0) throw new InvalidDataException("维度为负");
        }

        private static void ReadName(BinaryReader br, out string name)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miInt8) throw new InvalidDataException("变量名 tag 类型应为 miINT8");
            byte[] raw = br.ReadBytes((int)bytes);
            for (int i = (int)bytes; i < Align8(bytes); i++) br.ReadByte();
            name = Encoding.ASCII.GetString(raw);
        }

        private static void ReadNestedHeader(BinaryReader br, out long end, out uint cls, out uint rows, out uint cols)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miMatrix) throw new InvalidDataException("cell 内元素不是 miMATRIX");
            end = br.BaseStream.Position + bytes;

            uint flags;
            ReadSmallTag(br, miUInt32, 8, out flags);
            cls = flags & 0xFF;

            int r, c;
            ReadDimensions(br, out r, out c);
            rows = (uint)r;
            cols = (uint)c;

            string ignored;
            ReadName(br, out ignored);
        }

        private static string ReadCharPayload(BinaryReader br, int rows, int cols)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miUtf8 && type != miInt8) throw new InvalidDataException("字符内容 tag 类型异常：" + type);
            byte[] raw = br.ReadBytes((int)bytes);
            for (int i = (int)bytes; i < Align8(bytes); i++) br.ReadByte();
            if (rows * cols != raw.Length) throw new InvalidDataException("字符维度与内容长度不符");
            return Encoding.UTF8.GetString(raw);
        }

        private static uint ReadScalarUInt32(BinaryReader br)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miUInt32) throw new InvalidDataException("标量 tag 类型应为 miUINT32");
            if (bytes != 4) throw new InvalidDataException("标量长度应为 4");
            uint value = ReadUInt32Ordered(br);
            br.ReadBytes(4); // 补齐
            return value;
        }

        private static ushort ReadScalarUInt16(BinaryReader br)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miUInt16) throw new InvalidDataException("标量 tag 类型应为 miUINT16");
            if (bytes != 2) throw new InvalidDataException("标量长度应为 2");
            byte hi = br.ReadByte();
            byte lo = br.ReadByte();
            br.ReadBytes(6); // 2 字节 value 补齐到 8 + 4 字节补齐
            return _littleEndian ? (ushort)((lo << 8) | hi) : (ushort)((hi << 8) | lo);
        }

        private static byte[] ReadBytesPayload(BinaryReader br, int keep, int total)
        {
            uint type, bytes;
            ReadTag(br, out type, out bytes);
            if (type != miUInt8) throw new InvalidDataException("payload tag 类型应为 miUINT8");
            if ((int)bytes != total) throw new InvalidDataException("payload 字节数应为 " + total + "，实际 " + bytes);
            byte[] head = br.ReadBytes(keep);
            int remaining = (int)Align8(bytes) - keep;
            if (remaining > 0) br.ReadBytes(remaining);
            return head;
        }

        private static long Align8(long value) { return (value + 7) / 8 * 8; }

        private static uint ReadUInt32Ordered(BinaryReader br)
        {
            byte[] b = br.ReadBytes(4);
            if (b.Length != 4) throw new EndOfStreamException("读取 MAT tag 时文件结束");
            if (_littleEndian)
                return (uint)b[0] | ((uint)b[1] << 8) | ((uint)b[2] << 16) | ((uint)b[3] << 24);
            return ((uint)b[0] << 24) | ((uint)b[1] << 16) | ((uint)b[2] << 8) | b[3];
        }
    }
}
