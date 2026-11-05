using System;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// radar_adc.mat 写入器。
    ///
    /// 事实澄清：现有 2.0 GUI 并不生成 v7.3，而是手写 MATLAB 5 格式（v5）文件，
    /// 变量 ADCData 为 N×4 cell：{接收时间(yyyyMMddHHmmss.fff 字符), packetId(uint32),
    /// type(uint16), payload(1×2048 uint8)}，cell 按列主序排列。
    /// 为了"分析脚本可直接读"，这里逐字复刻那套写法（tag/维度/label/列主序顺序完全一致）。
    /// 每帧 2048 包，所以 N = 2048 × 帧数；单帧切片即 2048×4。
    ///
    /// 性能：上一版用 BinaryWriter/BinaryReader 逐字段读写，236 MB 要 41 s（约 5.7 MB/s）。
    /// 这里改成自管理缓冲（输出 4 MB / 输入 8 MB），只做顺序读写与整块拷贝。
    /// </summary>
    internal static class MatWriter
    {
        private const uint miInt8 = 1;
        private const uint miUInt8 = 2;
        private const uint miUInt16 = 4;
        private const uint miInt32 = 5;
        private const uint miUInt32 = 6;
        private const uint miUtf8 = 16;
        private const uint miMatrix = 14;
        private const uint mxCellClass = 1;
        private const uint mxCharClass = 4;
        private const uint mxUInt8Class = 9;
        private const uint mxUInt16Class = 11;
        private const uint mxUInt32Class = 13;

        private const int OutBufferSize = 4 * 1024 * 1024;
        private const int InBufferSize = 8 * 1024 * 1024;

        public static void Write(string packetLogPath, string matPath, int payloadBytes, long packetCount, DateTime createdLocal)
        {
            long timestampMatrixBytes = 80;   // 1×18 UTF-8 字符（时间串固定 18 字符）
            long scalarMatrixBytes = 64;
            long paddedPayload = Align8(payloadBytes);
            long payloadMatrixBytes = 8 + 16 + 16 + 8 + 8 + paddedPayload;
            long bytesPerRow = timestampMatrixBytes + scalarMatrixBytes + scalarMatrixBytes + payloadMatrixBytes;
            long outerContentBytes = 16 + 16 + 16 + checked(packetCount * bytesPerRow);
            if (outerContentBytes > uint.MaxValue)
            {
                throw new InvalidOperationException(
                    "单个 MAT v5 cell 矩阵超过 4 GiB（当前 " + packetCount + " 包）。请缩短单次采集，或改用分段保存。");
            }

            using (FileStream output = new FileStream(matPath, FileMode.Create, FileAccess.Write, FileShare.Read,
                       4 * 1024 * 1024, FileOptions.SequentialScan))
            using (MatSink sink = new MatSink(output, OutBufferSize))
            using (PacketLog log = new PacketLog(packetLogPath, payloadBytes, InBufferSize))
            {
                sink.WriteHeader(createdLocal);

                byte[] outerName = Encoding.ASCII.GetBytes("ADCData");
                sink.U32(miMatrix);
                sink.U32((uint)outerContentBytes);
                sink.ArrayFlags(mxCellClass);
                sink.Dimensions(checked((int)packetCount), 4);
                sink.Name(outerName);
                sink.Flush();

                // 第 1 列：接收时间字符串
                log.Rewind();
                for (long i = 0; i < packetCount; i++)
                {
                    if (!log.Next()) throw new EndOfStreamException("包日志提前结束（时间戳列，第 " + i + " 条）");
                    string stamp = new DateTime(log.Ticks).ToString("yyyyMMddHHmmss.fff", CultureInfo.InvariantCulture);
                    WriteNestedChar(sink, stamp);
                }

                // 第 2 列：packetId
                log.Rewind();
                for (long i = 0; i < packetCount; i++)
                {
                    if (!log.Next()) throw new EndOfStreamException("包日志提前结束（packetId 列，第 " + i + " 条）");
                    WriteNestedUInt32(sink, log.Id);
                }

                // 第 3 列：type（ADC 恒为 0x0B00）
                for (long i = 0; i < packetCount; i++) WriteNestedUInt16(sink, TinyFrameCodec.TypeAdc);

                // 第 4 列：payload
                log.Rewind();
                for (long i = 0; i < packetCount; i++)
                {
                    if (!log.NextWithPayload()) throw new EndOfStreamException("包日志提前结束（payload 列，第 " + i + " 条）");
                    WriteNestedPayload(sink, log);
                }

                sink.Flush();
            }
        }

        private static long Align8(long value) { return (value + 7) / 8 * 8; }

        private static void WriteNestedChar(MatSink sink, string value)
        {
            byte[] data = Encoding.UTF8.GetBytes(value);
            uint content = checked((uint)(16 + 16 + 8 + 8 + Align8(data.Length)));
            sink.U32(miMatrix);
            sink.U32(content);
            sink.ArrayFlags(mxCharClass);
            sink.Dimensions(1, data.Length);
            sink.EmptyName();
            sink.U32(miUtf8);
            sink.U32((uint)data.Length);
            sink.Bytes(data);
            sink.Zeros((int)(Align8(data.Length) - data.Length));
        }

        private static void WriteNestedUInt32(MatSink sink, uint value)
        {
            sink.U32(miMatrix);
            sink.U32(56);
            sink.ArrayFlags(mxUInt32Class);
            sink.Dimensions(1, 1);
            sink.EmptyName();
            sink.U32(miUInt32);
            sink.U32(4);
            sink.U32(value);
            sink.U32(0);
        }

        private static void WriteNestedUInt16(MatSink sink, ushort value)
        {
            sink.U32(miMatrix);
            sink.U32(56);
            sink.ArrayFlags(mxUInt16Class);
            sink.Dimensions(1, 1);
            sink.EmptyName();
            sink.U32(miUInt16);
            sink.U32(2);
            sink.U16(value);
            sink.U16(0);
            sink.U32(0);
        }

        private static void WriteNestedPayload(MatSink sink, PacketLog log)
        {
            int length = log.PayloadBytes;
            uint content = checked((uint)(16 + 16 + 8 + 8 + Align8(length)));
            sink.U32(miMatrix);
            sink.U32(content);
            sink.ArrayFlags(mxUInt8Class);
            sink.Dimensions(1, length);
            sink.EmptyName();
            sink.U32(miUInt8);
            sink.U32((uint)length);
            sink.Payload(log.PayloadBuffer, length);
            sink.Zeros((int)(Align8(length) - length));
        }

        // ------------------------------------------------------------ 输出缓冲

        private sealed class MatSink : IDisposable
        {
            private readonly FileStream _stream;
            private readonly byte[] _buffer;
            private int _position;

            public MatSink(FileStream stream, int bufferSize)
            {
                _stream = stream;
                _buffer = new byte[bufferSize];
            }

            private void Ensure(int bytes)
            {
                if (_position + bytes <= _buffer.Length) return;
                Flush();
                if (bytes > _buffer.Length) throw new InvalidOperationException("单次写入超过缓冲区：" + bytes);
            }

            public void Flush()
            {
                if (_position <= 0) return;
                _stream.Write(_buffer, 0, _position);
                _position = 0;
            }

            public void U8(byte value)
            {
                Ensure(1);
                _buffer[_position++] = value;
            }

            public void U16(ushort value)
            {
                Ensure(2);
                _buffer[_position] = (byte)value;
                _buffer[_position + 1] = (byte)(value >> 8);
                _position += 2;
            }

            public void U32(uint value)
            {
                Ensure(4);
                _buffer[_position] = (byte)value;
                _buffer[_position + 1] = (byte)(value >> 8);
                _buffer[_position + 2] = (byte)(value >> 16);
                _buffer[_position + 3] = (byte)(value >> 24);
                _position += 4;
            }

            public void Zeros(int count)
            {
                while (count > 0)
                {
                    Ensure(1);
                    int take = Math.Min(count, _buffer.Length - _position);
                    Array.Clear(_buffer, _position, take);
                    _position += take;
                    count -= take;
                }
            }

            public void Bytes(byte[] data)
            {
                Bytes(data, 0, data.Length);
            }

            public void Bytes(byte[] data, int offset, int count)
            {
                while (count > 0)
                {
                    Ensure(1);
                    int take = Math.Min(count, _buffer.Length - _position);
                    Buffer.BlockCopy(data, offset, _buffer, _position, take);
                    _position += take;
                    offset += take;
                    count -= take;
                }
            }

            public void Payload(byte[] data, int length)
            {
                Bytes(data, 0, length);
            }

            public void ArrayFlags(uint matrixClass)
            {
                U32(miUInt32);
                U32(8);
                U32(matrixClass);
                U32(0);
            }

            public void Dimensions(int rows, int columns)
            {
                U32(miInt32);
                U32(8);
                U32((uint)rows);
                U32((uint)columns);
            }

            public void Name(byte[] name)
            {
                U32(miInt8);
                U32((uint)name.Length);
                Bytes(name);
                Zeros((int)(Align8(name.Length) - name.Length));
            }

            public void EmptyName()
            {
                U32(miInt8);
                U32(0);
            }

            public void WriteHeader(DateTime createdLocal)
            {
                byte[] header = new byte[128];
                string description = "MATLAB 5.0 MAT-file, Platform: PCWIN64, Created by Acquire GUI on " +
                                     createdLocal.ToString("ddd MMM dd HH:mm:ss yyyy", CultureInfo.InvariantCulture);
                byte[] desc = Encoding.ASCII.GetBytes(description);
                Buffer.BlockCopy(desc, 0, header, 0, Math.Min(desc.Length, 116));
                for (int i = desc.Length; i < 116; i++) header[i] = 0x20;
                header[124] = 0x00;
                header[125] = 0x01;
                header[126] = (byte)'I';
                header[127] = (byte)'M';
                Bytes(header);
            }

            public void Dispose()
            {
                try { Flush(); } catch { }
            }
        }

        // ------------------------------------------------------------ 输入缓冲

        private sealed class PacketLog : IDisposable
        {
            private readonly FileStream _stream;
            private readonly byte[] _buffer;
            private int _start;
            private int _end;
            private long _remainingRecords = -1;

            public readonly int PayloadBytes;
            public readonly byte[] PayloadBuffer;
            public long Ticks;
            public uint Id;

            public PacketLog(string path, int payloadBytes, int bufferSize)
            {
                PayloadBytes = payloadBytes;
                PayloadBuffer = new byte[payloadBytes];
                _stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read,
                    1024 * 1024, FileOptions.SequentialScan);
                _buffer = new byte[bufferSize];
                _remainingRecords = _stream.Length / (12L + payloadBytes);
                _start = 0;
                _end = 0;
            }

            public void Rewind()
            {
                _stream.Position = 0;
                _start = 0;
                _end = 0;
                _remainingRecords = _stream.Length / (12L + PayloadBytes);
            }

            public bool Next()
            {
                if (_remainingRecords <= 0) return false;
                if (!Ensure(12)) return false;
                Ticks = ReadInt64();
                Id = ReadUInt32();
                if (!Skip(PayloadBytes)) return false;
                _remainingRecords--;
                return true;
            }

            public bool NextWithPayload()
            {
                if (_remainingRecords <= 0) return false;
                if (!Ensure(12)) return false;
                Ticks = ReadInt64();
                Id = ReadUInt32();
                if (!ReadInto(PayloadBuffer, PayloadBytes)) return false;
                _remainingRecords--;
                return true;
            }

            private bool ReadInto(byte[] destination, int count)
            {
                int written = 0;
                while (written < count)
                {
                    int available = _end - _start;
                    if (available <= 0)
                    {
                        if (!Ensure(Math.Min(count - written, _buffer.Length))) return false;
                        available = _end - _start;
                    }
                    int take = Math.Min(count - written, available);
                    Buffer.BlockCopy(_buffer, _start, destination, written, take);
                    _start += take;
                    written += take;
                }
                return true;
            }

            private bool Ensure(int count)
            {
                if (_end - _start >= count) return true;
                int keep = _end - _start;
                if (keep > 0) Buffer.BlockCopy(_buffer, _start, _buffer, 0, keep);
                _start = 0;
                _end = keep;
                while (_end < count)
                {
                    int read = _stream.Read(_buffer, _end, _buffer.Length - _end);
                    if (read <= 0) return false;
                    _end += read;
                }
                return true;
            }

            private long ReadInt64()
            {
                long value = _buffer[_start] | ((long)_buffer[_start + 1] << 8) |
                             ((long)_buffer[_start + 2] << 16) | ((long)_buffer[_start + 3] << 24) |
                             ((long)_buffer[_start + 4] << 32) | ((long)_buffer[_start + 5] << 40) |
                             ((long)_buffer[_start + 6] << 48) | ((long)_buffer[_start + 7] << 56);
                _start += 8;
                return value;
            }

            private uint ReadUInt32()
            {
                uint value = (uint)(_buffer[_start] | (_buffer[_start + 1] << 8) |
                                    (_buffer[_start + 2] << 16) | (_buffer[_start + 3] << 24));
                _start += 4;
                return value;
            }

            private bool Skip(int count)
            {
                while (count > 0)
                {
                    int available = _end - _start;
                    if (available <= 0)
                    {
                        if (!Ensure(Math.Min(count, _buffer.Length))) return false;
                        available = _end - _start;
                    }
                    int take = Math.Min(count, available);
                    _start += take;
                    count -= take;
                }
                return true;
            }

            public void Dispose()
            {
                try { _stream.Dispose(); } catch { }
            }
        }
    }
}
