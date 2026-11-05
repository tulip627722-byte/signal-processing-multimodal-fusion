using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal sealed class ManifestRow
    {
        public string Stream;
        public int Frame;
        public double RelMs;
        public string File;
    }

    /// <summary>
    /// 一个 take 目录的产物管理：目录结构、manifest.csv、radar_frames.csv、depth_scale.txt、meta.json。
    /// 三路共用同一个 Timebase 换算 t_utc，manifest 落盘时按 t_rel_ms 排序。
    /// </summary>
    internal sealed class TakeWriter
    {
        private readonly object _gate = new object();
        private readonly Timebase _tb;
        private readonly List<ManifestRow> _rows = new List<ManifestRow>();
        private readonly List<RadarFrameStat> _radarFrames = new List<RadarFrameStat>();
        private double? _depthScale;

        public string Directory { get; private set; }
        public string ColorDirectory { get; private set; }
        public string DepthDirectory { get; private set; }
        public string CloudDirectory { get; private set; }

        public TakeWriter(string rootDirectory, string takeName, Timebase tb)
        {
            if (string.IsNullOrEmpty(rootDirectory)) throw new ArgumentNullException("rootDirectory");
            if (string.IsNullOrEmpty(takeName)) throw new ArgumentNullException("takeName");
            _tb = tb;
            Directory = Path.Combine(rootDirectory, takeName);
            ColorDirectory = Path.Combine(Directory, "color");
            DepthDirectory = Path.Combine(Directory, "depth");
            CloudDirectory = Path.Combine(Directory, "cloud");
            System.IO.Directory.CreateDirectory(ColorDirectory);
            System.IO.Directory.CreateDirectory(DepthDirectory);
        }

        /// <summary>生成形如 20261026_173000_室内2米 的目录名，冲突时自动加序号。</summary>
        public static string MakeTakeName(string rootDirectory, DateTime localTime, string label)
        {
            string stem = localTime.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            if (!string.IsNullOrEmpty(label))
            {
                StringBuilder safe = new StringBuilder();
                foreach (char c in label.Trim())
                {
                    if (Array.IndexOf(Path.GetInvalidFileNameChars(), c) >= 0) continue;
                    safe.Append(c == ' ' ? '_' : c);
                }
                string cleaned = safe.ToString();
                if (cleaned.Length > 0) stem += "_" + cleaned;
            }
            string candidate = stem;
            int suffix = 2;
            while (System.IO.Directory.Exists(Path.Combine(rootDirectory, candidate)))
            {
                candidate = stem + "_" + suffix.ToString(CultureInfo.InvariantCulture);
                suffix++;
            }
            return candidate;
        }

        public void AddRow(string stream, int frame, double relMs, string file)
        {
            lock (_gate)
            {
                _rows.Add(new ManifestRow { Stream = stream, Frame = frame, RelMs = relMs, File = file });
            }
        }

        public void AddRadarFrame(RadarFrameStat stat)
        {
            lock (_gate) _radarFrames.Add(stat);
        }

        /// <summary>点云目录按需创建：不勾选“同时存点云”时不留空目录。</summary>
        public void EnsureCloudDirectory()
        {
            if (!System.IO.Directory.Exists(CloudDirectory))
                System.IO.Directory.CreateDirectory(CloudDirectory);
        }

        public void WriteDepthScale(double metersPerUnit)
        {
            lock (_gate) _depthScale = metersPerUnit;
            File.WriteAllText(Path.Combine(Directory, "depth_scale.txt"),
                metersPerUnit.ToString("0.######", CultureInfo.InvariantCulture) + "\r\n",
                new UTF8Encoding(false));
        }

        public int RowCount { get { lock (_gate) return _rows.Count; } }

        public void WriteAll(JsonObject meta)
        {
            List<ManifestRow> rows;
            List<RadarFrameStat> frames;
            double? depthScale;
            lock (_gate)
            {
                rows = new List<ManifestRow>(_rows);
                frames = new List<RadarFrameStat>(_radarFrames);
                depthScale = _depthScale;
            }

            rows.Sort(CompareRows);
            WriteManifest(rows);
            WriteRadarFrames(frames);
            if (meta != null) WriteMeta(meta);
            if (!depthScale.HasValue && !File.Exists(Path.Combine(Directory, "depth_scale.txt")))
            {
                // 没有深度通路时不伪造 scale 文件，由 meta.json 的 depth.available=false 说明
            }
        }

        private static int CompareRows(ManifestRow a, ManifestRow b)
        {
            int cmp = a.RelMs.CompareTo(b.RelMs);
            if (cmp != 0) return cmp;
            cmp = StreamRank(a.Stream).CompareTo(StreamRank(b.Stream));
            if (cmp != 0) return cmp;
            return a.Frame.CompareTo(b.Frame);
        }

        private static int StreamRank(string stream)
        {
            if (stream == "color") return 0;
            if (stream == "depth") return 1;
            if (stream == "cloud") return 2;
            if (stream == "radar") return 3;
            return 4;
        }

        private void WriteManifest(List<ManifestRow> rows)
        {
            StringBuilder sb = new StringBuilder(rows.Count * 64 + 64);
            sb.Append("stream,frame,t_rel_ms,t_utc,file\r\n");
            foreach (ManifestRow row in rows)
            {
                sb.Append(row.Stream).Append(',');
                sb.Append(row.Frame.ToString(CultureInfo.InvariantCulture)).Append(',');
                sb.Append(FormatRelMs(row.RelMs)).Append(',');
                sb.Append(_tb.UtcFromRelMs(row.RelMs).ToString("yyyy-MM-ddTHH:mm:ss.fffZ", CultureInfo.InvariantCulture)).Append(',');
                sb.Append(EscapeCsv(row.File)).Append("\r\n");
            }
            File.WriteAllText(Path.Combine(Directory, "manifest.csv"), sb.ToString(), new UTF8Encoding(false));
        }

        private void WriteRadarFrames(List<RadarFrameStat> frames)
        {
            frames.Sort(delegate(RadarFrameStat a, RadarFrameStat b) { return a.FrameNumber.CompareTo(b.FrameNumber); });
            StringBuilder sb = new StringBuilder(frames.Count * 48 + 64);
            sb.Append("frame,first_packet_t_rel_ms,last_packet_t_rel_ms,received,missing\r\n");
            foreach (RadarFrameStat s in frames)
            {
                sb.Append(s.FrameNumber.ToString(CultureInfo.InvariantCulture)).Append(',');
                sb.Append(double.IsNaN(s.FirstRelMs) ? "" : FormatRelMs(s.FirstRelMs)).Append(',');
                sb.Append(double.IsNaN(s.LastRelMs) ? "" : FormatRelMs(s.LastRelMs)).Append(',');
                sb.Append(s.Received.ToString(CultureInfo.InvariantCulture)).Append(',');
                sb.Append(s.Missing.ToString(CultureInfo.InvariantCulture)).Append("\r\n");
            }
            File.WriteAllText(Path.Combine(Directory, "radar_frames.csv"), sb.ToString(), new UTF8Encoding(false));
        }

        private void WriteMeta(JsonObject meta)
        {
            File.WriteAllText(Path.Combine(Directory, "meta.json"), MiniJson.Serialize(meta), new UTF8Encoding(false));
        }

        public static string FormatRelMs(double relMs)
        {
            return relMs.ToString("0.000", CultureInfo.InvariantCulture);
        }

        private static string EscapeCsv(string value)
        {
            if (string.IsNullOrEmpty(value)) return "";
            if (value.IndexOf(',') < 0 && value.IndexOf('"') < 0 && value.IndexOf('\r') < 0 && value.IndexOf('\n') < 0)
                return value;
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }
    }
}
