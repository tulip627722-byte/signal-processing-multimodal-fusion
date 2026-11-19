using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal sealed class AlignmentCoordinator
    {
        private sealed class CameraStamp { public int Frame; public double T; public string Stream; }
        private sealed class LidarStamp { public int Batch; public double First; public double Last; public int Points; public string File; }
        private sealed class Row
        {
            public RadarFrameStat Radar;
            public CameraStamp Depth;
            public double DepthDt;
            public CameraStamp Color;
            public double ColorDt;
            public List<LidarStamp> Lidar = new List<LidarStamp>();
            public bool Valid;
            public string Reason;
        }

        private readonly object _gate = new object();
        private readonly List<CameraStamp> _depth = new List<CameraStamp>();
        private readonly List<CameraStamp> _color = new List<CameraStamp>();
        private readonly List<LidarStamp> _lidar = new List<LidarStamp>();
        private readonly List<Row> _rows = new List<Row>();
        private readonly HashSet<int> _radarFrames = new HashSet<int>();
        private readonly double _windowMs;
        private readonly double _maxDeltaMs;

        public AlignmentCoordinator(double windowMs, double maxDeltaMs)
        {
            _windowMs = windowMs;
            _maxDeltaMs = maxDeltaMs;
        }

        public void AddCamera(string stream, int frame, double t)
        {
            if (double.IsNaN(t)) return;
            lock (_gate)
            {
                List<CameraStamp> target = stream == "depth" ? _depth : _color;
                target.Add(new CameraStamp { Stream = stream, Frame = frame, T = t });
            }
        }

        public void AddLidar(LidarBatch batch, string file)
        {
            if (batch == null || double.IsNaN(batch.RelMs)) return;
            lock (_gate)
            {
                _lidar.Add(new LidarStamp { Batch = batch.Sequence, First = batch.RelMs,
                    Last = double.IsNaN(batch.EndRelMs) ? batch.RelMs : batch.EndRelMs,
                    Points = batch.Count, File = "lidar/" + file });
            }
        }

        public void AddRadar(RadarFrameStat stat)
        {
            if (stat == null) return;
            lock (_gate)
            {
                if (!_radarFrames.Add(stat.FrameNumber)) return;
                RadarFrameStat copy = new RadarFrameStat { FrameNumber = stat.FrameNumber,
                    FirstRelMs = stat.FirstRelMs, LastRelMs = stat.LastRelMs,
                    Received = stat.Received, Expected = stat.Expected, Missing = stat.Missing };
                double center = double.IsNaN(copy.FirstRelMs) ? double.NaN :
                    (double.IsNaN(copy.LastRelMs) ? copy.FirstRelMs : (copy.FirstRelMs + copy.LastRelMs) / 2.0);
                Row row = new Row { Radar = copy, Reason = "" };
                row.Depth = Nearest(_depth, center, out row.DepthDt);
                row.Color = Nearest(_color, center, out row.ColorDt);
                double start = center - _windowMs;
                double end = center;
                for (int i = 0; i < _lidar.Count; i++)
                {
                    LidarStamp item = _lidar[i];
                    if (item.Last >= start && item.First <= end) row.Lidar.Add(item);
                }
                List<string> reasons = new List<string>();
                if (copy.Received == 0 || copy.Missing > 0) reasons.Add("radar_incomplete");
                if (row.Depth == null) reasons.Add("depth_missing");
                else if (row.DepthDt > _maxDeltaMs) reasons.Add("depth_delta");
                if (row.Color == null) reasons.Add("color_missing");
                else if (row.ColorDt > _maxDeltaMs) reasons.Add("color_delta");
                if (row.Lidar.Count == 0) reasons.Add("lidar_window_missing");
                row.Valid = reasons.Count == 0;
                row.Reason = string.Join(";", reasons.ToArray());
                _rows.Add(row);
            }
        }

        private static CameraStamp Nearest(List<CameraStamp> frames, double t, out double delta)
        {
            delta = double.NaN;
            if (frames.Count == 0 || double.IsNaN(t)) return null;
            CameraStamp best = null;
            double bestDelta = double.MaxValue;
            for (int i = 0; i < frames.Count; i++)
            {
                double d = Math.Abs(frames[i].T - t);
                if (d < bestDelta) { bestDelta = d; best = frames[i]; }
            }
            delta = bestDelta;
            return best;
        }

        public int Count { get { lock (_gate) return _rows.Count; } }
        public int ValidCount { get { lock (_gate) { int n = 0; for (int i = 0; i < _rows.Count; i++) if (_rows[i].Valid) n++; return n; } } }

        public void Write(string directory)
        {
            List<Row> rows;
            lock (_gate) rows = new List<Row>(_rows);
            rows.Sort(delegate(Row a, Row b) { return a.Radar.FrameNumber.CompareTo(b.Radar.FrameNumber); });
            StringBuilder sb = new StringBuilder(rows.Count * 260 + 256);
            sb.Append("radar_frame,radar_first_t_ms,radar_last_t_ms,radar_center_t_ms,radar_received,radar_expected,radar_missing,depth_frame,depth_t_ms,depth_dt_ms,color_frame,color_t_ms,color_dt_ms,lidar_window_start_t_ms,lidar_window_end_t_ms,lidar_first_batch,lidar_last_batch,lidar_batch_count,lidar_point_count,lidar_latest_dt_ms,valid,reason\r\n");
            for (int i = 0; i < rows.Count; i++)
            {
                Row r = rows[i];
                double center = double.IsNaN(r.Radar.FirstRelMs) ? double.NaN : (double.IsNaN(r.Radar.LastRelMs) ? r.Radar.FirstRelMs : (r.Radar.FirstRelMs + r.Radar.LastRelMs) / 2.0);
                double start = center - _windowMs;
                int points = 0; int firstBatch = 0; int lastBatch = 0; double latestDelta = double.NaN;
                if (r.Lidar.Count > 0)
                {
                    firstBatch = r.Lidar[0].Batch; lastBatch = r.Lidar[r.Lidar.Count - 1].Batch;
                    for (int j = 0; j < r.Lidar.Count; j++) { points += r.Lidar[j].Points; latestDelta = Math.Abs(center - r.Lidar[j].Last); }
                }
                Add(sb, r.Radar.FrameNumber); Add(sb, r.Radar.FirstRelMs); Add(sb, r.Radar.LastRelMs); Add(sb, center);
                Add(sb, r.Radar.Received); Add(sb, r.Radar.Expected); Add(sb, r.Radar.Missing);
                Add(sb, r.Depth == null ? 0 : r.Depth.Frame); Add(sb, r.Depth == null ? double.NaN : r.Depth.T); Add(sb, r.DepthDt);
                Add(sb, r.Color == null ? 0 : r.Color.Frame); Add(sb, r.Color == null ? double.NaN : r.Color.T); Add(sb, r.ColorDt);
                Add(sb, start); Add(sb, center); Add(sb, firstBatch); Add(sb, lastBatch); Add(sb, r.Lidar.Count); Add(sb, points); Add(sb, latestDelta);
                Add(sb, r.Valid ? "true" : "false", false); Add(sb, r.Reason, true);
            }
            File.WriteAllText(Path.Combine(directory, "aligned_frames.csv"), sb.ToString(), new UTF8Encoding(false));
        }

        private static void Add(StringBuilder sb, int value) { sb.Append(value.ToString(CultureInfo.InvariantCulture)).Append(','); }
        private static void Add(StringBuilder sb, double value) { sb.Append(double.IsNaN(value) ? "" : value.ToString("0.000", CultureInfo.InvariantCulture)).Append(','); }
        private static void Add(StringBuilder sb, string value, bool last)
        {
            string text = value ?? "";
            if (text.IndexOfAny(new char[] { ',', '"', '\r', '\n' }) >= 0) text = "\"" + text.Replace("\"", "\"\"") + "\"";
            sb.Append(text).Append(last ? "\r\n" : ",");
        }
    }
}
