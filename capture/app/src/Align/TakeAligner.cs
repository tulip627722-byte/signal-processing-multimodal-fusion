using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// 一次对齐的结果。
    /// </summary>
    internal sealed class AlignResult
    {
        public string TakeDirectory = "";
        public int RadarFrames;
        public int DepthFrames;
        public int ColorFrames;
        public double DepthDtMedian = double.NaN;
        public double DepthDtMax = double.NaN;
        public double ColorDtMedian = double.NaN;
        public double ColorDtMax = double.NaN;
        public string AlignFile = "";
    }

    /// <summary>
    /// 把 take 目录里的三路数据按时间戳配成一张表（align.csv）。
    ///
    /// 为什么需要它：雷达 3.8 fps、相机 30 fps，帧号完全对不上，
    /// 只能靠 manifest.csv 里的 t_rel_ms（统一 Stopwatch）做最近邻匹配。
    ///
    /// 注意：雷达帧的 t_rel_ms 是"首包到达时刻"，也就是这一帧 8.6 ms 采集结束的时刻；
    /// 所以匹配误差上限是半个相机帧（约 17 ms）。
    /// </summary>
    internal static class TakeAligner
    {
        public const string AlignFileName = "align.csv";
        private const string ManifestName = "manifest.csv";

        public static bool IsAlignable(string takeDir)
        {
            return Directory.Exists(takeDir) && File.Exists(Path.Combine(takeDir, ManifestName));
        }

        public static bool IsAligned(string takeDir)
        {
            return File.Exists(Path.Combine(takeDir, AlignFileName));
        }

        /// <summary>扫描 takes 根目录，返回所有可对齐但还没对齐的 take。</summary>
        public static List<string> FindPending(string takesRoot)
        {
            List<string> pending = new List<string>();
            if (!Directory.Exists(takesRoot)) return pending;
            string[] dirs;
            try { dirs = Directory.GetDirectories(takesRoot); }
            catch { return pending; }
            Array.Sort(dirs, StringComparer.OrdinalIgnoreCase);
            foreach (string dir in dirs)
            {
                if (IsAlignable(dir) && !IsAligned(dir)) pending.Add(dir);
            }
            return pending;
        }

        /// <summary>对一个 take 生成 align.csv。</summary>
        public static AlignResult AlignOne(string takeDir, bool overwrite)
        {
            if (!IsAlignable(takeDir))
                throw new InvalidOperationException("不是有效的 take 目录（缺 manifest.csv）：" + takeDir);
            string outPath = Path.Combine(takeDir, AlignFileName);
            if (File.Exists(outPath) && !overwrite)
                throw new InvalidOperationException("已经对齐过了：" + outPath);

            List<double> radarT = new List<double>();
            List<int> radarF = new List<int>();
            List<double> depthT = new List<double>();
            List<int> depthF = new List<int>();
            List<double> colorT = new List<double>();
            List<int> colorF = new List<int>();

            using (StreamReader reader = new StreamReader(Path.Combine(takeDir, ManifestName), Encoding.UTF8))
            {
                string line;
                bool header = true;
                while ((line = reader.ReadLine()) != null)
                {
                    if (header) { header = false; continue; }
                    if (line.Length == 0) continue;
                    string[] parts = line.Split(',');
                    if (parts.Length < 5) continue;
                    string stream = parts[0].Trim();
                    int frame;
                    double t;
                    if (!int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out frame)) continue;
                    if (!double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out t)) continue;

                    if (stream == "radar") { radarF.Add(frame); radarT.Add(t); }
                    else if (stream == "depth") { depthF.Add(frame); depthT.Add(t); }
                    else if (stream == "color") { colorF.Add(frame); colorT.Add(t); }
                }
            }

            AlignResult result = new AlignResult();
            result.TakeDirectory = takeDir;
            result.RadarFrames = radarF.Count;
            result.DepthFrames = depthF.Count;
            result.ColorFrames = colorF.Count;
            result.AlignFile = outPath;

            List<double> depthDt = new List<double>();
            List<double> colorDt = new List<double>();

            StringBuilder csv = new StringBuilder();
            csv.AppendLine("radar_frame,radar_t_ms,depth_frame,depth_dt_ms,color_frame,color_dt_ms");
            for (int i = 0; i < radarF.Count; i++)
            {
                double t = radarT[i];
                int di, ci;
                double dd, cd;
                Nearest(depthT, depthF, t, out di, out dd);
                Nearest(colorT, colorF, t, out ci, out cd);
                if (di >= 0) depthDt.Add(Math.Abs(dd));
                if (ci >= 0) colorDt.Add(Math.Abs(cd));
                csv.Append(radarF[i].ToString(CultureInfo.InvariantCulture)).Append(',')
                   .Append(t.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
                   .Append(di >= 0 ? di.ToString(CultureInfo.InvariantCulture) : "").Append(',')
                   .Append(di >= 0 ? dd.ToString("0.###", CultureInfo.InvariantCulture) : "").Append(',')
                   .Append(ci >= 0 ? ci.ToString(CultureInfo.InvariantCulture) : "").Append(',')
                   .Append(ci >= 0 ? cd.ToString("0.###", CultureInfo.InvariantCulture) : "")
                   .AppendLine();
            }
            File.WriteAllText(outPath, csv.ToString(), new UTF8Encoding(false));

            result.DepthDtMedian = Median(depthDt);
            result.DepthDtMax = Max(depthDt);
            result.ColorDtMedian = Median(colorDt);
            result.ColorDtMax = Max(colorDt);

            // 顺手写一份人看的摘要
            StringBuilder summary = new StringBuilder();
            summary.AppendLine("take: " + Path.GetFileName(takeDir));
            summary.AppendLine("雷达帧: " + result.RadarFrames + "，深度帧: " + result.DepthFrames + "，彩色帧: " + result.ColorFrames);
            summary.AppendLine("雷达↔深度 时间残差: 中位 " + Fmt(result.DepthDtMedian) + " ms，最大 " + Fmt(result.DepthDtMax) + " ms");
            summary.AppendLine("雷达↔彩色 时间残差: 中位 " + Fmt(result.ColorDtMedian) + " ms，最大 " + Fmt(result.ColorDtMax) + " ms");
            summary.AppendLine("说明: 残差 = 该雷达帧首包时刻 − 最近图像帧时刻，正值表示图像比雷达晚。");
            summary.AppendLine("      雷达帧本身只占 8.6 ms（128 chirp × 67 µs），263 ms 是传输时间，不参与对齐。");
            File.WriteAllText(Path.Combine(takeDir, "align_summary.txt"), summary.ToString(), new UTF8Encoding(false));
            return result;
        }

        /// <summary>批量对齐。progress(已完成, 总数)，cancel 返回 true 时中断。</summary>
        public static List<AlignResult> AlignAll(string takesRoot, Action<string> log,
                                                 Action<int, int> progress, Func<bool> cancel)
        {
            List<string> pending = FindPending(takesRoot);
            List<AlignResult> results = new List<AlignResult>();
            if (pending.Count == 0)
            {
                if (log != null) log("没有待对齐的 take（" + takesRoot + "）");
                if (progress != null) progress(1, 1);
                return results;
            }
            if (log != null) log("发现 " + pending.Count + " 个待对齐的 take");

            for (int i = 0; i < pending.Count; i++)
            {
                if (cancel != null && cancel()) { if (log != null) log("已取消对齐"); break; }
                string dir = pending[i];
                try
                {
                    AlignResult r = AlignOne(dir, false);
                    results.Add(r);
                    if (log != null)
                    {
                        log("对齐完成：" + Path.GetFileName(dir) +
                            "｜雷达 " + r.RadarFrames + " 帧，深度残差中位 " + Fmt(r.DepthDtMedian) + " ms" +
                            (r.ColorFrames > 0 ? "，彩色残差中位 " + Fmt(r.ColorDtMedian) + " ms" : "（无彩色）"));
                    }
                }
                catch (Exception ex)
                {
                    if (log != null) log("对齐失败：" + Path.GetFileName(dir) + "：" + ex.Message);
                }
                if (progress != null) progress(i + 1, pending.Count);
            }
            return results;
        }

        // ------------------------------------------------------------ 工具

        private static void Nearest(List<double> times, List<int> frames, double t, out int frame, out double dt)
        {
            frame = -1; dt = double.NaN;
            if (times.Count == 0) return;
            int lo = 0, hi = times.Count - 1;
            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (times[mid] < t) lo = mid + 1; else hi = mid;
            }
            int best = lo;
            if (lo > 0 && Math.Abs(times[lo - 1] - t) <= Math.Abs(times[lo] - t)) best = lo - 1;
            frame = frames[best];
            dt = times[best] - t;
        }

        private static double Median(List<double> values)
        {
            if (values.Count == 0) return double.NaN;
            values.Sort();
            int n = values.Count;
            return (n % 2 == 1) ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
        }

        private static double Max(List<double> values)
        {
            if (values.Count == 0) return double.NaN;
            double m = values[0];
            foreach (double v in values) if (v > m) m = v;
            return m;
        }

        private static string Fmt(double v)
        {
            return double.IsNaN(v) ? "—" : v.ToString("0.0", CultureInfo.InvariantCulture);
        }
    }
}
