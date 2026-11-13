using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal sealed class TakeValidation
    {
        public readonly List<string> Passed = new List<string>();
        public readonly List<string> Failed = new List<string>();
        public readonly List<string> Notes = new List<string>();

        public bool Ok { get { return Failed.Count == 0; } }

        public void Pass(string message) { Passed.Add(message); }
        public void Fail(string message) { Failed.Add(message); }
        public void Note(string message) { Notes.Add(message); }

        public void Require(bool condition, string message)
        {
            if (condition) Pass(message);
            else Fail(message);
        }

        public string Report()
        {
            StringBuilder sb = new StringBuilder();
            foreach (string s in Passed) sb.AppendLine("PASS  " + s);
            foreach (string s in Notes) sb.AppendLine("INFO  " + s);
            foreach (string s in Failed) sb.AppendLine("FAIL  " + s);
            sb.AppendLine(Ok ? "结论：通过" : "结论：失败（" + Failed.Count + " 项）");
            return sb.ToString();
        }
    }

    internal sealed class ManifestEntry
    {
        public string Stream;
        public int Frame;
        public double RelMs;
        public DateTime Utc;
        public string File;
    }

    /// <summary>
    /// take 目录验收器：结构、manifest、meta、MAT、PNG 位深、时间轴一致性。
    /// --self-test 与 --verify-take 共用同一套标准。
    /// </summary>
    internal static class TakeValidator
    {
        public static TakeValidation Validate(string takeDir, int packetsPerFrame, int payloadBytes,
            bool selfTest, int expectedDelaySeconds, int expectedDurationSeconds)
        {
            TakeValidation v = new TakeValidation();
            if (!Directory.Exists(takeDir))
            {
                v.Fail("take 目录不存在：" + takeDir);
                return v;
            }

            string manifestPath = Path.Combine(takeDir, "manifest.csv");
            string metaPath = Path.Combine(takeDir, "meta.json");
            string matPath = Path.Combine(takeDir, "radar_adc.mat");
            string framesPath = Path.Combine(takeDir, "radar_frames.csv");

            JsonObject meta = null;
            try
            {
                meta = MiniJson.Parse(File.ReadAllText(metaPath, Encoding.UTF8)) as JsonObject;
                v.Require(meta != null, "meta.json 可解析为 JSON 对象");
            }
            catch (Exception ex)
            {
                v.Fail("meta.json 解析失败：" + ex.Message);
                return v;
            }

            double actualDuration = GetDouble(meta, "actualDurationSeconds", -1);
            int delaySeconds = (int)GetDouble(meta, "delaySeconds", -1);
            int requestedDuration = (int)GetDouble(meta, "requestedDurationSeconds", -1);
            v.Require(delaySeconds == expectedDelaySeconds, "meta.delaySeconds = " + delaySeconds + "（期望 " + expectedDelaySeconds + "）");
            v.Require(requestedDuration == expectedDurationSeconds,
                "meta.requestedDurationSeconds = " + requestedDuration + "（期望 " + expectedDurationSeconds + "）");
            v.Require(actualDuration >= expectedDurationSeconds - 0.5 && actualDuration <= expectedDurationSeconds + 1.0,
                "实测时长 " + actualDuration.ToString("0.000") + " s 落在 [" +
                (expectedDurationSeconds - 0.5) + ", " + (expectedDurationSeconds + 1.0) + "] 内");

            // ---------------- manifest ----------------
            List<ManifestEntry> rows = new List<ManifestEntry>();
            if (!File.Exists(manifestPath))
            {
                v.Fail("缺少 manifest.csv");
            }
            else
            {
                try
                {
                    rows = ReadManifest(manifestPath, v);
                }
                catch (Exception ex)
                {
                    v.Fail("manifest.csv 解析失败：" + ex.Message);
                }
            }

            v.Require(rows.Count > 0, "manifest.csv 非空（" + rows.Count + " 行）");
            bool monotonic = true;
            double lastRel = double.NegativeInfinity;
            foreach (ManifestEntry row in rows)
            {
                if (row.RelMs < lastRel) monotonic = false;
                lastRel = row.RelMs;
            }
            v.Require(monotonic, "manifest t_rel_ms 单调不减");

            Dictionary<string, List<ManifestEntry>> byStream = new Dictionary<string, List<ManifestEntry>>();
            foreach (ManifestEntry row in rows)
            {
                if (!byStream.ContainsKey(row.Stream)) byStream[row.Stream] = new List<ManifestEntry>();
                byStream[row.Stream].Add(row);
            }
            bool colorRequired = GetBool(GetObject(meta, "color"), "available", true);
            bool depthRequired = GetBool(GetObject(meta, "depth"), "available", true);
            bool radarRequired = GetBool(GetObject(meta, "radar"), "enabled", true);
            bool lidarRequired = GetBool(GetObject(meta, "lidar"), "enabled", false);
            foreach (string stream in new[] { "radar", "color", "depth", "lidar" })
            {
                bool required = stream == "radar" ? radarRequired : (stream == "lidar" ? lidarRequired : (stream == "color" ? colorRequired : depthRequired));
                if (required) v.Require(byStream.ContainsKey(stream), "manifest 含 " + stream + " 流");
                else v.Note("manifest 无 " + stream + " 流（meta 标记该路未启用或无数据）");
            }

            bool filesOk = true;
            foreach (ManifestEntry row in rows)
            {
                string path = Path.Combine(takeDir, row.File.Replace('/', Path.DirectorySeparatorChar));
                if (!File.Exists(path))
                {
                    filesOk = false;
                    v.Fail("manifest 行指向的文件不存在：" + row.File);
                    break;
                }
            }
            if (filesOk) v.Pass("manifest 中 " + rows.Count + " 行的 file 字段全部存在");

            foreach (KeyValuePair<string, List<ManifestEntry>> pair in byStream)
            {
                List<int> frames = new List<int>();
                foreach (ManifestEntry row in pair.Value) frames.Add(row.Frame);
                bool sequential = true;
                bool radar = pair.Key == "radar";
                for (int i = 0; i < frames.Count; i++)
                {
                    if (radar) { if (i > 0 && frames[i] <= frames[i - 1]) sequential = false; }
                    else if (frames[i] != i + 1) sequential = false;
                }
                v.Require(sequential, radar
                    ? "radar 帧号严格递增（共 " + frames.Count + " 帧）"
                    : pair.Key + " 帧号从 1 连续递增（共 " + frames.Count + " 帧）");
            }

            // ---------------- meta 一致性 ----------------
            JsonObject colorObj = GetObject(meta, "color");
            JsonObject depthObj = GetObject(meta, "depth");
            JsonObject radarObj = GetObject(meta, "radar");
            JsonObject cameraObj = GetObject(meta, "camera");
            JsonObject timebaseObj = GetObject(meta, "timebase");
            JsonObject lidarObj = GetObject(meta, "lidar");

            v.Require(GetString(meta, "format", "") == "ACQUIRE_TAKE_V1", "meta.format = ACQUIRE_TAKE_V1");
            v.Require(GetDouble(timebaseObj, "frequencyHz", 0) > 0, "meta.timebase.frequencyHz > 0");
            v.Require(GetString(cameraObj, "model", "").Length > 0, "meta.camera.model 非空（" + GetString(cameraObj, "model", "") + "）");

            string colorDir = Path.Combine(takeDir, "color");
            string depthDir = Path.Combine(takeDir, "depth");
            string[] colorFiles = Directory.Exists(colorDir) ? Directory.GetFiles(colorDir, "*.png") : new string[0];
            string[] depthFiles = Directory.Exists(depthDir) ? Directory.GetFiles(depthDir, "*.png") : new string[0];
            Array.Sort(colorFiles, StringComparer.Ordinal);
            Array.Sort(depthFiles, StringComparer.Ordinal);

            bool colorAvailable = GetBool(colorObj, "available", true);
            if (colorAvailable)
                v.Require(colorFiles.Length > 0, "color 目录有图（" + colorFiles.Length + " 张）");
            else
                v.Note("本次无彩色帧（meta.color.available=false），不要求 color 目录有图（实际 " + colorFiles.Length + " 张）");
            bool depthExpected = GetBool(depthObj, "available", false);
            if (depthExpected)
                v.Require(depthFiles.Length > 0, "depth 目录有图（" + depthFiles.Length + " 张）");
            else
                v.Note("本次采集无深度流（meta.depth.available = false），depth 目录允许为空");
            v.Require((int)GetDouble(colorObj, "frames", -1) == colorFiles.Length,
                "meta.color.frames 与 color 目录张数一致（" + colorFiles.Length + "）");
            v.Require((int)GetDouble(depthObj, "frames", -1) == depthFiles.Length,
                "meta.depth.frames 与 depth 目录张数一致（" + depthFiles.Length + "）");

            if (colorFiles.Length > 0)
            {
                try
                {
                    using (Image img = Image.FromFile(colorFiles[0]))
                    {
                        v.Require(img.Width > 0 && img.Height > 0,
                            "彩色 PNG 可解码：" + Path.GetFileName(colorFiles[0]) + " " + img.Width + "×" + img.Height);
                    }
                }
                catch (Exception ex)
                {
                    v.Fail("彩色 PNG 解码失败：" + ex.Message);
                }
            }

            if (depthFiles.Length > 0)
            {
                int w, h;
                string error;
                if (Png16.InspectDepthPng(depthFiles[depthFiles.Length - 1], out w, out h, out error))
                    v.Pass("深度 PNG 是 16-bit 灰度：" + Path.GetFileName(depthFiles[depthFiles.Length - 1]) + " " + w + "×" + h);
                else
                    v.Fail("深度 PNG 位深检查失败：" + error);
            }

            string scalePath = Path.Combine(takeDir, "depth_scale.txt");
            if (!depthExpected)
            {
                v.Note("无深度流，不检查 depth_scale.txt");
            }
            else if (File.Exists(scalePath))
            {
                double scale;
                v.Require(double.TryParse(File.ReadAllText(scalePath).Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out scale) && scale > 0,
                    "depth_scale.txt 可解析且为正数");
                v.Require(Math.Abs(scale - GetDouble(depthObj, "depthScaleMetersPerUnit", 0)) < 1e-9,
                    "depth_scale.txt 与 meta.depth.depthScaleMetersPerUnit 一致");
            }
            else
            {
                v.Fail("缺少 depth_scale.txt");
            }

            ValidateIntrinsics(v, cameraObj, depthObj, depthExpected);
            ValidateCloud(v, takeDir, depthObj, byStream);
            ValidateLidar(v, takeDir, lidarObj, byStream);

            // ---------------- 雷达：MAT + radar_frames.csv ----------------
            long received = (long)GetDouble(radarObj, "receivedPackets", -1);
            long missing = (long)GetDouble(radarObj, "missingPackets", -1);
            int radarFrames = (int)GetDouble(radarObj, "frames", -1);

            if (radarRequired)
                v.Require(received > 0, "meta.radar.receivedPackets > 0（" + received + "）");
            else
                v.Note("雷达未启用（meta.radar.enabled=false），跳过 MAT / radar_frames 校验");
            if (radarRequired) v.Require(File.Exists(matPath), "radar_adc.mat 存在");
            if (File.Exists(matPath))
            {
                try
                {
                    MatInspection mat = MatVerifier.Verify(matPath, payloadBytes, TinyFrameCodec.TypeAdc);
                    v.Require(mat.Cols == 4, "ADCData 列数为 4（实际 " + mat.Cols + "）");
                    v.Require(mat.Rows == received, "ADCData 行数 = 收到包数（" + mat.Rows + "）");
                    if (missing == 0)
                        v.Require(mat.Rows % packetsPerFrame == 0,
                            "无丢包时 ADCData 行数是 " + packetsPerFrame + " 的整数倍（" + mat.Rows + " 行 = " +
                            (mat.Rows / packetsPerFrame) + " 帧 × " + packetsPerFrame + " 包）");
                    else
                        v.Note("本次有丢包（" + missing + " 包），ADCData 共 " + mat.Rows + " 行，不要求为 " +
                               packetsPerFrame + " 的整数倍");
                    v.Require(mat.PayloadBytes == payloadBytes, "payload 宽度 = " + payloadBytes);
                    v.Require(mat.Type == TinyFrameCodec.TypeAdc, "type 全部为 0x0B00");
                    v.Require(mat.LastId > mat.FirstId, "packetId 递增（" + mat.FirstId + " → " + mat.LastId + "）");
                    v.Note("MAT 结构：" + mat.VarName + " " + mat.Rows + "×" + mat.Cols + " cell，单帧 2048×4，文件 " +
                           (mat.FileBytes / 1024 / 1024) + " MB");

                    if (selfTest && mat.FirstPayloadHead != null && mat.FirstPayloadHead.Length >= 16)
                    {
                        bool patternOk = true;
                        for (int i = 0; i < 16; i++)
                        {
                            if (mat.FirstPayloadHead[i] != SyntheticRadar.PayloadByte(mat.FirstId, i)) patternOk = false;
                        }
                        v.Require(patternOk, "首包 payload 与合成数据模式一致（端到端数据未被改写）");
                    }
                }
                catch (Exception ex)
                {
                    v.Fail("MAT 结构校验失败：" + ex.Message);
                }
            }

            List<RadarRow> radarRows = new List<RadarRow>();
            if (!File.Exists(framesPath))
            {
                if (radarRequired) v.Fail("缺少 radar_frames.csv");
            }
            else
            {
                string[] lines = File.ReadAllLines(framesPath, Encoding.UTF8);
                v.Require(lines.Length > 0 && lines[0] == "frame,first_packet_t_rel_ms,last_packet_t_rel_ms,received,missing",
                    "radar_frames.csv 表头正确");
                long sumReceived = 0, sumMissing = 0;
                for (int i = 1; i < lines.Length; i++)
                {
                    if (lines[i].Trim().Length == 0) continue;
                    string[] parts = lines[i].Split(',');
                    if (parts.Length != 5)
                    {
                        v.Fail("radar_frames.csv 第 " + (i + 1) + " 行列数不对");
                        break;
                    }
                    RadarRow row = new RadarRow
                    {
                        Frame = int.Parse(parts[0], CultureInfo.InvariantCulture),
                        FirstRel = parts[1].Length == 0 ? double.NaN : double.Parse(parts[1], CultureInfo.InvariantCulture),
                        LastRel = parts[2].Length == 0 ? double.NaN : double.Parse(parts[2], CultureInfo.InvariantCulture),
                        Received = int.Parse(parts[3], CultureInfo.InvariantCulture),
                        Missing = int.Parse(parts[4], CultureInfo.InvariantCulture)
                    };
                    radarRows.Add(row);
                    sumReceived += row.Received;
                    sumMissing += row.Missing;
                }
                v.Require(radarRows.Count == radarFrames, "radar_frames.csv 帧数 = meta.radar.frames（" + radarRows.Count + "）");
                v.Require(sumReceived == received, "radar_frames.csv 收包合计 = meta.radar.receivedPackets（" + sumReceived + "）");
                v.Require(sumMissing == missing, "radar_frames.csv 丢包合计 = meta.radar.missingPackets（" + sumMissing + "）");
                v.Note("雷达丢包率：" + (missing + received == 0 ? 0 : 100.0 * missing / (missing + received)).ToString("0.0000") +
                       "%（含尾帧截断），排除尾帧后：" +
                       (100.0 * GetDouble(radarObj, "lossRateExcludingTail", 0)).ToString("0.0000") +
                       "%，尾帧 #" + (int)GetDouble(radarObj, "lastFrameNumber", 0) + " 收到 " +
                       (int)GetDouble(radarObj, "lastFrameReceived", 0) + "/" +
                       (int)GetDouble(radarObj, "lastFrameExpected", 0));
            }

            // ---------------- 时间轴质量 ----------------
            if (byStream.ContainsKey("color") && byStream["color"].Count > 2)
                ReportIntervals(v, "彩色", byStream["color"], 1000.0 / 30.0);
            if (byStream.ContainsKey("depth") && byStream["depth"].Count > 2)
                ReportIntervals(v, "深度", byStream["depth"], 1000.0 / 30.0);
            if (radarRows.Count > 2)
                ReportRadarIntervals(v, radarRows);

            if (byStream.ContainsKey("color") && byStream["color"].Count > 1 && radarRows.Count > 1)
                ReportCrossAlignment(v, byStream["color"], radarRows);

            double maxRel = rows.Count == 0 ? 0 : rows[rows.Count - 1].RelMs;
            v.Require(maxRel >= (expectedDurationSeconds - 0.6) * 1000.0 && maxRel <= (expectedDurationSeconds + 1.0) * 1000.0,
                "manifest 最后时间戳 " + maxRel.ToString("0.0") + " ms 与设定时长吻合");

            return v;
        }

        /// <summary>内参必须能支撑点云复算：正焦距、主点在图像内、与实际深度分辨率一致。</summary>
        private static void ValidateIntrinsics(TakeValidation v, JsonObject cameraObj, JsonObject depthObj, bool depthExpected)
        {
            JsonObject k = cameraObj == null ? null : cameraObj.Get("depthIntrinsics", null) as JsonObject;
            if (!depthExpected)
            {
                v.Note("无深度流，不检查深度内参");
                return;
            }
            if (k == null)
            {
                v.Fail("meta.camera.depthIntrinsics 缺失，深度点云无法复算");
                return;
            }

            double fx = GetDouble(k, "fx", 0);
            double fy = GetDouble(k, "fy", 0);
            double cx = GetDouble(k, "cx", 0);
            double cy = GetDouble(k, "cy", 0);
            int width = (int)GetDouble(k, "width", 0);
            int height = (int)GetDouble(k, "height", 0);
            double scale = GetDouble(k, "depthScale", 0);
            string source = GetString(k, "source", "");
            string note = GetString(k, "note", "");

            v.Require(fx > 0 && fy > 0, "内参 fx/fy > 0（" + Fmt(fx) + " / " + Fmt(fy) + "）");
            v.Require(width > 0 && height > 0 &&
                      width == (int)GetDouble(depthObj, "width", 0) &&
                      height == (int)GetDouble(depthObj, "height", 0),
                "内参分辨率 = 实际深度分辨率（" + width + "x" + height + "）");
            v.Require(cx > 0 && cx < width && cy > 0 && cy < height,
                "主点在图像内（cx=" + Fmt(cx) + ", cy=" + Fmt(cy) + "）");
            v.Require(Math.Abs(cx - (width - 1) / 2.0) < 1.0 && Math.Abs(cy - (height - 1) / 2.0) < 1.0,
                "主点接近图像中心（偏差 < 1 px）");
            v.Require(scale > 0 && Math.Abs(scale - GetDouble(depthObj, "depthScaleMetersPerUnit", 0)) < 1e-12,
                "内参 depthScale 与 depth.depthScaleMetersPerUnit 一致（" + Fmt(scale) + "）");
            v.Require(source == DepthIntrinsics.SourceCalibration || source == DepthIntrinsics.SourceDeviceFov ||
                      source == DepthIntrinsics.SourceFallback || source == DepthIntrinsics.SourceSynthetic,
                "内参来源在允许集内：" + source);
            v.Note("深度内参：fx=" + Fmt(fx) + " fy=" + Fmt(fy) + " cx=" + Fmt(cx) + " cy=" + Fmt(cy) +
                   " " + width + "x" + height + " depthScale=" + Fmt(scale) + " source=" + source +
                   (note.Length > 0 ? "（" + note + "）" : ""));
        }

        /// <summary>点云：文件数、PLY 头、顶点合计、与深度帧的对应关系。</summary>
        private static void ValidateCloud(TakeValidation v, string takeDir, JsonObject depthObj,
                                          Dictionary<string, List<ManifestEntry>> byStream)
        {
            JsonObject cloud = depthObj == null ? null : depthObj.Get("cloud", null) as JsonObject;
            if (!GetBool(cloud, "enabled", false))
            {
                v.Note("本次未存点云（meta.depth.cloud.enabled = false）");
                return;
            }

            int frames = (int)GetDouble(cloud, "frames", -1);
            long points = (long)GetDouble(cloud, "points", -1);
            int stride = (int)GetDouble(cloud, "stride", 4);
            string dir = Path.Combine(takeDir, "cloud");
            string[] files = Directory.Exists(dir) ? Directory.GetFiles(dir, "*.ply") : new string[0];
            Array.Sort(files, StringComparer.Ordinal);

            v.Require(files.Length > 0, "cloud 目录有 PLY（" + files.Length + " 个）");
            v.Require(files.Length == frames, "cloud 文件数 = meta.depth.cloud.frames（" + files.Length + " vs " + frames + "）");

            long total = 0;
            bool ok = true;
            string problem = null;
            for (int i = 0; i < files.Length; i++)
            {
                string error;
                string format;
                int count = DepthCloud.ReadPlyVertexCount(files[i], out error, out format);
                if (count < 0)
                {
                    ok = false;
                    problem = Path.GetFileName(files[i]) + "：" + error;
                    break;
                }
                total += count;
            }
            v.Require(ok, ok ? "每个 cloud/*.ply 的 PLY 头合法、文件长度自洽" : "cloud/*.ply 校验失败：" + problem);
            if (ok)
            {
                v.Require(total == points, "PLY 顶点合计 " + total + " = meta.depth.cloud.points（" + points + "）");
                if (frames > 0)
                    v.Note("点云：stride=" + stride + "，帧均 " + (total / frames) + " 点（640×480 ÷ 4² ≈ 19200）");
            }

            List<ManifestEntry> cloudRows;
            if (!byStream.TryGetValue("cloud", out cloudRows))
            {
                v.Fail("manifest 里没有 cloud 流（勾选了点云就该有）");
                return;
            }
            v.Require(cloudRows.Count == files.Length,
                "manifest 中 cloud 行数 = cloud 文件数（" + cloudRows.Count + "）");

            List<ManifestEntry> depthRows;
            if (byStream.TryGetValue("depth", out depthRows))
            {
                Dictionary<int, double> depthRel = new Dictionary<int, double>();
                foreach (ManifestEntry row in depthRows) depthRel[row.Frame] = row.RelMs;
                bool matched = true;
                foreach (ManifestEntry row in cloudRows)
                {
                    double rel;
                    if (!depthRel.TryGetValue(row.Frame, out rel) || Math.Abs(rel - row.RelMs) > 1e-9)
                    {
                        matched = false;
                        break;
                    }
                }
                v.Require(matched, "每个 cloud 行都与同号 depth 行的时间戳一致（点云取自同一帧）");
            }
        }

        private static void ValidateLidar(TakeValidation v, string takeDir, JsonObject lidarObj,
                                          Dictionary<string, List<ManifestEntry>> byStream)
        {
            if (!GetBool(lidarObj, "enabled", false))
            {
                v.Note("本次未启用 LiDAR");
                return;
            }
            string directory = Path.Combine(takeDir, "lidar");
            string[] files = Directory.Exists(directory) ? Directory.GetFiles(directory, "*.bin") : new string[0];
            Array.Sort(files, StringComparer.Ordinal);
            int batches = (int)GetDouble(lidarObj, "batches", -1);
            long expectedPoints = (long)GetDouble(lidarObj, "points", -1);
            v.Require(files.Length > 0, "lidar 目录有二进制批次（" + files.Length + " 个）");
            v.Require(files.Length == batches, "lidar 文件数 = meta.lidar.batches（" + files.Length + "）");
            long points = 0;
            bool readable = true;
            string error = "";
            foreach (string file in files)
            {
                try { points += LidarBinary.Read(file).Count; }
                catch (Exception ex) { readable = false; error = Path.GetFileName(file) + "：" + ex.Message; break; }
            }
            v.Require(readable, readable ? "每个 lidar/*.bin 的版本头与点记录可回读" : "LiDAR 文件校验失败：" + error);
            if (readable) v.Require(points == expectedPoints, "LiDAR 点数合计 " + points + " = meta.lidar.points");
            List<ManifestEntry> rows;
            if (!byStream.TryGetValue("lidar", out rows)) v.Fail("manifest 里没有 lidar 流");
            else v.Require(rows.Count == files.Length, "manifest 中 lidar 行数 = lidar 文件数（" + rows.Count + "）");
        }

        private static string Fmt(double value)
        {
            return value.ToString("0.####", CultureInfo.InvariantCulture);
        }

        private static void ReportIntervals(TakeValidation v, string name, List<ManifestEntry> rows, double expectedMs)
        {
            double sum = 0, max = 0, min = double.MaxValue;
            int count = 0;
            for (int i = 1; i < rows.Count; i++)
            {
                double delta = rows[i].RelMs - rows[i - 1].RelMs;
                sum += delta;
                if (delta > max) max = delta;
                if (delta < min) min = delta;
                count++;
            }
            double mean = count == 0 ? 0 : sum / count;
            double meanAbsErr = 0;
            int outliers = 0;
            for (int i = 1; i < rows.Count; i++)
            {
                double delta = rows[i].RelMs - rows[i - 1].RelMs;
                meanAbsErr += Math.Abs(delta - mean);
                if (delta > 2.0 * expectedMs) outliers++;
            }
            if (count > 0) meanAbsErr /= count;
            v.Note(name + "帧间隔：均值 " + mean.ToString("0.000") + " ms（标称 " + expectedMs.ToString("0.000") +
                   "），min " + min.ToString("0.000") + " / max " + max.ToString("0.000") +
                   "，抖动量 " + meanAbsErr.ToString("0.000") + " ms，超过 2 倍标称间隔的有 " + outliers + " 次");
        }

        private static void ReportRadarIntervals(TakeValidation v, List<RadarRow> rows)
        {
            double sum = 0, max = 0, min = double.MaxValue;
            int count = 0;
            for (int i = 1; i < rows.Count; i++)
            {
                if (double.IsNaN(rows[i].FirstRel) || double.IsNaN(rows[i - 1].FirstRel)) continue;
                double delta = rows[i].FirstRel - rows[i - 1].FirstRel;
                sum += delta;
                if (delta > max) max = delta;
                if (delta < min) min = delta;
                count++;
            }
            if (count == 0) return;
            v.Note("雷达帧周期：均值 " + (sum / count).ToString("0.000") + " ms，min " + min.ToString("0.000") +
                   " / max " + max.ToString("0.000") + "（" + count + " 个间隔）");
        }

        /// <summary>
        /// 三路对齐精度：以彩色帧为参照，看每个雷达帧首包时刻距最近彩色帧的残差。
        /// 残差上限天然等于彩色帧周期的一半，能直观说明"毫秒级还是几十毫秒级"。
        /// </summary>
        private static void ReportCrossAlignment(TakeValidation v, List<ManifestEntry> colorRows, List<RadarRow> radarRows)
        {
            double sum = 0, max = 0;
            int count = 0;
            foreach (RadarRow radar in radarRows)
            {
                if (double.IsNaN(radar.FirstRel)) continue;
                double best = double.MaxValue;
                foreach (ManifestEntry color in colorRows)
                {
                    double diff = Math.Abs(color.RelMs - radar.FirstRel);
                    if (diff < best) best = diff;
                }
                if (best == double.MaxValue) continue;
                sum += best;
                if (best > max) max = best;
                count++;
            }
            if (count == 0) return;
            v.Note("雷达帧首包与最近彩色帧的残差：均值 " + (sum / count).ToString("0.000") +
                   " ms，最大 " + max.ToString("0.000") + " ms（" + count + " 帧）");
        }

        private sealed class RadarRow
        {
            public int Frame;
            public double FirstRel;
            public double LastRel;
            public int Received;
            public int Missing;
        }

        private static List<ManifestEntry> ReadManifest(string path, TakeValidation v)
        {
            List<ManifestEntry> rows = new List<ManifestEntry>();
            string[] lines = File.ReadAllLines(path, Encoding.UTF8);
            if (lines.Length == 0 || lines[0] != "stream,frame,t_rel_ms,t_utc,file")
            {
                v.Fail("manifest.csv 表头不正确");
                return rows;
            }
            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i];
                if (line.Trim().Length == 0) continue;
                string[] parts = line.Split(',');
                if (parts.Length < 5)
                {
                    v.Fail("manifest 第 " + (i + 1) + " 行字段不足");
                    break;
                }
                ManifestEntry entry = new ManifestEntry();
                entry.Stream = parts[0];
                entry.Frame = int.Parse(parts[1], CultureInfo.InvariantCulture);
                entry.RelMs = double.Parse(parts[2], CultureInfo.InvariantCulture);
                entry.Utc = DateTime.Parse(parts[3], CultureInfo.InvariantCulture,
                    DateTimeStyles.AdjustToUniversal | DateTimeStyles.AssumeUniversal);
                entry.File = string.Join(",", parts, 4, parts.Length - 4);
                rows.Add(entry);
            }
            return rows;
        }

        private static JsonObject GetObject(JsonObject obj, string key)
        {
            return obj == null ? null : obj.Get(key, null) as JsonObject;
        }

        private static double GetDouble(JsonObject obj, string key, double fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            if (value == null) return fallback;
            try { return Convert.ToDouble(value, CultureInfo.InvariantCulture); }
            catch { return fallback; }
        }

        private static string GetString(JsonObject obj, string key, string fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            return value == null ? fallback : Convert.ToString(value, CultureInfo.InvariantCulture);
        }

        private static bool GetBool(JsonObject obj, string key, bool fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            if (value is bool) return (bool)value;
            return fallback;
        }
    }
}
