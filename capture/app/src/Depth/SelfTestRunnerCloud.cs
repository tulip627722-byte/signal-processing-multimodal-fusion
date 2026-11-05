using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// --self-test 里的点云验证段。单独成类，避免把 SelfTest.cs 撑得太大。
    ///
    /// 做三件事：
    ///   1. 无相机的离线自检（针孔公式、calib.json、16-bit PNG 往返、2 m 平面 → 点云数值）
    ///   2. 在刚生成的 take 上真的跑一遍 --export-cloud（含 --radar-frame / --frame / 坏参数）
    ///   3. 独立重数：不信任程序内部计数，直接解码每张 depth\*.png 数网格上的非零像素，
    ///      和 PLY 顶点数逐帧比对
    /// </summary>
    internal static class SelfTestRunnerCloud
    {
        public static void VerifyClouds(Action<string> log, string takeDir, TakeValidation validation)
        {
            log("");
            log("-- 点云 / 内参离线自检（无需相机） --");
            TakeValidation checks = new TakeValidation();
            try
            {
                CloudChecks.Run(checks);
            }
            catch (Exception ex)
            {
                checks.Fail("点云自检抛异常：" + ex.Message);
            }
            foreach (string line in checks.Passed) log("  PASS  " + line);
            foreach (string line in checks.Notes) log("  INFO  " + line);
            foreach (string line in checks.Failed) log("  FAIL  " + line);
            validation.Require(checks.Ok, "点云/内参离线自检全部通过（" + checks.Passed.Count + " 项）");

            log("");
            log("-- --export-cloud 离线导出 --");

            int code = -1;
            try
            {
                CloudExportOptions export = new CloudExportOptions();
                export.TakeDirectory = takeDir;
                export.Stride = 4;
                export.RadarFrame = true;
                export.OutputDirectory = Path.Combine(takeDir, "cloud_export");
                export.Log = log;
                code = CloudExporter.Run(export);
            }
            catch (Exception ex)
            {
                log("  异常：" + ex.Message);
            }
            validation.Require(code == 0, "export-cloud 全帧导出退出码 0（实际 " + code + "）");

            string frameOneCsv = Path.Combine(takeDir, "cloud_export_frame1", "000001.csv");
            int singleCode = -1;
            try
            {
                CloudExportOptions single = new CloudExportOptions();
                single.TakeDirectory = takeDir;
                single.Stride = 4;
                single.Frame = 1;
                single.Csv = true;
                single.OutputDirectory = Path.Combine(takeDir, "cloud_export_frame1");
                single.Log = delegate { };
                singleCode = CloudExporter.Run(single);
            }
            catch (Exception ex)
            {
                log("  --frame 1 异常：" + ex.Message);
            }
            validation.Require(singleCode == 0 && File.Exists(frameOneCsv),
                "--export-cloud --frame 1 --csv 只导出第 1 帧为 000001.csv");

            int badCode = -1;
            try
            {
                CloudExportOptions bad = new CloudExportOptions();
                bad.TakeDirectory = Path.Combine(takeDir, "__no_such_take__");
                bad.Log = delegate { };
                badCode = CloudExporter.Run(bad);
            }
            catch { }
            validation.Require(badCode == 2, "不存在的 take → export-cloud 退出码 2（实际 " + badCode + "）");

            CheckPointCounts(log, takeDir, validation);
        }

        private static void CheckPointCounts(Action<string> log, string takeDir, TakeValidation validation)
        {
            log("");
            log("-- 点数独立核对（解码 depth\\*.png 重新数） --");
            string depthDir = Path.Combine(takeDir, "depth");
            string[] depthFiles = Directory.Exists(depthDir) ? Directory.GetFiles(depthDir, "*.png") : new string[0];
            Array.Sort(depthFiles, StringComparer.Ordinal);

            const int stride = 4;
            long independentTotal = 0;
            long firstFrameExpected = -1;
            int mismatches = 0;
            int checkedFrames = 0;

            foreach (string file in depthFiles)
            {
                DepthImage image;
                try
                {
                    image = Png16Reader.Read(file);
                }
                catch (Exception ex)
                {
                    validation.Fail("解码深度图失败 " + Path.GetFileName(file) + "：" + ex.Message);
                    return;
                }

                int sampled = 0;
                int valid = 0;
                for (int v = 0; v < image.Height; v += stride)
                {
                    int row = v * image.Width;
                    for (int u = 0; u < image.Width; u += stride)
                    {
                        sampled++;
                        if (image.Data[row + u] != 0) valid++;
                    }
                }

                independentTotal += valid;
                if (firstFrameExpected < 0) firstFrameExpected = valid;

                string stem = Path.GetFileNameWithoutExtension(file);
                if (!ComparePly(Path.Combine(takeDir, "cloud_export", stem + ".ply"), valid)) mismatches++;
                if (!ComparePly(Path.Combine(takeDir, "cloud", stem + ".ply"), valid)) mismatches++;
                if (!ComparePly(Path.Combine(takeDir, "cloud_radar", stem + ".ply"), valid)) mismatches++;
                checkedFrames++;
            }

            validation.Require(mismatches == 0,
                "每帧 PLY 点数 = 该帧有效深度网格像素数（独立重数 " + checkedFrames + " 帧，失配 " + mismatches + " 处）");

            // 采集时写进 meta 的点数合计也要对得上
            string metaPath = Path.Combine(takeDir, "meta.json");
            long metaPoints = -1;
            int metaFrames = -1;
            try
            {
                JsonObject meta = MiniJson.Parse(File.ReadAllText(metaPath, Encoding.UTF8)) as JsonObject;
                JsonObject depth = meta == null ? null : meta.Get("depth", null) as JsonObject;
                JsonObject cloud = depth == null ? null : depth.Get("cloud", null) as JsonObject;
                if (cloud != null)
                {
                    metaPoints = (long)GetDouble(cloud, "points", -1);
                    metaFrames = (int)GetDouble(cloud, "frames", -1);
                }
            }
            catch (Exception ex)
            {
                log("  读取 meta.json 失败：" + ex.Message);
            }

            validation.Require(metaFrames == checkedFrames,
                "meta.depth.cloud.frames = 深度帧数（" + metaFrames + " vs " + checkedFrames + "）");
            validation.Require(metaPoints == independentTotal,
                "meta.depth.cloud.points = 独立重数合计（" + metaPoints + " vs " + independentTotal + "）");

            // CSV 单帧：表头 + 点数
            string csv = Path.Combine(takeDir, "cloud_export_frame1", "000001.csv");
            if (File.Exists(csv) && firstFrameExpected >= 0)
            {
                int lines = 0;
                using (StreamReader reader = new StreamReader(csv, Encoding.UTF8))
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (line.Trim().Length > 0) lines++;
                    }
                }
                validation.Require(lines == firstFrameExpected + 1,
                    "CSV 行数 = 第 1 帧点数 + 表头（" + lines + " = " + firstFrameExpected + " + 1）");
            }

            log("  合计：独立重数 " + independentTotal + " 点 / " + checkedFrames + " 帧，帧均 " +
                (checkedFrames == 0 ? 0 : independentTotal / checkedFrames) + " 点");
        }

        private static bool ComparePly(string path, int expected)
        {
            if (!File.Exists(path)) return false;
            string error;
            string format;
            int count = DepthCloud.ReadPlyVertexCount(path, out error, out format);
            return count == expected;
        }

        private static double GetDouble(JsonObject obj, string key, double fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            if (value == null) return fallback;
            try { return Convert.ToDouble(value, System.Globalization.CultureInfo.InvariantCulture); }
            catch { return fallback; }
        }
    }
}
