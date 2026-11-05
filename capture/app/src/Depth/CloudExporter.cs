using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    internal sealed class CloudExportOptions
    {
        public string TakeDirectory;
        public int Stride = DepthCloud.DefaultStride;
        /// <summary>0 = 导出全部帧；>0 = 只导出该帧号（与 depth\NNNNNN.png 的编号一致）。</summary>
        public int Frame;
        /// <summary>额外输出雷达坐标系版本（cloud_radar\）。</summary>
        public bool RadarFrame;
        /// <summary>输出 X,Y,Z 文本而不是二进制 PLY。</summary>
        public bool Csv;
        /// <summary>默认 &lt;take&gt;\cloud。</summary>
        public string OutputDirectory;
        public Action<string> Log;
    }

    /// <summary>
    /// 离线转换：take 里的 depth\*.png + meta.json 的内参 → cloud\*.ply（或 .csv）。
    /// 采集时存的是“原始深度图 + 内参”，点云永远是派生数据，所以任何 take 都能事后换 stride 重算。
    /// 退出码：0 成功；1 运行期失败（读不出图、写盘失败等）；2 参数/结构错误。
    /// </summary>
    internal static class CloudExporter
    {
        public static int Run(CloudExportOptions options)
        {
            Action<string> log = options != null && options.Log != null ? options.Log : delegate { };
            if (options == null)
            {
                log("内部错误：没有传入导出参数");
                return 2;
            }
            if (string.IsNullOrEmpty(options.TakeDirectory))
            {
                log("缺少 take 目录。用法：Acquire.exe --export-cloud <take目录> [--stride 4] [--frame 120] [--radar-frame] [--csv]");
                return 2;
            }

            string take;
            try { take = Path.GetFullPath(options.TakeDirectory); }
            catch (Exception ex) { log("路径无法解析：" + options.TakeDirectory + "（" + ex.Message + "）"); return 2; }

            if (!Directory.Exists(take))
            {
                log("take 目录不存在：" + take);
                return 2;
            }
            string metaPath = Path.Combine(take, "meta.json");
            if (!File.Exists(metaPath))
            {
                log("缺少 meta.json（这不是本程序生成的 take？）：" + metaPath);
                return 2;
            }
            if (options.Stride < 1 || options.Stride > 64)
            {
                log("--stride 必须在 1~64 之间（当前 " + options.Stride + "）");
                return 2;
            }
            if (options.Frame < 0)
            {
                log("--frame 必须 >= 1（当前 " + options.Frame + "）");
                return 2;
            }

            JsonObject meta;
            try
            {
                meta = MiniJson.Parse(File.ReadAllText(metaPath, Encoding.UTF8)) as JsonObject;
            }
            catch (Exception ex)
            {
                log("meta.json 解析失败：" + ex.Message);
                return 2;
            }
            if (meta == null)
            {
                log("meta.json 根节点不是 JSON 对象");
                return 2;
            }

            string warn;
            DepthIntrinsics intrinsics = LoadIntrinsics(meta, out warn);
            if (!string.IsNullOrEmpty(warn)) log("提示：" + warn);
            if (intrinsics == null || !intrinsics.Valid)
            {
                log("meta.json 里没有可用内参，也没有可参照的分辨率，无法生成点云。" +
                    "请用带深度内参的新版本重新采集，或在 exe 同目录放 calib.json。");
                return 1;
            }

            string depthDir = Path.Combine(take, "depth");
            List<string> depthFiles = Png16Reader.ListDepthFiles(depthDir);
            if (depthFiles.Count == 0)
            {
                log("depth 目录里没有深度图：" + depthDir);
                return 1;
            }

            List<string> targets = new List<string>();
            if (options.Frame > 0)
            {
                string name = options.Frame.ToString("D6", CultureInfo.InvariantCulture) + ".png";
                string candidate = Path.Combine(depthDir, name);
                if (!File.Exists(candidate))
                {
                    log("第 " + options.Frame + " 帧不存在：" + candidate +
                        "（该 take 的深度帧是 1~" + depthFiles.Count + "）");
                    return 2;
                }
                targets.Add(candidate);
            }
            else
            {
                targets.AddRange(depthFiles);
            }

            string cloudDir = string.IsNullOrEmpty(options.OutputDirectory)
                ? Path.Combine(take, "cloud")
                : Path.GetFullPath(options.OutputDirectory);
            string radarDir = Path.Combine(take, "cloud_radar");

            try
            {
                Directory.CreateDirectory(cloudDir);
                if (options.RadarFrame) Directory.CreateDirectory(radarDir);
            }
            catch (Exception ex)
            {
                log("创建输出目录失败：" + ex.Message);
                return 1;
            }

            log("take：" + take);
            log("内参：" + intrinsics.Describe());
            if (!string.IsNullOrEmpty(intrinsics.Detail)) log("      来源：" + intrinsics.Detail);
            log("公式：Z = depth * depthScale；X = (u - cx) * Z / fx；Y = (v - cy) * Z / fy");
            log("取样：每 " + options.Stride + "x" + options.Stride + " 像素取 1 点，深度为 0 的像素跳过");
            log("坐标系：相机 X右 / Y下 / Z前" +
                (options.RadarFrame ? "；另出雷达系 X右 / Y前 / Z上（Xr=Xc, Yr=Zc, Zr=-Yc）" : ""));
            log("输出：" + cloudDir + (options.RadarFrame ? " 与 " + radarDir : "") +
                "（" + (options.Csv ? "CSV X,Y,Z" : "PLY binary_little_endian") + "）");
            log("");

            DateTime started = DateTime.Now;
            int frames = 0;
            long sampledTotal = 0;
            long validTotal = 0;
            long bytes = 0;
            const int binCount = 2000;              // Z 直方图：0~20 m，1 cm 分箱
            int[] histogram = new int[binCount + 1];
            string ext = options.Csv ? ".csv" : ".ply";
            int reportEvery = targets.Count <= 30 ? 1 : 30;

            foreach (string file in targets)
            {
                DepthImage image;
                try
                {
                    image = Png16Reader.Read(file);
                }
                catch (Exception ex)
                {
                    log("读取失败 " + Path.GetFileName(file) + "：" + ex.Message);
                    return 1;
                }

                if (intrinsics.Width != image.Width || intrinsics.Height != image.Height)
                {
                    log("提示：深度图是 " + image.Width + "x" + image.Height + "，但内参记录的是 " +
                        intrinsics.Width + "x" + intrinsics.Height + "，改用 calib.json / 标称值重新推算");
                    intrinsics = DepthIntrinsics.Resolve(image.Width, image.Height, intrinsics.DepthScale,
                        null, null, DepthIntrinsics.CalibrationPath, log);
                    log("内参（重推）：" + intrinsics.Describe());
                }
                if (!string.IsNullOrEmpty(image.Warning)) log("提示：" + Path.GetFileName(file) + "：" + image.Warning);

                int sampled, valid;
                List<CloudPoint> points;
                try
                {
                    points = DepthCloud.Build(image.Data, image.Width, image.Height, intrinsics,
                        options.Stride, false, out sampled, out valid);
                }
                catch (Exception ex)
                {
                    log("生成点云失败 " + Path.GetFileName(file) + "：" + ex.Message);
                    return 1;
                }

                string stem = Path.GetFileNameWithoutExtension(file);
                try
                {
                    string outPath = Path.Combine(cloudDir, stem + ext);
                    Write(outPath, points, stem, "frame=camera", options.Csv);
                    bytes += new FileInfo(outPath).Length;

                    if (options.RadarFrame)
                    {
                        List<CloudPoint> radarPoints = new List<CloudPoint>(points.Count);
                        for (int i = 0; i < points.Count; i++)
                        {
                            CloudPoint p = points[i];
                            CloudPoint r = p;
                            r.X = p.X;
                            r.Y = p.Z;
                            r.Z = -p.Y;
                            radarPoints.Add(r);
                        }
                        string radarPath = Path.Combine(radarDir, stem + ext);
                        Write(radarPath, radarPoints, stem, "frame=radar (Xr=Xc, Yr=Zc, Zr=-Yc)", options.Csv);
                        bytes += new FileInfo(radarPath).Length;
                    }
                }
                catch (Exception ex)
                {
                    log("写文件失败（" + stem + "）：" + ex.Message);
                    return 1;
                }

                frames++;
                sampledTotal += sampled;
                validTotal += valid;
                for (int i = 0; i < points.Count; i++)
                {
                    int bin = (int)(points[i].Z / 0.01);
                    if (bin < 0) bin = 0;
                    if (bin > binCount) bin = binCount;
                    histogram[bin]++;
                }

                if (reportEvery == 1 || frames % reportEvery == 0 || frames == targets.Count)
                {
                    log("  " + stem + ext + "：采样 " + sampled + " / 有效 " + valid + " → " + points.Count +
                        " 点，" + DepthCloud.DescribeExtent(points));
                }
            }

            double seconds = (DateTime.Now - started).TotalSeconds;
            log("");
            log("完成：帧 " + frames + "，采样像素 " + sampledTotal + "，有效点 " + validTotal +
                "（" + (sampledTotal == 0 ? 0 : 100.0 * validTotal / sampledTotal).ToString("0.0", CultureInfo.InvariantCulture) +
                "%），写盘 " + (bytes / 1024).ToString(CultureInfo.InvariantCulture) + " KB，耗时 " +
                seconds.ToString("0.00", CultureInfo.InvariantCulture) + " s");
            if (validTotal > 0)
                log("Z 分布（相机系，1 cm 分箱）：5% " + ZFormat(Percentile(histogram, validTotal, 0.05)) +
                    " / 中位 " + ZFormat(Percentile(histogram, validTotal, 0.5)) +
                    " / 95% " + ZFormat(Percentile(histogram, validTotal, 0.95)) +
                    "　（对准已知距离的平面时，中位数应接近卷尺读数）");
            if (!options.RadarFrame)
                log("提示：需要雷达坐标系版本时加 --radar-frame（注意外参尚未标定，两套点云只能各自看）");
            return 0;
        }

        /// <summary>直方图百分位（1 cm 分箱），用于给“已知距离平面”验算提供一个可比的数字。</summary>
        private static double Percentile(int[] histogram, long total, double fraction)
        {
            if (total <= 0) return 0;
            long target = (long)Math.Floor(total * fraction);
            if (target < 1) target = 1;
            long accumulated = 0;
            for (int i = 0; i < histogram.Length; i++)
            {
                accumulated += histogram[i];
                if (accumulated >= target) return i * 0.01;
            }
            return (histogram.Length - 1) * 0.01;
        }

        private static string ZFormat(double meters)
        {
            return meters.ToString("0.00", CultureInfo.InvariantCulture) + " m";
        }

        private static void Write(string path, List<CloudPoint> points, string frame, string comment, bool csv)
        {
            if (csv) DepthCloud.WriteCsv(path, points);
            else DepthCloud.WritePly(path, points, frame, comment);
        }

        /// <summary>内参优先取 meta.camera.depthIntrinsics；老 take 没有时按 calib.json / 标称值重推。</summary>
        private static DepthIntrinsics LoadIntrinsics(JsonObject meta, out string warning)
        {
            warning = null;
            JsonObject camera = meta.Get("camera", null) as JsonObject;
            JsonObject stored = camera == null ? null : camera.Get("depthIntrinsics", null) as JsonObject;
            if (stored != null)
            {
                DepthIntrinsics k = FromJson(stored);
                if (k != null && k.Valid) return k;
                warning = "meta.camera.depthIntrinsics 字段不完整，改用标定/标称值推算";
            }

            JsonObject depth = meta.Get("depth", null) as JsonObject;
            int width = (int)Number(depth, "width", 0);
            int height = (int)Number(depth, "height", 0);
            double scale = Number(depth, "depthScaleMetersPerUnit", 0);
            if (width <= 0 || height <= 0)
            {
                JsonObject color = meta.Get("color", null) as JsonObject;
                width = (int)Number(color, "width", 0);
                height = (int)Number(color, "height", 0);
            }
            if (width <= 0 || height <= 0) return null;
            if (warning == null) warning = "meta.json 没有内参字段，按 calib.json / 标称 FOV 重新推算（老 take 兼容路径）";
            return DepthIntrinsics.Resolve(width, height, scale > 0 ? scale : 0.001,
                null, null, DepthIntrinsics.CalibrationPath, null);
        }

        private static DepthIntrinsics FromJson(JsonObject obj)
        {
            if (obj == null) return null;
            DepthIntrinsics k = new DepthIntrinsics();
            k.Fx = Number(obj, "fx", 0);
            k.Fy = Number(obj, "fy", 0);
            k.Cx = Number(obj, "cx", 0);
            k.Cy = Number(obj, "cy", 0);
            k.Width = (int)Number(obj, "width", 0);
            k.Height = (int)Number(obj, "height", 0);
            k.DepthScale = Number(obj, "depthScale", 0.001);
            if (k.DepthScale <= 0) k.DepthScale = 0.001;
            string source = obj.Get("source", null) as string;
            k.Source = string.IsNullOrEmpty(source) ? "unknown" : source;
            string note = obj.Get("note", null) as string;
            k.Detail = note ?? "";
            return k;
        }

        private static double Number(JsonObject obj, string key, double fallback)
        {
            if (obj == null) return fallback;
            object value = obj.Get(key, null);
            if (value == null) return fallback;
            if (value is double) return (double)value;
            try { return Convert.ToDouble(value, CultureInfo.InvariantCulture); }
            catch { return fallback; }
        }
    }
}
