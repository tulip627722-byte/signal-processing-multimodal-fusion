using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// 点云链路的离线自检：不需要相机，也不需要 take。
    /// 覆盖四件事：
    ///   1. FOV → fx/fy 的换算（含几何反算：投影半角要能还原视场角）
    ///   2. calib.json 的读取、缩放、优先级与坏文件降级
    ///   3. 16-bit PNG 写→读往返（Png16 ↔ Png16Reader）
    ///   4. 已知距离平面 → 点云的数值验证（Z 精确等于面距，X/Y 范围等于 2·tan(fov/2)·Z）
    ///      + PLY 落盘往返（顶点数、文件长度、首点坐标）
    /// </summary>
    internal static class CloudChecks
    {
        public static void Run(TakeValidation v)
        {
            string temp = Path.Combine(Path.GetTempPath(), "acquire-cloudcheck-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(temp);
            try
            {
                CheckFovMath(v);
                CheckCalibrationFile(v, temp);
                CheckPngRoundTrip(v, temp);
                CheckPlaneCloud(v, temp);
            }
            finally
            {
                try { Directory.Delete(temp, true); } catch { }
            }
        }

        private static void CheckFovMath(TakeValidation v)
        {
            DepthIntrinsics k = DepthIntrinsics.FromFov(640, 480,
                DepthIntrinsics.DegreesToRadians(58.0), DepthIntrinsics.DegreesToRadians(45.0),
                0.001, DepthIntrinsics.SourceDeviceFov, "check");

            v.Require(Math.Abs(k.Fx - 577.29) < 0.1,
                "FOV→fx：640 宽 / HFOV 58° → fx=" + k.Fx.ToString("0.00", CultureInfo.InvariantCulture) + "（期望 577.29）");
            v.Require(Math.Abs(k.Fy - 579.41) < 0.1,
                "FOV→fy：480 高 / VFOV 45° → fy=" + k.Fy.ToString("0.00", CultureInfo.InvariantCulture) + "（期望 579.41）");
            v.Require(Math.Abs(k.Cx - 319.5) < 1e-9 && Math.Abs(k.Cy - 239.5) < 1e-9,
                "主点 =(W-1)/2,(H-1)/2 → " + k.Cx.ToString("0.#", CultureInfo.InvariantCulture) + "," +
                k.Cy.ToString("0.#", CultureInfo.InvariantCulture));

            // 反算：由 fx 投影出的水平半角必须还原 58°
            double hfovDeg = 2.0 * Math.Atan((640 / 2.0) / k.Fx) * 180.0 / Math.PI;
            double vfovDeg = 2.0 * Math.Atan((480 / 2.0) / k.Fy) * 180.0 / Math.PI;
            v.Require(Math.Abs(hfovDeg - 58.0) < 0.05 && Math.Abs(vfovDeg - 45.0) < 0.05,
                "反算视场角：hfov=" + hfovDeg.ToString("0.000", CultureInfo.InvariantCulture) +
                "° vfov=" + vfovDeg.ToString("0.000", CultureInfo.InvariantCulture) + "°（应回到 58/45）");

            DepthIntrinsics nominal = DepthIntrinsics.Nominal(640, 480, 0.001, null);
            v.Require(nominal.Source == DepthIntrinsics.SourceFallback && Math.Abs(nominal.Fx - k.Fx) < 1e-9,
                "标称兜底档：source=" + nominal.Source + "，fx 与 58°/45° 一致");
        }

        private static void CheckCalibrationFile(TakeValidation v, string temp)
        {
            string path = Path.Combine(temp, "calib-good.json");
            File.WriteAllText(path,
                "{\r\n  \"fx\": 600.0,\r\n  \"fy\": 601.0,\r\n  \"cx\": 321.25,\r\n  \"cy\": 238.75,\r\n" +
                "  \"width\": 640,\r\n  \"height\": 480,\r\n  \"depthScale\": 0.0001\r\n}\r\n", new UTF8Encoding(false));

            string error;
            DepthIntrinsics k = DepthIntrinsics.TryLoadCalibration(path, 640, 480, 0.001, out error);
            v.Require(k != null && k.Source == DepthIntrinsics.SourceCalibration,
                "calib.json 读取成功，source=calibration" + (k == null ? "（失败：" + error + "）" : ""));
            if (k != null)
            {
                v.Require(Math.Abs(k.Fx - 600.0) < 1e-9 && Math.Abs(k.Cx - 321.25) < 1e-9 &&
                          Math.Abs(k.DepthScale - 0.0001) < 1e-12,
                    "calib.json 数值被原样采用：fx=600, cx=321.25, depthScale=0.0001");
            }

            // 标定在 320x240 上做、采集成 640x480：按比例缩放
            string half = Path.Combine(temp, "calib-half.json");
            File.WriteAllText(half,
                "{\"fx\":300,\"fy\":300.5,\"cx\":160.25,\"cy\":119.25,\"width\":320,\"height\":240}\r\n",
                new UTF8Encoding(false));
            DepthIntrinsics scaled = DepthIntrinsics.TryLoadCalibration(half, 640, 480, 0.001, out error);
            v.Require(scaled != null && Math.Abs(scaled.Fx - 600.0) < 1e-9 && Math.Abs(scaled.Cx - 320.5) < 1e-9,
                "calib 分辨率不同时按比例缩放（320x240 → 640x480：fx 300→600, cx 160.25→320.5）");

            // 优先级：calib.json 存在时压过设备 FOV
            DepthIntrinsics resolved = DepthIntrinsics.Resolve(640, 480, 0.001,
                DepthIntrinsics.DegreesToRadians(58.0), DepthIntrinsics.DegreesToRadians(45.0), path, null);
            v.Require(resolved.Source == DepthIntrinsics.SourceCalibration && Math.Abs(resolved.Fx - 600.0) < 1e-9,
                "calib.json 优先级高于设备 FOV");

            // 坏文件：不抛异常，返回 null + 原因（采集时会降级到下一档）
            string broken = Path.Combine(temp, "calib-broken.json");
            File.WriteAllText(broken, "{\"fy\": 10}\r\n", new UTF8Encoding(false));
            string brokenError;
            DepthIntrinsics bad = DepthIntrinsics.TryLoadCalibration(broken, 640, 480, 0.001, out brokenError);
            v.Require(bad == null && !string.IsNullOrEmpty(brokenError),
                "坏 calib.json 返回 null 且有原因（" + (brokenError ?? "") + "）");
        }

        private static void CheckPngRoundTrip(TakeValidation v, string temp)
        {
            const int width = 37;
            const int height = 23;
            ushort[] depth = new ushort[width * height];
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                    depth[y * width + x] = (ushort)((x * 1777 + y * 293) & 0xFFFF);
            }
            depth[0] = 0;
            depth[1] = 1;
            depth[2] = 65535;
            depth[3] = 1000;

            string path = Path.Combine(temp, "roundtrip.png");
            Png16.Write(path, depth, width, height);

            try
            {
                DepthImage image = Png16Reader.Read(path);
                bool ok = image.Width == width && image.Height == height && image.Data.Length == depth.Length;
                int firstMismatch = -1;
                if (ok)
                {
                    for (int i = 0; i < depth.Length; i++)
                    {
                        if (image.Data[i] != depth[i]) { firstMismatch = i; ok = false; break; }
                    }
                }
                v.Require(ok, string.IsNullOrEmpty(image.Warning)
                    ? "16-bit PNG 往返一致（" + width + "x" + height + " = " + depth.Length + " 像素）"
                    : "16-bit PNG 往返一致但 8-bit 告警：" + image.Warning);
                if (!ok && firstMismatch >= 0)
                    v.Fail("PNG 往返第 " + firstMismatch + " 个像素不符：" + depth[firstMismatch] + " → " + image.Data[firstMismatch]);
            }
            catch (Exception ex)
            {
                v.Fail("16-bit PNG 读取失败：" + ex.Message);
            }
        }

        private static void CheckPlaneCloud(TakeValidation v, string temp)
        {
            const int width = 640;
            const int height = 480;
            const double planeMeters = 2.0;
            const int stride = 4;

            ushort[] depth = new ushort[width * height];
            for (int i = 0; i < depth.Length; i++) depth[i] = (ushort)2000;   // 2000 mm
            for (int y = 100; y < 140; y++)
                for (int x = 100; x < 140; x++) depth[y * width + x] = 0;     // 40x40 空洞

            DepthIntrinsics k = DepthIntrinsics.FromFov(width, height,
                DepthIntrinsics.DegreesToRadians(58.0), DepthIntrinsics.DegreesToRadians(45.0),
                0.001, DepthIntrinsics.SourceDeviceFov, "plane-check");

            int sampled, valid;
            List<CloudPoint> points = DepthCloud.Build(depth, width, height, k, stride, false, out sampled, out valid);

            int gridCols = width / stride;
            int gridRows = height / stride;
            int holeCols = (136 - 100) / stride + 1;
            int holeRows = (136 - 100) / stride + 1;
            int expectedSampled = gridCols * gridRows;
            int expectedValid = expectedSampled - holeCols * holeRows;

            v.Require(sampled == expectedSampled && valid == expectedValid,
                "平面取样：网格 " + sampled + " 点（期望 " + expectedSampled + "），有效 " + valid +
                "（期望 " + expectedValid + " = 网格 - " + (holeCols * holeRows) + " 个空洞）");
            v.Require(points.Count == valid, "点数 = 有效网格像素数（" + points.Count + "）");

            // Z：整块平面必须严格落在 2 m（2000 × 0.001）
            double minZ = double.MaxValue, maxZ = double.MinValue;
            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            for (int i = 0; i < points.Count; i++)
            {
                CloudPoint p = points[i];
                if (p.Z < minZ) minZ = p.Z;
                if (p.Z > maxZ) maxZ = p.Z;
                if (p.X < minX) minX = p.X;
                if (p.X > maxX) maxX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.Y > maxY) maxY = p.Y;
            }
            v.Require(Math.Abs(minZ - planeMeters) < 1e-9 && Math.Abs(maxZ - planeMeters) < 1e-9,
                "2 m 平面自检：所有点 Z = " + minZ.ToString("0.000000", CultureInfo.InvariantCulture) +
                "~" + maxZ.ToString("0.000000", CultureInfo.InvariantCulture) + " m（期望 2.000000）");

            // X/Y 跨度必须等于 2·tan(hfov/2)·Z（几何上唯一自洽的答案）
            double halfWidth = planeMeters * Math.Tan(DepthIntrinsics.DegreesToRadians(58.0) / 2.0);
            double halfHeight = planeMeters * Math.Tan(DepthIntrinsics.DegreesToRadians(45.0) / 2.0);
            double imageWidthAtZ = width * planeMeters / k.Fx;
            v.Require(Math.Abs(imageWidthAtZ - 2.0 * halfWidth) < 0.002 * 2.0 * halfWidth,
                "图像物理宽度 " + imageWidthAtZ.ToString("0.0000", CultureInfo.InvariantCulture) +
                " m = 2·tan(29°)·Z = " + (2.0 * halfWidth).ToString("0.0000", CultureInfo.InvariantCulture) + " m");
            v.Require(minX > -halfWidth && maxX < halfWidth &&
                      (maxX - minX) > 2.0 * halfWidth - 2.0 * stride * planeMeters / k.Fx,
                "X 采样区间 " + minX.ToString("0.0000", CultureInfo.InvariantCulture) + "~" +
                maxX.ToString("0.0000", CultureInfo.InvariantCulture) + " m 落在视场 ±" +
                halfWidth.ToString("0.0000", CultureInfo.InvariantCulture) + " 内（网格末端比图像边缘少 1 个 stride）");
            double imageHeightAtZ = height * planeMeters / k.Fy;
            v.Require(Math.Abs(imageHeightAtZ - 2.0 * halfHeight) < 0.002 * 2.0 * halfHeight,
                "图像物理高度 " + imageHeightAtZ.ToString("0.0000", CultureInfo.InvariantCulture) +
                " m = 2·tan(22.5°)·Z = " + (2.0 * halfHeight).ToString("0.0000", CultureInfo.InvariantCulture) + " m");
            v.Require(minY > -halfHeight && maxY < halfHeight &&
                      (maxY - minY) > 2.0 * halfHeight - 2.0 * stride * planeMeters / k.Fy,
                "Y 采样区间 " + minY.ToString("0.0000", CultureInfo.InvariantCulture) + "~" +
                maxY.ToString("0.0000", CultureInfo.InvariantCulture) + " m 落在视场 ±" +
                halfHeight.ToString("0.0000", CultureInfo.InvariantCulture) + " 内");

            // 主点附近：u=320 距 cx=319.5 只有半像素
            CloudPoint center = FindPoint(points, 320, 240);
            double expectedCenterX = (320 - 319.5) * planeMeters / k.Fx;
            v.Require(Math.Abs(center.X - expectedCenterX) < 1e-12,
                "主点像素 (320,240) → X=" + center.X.ToString("0.000000", CultureInfo.InvariantCulture) +
                " m（= 0.5·Z/fx，半像素偏移，量级 1.7 mm）");

            // 雷达坐标系：Xr=Xc, Yr=Zc, Zr=-Yc
            List<CloudPoint> radar = new List<CloudPoint>(points);
            DepthCloud.ToRadarFrame(radar);
            CloudPoint origin = FindPoint(radar, 0, 0);
            CloudPoint cameraOrigin = FindPoint(points, 0, 0);
            v.Require(Math.Abs(origin.X - cameraOrigin.X) < 1e-12 &&
                      Math.Abs(origin.Y - planeMeters) < 1e-9 &&
                      Math.Abs(origin.Z + cameraOrigin.Y) < 1e-12,
                "雷达系转换 (0,0)：" + "Xr=" + origin.X.ToString("0.0000", CultureInfo.InvariantCulture) +
                " Yr=" + origin.Y.ToString("0.0000", CultureInfo.InvariantCulture) +
                " Zr=" + origin.Z.ToString("0.0000", CultureInfo.InvariantCulture) +
                "（Yr 应为 +Z=2.0000，Zr 应为 -Y=+" + (-cameraOrigin.Y).ToString("0.0000", CultureInfo.InvariantCulture) + "）");

            // PLY 往返
            string path = Path.Combine(temp, "plane.ply");
            DepthCloud.WritePly(path, points, "000001", "self-test plane");
            string error, format;
            int count = DepthCloud.ReadPlyVertexCount(path, out error, out format);
            long expectedBytes = Encoding.ASCII.GetByteCount(DepthCloud.BuildPlyHeader(points.Count, "000001", "self-test plane"));
            long fileBytes = new FileInfo(path).Length;
            v.Require(count == points.Count,
                "PLY 顶点数 " + count + " = 点云点数 " + points.Count + (error == null ? "" : "（" + error + "）"));
            v.Require(fileBytes == expectedBytes + (long)points.Count * 12,
                "PLY 文件长度 = 头 " + expectedBytes + " + " + points.Count + "×12 = " + fileBytes + " 字节（" +
                (format ?? "?") + "）");
            v.Require(FirstPlyVertexMatches(path, points[0]),
                "PLY 首点坐标可回读：" + points[0].X.ToString("0.0000", CultureInfo.InvariantCulture) + ", " +
                points[0].Y.ToString("0.0000", CultureInfo.InvariantCulture) + ", " +
                points[0].Z.ToString("0.0000", CultureInfo.InvariantCulture));

            // 采样密度：640x480 / 4² ≈ 1.9 万点
            v.Note("平面点云规模：" + points.Count + " 点 ≈ " + (width * height / (stride * stride)) + "（640×480 ÷ 4²）");
        }

        private static CloudPoint FindPoint(List<CloudPoint> points, int u, int v)
        {
            for (int i = 0; i < points.Count; i++)
            {
                if (points[i].U == u && points[i].V == v) return points[i];
            }
            throw new InvalidOperationException("点云里找不到像素 (" + u + "," + v + ")");
        }

        private static bool FirstPlyVertexMatches(string path, CloudPoint expected)
        {
            try
            {
                using (FileStream stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
                using (BinaryReader reader = new BinaryReader(stream))
                {
                    StringBuilder head = new StringBuilder();
                    while (true)
                    {
                        string line = ReadLine(reader);
                        if (line == null) return false;
                        head.Append(line).Append('\n');
                        if (line.Trim() == "end_header") break;
                    }
                    if (stream.Position + 12 > stream.Length) return false;
                    float x = reader.ReadSingle();
                    float y = reader.ReadSingle();
                    float z = reader.ReadSingle();
                    return Math.Abs(x - expected.X) < 1e-4 && Math.Abs(y - expected.Y) < 1e-4 && Math.Abs(z - expected.Z) < 1e-4;
                }
            }
            catch
            {
                return false;
            }
        }

        private static string ReadLine(BinaryReader reader)
        {
            StringBuilder sb = new StringBuilder();
            while (true)
            {
                int b = reader.BaseStream.ReadByte();
                if (b < 0) return sb.Length == 0 ? null : sb.ToString();
                if (b == '\n') return sb.ToString().TrimEnd('\r');
                sb.Append((char)b);
            }
        }
    }
}
