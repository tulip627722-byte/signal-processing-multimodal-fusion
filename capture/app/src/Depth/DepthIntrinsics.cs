using System;
using System.Globalization;
using System.IO;

namespace Acquire
{
    /// <summary>
    /// 深度相机针孔模型内参。Astra Pro 只输出 16-bit 深度图，点云必须靠这套参数算：
    ///
    ///   Z = depth(u,v) * depthScale
    ///   X = (u - cx) * Z / fx
    ///   Y = (v - cy) * Z / fy
    ///
    /// u/v 是像素列/行下标（0 起）。cx/cy 取 (W-1)/2、(H-1)/2：
    /// 640 宽的图像中心在连续坐标 320，像素 319 的中心是 319.5、像素 320 的中心是 320.5，
    /// 只有 cx=319.5 才能让这两个像素的 X 互为相反数。用 W/2 会整体偏半个像素。
    ///
    /// 获取顺序（从可信到兜底）：
    ///   1. calib.json（exe 同目录，用户可覆盖）→ source=calibration
    ///   2. OpenNI2 流属性 HORIZONTAL_FOV / VERTICAL_FOV → source=deviceFov
    ///   3. Astra Pro 标称 HFOV 58° / VFOV 45° → source=fallback
    /// </summary>
    internal sealed class DepthIntrinsics
    {
        public const string SourceCalibration = "calibration";
        public const string SourceDeviceFov = "deviceFov";
        public const string SourceFallback = "fallback";
        public const string SourceSynthetic = "synthetic";

        /// <summary>Astra Pro 标称视场角（厂家参数，仅在没有更好的来源时使用）。</summary>
        public const double AstraNominalHFovDeg = 58.0;
        public const double AstraNominalVFovDeg = 45.0;

        public double Fx;
        public double Fy;
        public double Cx;
        public double Cy;
        public int Width;
        public int Height;
        public double DepthScale = 0.001;

        /// <summary>calibration / deviceFov / fallback / synthetic。</summary>
        public string Source = SourceFallback;

        /// <summary>给人看的来源说明，例如 "OpenNI2：hfov=58.0°, vfov=45.2°"。</summary>
        public string Detail = "";

        public bool Valid
        {
            get { return Fx > 0 && Fy > 0 && Width > 0 && Height > 0; }
        }

        /// <summary>calib.json 默认位置：exe 同目录（用户可覆盖，不动 DLL）。</summary>
        public static string CalibrationPath
        {
            get { return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "calib.json"); }
        }

        public static double DegreesToRadians(double degrees)
        {
            return degrees * Math.PI / 180.0;
        }

        /// <summary>由视场角推内参：fx = (W/2)/tan(hfov/2)，fy = (H/2)/tan(vfov/2)。</summary>
        public static DepthIntrinsics FromFov(int width, int height, double hfovRadians, double vfovRadians,
                                              double depthScale, string source, string detail)
        {
            if (width <= 0 || height <= 0) throw new ArgumentException("图像尺寸非法");
            if (hfovRadians <= 0 || hfovRadians >= Math.PI || vfovRadians <= 0 || vfovRadians >= Math.PI)
                throw new ArgumentOutOfRangeException("hfovRadians", "视场角必须在 (0, π) 内，单位是弧度");

            DepthIntrinsics k = new DepthIntrinsics();
            k.Width = width;
            k.Height = height;
            k.Fx = (width / 2.0) / Math.Tan(hfovRadians / 2.0);
            k.Fy = (height / 2.0) / Math.Tan(vfovRadians / 2.0);
            k.Cx = (width - 1) / 2.0;
            k.Cy = (height - 1) / 2.0;
            k.DepthScale = depthScale > 0 ? depthScale : 0.001;
            k.Source = source;
            k.Detail = detail;
            return k;
        }

        /// <summary>标称视场角兜底。</summary>
        public static DepthIntrinsics Nominal(int width, int height, double depthScale, string source)
        {
            return FromFov(width, height, DegreesToRadians(AstraNominalHFovDeg), DegreesToRadians(AstraNominalVFovDeg),
                depthScale, source ?? SourceFallback,
                "标称 HFOV " + AstraNominalHFovDeg.ToString("0.#", CultureInfo.InvariantCulture) +
                "° / VFOV " + AstraNominalVFovDeg.ToString("0.#", CultureInfo.InvariantCulture) + "°（未标定）");
        }

        /// <summary>
        /// 按优先级挑一档内参。calib.json 读坏了不会中断采集，只记一条日志然后降级。
        /// </summary>
        public static DepthIntrinsics Resolve(int width, int height, double depthScale,
                                              double? hfovRadians, double? vfovRadians,
                                              string calibrationPath, Action<string> log)
        {
            if (!string.IsNullOrEmpty(calibrationPath) && File.Exists(calibrationPath))
            {
                string error;
                DepthIntrinsics calibrated = TryLoadCalibration(calibrationPath, width, height, depthScale, out error);
                if (calibrated != null) return calibrated;
                if (log != null) log("calib.json 存在但不可用（" + error + "），继续按设备 FOV / 标称值推算");
            }

            if (hfovRadians.HasValue && vfovRadians.HasValue)
            {
                return FromFov(width, height, hfovRadians.Value, vfovRadians.Value, depthScale,
                    SourceDeviceFov,
                    "OpenNI2 流属性：hfov=" + (hfovRadians.Value * 180.0 / Math.PI).ToString("0.0", CultureInfo.InvariantCulture) +
                    "°, vfov=" + (vfovRadians.Value * 180.0 / Math.PI).ToString("0.0", CultureInfo.InvariantCulture) + "°");
            }

            return Nominal(width, height, depthScale, SourceFallback);
        }

        /// <summary>
        /// 读 calib.json。字段：fx fy cx cy（必需 fx/fy），width height depthScale（可选）。
        /// 也接受把参数放在 "depth" 子对象里的写法。calib 的分辨率与当前流不同时按比例缩放并注明。
        /// </summary>
        public static DepthIntrinsics TryLoadCalibration(string path, int streamWidth, int streamHeight,
                                                        double depthScale, out string error)
        {
            error = null;
            try
            {
                if (!File.Exists(path))
                {
                    error = "文件不存在：" + path;
                    return null;
                }

                object parsed = MiniJson.Parse(File.ReadAllText(path));
                JsonObject root = parsed as JsonObject;
                if (root == null)
                {
                    error = "根节点不是 JSON 对象";
                    return null;
                }

                JsonObject source = root.Get("depth", null) as JsonObject;
                if (source == null) source = root;

                double fx = Number(source, "fx", double.NaN);
                double fy = Number(source, "fy", double.NaN);
                double cx = Number(source, "cx", double.NaN);
                double cy = Number(source, "cy", double.NaN);
                if (double.IsNaN(fx) || double.IsNaN(fy) || fx <= 0 || fy <= 0)
                {
                    error = "缺少有效的 fx / fy";
                    return null;
                }

                int calibWidth = (int)Number(source, "width", streamWidth);
                int calibHeight = (int)Number(source, "height", streamHeight);

                DepthIntrinsics k = new DepthIntrinsics();
                k.Width = streamWidth > 0 ? streamWidth : calibWidth;
                k.Height = streamHeight > 0 ? streamHeight : calibHeight;
                if (k.Width <= 0 || k.Height <= 0)
                {
                    error = "calib.json 没有 width/height，且当前没有深度流尺寸可参照";
                    return null;
                }

                double scaleX = 1.0;
                double scaleY = 1.0;
                if (calibWidth > 0 && calibHeight > 0 && (calibWidth != k.Width || calibHeight != k.Height))
                {
                    scaleX = (double)k.Width / calibWidth;
                    scaleY = (double)k.Height / calibHeight;
                }

                k.Fx = fx * scaleX;
                k.Fy = fy * scaleY;
                k.Cx = (double.IsNaN(cx) ? (calibWidth - 1) / 2.0 : cx) * scaleX;
                k.Cy = (double.IsNaN(cy) ? (calibHeight - 1) / 2.0 : cy) * scaleY;

                double calibScale = Number(source, "depthScale", 0);
                k.DepthScale = calibScale > 0 ? calibScale : (depthScale > 0 ? depthScale : 0.001);
                k.Source = SourceCalibration;
                k.Detail = "calib.json：" + Path.GetFileName(path) +
                           "（标定分辨率 " + calibWidth + "x" + calibHeight +
                           (scaleX == 1.0 && scaleY == 1.0 ? "" : "，已按比例缩放到 " + k.Width + "x" + k.Height) + "）";
                if (k.Cx < 0 || k.Cx > k.Width || k.Cy < 0 || k.Cy > k.Height)
                {
                    error = "cx/cy 落在图像外（" + k.Cx.ToString("0.##", CultureInfo.InvariantCulture) + ", " +
                            k.Cy.ToString("0.##", CultureInfo.InvariantCulture) + "）";
                    return null;
                }
                return k;
            }
            catch (Exception ex)
            {
                error = "解析失败：" + ex.Message;
                return null;
            }
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

        public JsonObject ToJson()
        {
            JsonObject o = new JsonObject();
            o.Add("fx", Math.Round(Fx, 4));
            o.Add("fy", Math.Round(Fy, 4));
            o.Add("cx", Math.Round(Cx, 4));
            o.Add("cy", Math.Round(Cy, 4));
            o.Add("width", Width);
            o.Add("height", Height);
            o.Add("depthScale", DepthScale);
            o.Add("source", Source);
            if (!string.IsNullOrEmpty(Detail)) o.Add("note", Detail);
            return o;
        }

        public DepthIntrinsics Clone()
        {
            DepthIntrinsics k = new DepthIntrinsics();
            k.Fx = Fx;
            k.Fy = Fy;
            k.Cx = Cx;
            k.Cy = Cy;
            k.Width = Width;
            k.Height = Height;
            k.DepthScale = DepthScale;
            k.Source = Source;
            k.Detail = Detail;
            return k;
        }

        public string Describe()
        {
            return "fx=" + Fx.ToString("0.00", CultureInfo.InvariantCulture) +
                   " fy=" + Fy.ToString("0.00", CultureInfo.InvariantCulture) +
                   " cx=" + Cx.ToString("0.##", CultureInfo.InvariantCulture) +
                   " cy=" + Cy.ToString("0.##", CultureInfo.InvariantCulture) +
                   " " + Width + "x" + Height +
                   " depthScale=" + DepthScale.ToString("0.######", CultureInfo.InvariantCulture) +
                   " source=" + Source;
        }
    }

    /// <summary>
    /// 相机源可选实现：能给出深度内参的后端才实现它。
    /// 独立成接口而不是塞进 ICameraSource，是为了不动 RealSense / 托管 OpenNI2 等已有实现。
    /// </summary>
    internal interface IDepthIntrinsicsProvider
    {
        /// <summary>Open() 之后调用才有值；没有深度流的后端返回 null。</summary>
        DepthIntrinsics GetDepthIntrinsics();
    }
}
