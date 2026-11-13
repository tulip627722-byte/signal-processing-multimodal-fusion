using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace Acquire
{
    internal abstract class PreviewControlBase : Control
    {
        protected string Caption;
        protected string Detail = "等待数据";
        protected readonly Color PanelBack = Color.FromArgb(13, 22, 35);

        protected PreviewControlBase(string caption)
        {
            Caption = caption;
            DoubleBuffered = true;
            BackColor = PanelBack;
            ForeColor = Color.White;
            Font = new Font("Microsoft YaHei UI", 9F);
            SetStyle(ControlStyles.ResizeRedraw, true);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
            e.Graphics.Clear(PanelBack);
            DrawContent(e.Graphics, new Rectangle(8, 34, Math.Max(1, Width - 16), Math.Max(1, Height - 62)));
            using (Font title = new Font(Font.FontFamily, 11F, FontStyle.Bold))
                e.Graphics.DrawString(Caption, title, Brushes.White, 10, 8);
            using (Brush muted = new SolidBrush(Color.FromArgb(160, 184, 205)))
                e.Graphics.DrawString(Detail, Font, muted, 10, Math.Max(8, Height - 23));
            using (Pen border = new Pen(Color.FromArgb(43, 65, 88)))
                e.Graphics.DrawRectangle(border, 0, 0, Math.Max(0, Width - 1), Math.Max(0, Height - 1));
        }

        protected abstract void DrawContent(Graphics graphics, Rectangle bounds);
    }

    internal sealed class ColorPreviewControl : PreviewControlBase
    {
        private Bitmap _image;
        public ColorPreviewControl() : base("RGB 彩色画面") { }

        public void SetFrame(CameraPreviewFrame frame, double fps)
        {
            if (frame == null || frame.ColorBgr == null) return;
            Bitmap next = ToBitmap(frame.ColorBgr, frame.ColorWidth, frame.ColorHeight);
            Bitmap old = _image;
            _image = next;
            if (old != null) old.Dispose();
            Detail = frame.ColorWidth + "×" + frame.ColorHeight + "  |  " + fps.ToString("0.0") +
                     " FPS  |  t=" + frame.ColorRelMs.ToString("0.0") + " ms";
            Invalidate();
        }

        protected override void DrawContent(Graphics graphics, Rectangle bounds)
        {
            if (_image == null) { DrawWaiting(graphics, bounds, "等待 RGB 帧"); return; }
            graphics.InterpolationMode = InterpolationMode.HighQualityBilinear;
            graphics.DrawImage(_image, Fit(bounds, _image.Width, _image.Height));
        }

        private static Bitmap ToBitmap(byte[] bgr, int width, int height)
        {
            Bitmap bitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
            BitmapData data = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            try
            {
                int row = width * 3;
                for (int y = 0; y < height; y++) Marshal.Copy(bgr, y * row, IntPtr.Add(data.Scan0, y * data.Stride), row);
            }
            finally { bitmap.UnlockBits(data); }
            return bitmap;
        }

        private static Rectangle Fit(Rectangle outer, int width, int height)
        {
            double scale = Math.Min(outer.Width / (double)width, outer.Height / (double)height);
            int w = Math.Max(1, (int)(width * scale));
            int h = Math.Max(1, (int)(height * scale));
            return new Rectangle(outer.X + (outer.Width - w) / 2, outer.Y + (outer.Height - h) / 2, w, h);
        }

        private static void DrawWaiting(Graphics graphics, Rectangle bounds, string text)
        {
            using (Brush brush = new SolidBrush(Color.FromArgb(90, 120, 145)))
                graphics.DrawString(text, SystemFonts.MessageBoxFont, brush, bounds.X + 12, bounds.Y + 12);
        }

        protected override void Dispose(bool disposing) { if (disposing && _image != null) _image.Dispose(); base.Dispose(disposing); }
    }

    internal sealed class DepthPreviewControl : PreviewControlBase
    {
        private Bitmap _image;
        public double MinMeters = 0.5;
        public double MaxMeters = 7.0;
        public DepthPreviewControl() : base("Depth 深度画面") { }

        public void SetFrame(CameraPreviewFrame frame, double fps)
        {
            if (frame == null || frame.Depth == null) return;
            Bitmap next = Colorize(frame.Depth, frame.DepthWidth, frame.DepthHeight, frame.DepthScaleMeters, MinMeters, MaxMeters);
            Bitmap old = _image;
            _image = next;
            if (old != null) old.Dispose();
            Detail = MinMeters.ToString("0.0") + "–" + MaxMeters.ToString("0.0") + " m  |  有效点 " +
                     (frame.ValidDepthRatio * 100).ToString("0.0") + "%  |  " + fps.ToString("0.0") + " FPS";
            Invalidate();
        }

        protected override void DrawContent(Graphics graphics, Rectangle bounds)
        {
            if (_image == null) return;
            graphics.InterpolationMode = InterpolationMode.NearestNeighbor;
            double scale = Math.Min(bounds.Width / (double)_image.Width, bounds.Height / (double)_image.Height);
            int width = (int)(_image.Width * scale), height = (int)(_image.Height * scale);
            graphics.DrawImage(_image, bounds.X + (bounds.Width - width) / 2, bounds.Y + (bounds.Height - height) / 2, width, height);
        }

        private static Bitmap Colorize(ushort[] depth, int width, int height, double scale, double min, double max)
        {
            Bitmap bitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);
            BitmapData data = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            byte[] row = new byte[width * 3];
            try
            {
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        double meters = depth[y * width + x] * scale;
                        double t = meters <= 0 ? -1 : (meters - min) / Math.Max(0.001, max - min);
                        Color color = t < 0 ? Color.Black : Turbo(Math.Max(0, Math.Min(1, t)));
                        int offset = x * 3;
                        row[offset] = color.B; row[offset + 1] = color.G; row[offset + 2] = color.R;
                    }
                    Marshal.Copy(row, 0, IntPtr.Add(data.Scan0, y * data.Stride), row.Length);
                }
            }
            finally { bitmap.UnlockBits(data); }
            return bitmap;
        }

        private static Color Turbo(double value)
        {
            double r = 0.13572138 + value * (4.61539260 + value * (-42.66032258 + value * (132.13108234 + value * (-152.94239396 + value * 59.28637943))));
            double g = 0.09140261 + value * (2.19418839 + value * (4.84296658 + value * (-14.18503333 + value * (4.27729857 + value * 2.82956604))));
            double b = 0.10667330 + value * (12.64194608 + value * (-60.58204836 + value * (110.36276771 + value * (-89.90310912 + value * 27.34824973))));
            return Color.FromArgb((int)(255 * Math.Max(0, Math.Min(1, r))), (int)(255 * Math.Max(0, Math.Min(1, g))), (int)(255 * Math.Max(0, Math.Min(1, b))));
        }

        protected override void Dispose(bool disposing) { if (disposing && _image != null) _image.Dispose(); base.Dispose(disposing); }
    }

    internal sealed class RadarPreviewControl : PreviewControlBase
    {
        private RadarPreviewFrame _frame;
        public RadarPreviewControl() : base("ADT7880 毫米波距离谱") { }

        public void SetFrame(RadarPreviewFrame frame, double fps)
        {
            if (frame == null) return;
            _frame = frame;
            Detail = "峰值 " + frame.PeakRangeMeters.ToString("0.00") + " m  |  " + frame.PeakMagnitudeDb.ToString("0.0") +
                     " dB  |  第 " + frame.FrameNumber + " 帧  |  " + fps.ToString("0.0") + " FPS";
            Invalidate();
        }

        protected override void DrawContent(Graphics graphics, Rectangle bounds)
        {
            using (Pen grid = new Pen(Color.FromArgb(35, 61, 82)))
                for (int i = 0; i <= 5; i++) graphics.DrawLine(grid, bounds.X, bounds.Y + bounds.Height * i / 5, bounds.Right, bounds.Y + bounds.Height * i / 5);
            if (_frame == null || _frame.MagnitudeDb == null) return;
            double min = double.MaxValue, max = double.MinValue;
            for (int i = 1; i < _frame.MagnitudeDb.Length; i++) { min = Math.Min(min, _frame.MagnitudeDb[i]); max = Math.Max(max, _frame.MagnitudeDb[i]); }
            if (max - min < 1) max = min + 1;
            PointF[] points = new PointF[_frame.MagnitudeDb.Length - 1];
            for (int i = 1; i < _frame.MagnitudeDb.Length; i++)
                points[i - 1] = new PointF(bounds.X + (i - 1) * bounds.Width / (float)(points.Length - 1), bounds.Bottom - (float)((_frame.MagnitudeDb[i] - min) / (max - min) * bounds.Height));
            using (Pen line = new Pen(Color.FromArgb(48, 213, 200), 2F)) graphics.DrawLines(line, points);
            int peakX = bounds.X + (int)(_frame.PeakRangeMeters / _frame.RangeMeters[_frame.RangeMeters.Length - 1] * bounds.Width);
            using (Pen peak = new Pen(Color.FromArgb(255, 184, 76), 1.5F)) graphics.DrawLine(peak, peakX, bounds.Y, peakX, bounds.Bottom);
        }
    }

    internal sealed class PointCloudPreviewControl : PreviewControlBase
    {
        private LidarBatch _batch;
        private float _yaw = -0.55F;
        private float _pitch = 0.42F;
        private float _zoom = 48F;
        private Point _dragStart;
        private bool _dragging;

        public PointCloudPreviewControl() : base("Livox MID-70 激光点云")
        {
            MouseDown += delegate(object sender, MouseEventArgs e) { if (e.Button == MouseButtons.Left) { _dragging = true; _dragStart = e.Location; } };
            MouseUp += delegate { _dragging = false; };
            MouseMove += OnMouseMove;
            MouseWheel += delegate(object sender, MouseEventArgs e) { _zoom = Math.Max(10, Math.Min(180, _zoom * (e.Delta > 0 ? 1.12F : 0.89F))); Invalidate(); };
        }

        public void SetBatch(LidarBatch batch, double fps, string state)
        {
            if (batch == null) return;
            _batch = batch;
            Detail = batch.Count.ToString("N0") + " 点/批  |  " + fps.ToString("0.0") + " 批/s  |  " + state + "  |  拖动旋转 · 滚轮缩放";
            Invalidate();
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (!_dragging) return;
            _yaw += (e.X - _dragStart.X) * 0.01F;
            _pitch += (e.Y - _dragStart.Y) * 0.01F;
            _dragStart = e.Location;
            Invalidate();
        }

        protected override void DrawContent(Graphics graphics, Rectangle bounds)
        {
            DrawAxes(graphics, bounds);
            if (_batch == null || _batch.Points == null) return;
            double cy = Math.Cos(_yaw), sy = Math.Sin(_yaw), cp = Math.Cos(_pitch), sp = Math.Sin(_pitch);
            int step = Math.Max(1, _batch.Count / 8000);
            for (int i = 0; i < _batch.Count; i += step)
            {
                LidarPoint point = _batch.Points[i];
                double x1 = point.X * cy + point.Z * sy;
                double z1 = -point.X * sy + point.Z * cy;
                double y1 = point.Y * cp - z1 * sp;
                double z2 = point.Y * sp + z1 * cp;
                float sx = bounds.X + bounds.Width / 2F + (float)(x1 * _zoom);
                float sy2 = bounds.Y + bounds.Height / 2F - (float)(y1 * _zoom);
                if (!bounds.Contains((int)sx, (int)sy2)) continue;
                int intensity = point.Reflectivity;
                using (Brush brush = new SolidBrush(Color.FromArgb(210, Math.Min(255, 40 + intensity), Math.Min(255, 210 - intensity / 3), Math.Min(255, 255 - intensity / 2))))
                    graphics.FillRectangle(brush, sx, sy2, z2 < 2 ? 2.2F : 1.5F, z2 < 2 ? 2.2F : 1.5F);
            }
        }

        private static void DrawAxes(Graphics graphics, Rectangle bounds)
        {
            Point origin = new Point(bounds.X + 38, bounds.Bottom - 28);
            using (Pen x = new Pen(Color.FromArgb(230, 94, 94), 2)) graphics.DrawLine(x, origin, new Point(origin.X + 28, origin.Y));
            using (Pen y = new Pen(Color.FromArgb(97, 210, 128), 2)) graphics.DrawLine(y, origin, new Point(origin.X, origin.Y - 24));
            using (Pen z = new Pen(Color.FromArgb(91, 165, 255), 2)) graphics.DrawLine(z, origin, new Point(origin.X + 17, origin.Y - 16));
        }
    }
}
