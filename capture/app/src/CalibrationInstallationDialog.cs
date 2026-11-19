using System;
using System.Drawing;
using System.Windows.Forms;

namespace Acquire
{
    internal sealed class CalibrationInstallationDialog : Form
    {
        private readonly NumericUpDown[] _camera = new NumericUpDown[3];
        private readonly NumericUpDown[] _lidar = new NumericUpDown[3];
        private readonly CalibrationInstallationConfig _config;

        private CalibrationInstallationDialog(CalibrationInstallationConfig config)
        {
            _config = config;
            Text = "传感器安装位置（毫米波坐标）";
            StartPosition = FormStartPosition.CenterParent;
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            ClientSize = new Size(600, 315);
            Font = new Font("Microsoft YaHei UI", 9F);

            Label title = new Label { Text = "更换安装位置时修改这里；MATLAB 标定会自动读取保存值。", AutoSize = true, Location = new Point(18, 15) };
            Controls.Add(title);
            Label frame = new Label { Text = "坐标：毫米波 ADT7880，X=右，Y=前，Z=上；单位：米", AutoSize = true, ForeColor = Color.FromArgb(80, 95, 110), Location = new Point(18, 42) };
            Controls.Add(frame);
            Controls.Add(new Label { Text = "相机光心相对毫米波 [右, 前, 上]", AutoSize = true, Location = new Point(18, 82) });
            Controls.Add(new Label { Text = "LiDAR 光心相对毫米波 [右, 前, 上]", AutoSize = true, Location = new Point(18, 137) });
            string[] names = { "右 X", "前 Y", "上 Z" };
            for (int i = 0; i < 3; i++)
            {
                int x = 210 + i * 122;
                Controls.Add(new Label { Text = names[i], AutoSize = true, Location = new Point(x, 62) });
                _camera[i] = MakeNumber(x, 78, _config.CameraOriginInRadarM[i]);
                Controls.Add(_camera[i]);
                Controls.Add(new Label { Text = names[i], AutoSize = true, Location = new Point(x, 117) });
                _lidar[i] = MakeNumber(x, 133, _config.LidarOriginInRadarM[i]);
                Controls.Add(_lidar[i]);
            }
            Label hint = new Label { Text = "例如 LiDAR 在雷达正下方 15 cm：右=0，前=0，上=-0.15。\r\n只填写安装平移；方向保持平行时不要修改旋转。", AutoSize = true, ForeColor = Color.FromArgb(80, 95, 110), Location = new Point(18, 185) };
            Controls.Add(hint);
            Button ok = new Button { Text = "保存并使用", DialogResult = DialogResult.OK, Bounds = new Rectangle(370, 258, 100, 32) };
            Button cancel = new Button { Text = "取消", DialogResult = DialogResult.Cancel, Bounds = new Rectangle(480, 258, 80, 32) };
            Controls.Add(ok); Controls.Add(cancel);
            AcceptButton = ok; CancelButton = cancel;
        }

        public static bool ShowDialog(IWin32Window owner, CalibrationInstallationConfig config)
        {
            using (CalibrationInstallationDialog dialog = new CalibrationInstallationDialog(config))
            {
                if (dialog.ShowDialog(owner) != DialogResult.OK) return false;
                for (int i = 0; i < 3; i++)
                {
                    config.CameraOriginInRadarM[i] = (double)dialog._camera[i].Value;
                    config.LidarOriginInRadarM[i] = (double)dialog._lidar[i].Value;
                }
                config.Save();
                return true;
            }
        }

        private static NumericUpDown MakeNumber(int x, int y, double value)
        {
            return new NumericUpDown
            {
                Bounds = new Rectangle(x, y, 100, 25),
                Minimum = -100,
                Maximum = 100,
                DecimalPlaces = 4,
                Increment = 0.01M,
                Value = (decimal)Math.Max(-100, Math.Min(100, value))
            };
        }
    }
}
