using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Forms;

namespace Acquire
{
    internal sealed class MainForm : Form
    {
        private ComboBox _cameraBox, _lidarMode;
        private Button _refreshButton, _browseButton, _startButton, _stopButton, _alignButton;
        private Label _cameraInfo, _stateLabel, _detailLabel;
        private CheckBox _radarEnabled, _lidarEnabled, _saveCloud, _soundEnabled;
        private TextBox _localIp, _boardIp, _lidarCode, _labelBox, _outputBox, _logBox;
        private NumericUpDown _localPort, _delayBox, _durationBox, _depthMin, _depthMax;
        private ProgressBar _progress;
        private ToolStripStatusLabel _radarStatus, _cameraStatus, _lidarStatus, _remainStatus;
        private ColorPreviewControl _colorPreview;
        private DepthPreviewControl _depthPreview;
        private RadarPreviewControl _radarPreview;
        private PointCloudPreviewControl _lidarPreview;
        private System.Windows.Forms.Timer _timer;
        private SessionController _controller;
        private List<CameraOption> _cameras = new List<CameraOption>();
        private readonly bool _demoMode;
        private bool _closing, _wasBusy, _alignRunning;
        private string _lastShownError = "";

        public MainForm() : this(false) { }

        public MainForm(bool demoMode)
        {
            _demoMode = demoMode;
            BuildUi();
            _controller = new SessionController();
            _controller.Log += AppendLog;
            ProbeCameras();
            _timer = new System.Windows.Forms.Timer { Interval = 100 };
            _timer.Tick += delegate { RefreshStatus(); };
            _timer.Start();
            FormClosing += OnFormClosing;
            if (_demoMode) Shown += delegate { BeginInvoke(new Action(StartDemo)); };
            AppendLog("就绪。四路预览与原始数据采集相互独立，预览慢不会阻塞落盘。");
        }

        private void BuildUi()
        {
            Text = "多模态同步采集与可视化平台 - ADT7880 / RGB-D / Livox MID-70";
            AutoScaleMode = AutoScaleMode.Dpi;
            AutoScaleDimensions = new SizeF(96F, 96F);
            StartPosition = FormStartPosition.CenterScreen;
            ClientSize = new Size(1420, 900);
            MinimumSize = new Size(1180, 760);
            Font = new Font("Microsoft YaHei UI", 9F);
            BackColor = Color.FromArgb(239, 244, 249);

            Panel header = new Panel { Dock = DockStyle.Top, Height = 56, BackColor = Color.FromArgb(13, 55, 100) };
            Label title = new Label { Text = "多模态同步采集与可视化平台", ForeColor = Color.White, Font = new Font("Microsoft YaHei UI", 18F, FontStyle.Bold), AutoSize = true, Location = new Point(18, 8) };
            Label subtitle = new Label { Text = "ADT7880 毫米波雷达  ·  RGB / Depth  ·  Livox MID-70", ForeColor = Color.FromArgb(181, 211, 239), AutoSize = true, Location = new Point(420, 22) };
            Panel badgePanel = new Panel { Dock = DockStyle.Right, Width = 150 };
            Label badge = new Label { Text = _demoMode ? "合成数据演示" : "统一时间基准", ForeColor = Color.White, BackColor = _demoMode ? Color.FromArgb(204, 111, 26) : Color.FromArgb(26, 148, 136), AutoSize = false, TextAlign = ContentAlignment.MiddleCenter, Size = new Size(120, 26), Location = new Point(15, 15) };
            badgePanel.Controls.Add(badge);
            header.Controls.Add(title); header.Controls.Add(subtitle); header.Controls.Add(badgePanel); Controls.Add(header);

            Panel config = new Panel { Dock = DockStyle.Top, Height = 150, Padding = new Padding(12, 8, 12, 4), BackColor = Color.White };
            Controls.Add(config);
            config.Controls.Add(MakeLabel("相机", 12, 11, 38));
            _cameraBox = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Bounds = new Rectangle(52, 7, 340, 25) };
            _cameraBox.SelectedIndexChanged += delegate { ShowCameraDetail(); }; config.Controls.Add(_cameraBox);
            _refreshButton = new Button { Text = "重新探测", Bounds = new Rectangle(398, 6, 82, 27) };
            _refreshButton.Click += delegate { ProbeCameras(); }; config.Controls.Add(_refreshButton);
            _cameraInfo = new Label { Bounds = new Rectangle(490, 11, 500, 20), ForeColor = Color.FromArgb(83, 99, 113) }; config.Controls.Add(_cameraInfo);

            _radarEnabled = new CheckBox { Text = "ADT7880", Checked = true, Bounds = new Rectangle(12, 41, 88, 24) }; config.Controls.Add(_radarEnabled);
            config.Controls.Add(MakeLabel("本机", 104, 44, 34));
            _localIp = new TextBox { Text = "192.168.220.10", Bounds = new Rectangle(140, 40, 116, 25) }; config.Controls.Add(_localIp);
            _localPort = Number(1, 65535, 8080, 262, 40, 68); config.Controls.Add(_localPort);
            config.Controls.Add(MakeLabel("雷达", 338, 44, 34));
            _boardIp = new TextBox { Text = "192.168.220.100", Bounds = new Rectangle(374, 40, 116, 25) }; config.Controls.Add(_boardIp);

            _lidarEnabled = new CheckBox { Text = "Livox MID-70", Checked = false, Bounds = new Rectangle(510, 41, 110, 24) }; config.Controls.Add(_lidarEnabled);
            _lidarMode = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Bounds = new Rectangle(622, 40, 180, 25) };
            _lidarMode.Items.AddRange(new object[] { "模拟点云（无硬件）", "MID-70 真机" }); _lidarMode.SelectedIndex = 0;
            _lidarMode.SelectedIndexChanged += delegate { _lidarCode.Enabled = _lidarMode.SelectedIndex == 1; }; config.Controls.Add(_lidarMode);
            _lidarCode = new TextBox { Bounds = new Rectangle(808, 40, 182, 25), Text = "广播码（真机）", Enabled = false }; config.Controls.Add(_lidarCode);

            config.Controls.Add(MakeLabel("延迟", 12, 77, 34)); _delayBox = Number(0, 60, 3, 50, 73, 52); config.Controls.Add(_delayBox);
            config.Controls.Add(MakeLabel("s   时长", 106, 77, 55)); _durationBox = Number(1, 600, 20, 162, 73, 60); config.Controls.Add(_durationBox);
            config.Controls.Add(MakeLabel("s   标签", 226, 77, 55)); _labelBox = new TextBox { Bounds = new Rectangle(282, 73, 126, 25), Text = "室内2米" }; config.Controls.Add(_labelBox);
            config.Controls.Add(MakeLabel("输出", 416, 77, 34)); _outputBox = new TextBox { Bounds = new Rectangle(452, 73, 420, 25), Text = DefaultTakeRoot() }; config.Controls.Add(_outputBox);
            _browseButton = new Button { Text = "浏览…", Bounds = new Rectangle(878, 72, 64, 27) }; _browseButton.Click += BrowseOutput; config.Controls.Add(_browseButton);
            _saveCloud = new CheckBox { Text = "保存相机派生点云", Bounds = new Rectangle(12, 108, 145, 24) }; config.Controls.Add(_saveCloud);
            _soundEnabled = new CheckBox { Text = "采集提示音", Checked = true, Bounds = new Rectangle(162, 108, 105, 24) }; config.Controls.Add(_soundEnabled);
            config.Controls.Add(MakeLabel("深度显示", 282, 111, 60));
            _depthMin = DecimalNumber(0.1M, 20, 0.5M, 344, 107, 58); _depthMax = DecimalNumber(0.2M, 50, 7.0M, 434, 107, 58);
            config.Controls.Add(_depthMin); config.Controls.Add(MakeLabel("—", 408, 111, 20)); config.Controls.Add(_depthMax); config.Controls.Add(MakeLabel("m", 496, 111, 20));

            _startButton = new Button { Text = "开始同步采集", Bounds = new Rectangle(1010, 10, 180, 38), BackColor = Color.FromArgb(22, 125, 116), ForeColor = Color.White, FlatStyle = FlatStyle.Flat, Font = new Font(Font, FontStyle.Bold) };
            _startButton.Click += delegate { StartCapture(false); }; config.Controls.Add(_startButton);
            _stopButton = new Button { Text = "停止并落盘", Bounds = new Rectangle(1198, 10, 180, 38), Enabled = false };
            _stopButton.Click += delegate { _controller.Stop(); }; config.Controls.Add(_stopButton);
            _alignButton = new Button { Text = "对齐未处理采集", Bounds = new Rectangle(1198, 57, 180, 30) };
            _alignButton.Click += delegate { StartAlign(); }; config.Controls.Add(_alignButton);
            _stateLabel = new Label { Text = "空闲", Font = new Font(Font.FontFamily, 12F, FontStyle.Bold), Bounds = new Rectangle(1010, 98, 95, 28) }; config.Controls.Add(_stateLabel);
            _detailLabel = new Label { Text = "等待采集", Bounds = new Rectangle(1110, 102, 268, 22), ForeColor = Color.FromArgb(83, 99, 113) }; config.Controls.Add(_detailLabel);
            _progress = new ProgressBar { Bounds = new Rectangle(1010, 128, 368, 10), Maximum = 100 }; config.Controls.Add(_progress);

            TableLayoutPanel previews = new TableLayoutPanel { Dock = DockStyle.None, Bounds = new Rectangle(0, 206, ClientSize.Width, ClientSize.Height - 312), Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right, Padding = new Padding(12, 8, 12, 5), BackColor = Color.FromArgb(226, 234, 242), ColumnCount = 2, RowCount = 2 };
            previews.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50)); previews.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            previews.RowStyles.Add(new RowStyle(SizeType.Percent, 50)); previews.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            _colorPreview = new ColorPreviewControl { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 4, 4) };
            _depthPreview = new DepthPreviewControl { Dock = DockStyle.Fill, Margin = new Padding(4, 0, 0, 4) };
            _radarPreview = new RadarPreviewControl { Dock = DockStyle.Fill, Margin = new Padding(0, 4, 4, 0) };
            _lidarPreview = new PointCloudPreviewControl { Dock = DockStyle.Fill, Margin = new Padding(4, 4, 0, 0) };
            previews.Controls.Add(_colorPreview, 0, 0); previews.Controls.Add(_depthPreview, 1, 0); previews.Controls.Add(_radarPreview, 0, 1); previews.Controls.Add(_lidarPreview, 1, 1); Controls.Add(previews);

            Panel logPanel = new Panel { Dock = DockStyle.Bottom, Height = 84, Padding = new Padding(12, 3, 12, 3), BackColor = Color.White };
            _logBox = new TextBox { Multiline = true, ScrollBars = ScrollBars.Vertical, ReadOnly = true, Dock = DockStyle.Fill, Font = new Font("Consolas", 8.5F), BackColor = Color.FromArgb(248, 250, 252) };
            logPanel.Controls.Add(_logBox); Controls.Add(logPanel);
            StatusStrip status = new StatusStrip();
            _radarStatus = new ToolStripStatusLabel("毫米波 -") { BorderSides = ToolStripStatusLabelBorderSides.Right };
            _cameraStatus = new ToolStripStatusLabel("RGB-D -") { BorderSides = ToolStripStatusLabelBorderSides.Right };
            _lidarStatus = new ToolStripStatusLabel("MID-70 -") { BorderSides = ToolStripStatusLabelBorderSides.Right };
            _remainStatus = new ToolStripStatusLabel("剩余 -");
            status.Items.Add(_radarStatus); status.Items.Add(_cameraStatus); status.Items.Add(_lidarStatus); status.Items.Add(new ToolStripStatusLabel { Spring = true }); status.Items.Add(_remainStatus); Controls.Add(status);
        }

        private static Label MakeLabel(string text, int x, int y, int width) { return new Label { Text = text, Bounds = new Rectangle(x, y, width, 20) }; }
        private static NumericUpDown Number(decimal min, decimal max, decimal value, int x, int y, int width) { return new NumericUpDown { Minimum = min, Maximum = max, Value = value, Bounds = new Rectangle(x, y, width, 25) }; }
        private static NumericUpDown DecimalNumber(decimal min, decimal max, decimal value, int x, int y, int width) { return new NumericUpDown { Minimum = min, Maximum = max, Value = value, Increment = 0.1M, DecimalPlaces = 1, Bounds = new Rectangle(x, y, width, 25) }; }

        private void BrowseOutput(object sender, EventArgs e)
        {
            using (FolderBrowserDialog dialog = new FolderBrowserDialog())
            {
                dialog.Description = "选择 take 输出目录"; if (Directory.Exists(_outputBox.Text)) dialog.SelectedPath = _outputBox.Text;
                if (dialog.ShowDialog(this) == DialogResult.OK) _outputBox.Text = dialog.SelectedPath;
            }
        }

        private static string DefaultTakeRoot()
        {
            string exeDir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
            try { DirectoryInfo bin = new DirectoryInfo(exeDir); if (bin.Parent != null && bin.Name.Equals("bin", StringComparison.OrdinalIgnoreCase) && bin.Parent.Parent != null) return Path.Combine(bin.Parent.Parent.FullName, "takes"); } catch { }
            return Path.Combine(exeDir, "takes");
        }

        private void ProbeCameras()
        {
            _refreshButton.Enabled = false;
            try
            {
                _cameras = CameraProbe.Probe(AppendLog); _cameraBox.Items.Clear(); int preferred = -1, synthetic = -1, first = -1;
                for (int i = 0; i < _cameras.Count; i++)
                {
                    CameraOption option = _cameras[i]; _cameraBox.Items.Add(option);
                    if (option.Available && first < 0) first = i;
                    if (option.Available && option.Kind == CameraKind.Null) synthetic = i;
                    if (option.Available && (option.Kind == CameraKind.RealSense || option.Kind == CameraKind.OpenNi2) && preferred < 0) preferred = i;
                }
                int selected = _demoMode && synthetic >= 0 ? synthetic : (preferred >= 0 ? preferred : first); if (selected >= 0) _cameraBox.SelectedIndex = selected;
                _cameraInfo.Text = CameraProbe.Summarize(_cameras);
            }
            catch (Exception ex) { AppendLog("相机探测异常：" + ex.Message); }
            finally { _refreshButton.Enabled = true; }
        }

        private void ShowCameraDetail()
        {
            CameraOption option = _cameraBox.SelectedItem as CameraOption; if (option == null) return;
            _cameraInfo.Text = option.Display + "  |  " + option.Reason + (option.Kind == CameraKind.Null ? "  【合成画面】" : "");
            _cameraInfo.ForeColor = option.Kind == CameraKind.Null ? Color.FromArgb(187, 91, 15) : Color.FromArgb(83, 99, 113);
        }

        private void StartDemo() { _delayBox.Value = 0; _durationBox.Value = 600; _soundEnabled.Checked = false; _labelBox.Text = "multiview-demo"; _lidarEnabled.Checked = true; _lidarMode.SelectedIndex = 0; StartCapture(true); }

        private void StartCapture(bool demo)
        {
            if (_controller.IsBusy) return;
            CameraOption option = _cameraBox.SelectedItem as CameraOption;
            if (option == null || !option.Available) { if (!demo) MessageBox.Show(this, "请选择可用相机来源。", "无法开始"); return; }
            if (option.Kind == CameraKind.Null && !demo && MessageBox.Show(this, "当前为合成测试画面，不是真实相机。仍要继续？", "合成数据", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes) return;
            if (_lidarEnabled.Checked && _lidarMode.SelectedIndex == 0 && !demo && MessageBox.Show(this, "当前 LiDAR 来源为模拟点云，不是真实 MID-70。仍要写入本次采集？", "合成数据", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) != DialogResult.Yes) return;
            int port = demo ? FindFreeUdpPort() : (int)_localPort.Value;
            SessionOptions session = new SessionOptions
            {
                OutputRoot = _outputBox.Text.Trim(), Label = _labelBox.Text.Trim(), DelaySeconds = (int)_delayBox.Value, DurationSeconds = (int)_durationBox.Value,
                RadarEnabled = _radarEnabled.Checked, RadarSynthetic = demo, LocalIp = demo ? "127.0.0.1" : _localIp.Text.Trim(), LocalPort = port, BoardIp = demo ? "" : _boardIp.Text.Trim(),
                Camera = option, SaveCloud = _saveCloud.Checked, SoundCues = _soundEnabled.Checked, SelfTest = option.Kind == CameraKind.Null,
                LidarEnabled = _lidarEnabled.Checked, LidarSynthetic = _lidarMode.SelectedIndex == 0, LidarBroadcastCode = _lidarCode.Text.Trim()
            };
            try { _controller.Start(session); } catch (Exception ex) { if (!demo) MessageBox.Show(this, ex.Message, "启动失败", MessageBoxButtons.OK, MessageBoxIcon.Error); AppendLog("启动失败：" + ex.Message); }
        }

        private static int FindFreeUdpPort() { using (UdpClient probe = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0))) return ((IPEndPoint)probe.Client.LocalEndPoint).Port; }

        private void RefreshStatus()
        {
            if (_closing) return;
            SessionSnapshot s = _controller.Snapshot();
            CameraPreviewFrame camera = _controller.TakeCameraPreview();
            if (camera != null) { _colorPreview.SetFrame(camera, s.ColorFps); _depthPreview.MinMeters = (double)_depthMin.Value; _depthPreview.MaxMeters = (double)_depthMax.Value; _depthPreview.SetFrame(camera, s.DepthFps); }
            RadarPreviewFrame radar = _controller.TakeRadarPreview(); if (radar != null) _radarPreview.SetFrame(radar, s.RadarFps);
            LidarBatch lidar = _controller.TakeLidarPreview(); if (lidar != null) _lidarPreview.SetBatch(lidar, s.LidarFps, s.LidarHealth);
            switch (s.State)
            {
                case SessionState.Countdown: _stateLabel.Text = "倒计时 " + s.CountdownRemaining + " s"; _stateLabel.ForeColor = Color.DarkOrange; break;
                case SessionState.Recording: _stateLabel.Text = "同步记录中"; _stateLabel.ForeColor = Color.FromArgb(190, 45, 45); break;
                case SessionState.Saving: _stateLabel.Text = "正在落盘"; _stateLabel.ForeColor = Color.RoyalBlue; break;
                default: _stateLabel.Text = "空闲"; _stateLabel.ForeColor = Color.FromArgb(45, 55, 65); break;
            }
            bool idle = s.State == SessionState.Idle;
            if (_wasBusy && idle && !string.IsNullOrEmpty(s.Error) && s.Error != _lastShownError && !_demoMode) { _lastShownError = s.Error; MessageBox.Show(this, FriendlyError(s.Error), "采集失败", MessageBoxButtons.OK, MessageBoxIcon.Error); }
            _wasBusy = !idle; SetEditingEnabled(idle);
            if (s.State == SessionState.Recording) { _progress.Value = Math.Min(100, (int)(s.ElapsedSeconds * 100 / Math.Max(1, (double)_durationBox.Value))); _detailLabel.Text = s.ElapsedSeconds.ToString("0.0") + " s  |  预览 " + s.PreviewLatencyMs.ToString("0") + " ms  |  覆盖 " + s.PreviewDropped; }
            else { _progress.Value = 0; _detailLabel.Text = string.IsNullOrEmpty(s.Error) ? s.Message : "错误：" + FriendlyErrorTitle(s.Error); }
            _radarStatus.Text = "毫米波 " + s.RadarFps.ToString("0.0") + " FPS  帧 " + s.RadarFrames + "  丢 " + s.RadarMissing + (s.RadarRestarts > 0 ? "  恢复 " + s.RadarRestarts : "");
            _cameraStatus.Text = "RGB " + s.ColorFps.ToString("0.0") + " / Depth " + s.DepthFps.ToString("0.0") + " FPS";
            _lidarStatus.Text = "MID-70 " + s.LidarHealth + "  " + s.LidarFps.ToString("0.0") + " 批/s  " + s.LidarPoints.ToString("N0") + " 点";
            _remainStatus.Text = s.State == SessionState.Recording ? "剩余 " + s.RemainingSeconds.ToString("0.0") + " s" : ShortPath(s.TakeDirectory);
        }

        private void SetEditingEnabled(bool enabled)
        {
            _startButton.Enabled = enabled; _stopButton.Enabled = !enabled; _cameraBox.Enabled = enabled; _refreshButton.Enabled = enabled; _radarEnabled.Enabled = enabled; _lidarEnabled.Enabled = enabled; _lidarMode.Enabled = enabled;
            _lidarCode.Enabled = enabled && _lidarMode.SelectedIndex == 1; _localIp.Enabled = enabled; _localPort.Enabled = enabled; _boardIp.Enabled = enabled; _delayBox.Enabled = enabled; _durationBox.Enabled = enabled;
            _labelBox.Enabled = enabled; _outputBox.Enabled = enabled; _browseButton.Enabled = enabled; _saveCloud.Enabled = enabled; _soundEnabled.Enabled = enabled;
        }

        private void StartAlign()
        {
            if (_alignRunning || !Directory.Exists(_outputBox.Text.Trim())) return; _alignRunning = true; _alignButton.Enabled = false;
            Thread worker = new Thread(delegate() { try { TakeAligner.AlignAll(_outputBox.Text.Trim(), AppendLog, null, null); } catch (Exception ex) { AppendLog("对齐异常：" + ex.Message); } finally { try { BeginInvoke(new Action(delegate { _alignRunning = false; _alignButton.Enabled = true; })); } catch { } } });
            worker.IsBackground = true; worker.Start();
        }

        private static string FriendlyError(string error) { return FriendlyErrorTitle(error) + "\r\n\r\n详细信息请查看日志。"; }
        private static string FriendlyErrorTitle(string error) { if (string.IsNullOrEmpty(error)) return "采集失败"; string text = error.Replace('\r', ' ').Replace('\n', ' ').Trim(); int sentence = text.IndexOf('。'); if (sentence >= 0) text = text.Substring(0, sentence + 1); return text.Length > 90 ? text.Substring(0, 87) + "..." : text; }
        private static string ShortPath(string path) { return string.IsNullOrEmpty(path) ? "" : Path.GetFileName(path.TrimEnd(Path.DirectorySeparatorChar)); }

        internal void AppendLog(string line)
        {
            if (_closing) return; if (InvokeRequired) { try { BeginInvoke(new Action<string>(AppendLog), line); } catch { } return; }
            if (_logBox.TextLength > 150000) _logBox.Clear(); _logBox.AppendText(line + "\r\n");
        }

        internal void SaveScreenshot(string path)
        {
            string directory = Path.GetDirectoryName(Path.GetFullPath(path));
            if (!Directory.Exists(directory)) Directory.CreateDirectory(directory);
            using (Bitmap image = new Bitmap(ClientSize.Width, ClientSize.Height))
            {
                DrawToBitmap(image, new Rectangle(Point.Empty, ClientSize));
                image.Save(path, System.Drawing.Imaging.ImageFormat.Png);
            }
        }

        private void OnFormClosing(object sender, FormClosingEventArgs e)
        {
            if (_controller != null && _controller.IsBusy && !_demoMode && MessageBox.Show(this, "采集仍在进行，关闭会停止并落盘。继续？", "确认关闭", MessageBoxButtons.YesNo) != DialogResult.Yes) { e.Cancel = true; return; }
            _closing = true; if (_timer != null) _timer.Stop(); if (_controller != null) try { _controller.Dispose(); } catch { }
        }
    }
}
