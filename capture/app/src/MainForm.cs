using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Windows.Forms;

namespace Acquire
{
    internal sealed class MainForm : Form
    {
        private ComboBox _cameraBox;
        private Button _refreshButton;
        private Label _cameraInfo;
        private CheckBox _radarEnabled;
        private CheckBox _saveCloud;
        private CheckBox _soundEnabled;
        private TextBox _localIp;
        private NumericUpDown _localPort;
        private TextBox _boardIp;
        private NumericUpDown _delayBox;
        private NumericUpDown _durationBox;
        private TextBox _labelBox;
        private TextBox _outputBox;
        private Button _browseButton;
        private Button _startButton;
        private Button _stopButton;
        private Button _alignButton;
        private bool _alignRunning;
        private Label _stateLabel;
        private Label _detailLabel;
        private Label _sizeHint;
        private ProgressBar _progress;
        private TextBox _logBox;
        private StatusStrip _statusStrip;
        private ToolStripStatusLabel _radarStatus;
        private ToolStripStatusLabel _colorStatus;
        private ToolStripStatusLabel _depthStatus;
        private ToolStripStatusLabel _remainStatus;
        private Timer _timer;
        private SessionController _controller;
        private List<CameraOption> _cameras = new List<CameraOption>();
        private bool _closing;

        public MainForm()
        {
            BuildUi();
            _controller = new SessionController();
            _controller.Log += AppendLog;
            ProbeCameras();

            _timer = new Timer();
            _timer.Interval = 200;
            _timer.Tick += delegate { RefreshStatus(); };
            _timer.Start();
            UpdateSizeHint();
            FormClosing += OnFormClosing;

            AppendLog("就绪。点“开始采集”前请确认相机与雷达网络。没有相机硬件时可选 NullSource 验证流程。");
        }

        // ------------------------------------------------------------ 界面

        private void BuildUi()
        {
            Text = "采集 GUI - 雷达 ADC + 彩色 + 深度";
            StartPosition = FormStartPosition.CenterScreen;
            ClientSize = new Size(944, 666);
            MinimumSize = new Size(880, 616);
            Font = new Font("Microsoft YaHei UI", 9F);

            GroupBox settings = new GroupBox();
            settings.Text = "采集设置";
            settings.Bounds = new Rectangle(12, 8, 920, 262);
            Controls.Add(settings);

            settings.Controls.Add(MakeLabel("相机", 12, 26, 40));
            _cameraBox = new ComboBox();
            _cameraBox.DropDownStyle = ComboBoxStyle.DropDownList;
            _cameraBox.Bounds = new Rectangle(56, 22, 430, 24);
            _cameraBox.SelectedIndexChanged += delegate { ShowCameraDetail(); };
            settings.Controls.Add(_cameraBox);

            _refreshButton = new Button();
            _refreshButton.Text = "重新探测";
            _refreshButton.Bounds = new Rectangle(494, 21, 82, 26);
            _refreshButton.Click += delegate { ProbeCameras(); };
            settings.Controls.Add(_refreshButton);

            _cameraInfo = new Label();
            _cameraInfo.Bounds = new Rectangle(12, 48, 896, 36);
            _cameraInfo.ForeColor = Color.FromArgb(90, 90, 90);
            settings.Controls.Add(_cameraInfo);

            settings.Controls.Add(MakeLabel("雷达 UDP", 12, 94, 62));
            _radarEnabled = new CheckBox();
            _radarEnabled.Text = "启用";
            _radarEnabled.Checked = true;
            _radarEnabled.Bounds = new Rectangle(78, 92, 56, 22);
            _radarEnabled.CheckedChanged += delegate { UpdateSizeHint(); };
            settings.Controls.Add(_radarEnabled);

            settings.Controls.Add(MakeLabel("本机 IP", 140, 94, 54));
            _localIp = new TextBox();
            _localIp.Text = "192.168.220.10";
            _localIp.Bounds = new Rectangle(196, 90, 118, 24);
            settings.Controls.Add(_localIp);

            _localPort = new NumericUpDown();
            _localPort.Minimum = 1;
            _localPort.Maximum = 65535;
            _localPort.Value = 8080;
            _localPort.Bounds = new Rectangle(320, 90, 70, 24);
            settings.Controls.Add(_localPort);

            settings.Controls.Add(MakeLabel("只收此源", 400, 94, 62));
            _boardIp = new TextBox();
            _boardIp.Text = "192.168.220.100";
            _boardIp.Bounds = new Rectangle(462, 90, 118, 24);
            settings.Controls.Add(_boardIp);
            settings.Controls.Add(MakeLabel("（留空 = 不过滤）", 586, 94, 130));

            settings.Controls.Add(MakeLabel("延迟启动", 12, 126, 60));
            _delayBox = new NumericUpDown();
            _delayBox.Minimum = 0;
            _delayBox.Maximum = 60;
            _delayBox.Value = 3;
            _delayBox.Bounds = new Rectangle(76, 122, 56, 24);
            _delayBox.ValueChanged += delegate { UpdateSizeHint(); };
            settings.Controls.Add(_delayBox);
            settings.Controls.Add(MakeLabel("秒", 136, 126, 20));

            settings.Controls.Add(MakeLabel("采集时长", 178, 126, 60));
            _durationBox = new NumericUpDown();
            _durationBox.Minimum = 1;
            _durationBox.Maximum = 600;
            _durationBox.Value = 20;
            _durationBox.Bounds = new Rectangle(242, 122, 64, 24);
            _durationBox.ValueChanged += delegate { UpdateSizeHint(); };
            settings.Controls.Add(_durationBox);
            settings.Controls.Add(MakeLabel("秒", 310, 126, 20));

            settings.Controls.Add(MakeLabel("标签", 350, 126, 34));
            _labelBox = new TextBox();
            _labelBox.Bounds = new Rectangle(386, 122, 150, 24);
            _labelBox.Text = "室内2米";
            settings.Controls.Add(_labelBox);

            settings.Controls.Add(MakeLabel("输出目录", 552, 126, 60));
            _outputBox = new TextBox();
            _outputBox.Bounds = new Rectangle(614, 122, 294, 24);
            _outputBox.Text = DefaultTakeRoot();
            settings.Controls.Add(_outputBox);

            settings.Controls.Add(MakeLabel("雷达参数", 12, 158, 60));
            Label radarHint = MakeLabel("2048 包/帧 × 2048 B，帧号 = floor(packetId/2048)+1", 76, 158, 500);
            radarHint.ForeColor = Color.FromArgb(110, 110, 110);
            settings.Controls.Add(radarHint);

            _browseButton = new Button();
            _browseButton.Text = "浏览…";
            _browseButton.Bounds = new Rectangle(614, 154, 82, 26);            _browseButton.Click += delegate
            {
                using (FolderBrowserDialog dialog = new FolderBrowserDialog())
                {
                    dialog.Description = "选择 take 输出目录";
                    if (Directory.Exists(_outputBox.Text)) dialog.SelectedPath = _outputBox.Text;
                    if (dialog.ShowDialog(this) == DialogResult.OK) _outputBox.Text = dialog.SelectedPath;
                }
            };
            settings.Controls.Add(_browseButton);

            _sizeHint = MakeLabel("", 12, 230, 890);
            settings.Controls.Add(_sizeHint);

            _saveCloud = new CheckBox();
            _saveCloud.Text = "同时存点云 (.ply)：每 4×4 像素取 1 点，无效深度跳过（默认关；原始深度图无论如何都存）";
            _saveCloud.Checked = false;
            _saveCloud.Bounds = new Rectangle(12, 178, 700, 22);
            _saveCloud.ForeColor = Color.FromArgb(60, 60, 60);
            settings.Controls.Add(_saveCloud);

            _soundEnabled = new CheckBox();
            _soundEnabled.Text = "提示音：预备倒计时结束时响一声，拍摄结束时再响一声";
            _soundEnabled.Checked = true;
            _soundEnabled.Bounds = new Rectangle(12, 204, 890, 22);
            _soundEnabled.ForeColor = Color.FromArgb(60, 60, 60);
            settings.Controls.Add(_soundEnabled);

            GroupBox stateGroup = new GroupBox();
            stateGroup.Text = "状态";
            stateGroup.Bounds = new Rectangle(12, 278, 920, 136);
            Controls.Add(stateGroup);

            _stateLabel = new Label();
            _stateLabel.Text = "空闲";
            _stateLabel.Font = new Font("Microsoft YaHei UI", 14F, FontStyle.Bold);
            _stateLabel.Bounds = new Rectangle(16, 22, 300, 30);
            stateGroup.Controls.Add(_stateLabel);

            _progress = new ProgressBar();
            _progress.Bounds = new Rectangle(16, 58, 700, 16);
            _progress.Minimum = 0;
            _progress.Maximum = 100;
            stateGroup.Controls.Add(_progress);

            _detailLabel = new Label();
            _detailLabel.Bounds = new Rectangle(16, 78, 700, 20);
            _detailLabel.ForeColor = Color.FromArgb(90, 90, 90);
            stateGroup.Controls.Add(_detailLabel);

            _startButton = new Button();
            _startButton.Text = "开始采集";
            _startButton.Bounds = new Rectangle(734, 20, 168, 34);
            _startButton.Font = new Font("Microsoft YaHei UI", 10F, FontStyle.Bold);
            _startButton.Click += delegate { StartCapture(); };
            stateGroup.Controls.Add(_startButton);

            _stopButton = new Button();
            _stopButton.Text = "停止";
            _stopButton.Bounds = new Rectangle(734, 60, 168, 30);
            _stopButton.Enabled = false;
            _stopButton.Click += delegate { _controller.Stop(); };
            stateGroup.Controls.Add(_stopButton);

            _alignButton = new Button();
            _alignButton.Text = "对齐未处理的采集";
            _alignButton.Bounds = new Rectangle(734, 96, 168, 26);
            _alignButton.Click += delegate { StartAlign(); };
            stateGroup.Controls.Add(_alignButton);

            _logBox = new TextBox();
            _logBox.Multiline = true;
            _logBox.ScrollBars = ScrollBars.Vertical;
            _logBox.ReadOnly = true;
            _logBox.Bounds = new Rectangle(12, 422, 920, 192);
            _logBox.Font = new Font("Consolas", 9F);
            _logBox.BackColor = Color.FromArgb(250, 250, 250);
            _logBox.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom;
            Controls.Add(_logBox);

            _statusStrip = new StatusStrip();
            _radarStatus = new ToolStripStatusLabel("雷达 -");
            _colorStatus = new ToolStripStatusLabel("彩色 -");
            _depthStatus = new ToolStripStatusLabel("深度 -");
            _remainStatus = new ToolStripStatusLabel("剩余 -");
            _radarStatus.BorderSides = ToolStripStatusLabelBorderSides.Right;
            _colorStatus.BorderSides = ToolStripStatusLabelBorderSides.Right;
            _depthStatus.BorderSides = ToolStripStatusLabelBorderSides.Right;
            _statusStrip.Items.Add(_radarStatus);
            _statusStrip.Items.Add(_colorStatus);
            _statusStrip.Items.Add(_depthStatus);
            _statusStrip.Items.Add(new ToolStripStatusLabel { Spring = true });
            _statusStrip.Items.Add(_remainStatus);
            Controls.Add(_statusStrip);
        }

        private static Label MakeLabel(string text, int x, int y, int width)
        {
            Label label = new Label();
            label.Text = text;
            label.Bounds = new Rectangle(x, y, width, 20);
            return label;
        }

        private static string DefaultTakeRoot()
        {
            string exeDir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
            try
            {
                DirectoryInfo bin = new DirectoryInfo(exeDir);
                if (bin.Parent != null && string.Equals(bin.Name, "bin", StringComparison.OrdinalIgnoreCase) &&
                    bin.Parent.Parent != null)
                {
                    return Path.Combine(bin.Parent.Parent.FullName, "takes");
                }
            }
            catch { }
            return Path.Combine(exeDir, "takes");
        }

        // ------------------------------------------------------------ 交互

        private void ProbeCameras()
        {
            _refreshButton.Enabled = false;
            try
            {
                _cameras = CameraProbe.Probe(AppendLog);
                _cameraBox.Items.Clear();
                int preferred = -1;
                int preferredOpenNi = -1;
                int firstAvailable = -1;
                for (int i = 0; i < _cameras.Count; i++)
                {
                    CameraOption option = _cameras[i];
                    _cameraBox.Items.Add(option);
                    if (option.Available && firstAvailable < 0) firstAvailable = i;
                    if (option.Available && option.Kind == CameraKind.RealSense && preferred < 0) preferred = i;
                    if (option.Available && option.Kind == CameraKind.OpenNi2 && preferredOpenNi < 0) preferredOpenNi = i;
                }
                if (preferred >= 0) _cameraBox.SelectedIndex = preferred;
                else if (preferredOpenNi >= 0) _cameraBox.SelectedIndex = preferredOpenNi;
                else if (firstAvailable >= 0) _cameraBox.SelectedIndex = firstAvailable;
                _cameraInfo.Text = CameraProbe.Summarize(_cameras);
                AppendLog("相机探测完成：" + CameraProbe.Summarize(_cameras));
            }
            catch (Exception ex)
            {
                AppendLog("相机探测异常：" + ex.Message);
                _cameraInfo.Text = "相机探测异常：" + ex.Message;
            }
            finally
            {
                _refreshButton.Enabled = true;
            }
        }

        private void ShowCameraDetail()
        {
            CameraOption option = _cameraBox.SelectedItem as CameraOption;
            if (option == null) return;
            string text = option.Display + "　" + option.Reason;
            if (option.Available && !option.HasDepth) text += "　【深度不可用】";
            if (!option.Available) text += "　【不可用，无法开始】";
            _cameraInfo.Text = text;
        }

        /// <summary>
        /// MAT v5 单个 cell 变量上限 4 GiB，10 fps × 2048 包时每包约占 2312 字节 → 约 47 MB/s，
        /// 也就是 90 秒左右就封顶。提前算给用户看，别等落盘才失败。
        /// </summary>
        private void UpdateSizeHint()
        {
            if (_sizeHint == null) return;
            if (!_radarEnabled.Checked)
            {
                _sizeHint.Text = "雷达未启用：本次只录相机，不生成 radar_adc.mat";
                _sizeHint.ForeColor = Color.FromArgb(110, 110, 110);
                return;
            }
            double seconds = (double)_durationBox.Value;
            double bytesPerSecond = 10.0 * 2048 * 2312.0; // 10 fps × 2048 包 × 每行 MAT 字节
            double gb = seconds * bytesPerSecond / (1024.0 * 1024.0 * 1024.0);
            string text = "预计 radar_adc.mat ≈ " + gb.ToString("0.00") +
                          " GB（按 10 fps、2048 包/帧估算）；MAT v5 单变量上限 4 GiB ≈ 90 s @10 fps";
            bool tooBig = gb > 3.8;
            if (tooBig) text += "　【本次会超限：将保留 .packets，不生成 .mat】";
            _sizeHint.Text = text;
            _sizeHint.ForeColor = tooBig ? Color.FromArgb(180, 30, 30) : Color.FromArgb(110, 110, 110);
        }

        /// <summary>
        /// 手动触发：扫描输出目录，把还没对齐（没 align.csv）的 take 全部生成对齐表。
        /// 进度复用采集那根进度条。
        /// </summary>
        private void StartAlign()
        {
            if (_alignRunning) return;
            string root = _outputBox.Text.Trim();
            if (root.Length == 0)
            {
                MessageBox.Show(this, "请先填写输出目录。", "无法对齐", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!System.IO.Directory.Exists(root))
            {
                MessageBox.Show(this, "输出目录不存在：\r\n" + root, "无法对齐", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            _alignRunning = true;
            _alignButton.Enabled = false;
            _stateLabel.Text = "对齐中";
            _progress.Value = 0;
            AppendLog("开始对齐：" + root);

            System.Threading.Thread worker = new System.Threading.Thread(delegate()
            {
                try
                {
                    TakeAligner.AlignAll(root,
                        delegate(string line) { AppendLog(line); },
                        delegate(int done, int total)
                        {
                            int pct = total <= 0 ? 100 : (int)(100.0 * done / total);
                            try
                            {
                                BeginInvoke(new Action(delegate
                                {
                                    _progress.Value = Math.Max(0, Math.Min(100, pct));
                                    _detailLabel.Text = "对齐进度 " + done + "/" + total;
                                }));
                            }
                            catch { }
                        },
                        null);
                }
                catch (Exception ex)
                {
                    AppendLog("对齐异常：" + ex.Message);
                }
                finally
                {
                    try
                    {
                        BeginInvoke(new Action(delegate
                        {
                            _alignRunning = false;
                            _alignButton.Enabled = true;
                            _stateLabel.Text = "空闲";
                            _progress.Value = 100;
                            _detailLabel.Text = "对齐结束";
                        }));
                    }
                    catch { }
                }
            });
            worker.IsBackground = true;
            worker.Name = "Acquire align";
            worker.Start();
        }

        private void StartCapture()        {
            if (_controller.IsBusy) return;
            CameraOption option = _cameraBox.SelectedItem as CameraOption;
            if (option == null)
            {
                MessageBox.Show(this, "请先选择一个相机来源。", "无法开始", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (!option.Available)
            {
                MessageBox.Show(this, option.Display + "\r\n\r\n原因：" + option.Reason,
                    "相机不可用", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            string output = _outputBox.Text.Trim();
            if (output.Length == 0)
            {
                MessageBox.Show(this, "请填写输出目录。", "无法开始", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            SessionOptions session = new SessionOptions
            {
                OutputRoot = output,
                Label = _labelBox.Text.Trim(),
                DelaySeconds = (int)_delayBox.Value,
                DurationSeconds = (int)_durationBox.Value,
                RadarEnabled = _radarEnabled.Checked,
                LocalIp = _localIp.Text.Trim(),
                LocalPort = (int)_localPort.Value,
                BoardIp = _boardIp.Text.Trim(),
                Camera = option,
                SaveCloud = _saveCloud.Checked,
                SoundCues = _soundEnabled.Checked,
                SelfTest = option.Kind == CameraKind.Null
            };

            try
            {
                _controller.Start(session);
                AppendLog("开始任务：延迟 " + session.DelaySeconds + " s / 时长 " + session.DurationSeconds + " s / 相机 " + option.Display);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "启动失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        // ------------------------------------------------------------ 状态刷新

        private void RefreshStatus()
        {
            if (_closing) return;
            SessionSnapshot s = _controller.Snapshot();

            switch (s.State)
            {
                case SessionState.Countdown:
                    _stateLabel.Text = "倒计时 " + s.CountdownRemaining + " s";
                    _stateLabel.ForeColor = Color.FromArgb(200, 120, 0);
                    break;
                case SessionState.Recording:
                    _stateLabel.Text = "记录中";
                    _stateLabel.ForeColor = Color.FromArgb(180, 30, 30);
                    break;
                case SessionState.Saving:
                    _stateLabel.Text = "正在落盘";
                    _stateLabel.ForeColor = Color.FromArgb(30, 90, 180);
                    break;
                default:
                    _stateLabel.Text = "空闲";
                    _stateLabel.ForeColor = Color.FromArgb(50, 50, 50);
                    break;
            }

            bool idle = s.State == SessionState.Idle;
            _startButton.Enabled = idle;
            _stopButton.Enabled = !idle;
            _cameraBox.Enabled = idle;
            _refreshButton.Enabled = idle;
            _radarEnabled.Enabled = idle;
            _saveCloud.Enabled = idle;
            _soundEnabled.Enabled = idle;
            _localIp.Enabled = idle;
            _localPort.Enabled = idle;
            _boardIp.Enabled = idle;
            _delayBox.Enabled = idle;
            _durationBox.Enabled = idle;
            _labelBox.Enabled = idle;
            _outputBox.Enabled = idle;
            _browseButton.Enabled = idle;

            if (s.State == SessionState.Recording && _durationBox.Value > 0)
            {
                int percent = (int)Math.Min(100, s.ElapsedSeconds * 100.0 / (double)_durationBox.Value);
                _progress.Value = percent;
                _detailLabel.Text = "已录 " + s.ElapsedSeconds.ToString("0.0") + " s / " +
                                    ((int)_durationBox.Value) + " s　" + s.Message;
            }
            else
            {
                _progress.Value = 0;
                _detailLabel.Text = s.Message;
            }

            if (!string.IsNullOrEmpty(s.Error)) _detailLabel.Text = "错误：" + s.Error;

            _radarStatus.Text = _radarEnabled.Checked
                ? "雷达 " + s.RadarFps.ToString("0.00") + " fps | 帧 " + s.RadarFrames + " | 包 " + s.RadarPackets + " | 丢 " + s.RadarMissing
                : "雷达 已关闭";
            _colorStatus.Text = "彩色 " + s.ColorFps.ToString("0.0") + " fps | 帧 " + s.ColorFrames;
            _depthStatus.Text = "深度 " + s.DepthFps.ToString("0.0") + " fps | 帧 " + s.DepthFrames +
                                (s.CloudFrames > 0 ? " | 点云 " + s.CloudFrames + " 帧/" + s.CloudPoints + " 点" : "");
            _remainStatus.Text = s.State == SessionState.Recording
                ? "剩余 " + s.RemainingSeconds.ToString("0.0") + " s"
                : (string.IsNullOrEmpty(s.TakeDirectory) ? "剩余 -" : ShortPath(s.TakeDirectory));
        }

        private static string ShortPath(string path)        {
            if (string.IsNullOrEmpty(path)) return "";
            string name = Path.GetFileName(path.TrimEnd(Path.DirectorySeparatorChar));
            return name.Length > 40 ? name.Substring(0, 37) + "..." : name;
        }

        internal void AppendLog(string line)
        {
            if (_closing) return;
            if (InvokeRequired)
            {
                try { BeginInvoke(new Action<string>(AppendLog), line); } catch { }
                return;
            }
            if (_logBox.TextLength > 400000) _logBox.Clear();
            _logBox.AppendText(line + "\r\n");
        }

        private void OnFormClosing(object sender, FormClosingEventArgs e)
        {
            if (_controller != null && _controller.IsBusy)
            {
                DialogResult result = MessageBox.Show(this, "采集仍在进行，关闭窗口会停止并落盘。继续？",
                    "确认关闭", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
                if (result != DialogResult.Yes)
                {
                    e.Cancel = true;
                    return;
                }
            }
            _closing = true;
            if (_timer != null) _timer.Stop();
            if (_controller != null)
            {
                try { _controller.Dispose(); } catch { }
            }
        }
    }
}
