using System;

namespace Acquire
{
    /// <summary>相机源可以报告“实际生效的模式”，由 SessionController 写进 meta.json。</summary>
    internal interface ICameraModeReporter
    {
        string DescribeActiveModes();
    }

    /// <summary>
    /// OpenNI2 相机源的门面：优先用托管包装（OpenNI2.Net.dll，反射），
    /// 打开失败或没装包装时退回原生 C API（OpenNI2.dll）。
    ///
    /// 之所以要这层门面：官方 OpenNI SDK 2.3 的 Windows 包只带原生库，
    /// 所以“装了 SDK 但没有 OpenNI2.Net.dll”是最常见的情形，不能因此判相机不可用。
    /// </summary>
    internal sealed class OpenNi2Source : ICameraSource, ICameraModeReporter, IDepthIntrinsicsProvider
    {
        private readonly CameraOption _option;
        private readonly Action<string> _log;
        private readonly Func<double> _clock;
        private ICameraSource _inner;
        private bool _hasColor;
        private bool _hasDepth;

        public OpenNi2Source(CameraOption option, Action<string> log, Func<double> clock)
        {
            _option = option;
            _log = log ?? delegate { };
            _clock = clock ?? delegate { return 0; };
            _hasColor = option != null && option.HasColor;
            _hasDepth = option != null && option.HasDepth;
        }

        public CameraKind Kind { get { return CameraKind.OpenNi2; } }
        public string Model { get { return _inner != null ? _inner.Model : (_option == null ? "" : _option.Model); } }
        public string Serial { get { return _inner != null ? _inner.Serial : (_option == null ? "" : _option.Serial); } }
        public bool HasColor { get { return _inner != null ? _inner.HasColor : _hasColor; } }
        public bool HasDepth { get { return _inner != null ? _inner.HasDepth : _hasDepth; } }
        public double DepthScaleMeters { get { return _inner != null ? _inner.DepthScaleMeters : 0.001; } }

        public string DescribeActiveModes()
        {
            ICameraModeReporter reporter = _inner as ICameraModeReporter;
            return reporter == null ? "" : reporter.DescribeActiveModes();
        }

        /// <summary>
        /// 内参跟着实际打开的那条通道走：原生 C API 已实现（读 FOV），
        /// 托管包装没做，返回 null 由上层按标称值兜底。
        /// </summary>
        public DepthIntrinsics GetDepthIntrinsics()
        {
            IDepthIntrinsicsProvider provider = _inner as IDepthIntrinsicsProvider;
            return provider == null ? null : provider.GetDepthIntrinsics();
        }

        public void Open()
        {
            if (_inner != null) return;

            bool preferNative = _option != null && string.Equals(_option.Backend, "native", StringComparison.OrdinalIgnoreCase);
            string managedProblem = null;

            if (!preferNative)
            {
                OpenNi2ManagedSource managed = new OpenNi2ManagedSource(_option, _log, _clock);
                try
                {
                    managed.Open();
                    _inner = managed;
                    return;
                }
                catch (Exception ex)
                {
                    managedProblem = DynHelper.DescribeException(ex);
                    try { managed.Dispose(); } catch { }
                    _log("OpenNI2 托管包装路线失败：" + managedProblem + "；改用原生 C API（OpenNI2.dll）");
                }
            }

            OpenNi2NativeSource native = new OpenNi2NativeSource(_option, _log, _clock);
            try
            {
                native.Open();
            }
            catch (Exception ex)
            {
                string message = DynHelper.DescribeException(ex);
                if (managedProblem != null) message = "托管包装：" + managedProblem + "；原生 C API：" + message;
                try { native.Dispose(); } catch { }
                throw new InvalidOperationException(message);
            }
            _inner = native;

            if (_option != null)
            {
                _hasColor = native.HasColor;
                _hasDepth = native.HasDepth;
            }
        }

        public CameraFrame ReadFrame(int timeoutMs)
        {
            return _inner == null ? null : _inner.ReadFrame(timeoutMs);
        }

        public void Close()
        {
            if (_inner == null) return;
            try { _inner.Close(); } catch { }
        }

        public void Dispose()
        {
            if (_inner == null) return;
            try { _inner.Dispose(); } catch { }
            _inner = null;
        }
    }
}
