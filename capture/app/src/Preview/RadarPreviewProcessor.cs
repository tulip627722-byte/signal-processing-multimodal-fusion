using System;
using System.Threading;

namespace Acquire
{
    internal sealed class RadarPreviewProcessor : IDisposable
    {
        private const int Samples = 1024;
        private const int Bins = Samples / 2;
        private const double SpeedOfLight = 299792458.0;
        private const double BandwidthHz = 4.0e9;
        private readonly object _gate = new object();
        private readonly LatestPreview<RadarPreviewFrame> _output;
        private readonly Thread _worker;
        private double[] _frameSamples;
        private volatile int _frameIndex = -1;
        private double _firstRelMs;
        private Work _pending;
        private volatile bool _stop;

        private sealed class Work
        {
            public double[] Samples;
            public int FrameIndex;
            public double RelMs;
        }

        public RadarPreviewProcessor(LatestPreview<RadarPreviewFrame> output)
        {
            _output = output;
            _worker = new Thread(WorkerLoop) { IsBackground = true, Name = "Acquire radar preview", Priority = ThreadPriority.BelowNormal };
            _worker.Start();
        }

        public void AddChirp(int frameIndex, byte[] payload, int offset, int count, double relMs)
        {
            if (payload == null || count < Samples * 2) return;
            if (frameIndex == _frameIndex) return;
            lock (_gate)
            {
                if (frameIndex == _frameIndex) return;
                if (_frameIndex >= 0 && frameIndex != _frameIndex)
                {
                    if (_frameSamples != null)
                    {
                        _pending = new Work { Samples = _frameSamples, FrameIndex = _frameIndex, RelMs = _firstRelMs };
                        Monitor.PulseAll(_gate);
                    }
                    _frameSamples = null;
                }
                if (frameIndex != _frameIndex) { _frameIndex = frameIndex; _firstRelMs = relMs; }
                if (_frameSamples == null)
                {
                    _frameSamples = new double[Samples];
                    for (int sample = 0; sample < Samples; sample++)
                    {
                        int index = offset + sample * 2;
                        _frameSamples[sample] = (payload[index] | (payload[index + 1] << 8)) - 2048;
                    }
                }
            }
        }

        private void WorkerLoop()
        {
            DateTime lastPublish = DateTime.MinValue;
            while (!_stop)
            {
                Work work;
                lock (_gate)
                {
                    while (_pending == null && !_stop) Monitor.Wait(_gate, 200);
                    if (_stop) return;
                    work = _pending;
                    _pending = null;
                }
                if ((DateTime.UtcNow - lastPublish).TotalMilliseconds < 90) continue;
                Publish(work);
                lastPublish = DateTime.UtcNow;
            }
        }

        private void Publish(Work work)
        {
            double[] real = work.Samples;
            double[] imaginary = new double[Samples];
            for (int sample = 0; sample < Samples; sample++)
                real[sample] *= 0.54 - 0.46 * Math.Cos(2 * Math.PI * sample / (Samples - 1));
            Fft(real, imaginary);
            double[] range = new double[Bins];
            double[] magnitude = new double[Bins];
            int peak = 1;
            double peakDb = double.MinValue;
            double resolution = SpeedOfLight / (2.0 * BandwidthHz);
            for (int i = 0; i < Bins; i++)
            {
                range[i] = i * resolution;
                double amplitude = Math.Sqrt(real[i] * real[i] + imaginary[i] * imaginary[i]);
                magnitude[i] = 20.0 * Math.Log10(Math.Max(amplitude, 1e-9));
                if (i > 0 && magnitude[i] > peakDb) { peakDb = magnitude[i]; peak = i; }
            }
            _output.Publish(new RadarPreviewFrame
            {
                RangeMeters = range, MagnitudeDb = magnitude, PeakRangeMeters = range[peak], PeakMagnitudeDb = magnitude[peak],
                FrameNumber = work.FrameIndex + 1, RelMs = work.RelMs
            });
        }

        internal static void Fft(double[] real, double[] imaginary)
        {
            int n = real.Length;
            for (int j = 1, i = 0; j < n; j++)
            {
                int bit = n >> 1; for (; (i & bit) != 0; bit >>= 1) i ^= bit; i ^= bit;
                if (j < i) { double swap = real[j]; real[j] = real[i]; real[i] = swap; swap = imaginary[j]; imaginary[j] = imaginary[i]; imaginary[i] = swap; }
            }
            for (int length = 2; length <= n; length <<= 1)
            {
                double angle = -2 * Math.PI / length, wLengthReal = Math.Cos(angle), wLengthImaginary = Math.Sin(angle);
                for (int i = 0; i < n; i += length)
                {
                    double wReal = 1, wImaginary = 0;
                    for (int j = 0; j < length / 2; j++)
                    {
                        int even = i + j, odd = even + length / 2;
                        double oddReal = real[odd] * wReal - imaginary[odd] * wImaginary;
                        double oddImaginary = real[odd] * wImaginary + imaginary[odd] * wReal;
                        real[odd] = real[even] - oddReal; imaginary[odd] = imaginary[even] - oddImaginary;
                        real[even] += oddReal; imaginary[even] += oddImaginary;
                        double nextReal = wReal * wLengthReal - wImaginary * wLengthImaginary;
                        wImaginary = wReal * wLengthImaginary + wImaginary * wLengthReal; wReal = nextReal;
                    }
                }
            }
        }

        public void Dispose()
        {
            _stop = true;
            lock (_gate) Monitor.PulseAll(_gate);
            if (_worker != null && _worker.IsAlive) _worker.Join(1000);
        }
    }
}
