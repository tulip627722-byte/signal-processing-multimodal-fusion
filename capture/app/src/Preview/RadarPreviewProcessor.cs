using System;
using System.Collections.Generic;
using System.Threading;

namespace Acquire
{
    internal sealed class RadarPreviewProcessor : IDisposable
    {
        private const int Samples = 1024;
        private const int Bins = Samples / 2;
        private const double SpeedOfLight = 3.0e8;
        private const double SampleRateHz = 16.667e6;
        private const double BandwidthHz = 4.0e9;
        private const double RampSeconds = 60.0e-6;
        private const int ChirpsPerFrame = 2048;
        private const int EightTransmitStart = 1024;
        private const int TransmitChannels = 8;
        private const int ChirpsPerTransmit = 16;
        private const int FirstTargetBin = 14;
        private const int PeakHalfWidth = 5;
        private readonly object _gate = new object();
        private readonly LatestPreview<RadarPreviewFrame> _output;
        private readonly Thread _worker;
        private List<double[]> _frameChirps;
        private volatile int _frameIndex = -1;
        private double _firstRelMs;
        private Work _pending;
        private volatile bool _stop;

        private sealed class Work
        {
            public List<double[]> Chirps;
            public int FrameIndex;
            public double RelMs;
        }

        public RadarPreviewProcessor(LatestPreview<RadarPreviewFrame> output)
        {
            _output = output;
            _worker = new Thread(WorkerLoop) { IsBackground = true, Name = "Acquire radar preview", Priority = ThreadPriority.BelowNormal };
            _worker.Start();
        }

        public void AddChirp(int frameIndex, int chirpInFrame, byte[] payload, int offset, int count, double relMs)
        {
            if (payload == null || count < Samples * 2) return;
            if (chirpInFrame < EightTransmitStart || chirpInFrame >= ChirpsPerFrame) return;
            int eightTransmitOffset = chirpInFrame - EightTransmitStart;
            int chirpSequence = (eightTransmitOffset / TransmitChannels) % ChirpsPerTransmit;
            if ((chirpSequence % 4) != 0) return;
            lock (_gate)
            {
                if (_frameIndex >= 0 && frameIndex != _frameIndex)
                {
                    if (_frameChirps != null && _frameChirps.Count > 0)
                    {
                        _pending = new Work { Chirps = _frameChirps, FrameIndex = _frameIndex, RelMs = _firstRelMs };
                        Monitor.PulseAll(_gate);
                    }
                    _frameChirps = null;
                }
                if (frameIndex != _frameIndex) { _frameIndex = frameIndex; _firstRelMs = relMs; }
                if (_frameChirps == null) _frameChirps = new List<double[]>(128);
                double[] samples = new double[Samples];
                for (int sample = 0; sample < Samples; sample++)
                {
                    int index = offset + sample * 2;
                    samples[sample] = (payload[index] | (payload[index + 1] << 8)) - 2048;
                }
                _frameChirps.Add(samples);
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
            double[] amplitudeSum = new double[Bins];
            foreach (double[] chirp in work.Chirps)
            {
                double[] real = chirp;
                double[] imaginary = new double[Samples];
                for (int sample = 0; sample < Samples; sample++)
                    real[sample] *= 0.54 - 0.46 * Math.Cos(2 * Math.PI * sample / (Samples - 1));
                Fft(real, imaginary);
                for (int i = 0; i < Bins; i++)
                    amplitudeSum[i] += Math.Sqrt(real[i] * real[i] + imaginary[i] * imaginary[i]);
            }
            double[] range = new double[Bins];
            double[] magnitude = new double[Bins];
            double resolution = DistancePerBin;
            for (int i = 0; i < Bins; i++)
            {
                range[i] = i * resolution;
                double amplitude = amplitudeSum[i] / Math.Max(1, work.Chirps.Count);
                magnitude[i] = 20.0 * Math.Log10(Math.Max(amplitude, 1e-9));
            }
            int peak = FindTargetPeak(amplitudeSum);
            _output.Publish(new RadarPreviewFrame
            {
                RangeMeters = range, MagnitudeDb = magnitude, PeakRangeMeters = range[peak], PeakMagnitudeDb = magnitude[peak],
                LeakCutoffMeters = range[FirstTargetBin], AveragedChirps = work.Chirps.Count,
                FrameNumber = work.FrameIndex + 1, RelMs = work.RelMs
            });
        }

        internal static double DistancePerBin
        {
            get { return SpeedOfLight * SampleRateHz / (2.0 * (BandwidthHz / RampSeconds) * Samples); }
        }

        internal static int FindTargetPeak(double[] amplitude)
        {
            if (amplitude == null || amplitude.Length <= FirstTargetBin) return 0;
            int strongestLocal = -1;
            double strongestValue = double.MinValue;
            int last = amplitude.Length - PeakHalfWidth;
            for (int i = Math.Max(FirstTargetBin, PeakHalfWidth); i < last; i++)
            {
                bool local = true;
                for (int d = 1; d <= PeakHalfWidth && local; d++)
                    local = amplitude[i] > amplitude[i - d] && amplitude[i] > amplitude[i + d];
                if (local && amplitude[i] > strongestValue)
                {
                    strongestValue = amplitude[i];
                    strongestLocal = i;
                }
            }
            if (strongestLocal >= 0) return strongestLocal;

            int strongest = FirstTargetBin;
            for (int i = FirstTargetBin + 1; i < amplitude.Length; i++)
                if (amplitude[i] > amplitude[strongest]) strongest = i;
            return strongest;
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
