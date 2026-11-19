using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Threading;

namespace Acquire
{
    internal sealed class RadarPointCloudProcessor : IDisposable
    {
        private const int Samples = 1024;
        private const int RangeBins = 512;
        private const int DopplerBins = 128;
        private const int ChirpsPerFrame = 2048;
        private const int PayloadBytes = 2048;
        private const int MaxDetections = 80;
        private const double Lambda = 3.0e8 / 77.0e9;
        private const double ChirpSeconds = 67.0e-6;
        private const double RangeResolution = 3.0e8 * 16.667e6 / (2.0 * (4.0e9 / 60.0e-6) * 1024.0);
        private readonly object _gate = new object();
        private readonly LatestPreview<RadarCloudFrame> _output;
        private readonly Thread _worker;
        private byte[] _frame;
        private bool[] _received;
        private int _receivedCount;
        private int _frameIndex = -1;
        private double _firstRelMs;
        private Work _pending;
        private volatile bool _stop;

        private sealed class Work
        {
            public byte[] Raw;
            public int FrameIndex;
            public double RelMs;
        }

        private sealed class Candidate
        {
            public int Range;
            public int Doppler;
            public double Amplitude;
        }

        public RadarPointCloudProcessor(LatestPreview<RadarCloudFrame> output)
        {
            _output = output;
            _worker = new Thread(WorkerLoop)
            {
                IsBackground = true,
                Name = "Acquire PC radar point cloud",
                Priority = ThreadPriority.BelowNormal
            };
            _worker.Start();
        }

        public void AddChirp(int frameIndex, int chirpInFrame, byte[] payload, int offset, int count, double relMs)
        {
            if (payload == null || count < PayloadBytes || chirpInFrame < 0 || chirpInFrame >= ChirpsPerFrame) return;
            lock (_gate)
            {
                if (_frameIndex >= 0 && frameIndex != _frameIndex)
                {
                    if (_receivedCount == ChirpsPerFrame)
                    {
                        _pending = new Work { Raw = _frame, FrameIndex = _frameIndex, RelMs = _firstRelMs };
                        Monitor.PulseAll(_gate);
                    }
                    _frame = null;
                    _received = null;
                }
                if (frameIndex != _frameIndex)
                {
                    _frameIndex = frameIndex;
                    _firstRelMs = relMs;
                    _frame = new byte[ChirpsPerFrame * PayloadBytes];
                    _received = new bool[ChirpsPerFrame];
                    _receivedCount = 0;
                }
                if (_received[chirpInFrame]) return;
                Buffer.BlockCopy(payload, offset, _frame, chirpInFrame * PayloadBytes, PayloadBytes);
                _received[chirpInFrame] = true;
                _receivedCount++;
            }
        }

        private void WorkerLoop()
        {
            while (!_stop)
            {
                Work work;
                lock (_gate)
                {
                    while (_pending == null && !_stop) Monitor.Wait(_gate, 250);
                    if (_stop) return;
                    work = _pending;
                    _pending = null;
                }
                try { Process(work); }
                catch { }
            }
        }

        private void Process(Work work)
        {
            Stopwatch timer = Stopwatch.StartNew();
            double[] oneTransmitProfile = ComputeOneTransmitProfile(work.Raw);
            double[] power = ComputeEightTransmitPower(work.Raw);
            List<Candidate> candidates = Detect(power, oneTransmitProfile);
            if (candidates.Count == 0)
            {
                Publish(work, new RadarCloudPoint[0], 0, timer.Elapsed.TotalMilliseconds);
                return;
            }
            Complex[] vectors = ExtractCandidateVectors(work.Raw, candidates);
            RadarCloudPoint[] points = BuildPoints(candidates, vectors);
            uint clusters = Cluster(points);
            Publish(work, points, clusters, timer.Elapsed.TotalMilliseconds);
        }

        private static double[] ComputeOneTransmitProfile(byte[] raw)
        {
            double[] rd = new double[RangeBins * DopplerBins];
            Complex[] range = new Complex[DopplerBins * RangeBins];
            double[] real = new double[Samples];
            double[] imaginary = new double[Samples];
            double[] dopplerReal = new double[DopplerBins];
            double[] dopplerImaginary = new double[DopplerBins];
            for (int receiver = 0; receiver < 8; receiver++)
            {
                for (int chirp = 0; chirp < DopplerBins; chirp++)
                {
                    DecodeRangeFft(raw, chirp + DopplerBins * receiver, real, imaginary);
                    int target = chirp * RangeBins;
                    for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
                        range[target + rangeBin] = new Complex(real[rangeBin], imaginary[rangeBin]);
                }
                for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
                {
                    for (int chirp = 0; chirp < DopplerBins; chirp++)
                    {
                        double window = Hamming(chirp, DopplerBins);
                        Complex value = range[chirp * RangeBins + rangeBin];
                        dopplerReal[chirp] = value.Real * window;
                        dopplerImaginary[chirp] = value.Imaginary * window;
                    }
                    RadarPreviewProcessor.Fft(dopplerReal, dopplerImaginary);
                    int row = rangeBin * DopplerBins;
                    for (int doppler = 0; doppler < DopplerBins; doppler++)
                        rd[row + doppler] += Hypot(dopplerReal[doppler], dopplerImaginary[doppler]);
                }
            }
            double[] profile = new double[RangeBins];
            for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
            {
                double maximum = 0;
                int row = rangeBin * DopplerBins;
                for (int doppler = 0; doppler < DopplerBins; doppler++) maximum = Math.Max(maximum, rd[row + doppler]);
                profile[rangeBin] = maximum;
            }
            return profile;
        }

        private static double[] ComputeEightTransmitPower(byte[] raw)
        {
            double[] power = new double[RangeBins * DopplerBins];
            double[] real = new double[Samples];
            double[] imaginary = new double[Samples];
            Complex[] range = new Complex[16 * RangeBins];
            double[] dopplerReal = new double[DopplerBins];
            double[] dopplerImaginary = new double[DopplerBins];
            for (int channel = 0; channel < 64; channel++)
            {
                FillEightTransmitRange(raw, channel, range, real, imaginary);
                for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
                {
                    Array.Clear(dopplerReal, 0, DopplerBins);
                    Array.Clear(dopplerImaginary, 0, DopplerBins);
                    for (int chirp = 0; chirp < 16; chirp++)
                    {
                        Complex value = range[chirp * RangeBins + rangeBin];
                        double window = Hamming(chirp, 16);
                        dopplerReal[chirp] = value.Real * window;
                        dopplerImaginary[chirp] = value.Imaginary * window;
                    }
                    RadarPreviewProcessor.Fft(dopplerReal, dopplerImaginary);
                    int row = rangeBin * DopplerBins;
                    for (int doppler = 0; doppler < DopplerBins; doppler++)
                    {
                        int shifted = (doppler + 64) & 127;
                        power[row + doppler] += Hypot(dopplerReal[shifted], dopplerImaginary[shifted]);
                    }
                }
            }
            return power;
        }

        private static List<Candidate> Detect(double[] power, double[] oneTransmitProfile)
        {
            double[] baseline = new double[RangeBins];
            for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
            {
                double sum = 0;
                int first = Math.Max(0, rangeBin - 32), last = Math.Min(RangeBins - 1, rangeBin + 32);
                for (int index = first; index <= last; index++) sum += oneTransmitProfile[index];
                baseline[rangeBin] = sum / (last - first + 1);
            }
            int width = DopplerBins + 1;
            double[] integral = new double[(RangeBins + 1) * width];
            for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
            {
                double rowSum = 0;
                for (int doppler = 0; doppler < DopplerBins; doppler++)
                {
                    rowSum += power[rangeBin * DopplerBins + doppler];
                    integral[(rangeBin + 1) * width + doppler + 1] = integral[rangeBin * width + doppler + 1] + rowSum;
                }
            }
            List<Candidate> result = new List<Candidate>();
            for (int rangeBin = 19; rangeBin <= 492; rangeBin++)
            {
                double meters = rangeBin * RangeResolution;
                if (meters < 1.0 || meters > 8.0 || oneTransmitProfile[rangeBin] < 0.5 * baseline[rangeBin]) continue;
                for (int doppler = 7; doppler <= 120; doppler++)
                {
                    int r1 = rangeBin - 18, r2 = rangeBin + 18, d1 = doppler - 6, d2 = doppler + 6;
                    double sum = RectangleSum(integral, width, r1, d1, r2, d2);
                    double value = power[rangeBin * DopplerBins + doppler];
                    double noise = (sum - value) / 480.0;
                    if (value > 1.53 * noise)
                        result.Add(new Candidate { Range = rangeBin, Doppler = doppler, Amplitude = value });
                }
            }
            result.Sort(delegate(Candidate left, Candidate right) { return right.Amplitude.CompareTo(left.Amplitude); });
            if (result.Count > MaxDetections) result.RemoveRange(MaxDetections, result.Count - MaxDetections);
            return result;
        }

        private static Complex[] ExtractCandidateVectors(byte[] raw, List<Candidate> candidates)
        {
            Complex[] vectors = new Complex[candidates.Count * 64];
            Dictionary<int, List<int>> byRange = new Dictionary<int, List<int>>();
            for (int index = 0; index < candidates.Count; index++)
            {
                List<int> indices;
                if (!byRange.TryGetValue(candidates[index].Range, out indices))
                {
                    indices = new List<int>();
                    byRange.Add(candidates[index].Range, indices);
                }
                indices.Add(index);
            }
            double[] real = new double[Samples];
            double[] imaginary = new double[Samples];
            Complex[] range = new Complex[16 * RangeBins];
            double[] dopplerReal = new double[DopplerBins];
            double[] dopplerImaginary = new double[DopplerBins];
            for (int channel = 0; channel < 64; channel++)
            {
                FillEightTransmitRange(raw, channel, range, real, imaginary);
                foreach (KeyValuePair<int, List<int>> item in byRange)
                {
                    Array.Clear(dopplerReal, 0, DopplerBins);
                    Array.Clear(dopplerImaginary, 0, DopplerBins);
                    for (int chirp = 0; chirp < 16; chirp++)
                    {
                        Complex value = range[chirp * RangeBins + item.Key];
                        double window = Hamming(chirp, 16);
                        dopplerReal[chirp] = value.Real * window;
                        dopplerImaginary[chirp] = value.Imaginary * window;
                    }
                    RadarPreviewProcessor.Fft(dopplerReal, dopplerImaginary);
                    foreach (int candidateIndex in item.Value)
                    {
                        int shifted = (candidates[candidateIndex].Doppler + 64) & 127;
                        vectors[candidateIndex * 64 + channel] = new Complex(dopplerReal[shifted], dopplerImaginary[shifted]);
                    }
                }
            }
            return vectors;
        }

        private static RadarCloudPoint[] BuildPoints(List<Candidate> candidates, Complex[] vectors)
        {
            int[] map;
            double[] positionX, positionZ;
            BuildVirtualArray(out map, out positionX, out positionZ);
            double[] receiverCorrection = { 0, 313, 221, 21, 158, 346, 225, 241 };
            double[] transmitterCorrection = { 48, 73, 193, 178, 0, 212, 320, 307 };
            List<RadarCloudPoint> points = new List<RadarCloudPoint>();
            for (int candidateIndex = 0; candidateIndex < candidates.Count; candidateIndex++)
            {
                Candidate candidate = candidates[candidateIndex];
                double velocity = (candidate.Doppler - 64) * Lambda / (2.0 * DopplerBins * 8.0 * ChirpSeconds);
                Complex[] virtualChannels = new Complex[positionX.Length];
                for (int channel = 0; channel < 64; channel++)
                {
                    int receiver = channel % 8, transmitter = channel / 8;
                    double phaseDegrees = receiverCorrection[receiver] + transmitterCorrection[transmitter];
                    double tdmPhase = -2.0 * Math.PI * (2.0 * velocity / Lambda) * transmitter * ChirpSeconds;
                    Complex correction = Complex.FromPolarCoordinates(1.0, phaseDegrees * Math.PI / 180.0 + tdmPhase);
                    virtualChannels[map[channel]] += vectors[candidateIndex * 64 + channel] * correction;
                }
                double rangeMeters = candidate.Range * RangeResolution;
                double azimuth, elevation;
                EstimateAngle(virtualChannels, positionX, positionZ, rangeMeters, out azimuth, out elevation);
                double azimuthRadians = azimuth * Math.PI / 180.0;
                double elevationRadians = elevation * Math.PI / 180.0;
                double x = rangeMeters * Math.Cos(elevationRadians) * Math.Sin(azimuthRadians);
                double y = rangeMeters * Math.Cos(elevationRadians) * Math.Cos(azimuthRadians);
                double z = rangeMeters * Math.Sin(elevationRadians);
                if (x < -3 || x > 3 || y < 1 || y > 8 || z < -1 || z > 1.5) continue;
                points.Add(new RadarCloudPoint
                {
                    X = (float)x, Y = (float)y, Z = (float)z,
                    MagnitudeDb = (float)(20.0 * Math.Log10(candidate.Amplitude + 1e-12)),
                    RangeMeters = (float)rangeMeters, VelocityMetersPerSecond = (float)velocity
                });
            }
            return points.ToArray();
        }

        private static void EstimateAngle(Complex[] channels, double[] x, double[] z, double range,
            out double bestAzimuth, out double bestElevation)
        {
            bestAzimuth = 0; bestElevation = 0; double best = double.MinValue;
            for (double elevation = -14; elevation <= 14; elevation += 4)
                for (double azimuth = -60; azimuth <= 60; azimuth += 4)
                {
                    double score = Bartlett(channels, x, z, range, azimuth, elevation);
                    if (score > best) { best = score; bestAzimuth = azimuth; bestElevation = elevation; }
                }
            double coarseAzimuth = bestAzimuth, coarseElevation = bestElevation;
            for (double elevation = Math.Max(-14, coarseElevation - 2); elevation <= Math.Min(14, coarseElevation + 2); elevation += 0.5)
                for (double azimuth = coarseAzimuth - 2; azimuth <= coarseAzimuth + 2; azimuth += 0.5)
                {
                    double score = Bartlett(channels, x, z, range, azimuth, elevation);
                    if (score > best) { best = score; bestAzimuth = azimuth; bestElevation = elevation; }
                }
        }

        private static double Bartlett(Complex[] channels, double[] x, double[] z, double range,
            double azimuthDegrees, double elevationDegrees)
        {
            range = Math.Max(range, 0.3);
            double azimuth = azimuthDegrees * Math.PI / 180.0;
            double elevation = elevationDegrees * Math.PI / 180.0;
            double directionX = Math.Sin(azimuth) * Math.Cos(elevation);
            double directionZ = Math.Sin(elevation);
            Complex sum = Complex.Zero;
            for (int index = 0; index < channels.Length; index++)
            {
                double projection = x[index] * directionX + z[index] * directionZ;
                double correction = (x[index] * x[index] + z[index] * z[index] - projection * projection) / (2.0 * range);
                double steeringPhase = -(2.0 * Math.PI / Lambda) * (-projection + correction);
                sum += channels[index] * Complex.FromPolarCoordinates(1.0, -steeringPhase);
            }
            return sum.Real * sum.Real + sum.Imaginary * sum.Imaginary;
        }

        private static uint Cluster(RadarCloudPoint[] points)
        {
            uint next = 0;
            for (int start = 0; start < points.Length; start++)
            {
                if (points[start].Cluster != 0) continue;
                List<int> members = new List<int> { start };
                for (int cursor = 0; cursor < members.Count; cursor++)
                {
                    RadarCloudPoint current = points[members[cursor]];
                    for (int other = 0; other < points.Length; other++)
                    {
                        if (other == start || points[other].Cluster != 0 || members.Contains(other)) continue;
                        double dx = current.X - points[other].X, dy = current.Y - points[other].Y, dz = current.Z - points[other].Z;
                        if (dx * dx + dy * dy + dz * dz <= 0.35 * 0.35) members.Add(other);
                    }
                }
                if (members.Count < 2) continue;
                next++;
                foreach (int member in members)
                {
                    RadarCloudPoint point = points[member];
                    point.Cluster = next;
                    points[member] = point;
                }
            }
            return next;
        }

        private void Publish(Work work, RadarCloudPoint[] points, uint clusters, double processingMs)
        {
            _output.Publish(new RadarCloudFrame
            {
                Sequence = (uint)(work.FrameIndex + 1), Version = 0, Status = 0x44344452,
                Points = points, Count = points.Length, ClusterCount = clusters,
                ReceivedAtMs = work.RelMs, BoardTimestampMicroseconds = (uint)Math.Min(uint.MaxValue, processingMs * 1000.0)
            });
        }

        private static void FillEightTransmitRange(byte[] raw, int channel, Complex[] range, double[] real, double[] imaginary)
        {
            int receiver = channel % 8, transmitter = channel / 8;
            for (int chirp = 0; chirp < 16; chirp++)
            {
                int column = 1024 + transmitter + 8 * (chirp + 16 * receiver);
                DecodeRangeFft(raw, column, real, imaginary);
                int target = chirp * RangeBins;
                for (int rangeBin = 0; rangeBin < RangeBins; rangeBin++)
                    range[target + rangeBin] = new Complex(real[rangeBin], imaginary[rangeBin]);
            }
        }

        private static void DecodeRangeFft(byte[] raw, int column, double[] real, double[] imaginary)
        {
            int offset = column * PayloadBytes;
            for (int sample = 0; sample < Samples; sample++)
            {
                int index = offset + sample * 2;
                real[sample] = ((raw[index] | raw[index + 1] << 8) - 2048) * Hamming(sample, Samples);
                imaginary[sample] = 0;
            }
            RadarPreviewProcessor.Fft(real, imaginary);
        }

        private static void BuildVirtualArray(out int[] map, out double[] x, out double[] z)
        {
            int[] receiverX = { 0, -4, -8, -12, -16, -19, -21, -24 };
            int[] transmitterX = { 36, 27, 18, 9, 0, -24, -48, -72 };
            int[] transmitterZ = { 32, 24, 16, 8, 0, 0, 0, 0 };
            map = new int[64];
            Dictionary<string, int> unique = new Dictionary<string, int>();
            List<double> xs = new List<double>(), zs = new List<double>();
            for (int transmitter = 0; transmitter < 8; transmitter++)
                for (int receiver = 0; receiver < 8; receiver++)
                {
                    int px = transmitterX[transmitter] + receiverX[receiver], pz = transmitterZ[transmitter];
                    string key = px + "," + pz;
                    int index;
                    if (!unique.TryGetValue(key, out index))
                    {
                        index = unique.Count; unique.Add(key, index); xs.Add(px * 0.001); zs.Add(pz * 0.001);
                    }
                    map[transmitter * 8 + receiver] = index;
                }
            x = xs.ToArray(); z = zs.ToArray();
        }

        private static double RectangleSum(double[] integral, int width, int r1, int d1, int r2, int d2)
        {
            return integral[(r2 + 1) * width + d2 + 1] - integral[r1 * width + d2 + 1] -
                   integral[(r2 + 1) * width + d1] + integral[r1 * width + d1];
        }

        private static double Hamming(int index, int count)
        {
            return 0.54 - 0.46 * Math.Cos(2.0 * Math.PI * index / (count - 1));
        }

        private static double Hypot(double real, double imaginary)
        {
            return Math.Sqrt(real * real + imaginary * imaginary);
        }

        public void Dispose()
        {
            _stop = true;
            lock (_gate) Monitor.PulseAll(_gate);
            if (_worker.IsAlive) _worker.Join(2000);
        }
    }
}
