using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Threading;

namespace Acquire
{
    internal static class PreviewStressTest
    {
        private sealed class Stamp { public long PublishedTicks; }

        public static int Run(int seconds)
        {
            ConsoleIo.Ensure();
            seconds = Math.Max(1, seconds);
            LatestPreview<Stamp>[] streams =
            {
                new LatestPreview<Stamp>(), new LatestPreview<Stamp>(),
                new LatestPreview<Stamp>(), new LatestPreview<Stamp>()
            };
            int[] periods = { 33, 33, 100, 50 };
            long[] published = new long[4];
            long[] consumed = new long[4];
            List<double> latencies = new List<double>();
            object latencyGate = new object();
            volatile_box stop = new volatile_box();
            Thread[] producers = new Thread[4];
            for (int stream = 0; stream < streams.Length; stream++)
            {
                int index = stream;
                producers[stream] = new Thread(delegate()
                {
                    Stopwatch pacer = Stopwatch.StartNew();
                    double due = 0;
                    while (!stop.Value)
                    {
                        double now = pacer.Elapsed.TotalMilliseconds;
                        if (now < due) Thread.Sleep(Math.Max(1, (int)(due - now)));
                        streams[index].Publish(new Stamp { PublishedTicks = Stopwatch.GetTimestamp() });
                        Interlocked.Increment(ref published[index]);
                        due += periods[index];
                        if (pacer.Elapsed.TotalMilliseconds > due + periods[index]) due = pacer.Elapsed.TotalMilliseconds;
                    }
                }) { IsBackground = true, Name = "Preview stress producer " + index };
                producers[stream].Start();
            }

            Stopwatch test = Stopwatch.StartNew();
            while (test.Elapsed.TotalSeconds < seconds)
            {
                Thread.Sleep(100);
                for (int i = 0; i < streams.Length; i++)
                {
                    Stamp stamp = streams[i].Take();
                    if (stamp == null) continue;
                    Interlocked.Increment(ref consumed[i]);
                    double latency = (Stopwatch.GetTimestamp() - stamp.PublishedTicks) * 1000.0 / Stopwatch.Frequency;
                    lock (latencyGate) latencies.Add(latency);
                }
            }
            stop.Value = true;
            foreach (Thread thread in producers) thread.Join(1000);
            latencies.Sort();
            double p95 = latencies.Count == 0 ? double.NaN : latencies[(int)Math.Min(latencies.Count - 1, Math.Ceiling(latencies.Count * 0.95) - 1)];
            long totalPublished = 0, totalOverwritten = 0;
            for (int i = 0; i < streams.Length; i++) { totalPublished += published[i]; totalOverwritten += streams[i].Overwritten; }
            double overwriteRate = totalPublished == 0 ? 0 : totalOverwritten / (double)totalPublished;
            ConsoleIo.WriteLine("PREVIEW STRESS PASSED");
            ConsoleIo.WriteLine("durationSeconds=" + test.Elapsed.TotalSeconds.ToString("0.0", CultureInfo.InvariantCulture));
            ConsoleIo.WriteLine("published=" + totalPublished + " consumed=" + (consumed[0] + consumed[1] + consumed[2] + consumed[3]));
            ConsoleIo.WriteLine("previewOverwriteRate=" + overwriteRate.ToString("0.000000", CultureInfo.InvariantCulture));
            ConsoleIo.WriteLine("p95LatencyMs=" + p95.ToString("0.000", CultureInfo.InvariantCulture));
            ConsoleIo.WriteLine("note=覆盖表示 UI 只取最新帧，不代表原始采集丢帧");
            return double.IsNaN(p95) || p95 > 200 ? 1 : 0;
        }

        private sealed class volatile_box { public volatile bool Value; }
    }
}
