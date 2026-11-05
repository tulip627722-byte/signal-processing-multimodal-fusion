using System;
using System.Collections.Generic;
using System.IO;
using System.Media;
using System.Text;
using System.Threading;

namespace Acquire
{
    internal enum CueKind
    {
        /// <summary>预备/倒计时结束、开始记录的那一刻：短促两声。</summary>
        Start,
        /// <summary>拍摄结束那一刻：往下一声。</summary>
        End
    }

    /// <summary>
    /// 采集提示音：只在预备结束和拍摄结束各响一声。
    /// 波形在内存里合成成 16 bit / 44.1 kHz 单声道 WAV，用 SoundPlayer.Play()
    /// 异步播放（内部就是 winmm PlaySound + SND_ASYNC），所以不会阻塞采集线程，
    /// 也不落任何文件、不参与 take 数据。
    /// 想在现场换音色又不想重编译：把 cue-start.wav / cue-end.wav
    /// 放在 exe 同目录即可覆盖默认音。
    /// </summary>
    internal sealed class CuePlayer : IDisposable
    {
        private const int SampleRate = 44100;

        private readonly Dictionary<CueKind, SoundPlayer> _players = new Dictionary<CueKind, SoundPlayer>();
        private readonly List<IDisposable> _streams = new List<IDisposable>();
        private readonly object _gate = new object();
        private bool _disposed;

        public CuePlayer(Action<string> log)
        {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            Build(CueKind.Start, Path.Combine(dir, "cue-start.wav"), log, StartShape);
            Build(CueKind.End, Path.Combine(dir, "cue-end.wav"), log, EndShape);
            if (log != null) log("提示音已就绪：开始一声 / 结束一声");
        }

        /// <summary>非阻塞播放；出任何问题都只是没声音，绝不影响采集。</summary>
        public void Play(CueKind kind)
        {
            SoundPlayer player;
            lock (_gate)
            {
                if (_disposed) return;
                if (!_players.TryGetValue(kind, out player)) return;
            }
            try { player.Play(); } catch { }
        }

        /// <summary>立刻停止并释放（会掐掉正在播放的音）。</summary>
        public void Dispose()
        {
            DisposeNow();
        }

        /// <summary>延迟释放：收尾音还在响的时候不要把它掐断，交给后台线程等一会儿再关。</summary>
        public void DisposeSoon(int delayMs)
        {
            if (delayMs <= 0)
            {
                DisposeNow();
                return;
            }
            Thread worker = new Thread(delegate()
            {
                Thread.Sleep(delayMs);
                DisposeNow();
            });
            worker.IsBackground = true;
            worker.Name = "Acquire cue dispose";
            worker.Start();
        }

        private void DisposeNow()
        {
            lock (_gate)
            {
                if (_disposed) return;
                _disposed = true;
                foreach (SoundPlayer player in _players.Values)
                {
                    try { player.Stop(); } catch { }
                    try { player.Dispose(); } catch { }
                }
                _players.Clear();
                foreach (IDisposable stream in _streams)
                {
                    try { stream.Dispose(); } catch { }
                }
                _streams.Clear();
            }
        }

        // ------------------------------------------------------------ 装配

        private void Build(CueKind kind, string overridePath, Action<string> log, Func<Tone[]> shape)
        {
            try
            {
                SoundPlayer player;
                if (File.Exists(overridePath))
                {
                    player = new SoundPlayer(overridePath);
                    // 立刻加载进内存，现场播放时不再碰磁盘
                    player.Load();
                    if (log != null) log("提示音 " + kind + " 使用外部文件 " + Path.GetFileName(overridePath));
                }
                else
                {
                    MemoryStream stream = new MemoryStream(BuildWav(shape()));
                    player = new SoundPlayer(stream);
                    player.Load();
                    _streams.Add(stream);
                }
                _players[kind] = player;
            }
            catch (Exception ex)
            {
                if (log != null) log("提示音 " + kind + " 不可用（" + ex.Message + "），这一类不出声");
            }
        }

        // ------------------------------------------------------------ 音型

        private struct Tone
        {
            public double Freq;
            public double Seconds;
            public double Gain;
        }

        private static Tone[] StartShape()
        {
            // 上行两音：像"预备——走"
            return new[]
            {
                new Tone { Freq = 880.0, Seconds = 0.07, Gain = 0.30 },
                new Tone { Freq = 0.0, Seconds = 0.02, Gain = 0.0 },
                new Tone { Freq = 1320.0, Seconds = 0.14, Gain = 0.30 }
            };
        }

        private static Tone[] EndShape()
        {
            // 下行两音：落定
            return new[]
            {
                new Tone { Freq = 1046.5, Seconds = 0.10, Gain = 0.26 },
                new Tone { Freq = 0.0, Seconds = 0.02, Gain = 0.0 },
                new Tone { Freq = 784.0, Seconds = 0.16, Gain = 0.26 }
            };
        }

        // ------------------------------------------------------------ 合成

        /// <summary>
        /// 生成一段 16 bit 单声道 PCM WAV。每个音两头带升余弦淡入淡出，
        /// 否则起落那一下会"啪"一声（那是方波边沿，不是设计里的音）。
        /// </summary>
        private static byte[] BuildWav(Tone[] tones)
        {
            int[] counts = new int[tones.Length];
            int total = 0;
            for (int t = 0; t < tones.Length; t++)
            {
                counts[t] = Math.Max(1, (int)Math.Round(tones[t].Seconds * SampleRate));
                total += counts[t];
            }

            short[] samples = new short[total];
            int pos = 0;
            for (int t = 0; t < tones.Length; t++)
            {
                Tone tone = tones[t];
                int n = counts[t];
                int attack = Math.Min(n / 2, Math.Max(1, (int)(0.004 * SampleRate)));
                int release = Math.Min(n - attack, Math.Max(1, (int)(0.014 * SampleRate)));
                for (int i = 0; i < n; i++)
                {
                    double env = 1.0;
                    if (i < attack) env = 0.5 - 0.5 * Math.Cos(Math.PI * i / attack);
                    int tail = n - 1 - i;
                    if (tail < release)
                    {
                        double down = 0.5 - 0.5 * Math.Cos(Math.PI * tail / release);
                        if (down < env) env = down;
                    }

                    double value = 0.0;
                    if (tone.Freq > 0.0)
                    {
                        double phase = 2.0 * Math.PI * tone.Freq * i / SampleRate;
                        // 叠一点二次谐波，小喇叭上更容易听清
                        value = Math.Sin(phase) + 0.18 * Math.Sin(2.0 * phase);
                    }

                    double v = value * env * tone.Gain;
                    if (v > 1.0) v = 1.0;
                    if (v < -1.0) v = -1.0;
                    samples[pos + i] = (short)Math.Round(v * 32000.0);
                }
                pos += n;
            }

            using (MemoryStream ms = new MemoryStream(44 + samples.Length * 2))
            using (BinaryWriter w = new BinaryWriter(ms))
            {
                int dataBytes = samples.Length * 2;
                w.Write(Encoding.ASCII.GetBytes("RIFF"));
                w.Write(36 + dataBytes);
                w.Write(Encoding.ASCII.GetBytes("WAVE"));
                w.Write(Encoding.ASCII.GetBytes("fmt "));
                w.Write(16);                       // fmt 块长度
                w.Write((short)1);                 // PCM
                w.Write((short)1);                 // 单声道
                w.Write(SampleRate);
                w.Write(SampleRate * 2);           // 字节率
                w.Write((short)2);                 // 块对齐
                w.Write((short)16);                // 位深
                w.Write(Encoding.ASCII.GetBytes("data"));
                w.Write(dataBytes);
                for (int i = 0; i < samples.Length; i++) w.Write(samples[i]);
                w.Flush();
                return ms.ToArray();
            }
        }
    }
}
