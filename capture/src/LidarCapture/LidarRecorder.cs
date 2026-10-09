using System.Buffers;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Threading.Channels;

namespace LidarCapture;

public sealed class LidarRecorderOptions
{
    /// <summary>Folder of this scan, e.g. projects/my_cave/scan_01. Created if missing.</summary>
    public required string ScanDirectory { get; init; }

    /// <summary>
    /// Rough current UTC time in ns since 1970, only needs to be right to within +-0.5 s. It is sampled while the first
    /// packets arrive and decides the whole seconds of the lidar clock. Best is the rig's time label plus a stopwatch
    /// since the C command was sent. Default: the system clock.
    /// </summary>
    public Func<long>? TimeReferenceUnixNs { get; init; }

    /// <summary>UTC unix seconds that the first PPS pulse carried (label of the C command); only written to the metadata.</summary>
    public long? TimeLabelUnixSeconds { get; init; }

    public IPAddress LidarAddress { get; init; } = IPAddress.Parse("192.168.3.201");
    public IPAddress BindAddress { get; init; } = IPAddress.Any;
    public int Port { get; init; } = 3201;

    public double ChunkSeconds { get; init; } = 60;
    public int BatchPackets { get; init; } = 500;
    public int QueueCapacity { get; init; } = 20_000;
    public int SocketBufferBytes { get; init; } = 16 << 20;

    /// <summary>Added to the schema metadata of every file (project, rpm, firmware version, ...).</summary>
    public IReadOnlyDictionary<string, string> ExtraMetadata { get; init; } = new Dictionary<string, string>();

    /// <summary>Called for notable events (warnings, file changes).</summary>
    public Action<string>? Log { get; init; }
}

public sealed class LidarStats
{
    private long _received, _queueDropped, _rejected, _gapLost, _points;

    public long PacketsReceived => Interlocked.Read(ref _received);
    public long PacketsDroppedQueueFull => Interlocked.Read(ref _queueDropped);
    public long PacketsRejected => Interlocked.Read(ref _rejected);
    /// <summary>Packets missing according to gaps in the lidar timestamps (lost on the wire or in the socket buffer).</summary>
    public long PacketsLostEstimate => Interlocked.Read(ref _gapLost);
    public long PointsWritten => Interlocked.Read(ref _points);
    public long BytesWritten { get; internal set; }
    public int QueueDepth { get; internal set; }
    public int ChunkIndex { get; internal set; }

    /// <summary>
    /// Latest check of the lidar time against the reference clock (reference minus lidar), in ms. It should stay at a few
    /// ms; when it nears +-500 ms the whole seconds of the lidar clock would be wrong.
    /// </summary>
    public double ReferenceErrorMs { get; internal set; }
    public int ClockJumps { get; internal set; }

    internal void AddReceived() => Interlocked.Increment(ref _received);
    internal void AddQueueDropped() => Interlocked.Increment(ref _queueDropped);
    internal void AddRejected() => Interlocked.Increment(ref _rejected);
    internal void AddGapLost(long n) => Interlocked.Add(ref _gapLost, n);
    internal void AddPoints(long n) => Interlocked.Add(ref _points, n);
}

/// <summary>
/// Receives VLP-16 UDP packets and writes them as Arrow IPC stream files. One thread only listens on the socket and
/// queues packets; a second one parses them and writes, so slow disk writes do not drop packets.
/// </summary>
public sealed class LidarRecorder : IAsyncDisposable
{
    private readonly LidarRecorderOptions _o;
    private readonly Channel<(byte[] Buffer, int Length)> _queue;
    private readonly CancellationTokenSource _stop = new();

    private Thread? _receiver;
    private Task? _consumer;

    public LidarStats Stats { get; } = new();

    public LidarRecorder(LidarRecorderOptions options)
    {
        _o = options;
        _queue = Channel.CreateBounded<(byte[], int)>(new BoundedChannelOptions(options.QueueCapacity)
        {
            SingleReader = true,
            SingleWriter = true,
            FullMode = BoundedChannelFullMode.Wait,
        });
    }

    /// <summary>Opens the socket (throws if the port is taken) and starts both threads.</summary>
    public void Start()
    {
        var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp)
        {
            ReceiveBufferSize = _o.SocketBufferBytes,
        };
        try
        {
            socket.Bind(new IPEndPoint(_o.BindAddress, _o.Port));
        }
        catch
        {
            socket.Dispose();
            throw;
        }

        _consumer = Task.Run(ConsumeAsync);
        _receiver = new Thread(() => Receive(socket)) { Name = "lidar-udp", IsBackground = true, Priority = ThreadPriority.AboveNormal };
        _receiver.Start();
        _o.Log?.Invoke($"Listening on {_o.BindAddress}:{_o.Port} for packets from {_o.LidarAddress}, writing to {_o.ScanDirectory}");
    }

    /// <summary>Stops listening, writes everything still queued and closes the file.</summary>
    public async Task StopAsync()
    {
        _stop.Cancel();
        _receiver?.Join();
        if (_consumer is not null) await _consumer;
    }

    public async ValueTask DisposeAsync()
    {
        await StopAsync();
        _stop.Dispose();
    }

    //---- producer ---------------------------------------------------------------

    private void Receive(Socket socket)
    {
        using (socket)
        {
            EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
            try
            {
                while (!_stop.IsCancellationRequested)
                {
                    if (!socket.Poll(200_000, SelectMode.SelectRead)) continue;

                    byte[] buf = ArrayPool<byte>.Shared.Rent(Vlp16.PacketSize + 2);
                    int len = socket.ReceiveFrom(buf, ref remote);

                    if (len != Vlp16.PacketSize || !((IPEndPoint)remote).Address.Equals(_o.LidarAddress))
                    {
                        ArrayPool<byte>.Shared.Return(buf);
                        continue; // position packets (554 bytes) and strangers
                    }

                    Stats.AddReceived();
                    if (!_queue.Writer.TryWrite((buf, len)))
                    {
                        Stats.AddQueueDropped();
                        ArrayPool<byte>.Shared.Return(buf);
                    }
                }
            }
            catch (Exception e)
            {
                _o.Log?.Invoke($"Receiver stopped: {e.Message}");
            }
            finally
            {
                _queue.Writer.Complete();
            }
        }
    }

    //---- consumer ---------------------------------------------------------------

    //---- consumer ---------------------------------------------------------------

    private const int CalibrationPackets = 50;

    private async Task ConsumeAsync()
    {
        var clock = new LidarClock();
        var batch = new PointBatch(_o.BatchPackets * Vlp16.PointsPerPacket);
        var reference = _o.TimeReferenceUnixNs ?? SystemClockNs;

        // the first packets only serve to find the whole-second offset of the lidar clock, then they are processed as usual
        var samples = new List<(uint Micros, long ReferenceUnixNs)>(CalibrationPackets);
        var pending = new List<byte[]>(CalibrationPackets);

        ChunkedArrowWriter? writer = null;
        int packetsInBatch = 0;
        uint lastMicros = 0;
        bool haveLast = false, warnedMode = false, warnedDrift = false;
        int checkCounter = 0;

        void Flush()
        {
            if (batch.Count == 0 || writer is null) return;
            int before = writer.ChunkIndex;
            long points = batch.Count;
            writer.Write(batch);
            Stats.AddPoints(points);
            Stats.BytesWritten = writer.BytesWritten;
            Stats.ChunkIndex = writer.ChunkIndex;
            if (writer.ChunkIndex != before) _o.Log?.Invoke($"Started lidar_{writer.ChunkIndex:D4}.arrow");
            batch.Clear();
            packetsInBatch = 0;
        }

        void Process(ReadOnlySpan<byte> p)
        {
            uint micros = Vlp16.TimestampMicros(p);
            long startNs = clock.ToUnixNs(micros);

            // packets arrive every 12 x 110.592 us, bigger steps in the lidar clock mean lost packets,
            // steps of more than 100 ms are clock adjustments (e.g. the lidar locking onto the PPS)
            if (haveLast)
            {
                long gap = (long)micros - lastMicros;
                if (gap > 0 && gap <= 100_000)
                {
                    long missed = (long)Math.Round(gap / (Vlp16.BlockCount * Vlp16.BlockNs / 1000.0)) - 1;
                    if (missed > 0) Stats.AddGapLost(missed);
                }
                else if (Math.Abs(gap) > 100_000 && Math.Abs(gap) < 1_800_000_000)
                {
                    Stats.ClockJumps++;
                    _o.Log?.Invoke($"Lidar clock jumped by {gap / 1000.0:F1} ms. Recording should start after the lidar has locked onto the PPS.");
                }
            }
            lastMicros = micros;
            haveLast = true;

            // keep checking the whole-second offset against the reference (about every 0.7 s), it must never drift away
            if (++checkCounter >= 500)
            {
                checkCounter = 0;
                double err = (reference() - startNs) / 1e6;
                Stats.ReferenceErrorMs = err;
                bool bad = Math.Abs(err) > 300;
                if (bad && !warnedDrift)
                    _o.Log?.Invoke($"WARNING: lidar time and reference clock differ by {err:F0} ms, the whole seconds of the lidar clock may be wrong (limit 500 ms).");
                warnedDrift = bad;
            }

            Vlp16Parser.Parse(p, startNs, batch);
            if (++packetsInBatch >= _o.BatchPackets)
            {
                Stats.QueueDepth = _queue.Reader.Count;
                Flush();
            }
        }

        void Calibrate()
        {
            clock.Calibrate(samples);
            Stats.ReferenceErrorMs = clock.ReferenceErrorMs;
            writer = new ChunkedArrowWriter(_o.ScanDirectory, _o.ChunkSeconds, BuildMetadata(clock));
            _o.Log?.Invoke($"Lidar clock offset {clock.OffsetSeconds} s, reference clock off by {clock.ReferenceErrorMs:F1} ms");
            if (Math.Abs(clock.ReferenceErrorMs) > 300)
                _o.Log?.Invoke("WARNING: the reference clock is far from the lidar's PPS phase, the whole second may be wrong (limit is 500 ms).");
            foreach (var old in pending) Process(old);
            pending.Clear();
            samples.Clear();
        }

        try
        {
            await foreach (var (buf, len) in _queue.Reader.ReadAllAsync())
            {
                try
                {
                    var p = buf.AsSpan(0, len);

                    if (Vlp16.ReturnMode(p) != Vlp16.ReturnStrongest || Vlp16.Model(p) != Vlp16.ModelVlp16)
                    {
                        Stats.AddRejected();
                        if (!warnedMode)
                        {
                            warnedMode = true;
                            _o.Log?.Invoke($"Packets rejected: return mode 0x{Vlp16.ReturnMode(p):X2} / model 0x{Vlp16.Model(p):X2}, only strongest return on a VLP-16 (0x37 / 0x22) is supported.");
                        }
                        continue;
                    }

                    if (!LidarClock.IsValidMicros(Vlp16.TimestampMicros(p))) { Stats.AddRejected(); continue; }

                    if (!clock.IsCalibrated)
                    {
                        samples.Add((Vlp16.TimestampMicros(p), reference()));
                        pending.Add(p.ToArray());
                        if (samples.Count >= CalibrationPackets) Calibrate();
                    }
                    else
                    {
                        Process(p);
                    }
                }
                finally
                {
                    ArrayPool<byte>.Shared.Return(buf);
                }
            }

            if (!clock.IsCalibrated && samples.Count > 0) Calibrate(); // very short recording
            Flush();
        }
        finally
        {
            writer?.Close();
            if (writer is not null) Stats.BytesWritten = writer.BytesWritten;
            writer?.Dispose();
        }

        _o.Log?.Invoke("Lidar recording finished");
    }

    private static long SystemClockNs() => (DateTime.UtcNow - DateTime.UnixEpoch).Ticks * 100;

    private Dictionary<string, string> BuildMetadata(LidarClock clock)
    {
        var m = new Dictionary<string, string>
        {
            ["format"] = "cave_lidar.points/1",
            ["sensor"] = "Velodyne VLP-16",
            ["return_mode"] = "strongest",
            ["lidar_ip"] = _o.LidarAddress.ToString(),
            ["udp_port"] = _o.Port.ToString(),
            ["time_unit"] = "ns since 1970-01-01T00:00:00 UTC, interpolated per point",
            ["time_source"] = "lidar sub-second phase (locked to the rig PPS) + whole seconds from a rough reference clock",
            ["time_offset_s"] = clock.OffsetSeconds.ToString(),
            ["time_reference_error_ms"] = clock.ReferenceErrorMs.ToString("F1"),
            ["azimuth_unit"] = "0.01 deg, sensor frame, interpolated per point",
            ["elevation_unit"] = "deg, from the laser id",
            ["distance_unit"] = "2 mm",
            ["reflectivity_unit"] = "raw 0-255",
            ["dropped_points"] = "distance = 0 (no return)",
            ["software_version"] = typeof(LidarRecorder).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion ?? "unknown",
        };
        if (_o.TimeLabelUnixSeconds is { } label) m["time_label_unix_s"] = label.ToString();
        foreach (var (k, v) in _o.ExtraMetadata) m[k] = v;
        return m;
    }
}
