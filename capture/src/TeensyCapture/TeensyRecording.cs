using System.Diagnostics;
using System.Threading.Channels;
using Apache.Arrow;
using Apache.Arrow.Types;
using CaptureCore;

namespace TeensyCapture;

/// <summary>
/// Saves the Teensy stream into a scan folder: IMU and step samples as <c>rig_0001.arrow</c>, ..., index edges as
/// <c>edges_0001.arrow</c>, ... and the text messages (with the PC time they arrived) in <c>rig_messages.log</c>.
/// Packets go through a queue to a writer thread, so the serial reader never waits for the disk.
/// </summary>
public sealed class TeensyRecording : IDisposable
{
    private const int SampleBatch = 50; // 0.5 s

    private static readonly TimestampType TimeType = new(TimeUnit.Nanosecond, "UTC");

    private readonly Channel<object> _queue = Channel.CreateBounded<object>(new BoundedChannelOptions(20_000)
    {
        SingleReader = true,
        FullMode = BoundedChannelFullMode.DropOldest,
    });

    private readonly ChunkedStreamWriter _samples;
    private readonly ChunkedStreamWriter _edges;
    private readonly StreamWriter _log;
    private readonly IReadOnlyDictionary<string, string> _extra;
    private readonly Task _writer;
    private TeensyHeader? _header;

    public long SamplesWritten { get; private set; }
    public long EdgesWritten { get; private set; }
    public long BytesWritten => _samples.BytesWritten + _edges.BytesWritten;

    public TeensyRecording(string directory, double chunkSeconds, IReadOnlyDictionary<string, string>? extraMetadata = null)
    {
        Directory.CreateDirectory(directory);
        _extra = extraMetadata ?? new Dictionary<string, string>();
        _samples = new ChunkedStreamWriter(directory, "rig", chunkSeconds, i => SampleSchema(i));
        _edges = new ChunkedStreamWriter(directory, "edges", chunkSeconds, i => EdgeSchema(i));
        _log = new StreamWriter(Path.Combine(directory, "rig_messages.log"), append: true) { AutoFlush = true };
        _writer = Task.Run(WriteLoop);
    }

    // called on the serial reader thread, must not block
    internal void Add(TeensySample s) => _queue.Writer.TryWrite(s);
    internal void Add(TeensyEdge e) => _queue.Writer.TryWrite(e);
    internal void Add(TeensyHeader h) => _queue.Writer.TryWrite(h);
    internal void AddMessage(string text) => _queue.Writer.TryWrite(new PcMessage(DateTime.UtcNow, text));

    private sealed record PcMessage(DateTime Utc, string Text);

    private void WriteLoop()
    {
        var samples = new List<TeensySample>(SampleBatch);
        var edges = new List<TeensyEdge>();
        var lastFlush = Stopwatch.StartNew();

        void FlushSamples()
        {
            if (samples.Count > 0) { WriteSamples(samples); SamplesWritten += samples.Count; samples.Clear(); }
            if (edges.Count > 0) { WriteEdges(edges); EdgesWritten += edges.Count; edges.Clear(); }
            lastFlush.Restart();
        }

        var reader = _queue.Reader;
        while (reader.WaitToReadAsync().AsTask().GetAwaiter().GetResult())
        {
            while (reader.TryRead(out var item))
            {
                switch (item)
                {
                    case TeensySample s: samples.Add(s); break;
                    case TeensyEdge e: edges.Add(e); break;
                    case TeensyHeader h:
                        _header = h;
                        _log.WriteLine($"{DateTime.UtcNow:O} [header] version {h.Version}, {h.StepsPerRev} steps/rev, edge-to-vertical {h.VerticalAfterEdgeDeg} deg, {h.Rpm} rpm, first pulse {h.FirstPulseUnixS}");
                        break;
                    case PcMessage m: _log.WriteLine($"{m.Utc:O} {m.Text}"); break;
                }
                // The first chunk takes its schema metadata (steps per rev, offset to vertical, ...) from the stream header,
                // which arrives about 10 samples after the first one. The time trigger would flush a single sample before
                // that (the recording is created more than 1 s before the first sample), so it waits for the header.
                if (samples.Count >= SampleBatch || (lastFlush.Elapsed.TotalSeconds >= 1 && _header is not null)) FlushSamples();
            }
        }
        FlushSamples();
    }

    private void WriteSamples(List<TeensySample> list)
    {
        int n = list.Count;
        var time = new long[n]; var steps = new int[n];
        var qw = new float[n]; var qx = new float[n]; var qy = new float[n]; var qz = new float[n];
        var qacc = new ushort[n]; var cal = new byte[n];
        var lax = new float[n]; var lay = new float[n]; var laz = new float[n];
        var mx = new float[n]; var my = new float[n]; var mz = new float[n];
        var flags = new byte[n]; var drops = new byte[n];
        var gx = new float[n]; var gy = new float[n]; var gz = new float[n];
        var cal2 = new byte[n];

        for (int i = 0; i < n; i++)
        {
            var s = list[i];
            time[i] = s.TimeUs * 1000L; steps[i] = s.AxisSteps;
            qw[i] = s.Qw; qx[i] = s.Qx; qy[i] = s.Qy; qz[i] = s.Qz;
            qacc[i] = s.QAcc; cal[i] = s.Cal;
            lax[i] = s.Lax; lay[i] = s.Lay; laz[i] = s.Laz;
            mx[i] = s.Mx; my[i] = s.My; mz[i] = s.Mz;
            flags[i] = s.Flags; drops[i] = s.Drops;
            gx[i] = s.Gx; gy[i] = s.Gy; gz[i] = s.Gz;
            cal2[i] = s.Cal2;
        }

        static ArrowBuffer B<T>(T[] a, int n) where T : unmanaged => ChunkedStreamWriter.Buffer(a, n);
        _samples.Write(schema => new RecordBatch(schema, new IArrowArray[]
        {
            new TimestampArray(TimeType, B(time, n), ArrowBuffer.Empty, n, 0, 0),
            new Int32Array(B(steps, n), ArrowBuffer.Empty, n, 0, 0),
            new FloatArray(B(qw, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(qx, n), ArrowBuffer.Empty, n, 0, 0),
            new FloatArray(B(qy, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(qz, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt16Array(B(qacc, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(B(cal, n), ArrowBuffer.Empty, n, 0, 0),
            new FloatArray(B(lax, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(lay, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(laz, n), ArrowBuffer.Empty, n, 0, 0),
            new FloatArray(B(mx, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(my, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(mz, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(B(flags, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(B(drops, n), ArrowBuffer.Empty, n, 0, 0),
            new FloatArray(B(gx, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(gy, n), ArrowBuffer.Empty, n, 0, 0), new FloatArray(B(gz, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(B(cal2, n), ArrowBuffer.Empty, n, 0, 0),
        }, n));
    }

    private void WriteEdges(List<TeensyEdge> list)
    {
        int n = list.Count;
        var time = new long[n]; var steps = new int[n];
        for (int i = 0; i < n; i++) { time[i] = list[i].TimeUs * 1000L; steps[i] = list[i].StepsBeforeReset; }
        _edges.Write(schema => new RecordBatch(schema, new IArrowArray[]
        {
            new TimestampArray(TimeType, ChunkedStreamWriter.Buffer(time, n), ArrowBuffer.Empty, n, 0, 0),
            new Int32Array(ChunkedStreamWriter.Buffer(steps, n), ArrowBuffer.Empty, n, 0, 0),
        }, n));
    }

    private Dictionary<string, string> Metadata(int chunk, string kind, string columns)
    {
        var m = new Dictionary<string, string>
        {
            ["format"] = kind == "rig" ? "cave_lidar.rig/2" : $"cave_lidar.{kind}/1",   // rig/2: gyroscope appended
            ["sensor"] = "Teensy 4.1 + BNO085 + step counter",
            ["time_unit"] = "ns since 1970-01-01T00:00:00 UTC (simulated UTC of the rig, the clock the lidar is locked to)",
            ["columns"] = columns,
            ["chunk_index"] = chunk.ToString(),
            ["chunk_created_utc"] = DateTime.UtcNow.ToString("O"),
        };
        if (_header is { } h)
        {
            m["firmware_protocol_version"] = h.Version.ToString();
            m["steps_per_rev"] = h.StepsPerRev.ToString(System.Globalization.CultureInfo.InvariantCulture);
            m["vertical_after_edge_deg"] = h.VerticalAfterEdgeDeg.ToString(System.Globalization.CultureInfo.InvariantCulture);
            m["rpm"] = h.Rpm.ToString(System.Globalization.CultureInfo.InvariantCulture);
            m["first_pulse_unix_s"] = h.FirstPulseUnixS.ToString();
        }
        foreach (var (k, v) in _extra) m[k] = v;
        return m;
    }

    private Schema SampleSchema(int chunk)
    {
        var sb = new Schema.Builder()
            .Field(new Field("time", TimeType, false))
            .Field(new Field("axis_steps", Int32Type.Default, false))
            .Field(new Field("qw", FloatType.Default, false)).Field(new Field("qx", FloatType.Default, false))
            .Field(new Field("qy", FloatType.Default, false)).Field(new Field("qz", FloatType.Default, false))
            .Field(new Field("q_acc", UInt16Type.Default, false))
            .Field(new Field("cal", UInt8Type.Default, false))
            .Field(new Field("lax", FloatType.Default, false)).Field(new Field("lay", FloatType.Default, false)).Field(new Field("laz", FloatType.Default, false))
            .Field(new Field("mx", FloatType.Default, false)).Field(new Field("my", FloatType.Default, false)).Field(new Field("mz", FloatType.Default, false))
            .Field(new Field("flags", UInt8Type.Default, false))
            .Field(new Field("drops", UInt8Type.Default, false))
            .Field(new Field("gx", FloatType.Default, false)).Field(new Field("gy", FloatType.Default, false)).Field(new Field("gz", FloatType.Default, false))
            .Field(new Field("cal2", UInt8Type.Default, false));
        foreach (var (k, v) in Metadata(chunk, "rig", "axis_steps: steps since the last index edge; q*: IMU orientation (real part first); q_acc: 0.01 deg; cal: bits 0-1 rotation vector, bits 2-3 magnetometer calibration; la*: linear acceleration m/s^2 (NaN = not yet delivered); m*: magnetic field uT; flags: bit0 index edge seen, bit1 running; drops: wrapping counter of packets the Teensy dropped; g*: calibrated gyroscope angular velocity rad/s, sensor frame (NaN = not delivered, e.g. firmware protocol 1); cal2: bits 0-1 gyroscope calibration status. The values of the other reports are the latest delivered ones, up to 10 ms old"))
            sb.Metadata(k, v);
        return sb.Build();
    }

    private Schema EdgeSchema(int chunk)
    {
        var sb = new Schema.Builder()
            .Field(new Field("time", TimeType, false))
            .Field(new Field("steps_before_reset", Int32Type.Default, false));
        foreach (var (k, v) in Metadata(chunk, "edges", "steps_before_reset: step counter just before it was zeroed (about steps_per_rev without slip, meaningless for the first edge)"))
            sb.Metadata(k, v);
        return sb.Build();
    }

    /// <summary>Waits until everything queued is written. Call <see cref="TeensyLink.Attach"/> with null first.</summary>
    public void Dispose()
    {
        _queue.Writer.TryComplete();
        _writer.GetAwaiter().GetResult();
        _samples.Dispose();
        _edges.Dispose();
        _log.Dispose();
    }
}
