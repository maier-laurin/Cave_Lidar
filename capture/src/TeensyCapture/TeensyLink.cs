using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;

namespace TeensyCapture;

/// <summary>Live state of the rig, updated by the reader thread. Copy it with <see cref="TeensyLink.Snapshot"/>.</summary>
public sealed record RigState
{
    public TeensySample? LastSample { get; init; }
    public TeensyHeader? Header { get; init; }
    public string LastMessage { get; init; } = "";
    public long Samples { get; init; }
    public long Edges { get; init; }
    public long UsbDrops { get; init; }   // packets the Teensy had to drop, from the wrapping counter in the samples
    public long CrcErrors { get; init; }
    public double SampleRateHz { get; init; }
    public double SecondsSinceSample { get; init; } = double.PositiveInfinity;
}

/// <summary>
/// Serial connection to the Teensy. The port stays open and is read all the time (so the live state is available
/// before a scan starts); a <see cref="TeensyRecording"/> can be attached to save the stream.
/// </summary>
public sealed class TeensyLink : IDisposable
{
    private readonly SerialPort _port;
    private readonly TeensyPacketParser _parser = new();
    private readonly object _gate = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private Thread? _thread;
    private volatile bool _stop;

    private TeensySample? _last;
    private TeensyHeader? _header;
    private string _message = "";
    private long _samples, _edges, _drops;
    private long _lastSampleTicks;
    private int _lastDropByte = -1;
    private long _rateCount;
    private long _rateTicks = Stopwatch.GetTimestamp();
    private double _rate;

    private volatile TeensyRecording? _recording;
    private TaskCompletionSource<string>? _waiter;
    private Func<string, bool>? _waitFor;

    /// <summary>Raised for every text message of the firmware, on the reader thread.</summary>
    public event Action<string>? MessageReceived;

    public string PortName => _port.PortName;

    public TeensyLink(string portName)
    {
        _port = new SerialPort(portName, 115200) { DtrEnable = true, ReadTimeout = 200, WriteTimeout = 1000 };
        _parser.OnSample = HandleSample;
        _parser.OnEdge = e => { lock (_gate) _edges++; _recording?.Add(e); };
        _parser.OnHeader = h => { lock (_gate) _header = h; _recording?.Add(h); };
        _parser.OnMessage = HandleMessage;
    }

    public void Open()
    {
        _port.Open();
        _port.DiscardInBuffer();
        _thread = new Thread(ReadLoop) { Name = "teensy-serial", IsBackground = true };
        _thread.Start();
    }

    public void Send(string line) => _port.Write(line.EndsWith('\n') ? line : line + "\n");

    /// <summary>Starts the rig: <c>C,rpm,label</c>. The first PPS pulse follows one second later and carries the label.</summary>
    public void Configure(double rpm, long labelUnixSeconds) =>
        Send(string.Create(CultureInfo.InvariantCulture, $"C,{rpm},{labelUnixSeconds}"));

    public void Stop() => Send("x");

    /// <summary>Starts or stops saving the stream. Only one recording can be attached at a time.</summary>
    public void Attach(TeensyRecording? recording) => _recording = recording;

    /// <summary>Waits for a text message of the firmware that matches, e.g. "Config ok". Returns null on timeout.</summary>
    public async Task<string?> WaitForMessageAsync(Func<string, bool> match, TimeSpan timeout, CancellationToken ct = default)
    {
        var tcs = new TaskCompletionSource<string>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_gate) { _waiter = tcs; _waitFor = match; }
        try
        {
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            cts.CancelAfter(timeout);
            using (cts.Token.Register(() => tcs.TrySetCanceled()))
                return await tcs.Task;
        }
        catch (OperationCanceledException) { return null; }
        finally { lock (_gate) { _waiter = null; _waitFor = null; } }
    }

    /// <summary>
    /// Rough current time in simulated UTC ns since 1970: the latest sample plus the time since it arrived. Wrong by the
    /// USB latency only (milliseconds) and without any drift, since it is the clock the lidar is locked to. Null when no sample has arrived yet.
    /// </summary>
    public long? EstimateNowUnixNs()
    {
        long t, ticks;
        lock (_gate)
        {
            if (_last is null) return null;
            t = _last.Value.TimeUs;
            ticks = _lastSampleTicks;
        }
        return t * 1000L + (long)((Stopwatch.GetTimestamp() - ticks) * (1e9 / Stopwatch.Frequency));
    }

    public RigState Snapshot()
    {
        lock (_gate)
        {
            return new RigState
            {
                LastSample = _last,
                Header = _header,
                LastMessage = _message,
                Samples = _samples,
                Edges = _edges,
                UsbDrops = _drops,
                CrcErrors = _parser.CrcErrors,
                SampleRateHz = _rate,
                SecondsSinceSample = _last is null ? double.PositiveInfinity : (Stopwatch.GetTimestamp() - _lastSampleTicks) / (double)Stopwatch.Frequency,
            };
        }
    }

    private void HandleSample(TeensySample s)
    {
        long now = Stopwatch.GetTimestamp();
        lock (_gate)
        {
            _last = s;
            _lastSampleTicks = now;
            _samples++;
            if (_lastDropByte >= 0) _drops += (s.Drops - _lastDropByte + 256) % 256;
            _lastDropByte = s.Drops;

            _rateCount++;
            double dt = (now - _rateTicks) / (double)Stopwatch.Frequency;
            if (dt >= 1)
            {
                _rate = _rateCount / dt;
                _rateCount = 0;
                _rateTicks = now;
            }
        }
        _recording?.Add(s);
    }

    private void HandleMessage(string text)
    {
        TaskCompletionSource<string>? hit = null;
        lock (_gate)
        {
            _message = text;
            if (_waiter is not null && _waitFor!(text)) hit = _waiter;
        }
        hit?.TrySetResult(text);
        _recording?.AddMessage(text);
        MessageReceived?.Invoke(text);
    }

    private void ReadLoop()
    {
        var buf = new byte[8192];
        while (!_stop)
        {
            try
            {
                int n = _port.Read(buf, 0, buf.Length);
                if (n > 0) _parser.Feed(buf.AsSpan(0, n));
            }
            catch (TimeoutException) { }
            catch (Exception e) when (!_stop)
            {
                HandleMessage($"[serial error] {e.Message}");
                return;
            }
        }
    }

    public void Dispose()
    {
        _stop = true;
        _thread?.Join(1000);
        if (_port.IsOpen) _port.Close();
        _port.Dispose();
    }
}
