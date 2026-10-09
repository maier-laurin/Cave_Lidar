using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Reflection;
using System.Text.Json;
using LidarCapture;
using TeensyCapture;

namespace CaveScan;

public enum Phase { Idle, Starting, Scanning, Stopping }

public sealed class OrchestratorOptions
{
    public required string ProjectName { get; init; }
    public required string ProjectDirectory { get; init; }
    public double Rpm { get; init; } = 6;
    public IPAddress LidarAddress { get; init; } = IPAddress.Parse("192.168.3.201");
    public int LidarPort { get; init; } = 3201;
    public double ChunkSeconds { get; init; } = 60;

    /// <summary>
    /// Minimum time between the C command and the start of the lidar recording. The lidar needs about 3 s to lock onto
    /// the PPS and its clock jumps then; the shaft starts turning 10 s after the C command.
    /// </summary>
    public double LockWaitSeconds { get; init; } = 8;
}

/// <summary>
/// Runs a scan: starts the rig, waits for the lidar to lock, records lidar and Teensy side by side into
/// <c>&lt;project&gt;/scan_NN/</c>, and stops everything in the right order.
/// </summary>
public sealed class Orchestrator
{
    private readonly OrchestratorOptions _o;
    private readonly TeensyLink _rig;
    private readonly LidarStatusPoller _lidarStatus;
    private readonly object _gate = new();
    private readonly List<string> _log = new();

    private TeensyRecording? _recording;
    private volatile bool _forceStop;
    private string? _scanDir;
    private long _label;
    private DateTime _startedUtc;

    public Orchestrator(OrchestratorOptions options, TeensyLink rig, LidarStatusPoller lidarStatus)
    {
        _o = options;
        _rig = rig;
        _lidarStatus = lidarStatus;
        _rig.MessageReceived += m => Log($"rig: {m}");
    }

    public OrchestratorOptions Options => _o;
    public Phase Phase { get; private set; } = Phase.Idle;
    public string PhaseText { get; private set; } = "";
    public string? ScanName { get; private set; }
    public Stopwatch ScanTime { get; } = new();
    public LidarRecorder? Lidar { get; private set; }
    public TeensyRecording? Recording => _recording;
    public TeensyLink Rig => _rig;
    public LidarStatusPoller LidarStatus => _lidarStatus;

    public IReadOnlyList<string> RecentLog(int count)
    {
        lock (_gate) return _log.TakeLast(count).ToArray();
    }

    public void Log(string text)
    {
        string line = $"{DateTime.UtcNow:HH:mm:ss} {text}";
        lock (_gate)
        {
            _log.Add(line);
            if (_log.Count > 500) _log.RemoveRange(0, 100);
            if (_scanDir is not null)
            {
                try { File.AppendAllText(Path.Combine(_scanDir, "orchestrator.log"), line + Environment.NewLine); }
                catch (IOException) { }
            }
        }
    }

    //---- start ------------------------------------------------------------------

    public async Task StartScanAsync()
    {
        if (Phase != Phase.Idle) { Log("A scan is already running."); return; }
        Phase = Phase.Starting;
        _forceStop = false;
        bool configSent = false;

        try
        {
            PhaseText = "checking the rig";
            // the firmware only streams after a C command, so a silent rig is normal here; it proves it is alive by answering C
            var state = _rig.Snapshot();
            if (state.LastSample is { Running: true } && state.SecondsSinceSample < 2) throw new InvalidOperationException("The rig is still turning. Stop it first (x).");

            int number = ProjectStore.NextScanNumber(_o.ProjectDirectory);
            ScanName = $"scan_{number:D2}";
            _scanDir = ProjectStore.ScanDirectory(_o.ProjectDirectory, number);
            Directory.CreateDirectory(_scanDir);
            _startedUtc = DateTime.UtcNow;
            ScanTime.Restart();
            Log($"Starting {ScanName} in {_scanDir}");

            var extra = new Dictionary<string, string> { ["project"] = _o.ProjectName, ["scan"] = ScanName };
            _recording = new TeensyRecording(_scanDir, _o.ChunkSeconds, extra);
            _rig.Attach(_recording);

            // the rig clock starts with the first PPS pulse, one second after the C line, and carries the label
            PhaseText = "starting the rig";
            _label = DateTimeOffset.UtcNow.ToUnixTimeSeconds() + 1;
            var reply = _rig.WaitForMessageAsync(m => m.StartsWith("Config ok") || m.StartsWith("C rejected"), TimeSpan.FromSeconds(4));
            _rig.Configure(_o.Rpm, _label);
            configSent = true;
            var sinceC = Stopwatch.StartNew();
            string? answer = await reply;
            if (answer is null) throw new InvalidOperationException("The rig did not answer the C command.");
            if (answer.StartsWith("C rejected")) throw new InvalidOperationException(answer);

            PhaseText = "waiting for the lidar to lock onto the PPS";
            while (true)
            {
                if (_forceStop) throw new OperationCanceledException();
                bool timeOk = sinceC.Elapsed.TotalSeconds >= _o.LockWaitSeconds;
                var lidar = _lidarStatus.Status;
                bool locked = !lidar.Reachable || lidar.PpsState == "Locked"; // without the status page only the wait counts
                if (timeOk && locked && _rig.Snapshot().SecondsSinceSample < 1) break;
                if (sinceC.Elapsed.TotalSeconds > _o.LockWaitSeconds + 20)
                    throw new InvalidOperationException($"The lidar did not lock onto the PPS (status: {lidar.PpsState}).");
                await Task.Delay(200);
            }

            PhaseText = "starting the lidar recording";
            var lidarOptions = new LidarRecorderOptions
            {
                ScanDirectory = _scanDir,
                TimeReferenceUnixNs = () => _rig.EstimateNowUnixNs() ?? SystemClockNs(),
                TimeLabelUnixSeconds = _label,
                LidarAddress = _o.LidarAddress,
                Port = _o.LidarPort,
                ChunkSeconds = _o.ChunkSeconds,
                ExtraMetadata = new Dictionary<string, string>
                {
                    ["project"] = _o.ProjectName,
                    ["scan"] = ScanName,
                    ["rpm"] = _o.Rpm.ToString(CultureInfo.InvariantCulture),
                },
                Log = m => Log($"lidar: {m}"),
            };
            Lidar = new LidarRecorder(lidarOptions);
            Lidar.Start();

            WriteScanJson(null);
            Phase = Phase.Scanning;
            PhaseText = "recording";
            Log($"Scanning at {_o.Rpm} rpm.");
        }
        catch (Exception e)
        {
            Log(e is OperationCanceledException ? "Start cancelled." : $"Start failed: {e.Message}");
            if (configSent) { try { _rig.Stop(); } catch (Exception) { } }
            await FinishAsync(e is OperationCanceledException ? "cancelled while starting" : "start failed");
        }
    }

    //---- stop -------------------------------------------------------------------

    /// <summary>
    /// First call: stops the rig (it runs down to 0 deg) and keeps recording until it has stopped. A second call while
    /// stopping ends the recording immediately.
    /// </summary>
    public async Task StopScanAsync()
    {
        if (Phase == Phase.Stopping || Phase == Phase.Starting) { _forceStop = true; return; }
        if (Phase != Phase.Scanning) return;

        Phase = Phase.Stopping;
        PhaseText = "rig running down to 0 deg (press x again to end the recording now)";
        Log("Stopping the rig.");
        try { _rig.Stop(); } catch (Exception e) { Log($"Could not send x: {e.Message}"); }

        var stopped = _rig.WaitForMessageAsync(m => m.Contains("stopped") || m.StartsWith("Start cancelled"), TimeSpan.FromSeconds(90));
        while (!stopped.IsCompleted && !_forceStop) await Task.Delay(100);
        await FinishAsync(_forceStop && !stopped.IsCompleted ? "stopped by the user before the rig had stopped" : "stopped");
    }

    /// <summary>Immediate stop for exiting the program: rig stop command, then everything is closed.</summary>
    public async Task AbortAsync()
    {
        if (Phase == Phase.Idle) return;
        try { _rig.Stop(); } catch (Exception) { }
        await FinishAsync("aborted");
    }

    private async Task FinishAsync(string reason)
    {
        PhaseText = "closing the files";
        try
        {
            if (Lidar is not null) await Lidar.StopAsync();
            _rig.Attach(null);
            _recording?.Dispose();
            WriteScanJson(reason);
            if (_scanDir is not null) Log($"{ScanName} saved ({reason}).");
        }
        catch (Exception e)
        {
            Log($"Error while closing: {e.Message}");
        }
        finally
        {
            ScanTime.Stop();
            Lidar = null;
            _recording = null;
            Phase = Phase.Idle;
            PhaseText = "";
        }
    }

    //---- scan.json --------------------------------------------------------------

    private void WriteScanJson(string? endedBy)
    {
        if (_scanDir is null) return;
        var lidar = Lidar?.Stats;
        var rig = _rig.Snapshot();
        var info = new Dictionary<string, object?>
        {
            ["project"] = _o.ProjectName,
            ["scan"] = ScanName,
            ["started_utc"] = _startedUtc.ToString("O"),
            ["ended_utc"] = endedBy is null ? null : DateTime.UtcNow.ToString("O"),
            ["ended_by"] = endedBy,
            ["rpm"] = _o.Rpm,
            ["time_label_unix_s"] = _label,
            ["lidar_ip"] = _o.LidarAddress.ToString(),
            ["lidar_port"] = _o.LidarPort,
            ["software_version"] = typeof(Orchestrator).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion,
            ["lidar"] = lidar is null ? null : new Dictionary<string, object>
            {
                ["packets_received"] = lidar.PacketsReceived,
                ["packets_lost_estimate"] = lidar.PacketsLostEstimate,
                ["packets_dropped_queue_full"] = lidar.PacketsDroppedQueueFull,
                ["packets_rejected"] = lidar.PacketsRejected,
                ["points_written"] = lidar.PointsWritten,
                ["bytes_written"] = lidar.BytesWritten,
                ["files"] = lidar.ChunkIndex,
                ["clock_jumps"] = lidar.ClockJumps,
                ["time_reference_error_ms"] = Math.Round(lidar.ReferenceErrorMs, 1),
            },
            ["rig"] = new Dictionary<string, object>
            {
                ["samples_seen"] = rig.Samples,
                ["edges_seen"] = rig.Edges,
                ["usb_packets_dropped_by_teensy"] = rig.UsbDrops,
                ["crc_errors"] = rig.CrcErrors,
            },
        };
        File.WriteAllText(Path.Combine(_scanDir, "scan.json"), JsonSerializer.Serialize(info, new JsonSerializerOptions { WriteIndented = true }));
    }

    private static long SystemClockNs() => (DateTime.UtcNow - DateTime.UnixEpoch).Ticks * 100;
}
