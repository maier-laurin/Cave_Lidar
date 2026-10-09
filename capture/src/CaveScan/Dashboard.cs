using System.Diagnostics;
using System.Text;
using TeensyCapture;

namespace CaveScan;

/// <summary>Draws the live telemetry as a block of text that is redrawn in place.</summary>
public sealed class Dashboard
{
    private readonly Orchestrator _o;
    private readonly string _root;
    private int _lastLines;

    // for the rates
    private DateTime _prevTime = DateTime.UtcNow;
    private long _prevPackets, _prevPoints, _prevBytes;
    private double _packetsPerS, _pointsPerS, _bytesPerS;

    public Dashboard(Orchestrator orchestrator, string root)
    {
        _o = orchestrator;
        _root = root;
    }

    public void Draw()
    {
        var lines = Build();
        int width = Math.Max(40, Console.WindowWidth - 1);
        var sb = new StringBuilder();
        Console.SetCursorPosition(0, 0);
        foreach (var l in lines) sb.Append(l.Length > width ? l[..width] : l.PadRight(width)).Append('\n');
        for (int i = lines.Count; i < _lastLines; i++) sb.Append(new string(' ', width)).Append('\n');
        _lastLines = lines.Count;
        Console.Write(sb.ToString());
    }

    private List<string> Build()
    {
        var o = _o.Options;
        var rig = _o.Rig.Snapshot();
        var http = _o.LidarStatus.Status;
        var L = new List<string>();

        var phase = _o.Phase switch
        {
            Phase.Idle => "IDLE",
            Phase.Starting => "STARTING",
            Phase.Scanning => "SCANNING",
            _ => "STOPPING",
        };
        L.Add($"CAVE SCAN   project \"{o.ProjectName}\"   {DateTime.UtcNow:yyyy-MM-dd HH:mm:ss} UTC");
        L.Add($"State: {phase,-9} {(_o.ScanName is not null && _o.Phase != Phase.Idle ? _o.ScanName + "  " + _o.ScanTime.Elapsed.ToString(@"hh\:mm\:ss") : "")}  {_o.PhaseText}");
        L.Add(_o.Phase switch
        {
            Phase.Idle => "Keys: [s] start scan   [q] quit",
            Phase.Scanning => "Keys: [x] stop (rig runs down, then the files are closed)",
            _ => "Keys: [x] end the recording now",
        });
        L.Add("");

        // ---- rig
        L.Add($"-- Rig ({_o.Rig.PortName}) " + new string('-', 40));
        if (rig.LastSample is { } s && rig.SecondsSinceSample < 3)
        {
            string axis = s.IndexSeen ? $"{s.AxisSteps} steps" + (rig.Header is { } h && h.StepsPerRev > 0 ? $" = {s.AxisSteps / h.StepsPerRev * 360:F1} deg after the index edge" : "") : "no index edge seen yet";
            L.Add($" stream {rig.SampleRateHz,5:F1} Hz   samples {rig.Samples}   usb drops {rig.UsbDrops}   crc errors {rig.CrcErrors}   index edges {rig.Edges}");
            L.Add($" shaft: {(s.Running ? "running" : "holding")}, {axis}");
            L.Add($" IMU q = [{s.Qw:F3} {s.Qx:F3} {s.Qy:F3} {s.Qz:F3}]  accuracy {s.QAcc / 100.0:F1} deg  cal rot {s.CalRotation}/3 mag {s.CalMagnet}/3");
            L.Add($" rig time {DateTimeOffset.FromUnixTimeMilliseconds(s.TimeUs / 1000):HH:mm:ss.fff} (simulated UTC)");
        }
        else
        {
            L.Add(rig.Samples > 0
                ? $" NO DATA from the rig for {rig.SecondsSinceSample:F0} s"
                : " no stream yet (the rig only streams after a scan was started)");
            L.Add(""); L.Add(""); L.Add("");
        }
        L.Add($" last message: {rig.LastMessage}");
        L.Add("");

        // ---- lidar
        L.Add($"-- Lidar ({o.LidarAddress}) " + new string('-', 40));
        L.Add(http.Reachable
            ? $" status page: PPS {http.PpsState}, motor {http.MotorState} {http.Rpm} rpm, laser {http.LaserState}"
            : " status page: not reachable");

        var lidar = _o.Lidar?.Stats;
        long bytes = (lidar?.BytesWritten ?? 0) + (_o.Recording?.BytesWritten ?? 0);
        if (lidar is not null)
        {
            UpdateRates(lidar.PacketsReceived, lidar.PointsWritten, bytes);
            L.Add($" packets {_packetsPerS,6:F0}/s   points {_pointsPerS / 1000,6:F0}k/s   total {lidar.PointsWritten / 1e6:F1} M points");
            L.Add($" lost (clock gaps) {lidar.PacketsLostEstimate}   dropped (queue) {lidar.PacketsDroppedQueueFull}   rejected {lidar.PacketsRejected}   clock jumps {lidar.ClockJumps}   queue {lidar.QueueDepth}");
            L.Add($" time check: lidar vs rig clock {lidar.ReferenceErrorMs:F1} ms (must stay within +-500)   file lidar_{Math.Max(lidar.ChunkIndex, 1):D4}.arrow");
        }
        else
        {
            _prevTime = DateTime.UtcNow;
            _prevPackets = _prevPoints = _prevBytes = 0;
            L.Add(" not recording"); L.Add(""); L.Add("");
        }
        L.Add("");

        // ---- disk
        L.Add("-- Disk " + new string('-', 40));
        try
        {
            var drive = new DriveInfo(Path.GetPathRoot(Path.GetFullPath(_root))!);
            double freeGb = drive.AvailableFreeSpace / 1e9;
            string eta = _bytesPerS > 1e4 ? $", about {drive.AvailableFreeSpace / _bytesPerS / 3600:F1} h left at the current rate" : "";
            L.Add($" {_root}: {freeGb:F1} GB free{eta}");
        }
        catch (Exception)
        {
            L.Add($" {_root}");
        }
        L.Add(lidar is null ? " scan data: -" : $" scan data: {bytes / 1048576.0:F1} MB   ({_bytesPerS / 1048576.0:F2} MB/s)");
        L.Add("");

        L.Add("-- Messages " + new string('-', 40));
        var log = _o.RecentLog(8);
        for (int i = 0; i < 8; i++) L.Add(i < log.Count ? " " + log[i] : "");
        return L;
    }

    private void UpdateRates(long packets, long points, long bytes)
    {
        var now = DateTime.UtcNow;
        double dt = (now - _prevTime).TotalSeconds;
        if (dt < 1) return;
        _packetsPerS = (packets - _prevPackets) / dt;
        _pointsPerS = (points - _prevPoints) / dt;
        _bytesPerS = (bytes - _prevBytes) / dt;
        _prevTime = now; _prevPackets = packets; _prevPoints = points; _prevBytes = bytes;
    }
}
