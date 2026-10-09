using System.Diagnostics;
using System.IO.Ports;
using System.Net;
using LidarCapture;

// Standalone lidar recorder, mainly for testing. The orchestrator will use LidarRecorder directly.
//   LidarCapture.Cli --out <scan folder> [--teensy COM4 --rpm 6] [--lidar-ip 192.168.3.201] [--port 3201]
//                    [--chunk-seconds 60] [--duration <s>] [--lock-wait 8]
//   LidarCapture.Cli inspect <scan folder or file>
//   LidarCapture.Cli sniff <udp port>
//
// With --teensy the program also starts the rig: it sends "C,<rpm>,<label>", waits for the lidar to lock onto the PPS
// and uses "label + stopwatch since the C command" as the rough time reference for the lidar clock.
// Without it the PC clock is the reference.

if (args.Length == 2 && args[0] == "inspect") return await Inspect.RunAsync(args[1]);
if (args.Length == 2 && args[0] == "sniff") return Sniff.Run(int.Parse(args[1]), 4);

string? outDir = null;
string? teensyPort = null;
double rpm = 6;
var lidarIp = IPAddress.Parse("192.168.3.201");
int port = 3201;
double chunkSeconds = 60;
double? duration = null;
double lockWait = 8;

try
{
    for (int i = 0; i < args.Length; i++)
    {
        string Next() => args[++i];
        switch (args[i])
        {
            case "--out": outDir = Next(); break;
            case "--teensy": teensyPort = Next(); break;
            case "--rpm": rpm = double.Parse(Next(), System.Globalization.CultureInfo.InvariantCulture); break;
            case "--lidar-ip": lidarIp = IPAddress.Parse(Next()); break;
            case "--port": port = int.Parse(Next()); break;
            case "--chunk-seconds": chunkSeconds = double.Parse(Next(), System.Globalization.CultureInfo.InvariantCulture); break;
            case "--duration": duration = double.Parse(Next(), System.Globalization.CultureInfo.InvariantCulture); break;
            case "--lock-wait": lockWait = double.Parse(Next(), System.Globalization.CultureInfo.InvariantCulture); break;
            default: throw new ArgumentException($"unknown argument {args[i]}");
        }
    }
    if (outDir is null) throw new ArgumentException("--out is required");
}
catch (Exception e) when (e is ArgumentException or FormatException or IndexOutOfRangeException)
{
    Console.Error.WriteLine(e.Message);
    Console.Error.WriteLine("usage: LidarCapture.Cli --out <scan folder> [--teensy COM4 --rpm 6] [--lidar-ip <ip>] [--port <n>] [--chunk-seconds <s>] [--duration <s>] [--lock-wait <s>]");
    return 2;
}

if (Directory.Exists(outDir) && Directory.EnumerateFileSystemEntries(outDir).Any())
{
    Console.Error.WriteLine($"{outDir} already contains files, refusing to write into it.");
    return 1;
}

long label = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
Func<long>? reference = null;
SerialPort? teensy = null;
CancellationTokenSource drain = new();
Task? drainTask = null;

if (teensyPort is not null)
{
    teensy = new SerialPort(teensyPort, 115200) { DtrEnable = true, ReadTimeout = 200 };
    teensy.Open();
    teensy.DiscardInBuffer();

    var sinceC = Stopwatch.StartNew();
    teensy.Write($"C,{rpm.ToString(System.Globalization.CultureInfo.InvariantCulture)},{label}\n");
    sinceC.Restart();
    Console.WriteLine($"[rig] sent C,{rpm},{label} on {teensyPort}");

    // the first PPS pulse follows one second after the C line and carries the label
    reference = () => label * 1_000_000_000L + (long)(sinceC.Elapsed.TotalSeconds * 1e9) - 1_000_000_000L;

    // the firmware streams binary data continuously, keep the port drained
    drainTask = Task.Run(() =>
    {
        var buf = new byte[8192];
        while (!drain.IsCancellationRequested)
        {
            try { teensy.Read(buf, 0, buf.Length); }
            catch (TimeoutException) { }
            catch (Exception) { break; }
        }
    });

    Console.WriteLine($"[rig] waiting {lockWait:F0} s for the lidar to lock onto the PPS...");
    await Task.Delay(TimeSpan.FromSeconds(lockWait));
}
else
{
    Console.WriteLine("No --teensy given, using the PC clock as time reference.");
}

await using var recorder = new LidarRecorder(new LidarRecorderOptions
{
    ScanDirectory = outDir,
    TimeLabelUnixSeconds = teensyPort is null ? null : label,
    TimeReferenceUnixNs = reference,
    LidarAddress = lidarIp,
    Port = port,
    ChunkSeconds = chunkSeconds,
    ExtraMetadata = teensyPort is null ? new Dictionary<string, string>() : new Dictionary<string, string> { ["rpm"] = rpm.ToString(System.Globalization.CultureInfo.InvariantCulture) },
    Log = m => Console.WriteLine($"[lidar] {m}"),
});

recorder.Start();

using var quit = duration is null ? new CancellationTokenSource() : new CancellationTokenSource(TimeSpan.FromSeconds(duration.Value));
Console.CancelKeyPress += (_, e) => { e.Cancel = true; quit.Cancel(); };
Console.WriteLine("Recording. Press Ctrl+C to stop.");

var s = recorder.Stats;
try
{
    while (true)
    {
        await Task.Delay(2000, quit.Token);
        Console.WriteLine($"packets {s.PacketsReceived}  lost~{s.PacketsLostEstimate}  queue-dropped {s.PacketsDroppedQueueFull}  rejected {s.PacketsRejected}  points {s.PointsWritten}  {s.BytesWritten / 1048576.0:F1} MB  queue {s.QueueDepth}");
    }
}
catch (OperationCanceledException) { }

await recorder.StopAsync();
Console.WriteLine($"Done: {s.PointsWritten} points, {s.BytesWritten / 1048576.0:F1} MB, {s.ChunkIndex} file(s).");

if (teensy is not null)
{
    teensy.Write("x\n");
    Console.WriteLine("[rig] sent x, the shaft runs down to 0 deg");
    await Task.Delay(1000);
    drain.Cancel();
    if (drainTask is not null) await drainTask;
    teensy.Close();
}
return 0;
