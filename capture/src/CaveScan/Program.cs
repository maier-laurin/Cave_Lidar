using System.Globalization;
using System.Net;
using CaveScan;
using TeensyCapture;

// Orchestrator: asks for a project name, then records lidar and rig (Teensy) into <root>/<project>/scan_NN/.
//   CaveScan [--root scans] [--project <name>] [--teensy COM4] [--rpm 6] [--lidar-ip 192.168.3.201] [--port 3201]
//            [--chunk-seconds 60] [--lock-wait 8] [--auto <seconds>]

CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

string root = "scans";
string? project = null;
string teensyPort = "COM4";
double rpm = 6;
var lidarIp = IPAddress.Parse("192.168.3.201");
int lidarPort = 3201;
double chunkSeconds = 60, lockWait = 8;
double? auto = null; // unattended: start a scan, record this many seconds, stop, exit (no keys, no dashboard)

try
{
    for (int i = 0; i < args.Length; i++)
    {
        string Next() => args[++i];
        switch (args[i])
        {
            case "--root": root = Next(); break;
            case "--project": project = Next(); break;
            case "--teensy": teensyPort = Next(); break;
            case "--rpm": rpm = double.Parse(Next()); break;
            case "--lidar-ip": lidarIp = IPAddress.Parse(Next()); break;
            case "--port": lidarPort = int.Parse(Next()); break;
            case "--chunk-seconds": chunkSeconds = double.Parse(Next()); break;
            case "--lock-wait": lockWait = double.Parse(Next()); break;
            case "--auto": auto = double.Parse(Next()); break;
            default: throw new ArgumentException($"unknown argument {args[i]}");
        }
    }
}
catch (Exception e) when (e is ArgumentException or FormatException or IndexOutOfRangeException)
{
    Console.Error.WriteLine(e.Message);
    Console.Error.WriteLine("usage: CaveScan [--root <folder>] [--project <name>] [--teensy COM4] [--rpm 6] [--lidar-ip <ip>] [--port <n>] [--chunk-seconds <s>] [--lock-wait <s>]");
    return 2;
}

// ---- project name
while (true)
{
    if (project is null)
    {
        Console.Write("Project name: ");
        project = Console.ReadLine();
        if (project is null) return 2; // input closed
    }
    string? problem = ProjectStore.ValidateName(project);
    if (problem is null) break;
    Console.WriteLine(problem);
    project = null;
}

string projectDir = ProjectStore.ProjectDirectory(root, project);
var existing = ProjectStore.ExistingScans(projectDir);
Directory.CreateDirectory(projectDir);
Console.WriteLine(existing.Count == 0
    ? $"New project in {Path.GetFullPath(projectDir)}"
    : $"Project {project} exists with {existing.Count} scan(s), the next scan will be scan_{ProjectStore.NextScanNumber(projectDir):D2}.");

// ---- connections
using var rig = new TeensyLink(teensyPort);
try
{
    rig.Open();
}
catch (Exception e)
{
    Console.Error.WriteLine($"Cannot open {teensyPort}: {e.Message}");
    return 1;
}

using var lidarStatus = new LidarStatusPoller(lidarIp.ToString());
var orchestrator = new Orchestrator(new OrchestratorOptions
{
    ProjectName = project,
    ProjectDirectory = projectDir,
    Rpm = rpm,
    LidarAddress = lidarIp,
    LidarPort = lidarPort,
    ChunkSeconds = chunkSeconds,
    LockWaitSeconds = lockWait,
}, rig, lidarStatus);

var dashboard = new Dashboard(orchestrator, root);
bool quit = false;
Console.CancelKeyPress += (_, e) => { e.Cancel = true; quit = true; };

bool interactive = auto is null && !Console.IsInputRedirected && !Console.IsOutputRedirected;
if (interactive)
{
    Console.Clear();
    Console.CursorVisible = false;
}
Task? running = null; // the start or stop that is in progress

try
{
    if (interactive)
    {
        var lastDraw = DateTime.MinValue;
        while (!quit)
        {
            while (Console.KeyAvailable)
            {
                var key = Console.ReadKey(true).Key;
                if (key == ConsoleKey.S && orchestrator.Phase == Phase.Idle && (running is null || running.IsCompleted))
                    running = orchestrator.StartScanAsync();
                else if (key == ConsoleKey.X && orchestrator.Phase != Phase.Idle)
                    running = orchestrator.StopScanAsync();
                else if (key == ConsoleKey.Q)
                {
                    if (orchestrator.Phase == Phase.Idle) quit = true;
                    else orchestrator.Log("A scan is in progress. Stop it first with x.");
                }
            }

            if ((DateTime.UtcNow - lastDraw).TotalMilliseconds >= 500)
            {
                lastDraw = DateTime.UtcNow;
                dashboard.Draw();
            }
            await Task.Delay(50);
        }
    }
    else
    {
        // unattended: the same steps without keys, progress as plain text lines
        int printed = 0;
        void PrintNewLog()
        {
            var all = orchestrator.RecentLog(500);
            for (; printed < all.Count; printed++) Console.WriteLine(all[printed]);
        }

        running = orchestrator.StartScanAsync();
        while (!quit && !running.IsCompleted) { PrintNewLog(); await Task.Delay(200); }
        await running;
        PrintNewLog();

        var until = DateTime.UtcNow.AddSeconds(auto ?? 0);
        var lastStatus = DateTime.MinValue;
        while (!quit && orchestrator.Phase == Phase.Scanning && DateTime.UtcNow < until)
        {
            PrintNewLog();
            if ((DateTime.UtcNow - lastStatus).TotalSeconds >= 5)
            {
                lastStatus = DateTime.UtcNow;
                var st = orchestrator.Lidar?.Stats;
                var rs = rig.Snapshot();
                if (st is not null)
                    Console.WriteLine($"  lidar: {st.PacketsReceived} packets, {st.PointsWritten / 1e6:F1} M points, lost~{st.PacketsLostEstimate}, queue-dropped {st.PacketsDroppedQueueFull}, time check {st.ReferenceErrorMs:F1} ms | rig: {rs.SampleRateHz:F0} Hz, drops {rs.UsbDrops}, crc {rs.CrcErrors}, edges {rs.Edges}");
            }
            await Task.Delay(200);
        }

        if (orchestrator.Phase == Phase.Scanning)
        {
            running = orchestrator.StopScanAsync();
            while (!running.IsCompleted) { PrintNewLog(); await Task.Delay(200); }
        }
        while (orchestrator.Phase != Phase.Idle) { PrintNewLog(); await Task.Delay(200); }
        PrintNewLog();
    }
}
finally
{
    // the firmware keeps the shaft turning if the PC just disappears, so make sure it is told to stop
    if (interactive)
    {
        Console.CursorVisible = true;
        Console.SetCursorPosition(0, Math.Min(Console.CursorTop + 1, Console.BufferHeight - 1));
    }
    if (running is not null) { try { await running; } catch (Exception) { } }
    if (orchestrator.Phase != Phase.Idle)
    {
        Console.WriteLine("Closing the scan...");
        await orchestrator.AbortAsync();
    }
    var last = rig.Snapshot().LastSample;
    if (last is { Running: true }) { rig.Stop(); Console.WriteLine("Sent stop to the rig."); }
}

return 0;
