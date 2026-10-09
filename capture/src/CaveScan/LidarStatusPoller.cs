using System.Net.Http;
using System.Text.Json;

namespace CaveScan;

public sealed record LidarHttpStatus(bool Reachable, string PpsState, string MotorState, int Rpm, string LaserState, DateTime? UpdatedUtc);

/// <summary>Reads the lidar's own status page (<c>/cgi/status.json</c>) every 2 s: PPS lock, motor speed, laser state.</summary>
public sealed class LidarStatusPoller : IDisposable
{
    private static readonly LidarHttpStatus Unknown = new(false, "?", "?", 0, "?", null);

    private readonly HttpClient _http = new() { Timeout = TimeSpan.FromSeconds(2) };
    private readonly string _url;
    private readonly CancellationTokenSource _stop = new();
    private readonly Task _task;
    private volatile LidarHttpStatus _status = Unknown;

    public LidarStatusPoller(string lidarAddress)
    {
        _url = $"http://{lidarAddress}/cgi/status.json";
        _task = Task.Run(LoopAsync);
    }

    public LidarHttpStatus Status => _status;

    private async Task LoopAsync()
    {
        while (!_stop.IsCancellationRequested)
        {
            try
            {
                using var doc = JsonDocument.Parse(await _http.GetStringAsync(_url, _stop.Token));
                var r = doc.RootElement;
                _status = new LidarHttpStatus(
                    true,
                    r.GetProperty("gps").GetProperty("pps_state").GetString() ?? "?",
                    r.GetProperty("motor").GetProperty("state").GetString() ?? "?",
                    r.GetProperty("motor").GetProperty("rpm").GetInt32(),
                    r.GetProperty("laser").GetProperty("state").GetString() ?? "?",
                    DateTime.UtcNow);
            }
            catch (OperationCanceledException) when (_stop.IsCancellationRequested) { return; }
            catch (Exception)
            {
                _status = Unknown;
            }

            try { await Task.Delay(2000, _stop.Token); } catch (OperationCanceledException) { return; }
        }
    }

    public void Dispose()
    {
        _stop.Cancel();
        try { _task.Wait(1000); } catch (AggregateException) { }
        _http.Dispose();
        _stop.Dispose();
    }
}
