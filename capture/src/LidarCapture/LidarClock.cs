namespace LidarCapture;

/// <summary>
/// Turns the lidar's "microseconds since the top of the hour" into absolute UTC nanoseconds since 1970.
///
/// The lidar locks its seconds to the rig's PPS pulses, so its sub-second phase is exact, but its whole seconds are
/// not necessarily the rig's. The whole-second offset is found once from a rough reference clock (accurate to better
/// than +-0.5 s, e.g. the time label plus a stopwatch since the C command) and then stays fixed.
/// </summary>
public sealed class LidarClock
{
    private const long NsPerSecond = 1_000_000_000L;
    private const long NsPerHour = 3600 * NsPerSecond;
    private const uint HalfHourMicros = 1_800_000_000u;

    private uint _lastMicros;
    private long _wraps;
    private bool _started;
    private long _offsetNs;

    public bool IsCalibrated { get; private set; }

    /// <summary>Whole seconds added to the lidar time (lidar time + this = unix time).</summary>
    public long OffsetSeconds => _offsetNs / NsPerSecond;

    /// <summary>
    /// Median difference between the reference and the calibrated time, in ms. It is the latency and error of the
    /// rough reference; it has to stay well below 500 ms for the whole second to be right.
    /// </summary>
    public double ReferenceErrorMs { get; private set; }

    /// <summary>Finds the whole-second offset from (lidar micros, rough reference unix ns) pairs taken at the same moment.</summary>
    public void Calibrate(IReadOnlyList<(uint Micros, long ReferenceUnixNs)> samples)
    {
        if (samples.Count == 0) throw new ArgumentException("no samples", nameof(samples));

        var diffs = new long[samples.Count];
        for (int i = 0; i < samples.Count; i++)
            diffs[i] = samples[i].ReferenceUnixNs - Unwrap(samples[i].Micros);
        Array.Sort(diffs);
        long median = diffs[diffs.Length / 2];

        long seconds = (long)Math.Round(median / (double)NsPerSecond);
        _offsetNs = seconds * NsPerSecond;
        ReferenceErrorMs = (median - _offsetNs) / 1e6;
        IsCalibrated = true;

        // the same packets are converted again afterwards, so start the unwrapping from scratch
        _started = false;
        _wraps = 0;
    }

    public long ToUnixNs(uint micros)
    {
        if (!IsCalibrated) throw new InvalidOperationException("clock not calibrated");
        return Unwrap(micros) + _offsetNs;
    }

    public static bool IsValidMicros(uint micros) => micros < 3_600_000_000u;

    /// <summary>Lidar time as continuous nanoseconds, counting the wraps over the hour.</summary>
    private long Unwrap(uint micros)
    {
        if (_started && micros + HalfHourMicros < _lastMicros) _wraps++;
        _started = true;
        _lastMicros = micros;
        return _wraps * NsPerHour + micros * 1000L;
    }
}
