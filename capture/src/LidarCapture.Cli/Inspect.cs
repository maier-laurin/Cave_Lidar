using Apache.Arrow;
using Apache.Arrow.Ipc;

static class Inspect
{
    /// <summary>Prints a summary of every lidar_*.arrow file in a scan folder (or of one file).</summary>
    public static async Task<int> RunAsync(string path)
    {
        var files = Directory.Exists(path)
            ? Directory.GetFiles(path, "lidar_*.arrow").OrderBy(f => f).ToArray()
            : new[] { path };

        long prevTime = long.MinValue;
        foreach (var f in files)
        {
            using var fs = File.OpenRead(f);
            using var reader = new ArrowStreamReader(fs);

            long rows = 0, first = long.MaxValue, last = long.MinValue, backwards = 0, jumps = 0;
            long maxJump = 0;
            int minDist = int.MaxValue, maxDist = 0;
            var azHist = new int[36];
            var elHist = new Dictionary<int, long>();

            RecordBatch? b;
            while ((b = await reader.ReadNextRecordBatchAsync()) is not null)
            {
                var t = ((TimestampArray)b.Column("time")).Values;
                var az = ((UInt16Array)b.Column("azimuth")).Values;
                var el = ((Int8Array)b.Column("elevation")).Values;
                var d = ((UInt16Array)b.Column("distance")).Values;
                for (int i = 0; i < b.Length; i++)
                {
                    long ti = t[i];
                    if (prevTime != long.MinValue)
                    {
                        long dt = ti - prevTime;
                        if (dt < 0 || dt > 20_000_000) Console.WriteLine($"  [event] row {rows + i}: dt = {dt / 1e6:F3} ms, time {FromNs(ti):HH:mm:ss.ffffff}");
                        if (dt < 0) backwards++;
                        else if (dt > 20_000_000) { jumps++; maxJump = Math.Max(maxJump, dt); } // more than 20 ms
                    }
                    prevTime = ti;
                    first = Math.Min(first, ti); last = Math.Max(last, ti);
                    azHist[az[i] / 1000 % 36]++;
                    elHist[el[i]] = elHist.GetValueOrDefault(el[i]) + 1;
                    if (d[i] < minDist) minDist = d[i];
                    if (d[i] > maxDist) maxDist = d[i];
                }
                rows += b.Length;
            }

            Console.WriteLine($"{Path.GetFileName(f)}: {rows} points");
            if (rows == 0) continue;
            Console.WriteLine($"  time   {FromNs(first):O} .. {FromNs(last):O}  ({(last - first) / 1e9:F2} s)");
            Console.WriteLine($"  steps backwards: {backwards}, jumps > 20 ms: {jumps} (largest {maxJump / 1e6:F1} ms)");
            Console.WriteLine($"  distance {minDist * 0.002:F2} .. {maxDist * 0.002:F2} m, azimuth bins (10 deg, in thousands): {string.Join(' ', azHist.Select(x => x / 1000))}");
            Console.WriteLine($"  elevations present: {string.Join(' ', elHist.Keys.OrderBy(x => x))}");
            foreach (var (k, v) in reader.Schema.Metadata.OrderBy(kv => kv.Key))
                Console.WriteLine($"  meta {k} = {v}");
        }
        return 0;
    }

    private static DateTimeOffset FromNs(long ns) => DateTimeOffset.UnixEpoch.AddTicks(ns / 100);
}
