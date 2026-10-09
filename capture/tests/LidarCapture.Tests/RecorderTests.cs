using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using LidarCapture;

namespace LidarCapture.Tests;

public class ParserTests
{
    /// <summary>A packet whose blocks all see azimuth start + 20*block (0.01 deg) and the same distance on every laser.</summary>
    internal static byte[] MakePacket(uint micros, int startAz, ushort dist, byte refl = 50)
    {
        var p = new byte[Vlp16.PacketSize];
        for (int b = 0; b < 12; b++)
        {
            int o = b * 100;
            p[o] = 0xFF; p[o + 1] = 0xEE;
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(o + 2), (ushort)((startAz + 20 * b) % 36000));
            for (int i = 0; i < 32; i++)
            {
                int e = o + 4 + i * 3;
                BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(e), dist);
                p[e + 2] = refl;
            }
        }
        BinaryPrimitives.WriteUInt32LittleEndian(p.AsSpan(1200), micros);
        p[1204] = Vlp16.ReturnStrongest;
        p[1205] = Vlp16.ModelVlp16;
        return p;
    }

    [Fact]
    public void ParsesAllPointsWithInterpolatedTimeAndAzimuth()
    {
        var batch = new PointBatch(Vlp16.PointsPerPacket);
        Vlp16Parser.Parse(MakePacket(1000, 100, 500), 5_000_000_000, batch);

        Assert.Equal(384, batch.Count);
        // first point: block 0, sequence 0, laser 0
        Assert.Equal(5_000_000_000, batch.TimeNs[0]);
        Assert.Equal(100, batch.Azimuth[0]);
        Assert.Equal(-15, batch.Elevation[0]);
        Assert.Equal(500, batch.Distance[0]);
        // second sequence, laser 0 of block 0 is half a block later -> azimuth +10
        Assert.Equal(5_000_000_000 + 55_296, batch.TimeNs[16]);
        Assert.Equal(110, batch.Azimuth[16]);
        // last point: block 11, sequence 1, laser 15
        long expect = 5_000_000_000 + 11 * 110_592 + 55_296 + 15 * 2_304;
        Assert.Equal(expect, batch.TimeNs[383]);
        Assert.Equal(15, batch.Elevation[383]);
        Assert.Equal(100 + 220 + 16, batch.Azimuth[383]); // last block reuses the speed of the one before (16.25 rounded)
    }

    [Fact]
    public void DropsZeroDistancePointsAndWrapsAzimuth()
    {
        var p = MakePacket(0, 35990, 100);
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(4), 0); // block 0, laser 0, sequence 0
        var batch = new PointBatch(Vlp16.PointsPerPacket);
        Vlp16Parser.Parse(p, 0, batch);

        Assert.Equal(383, batch.Count);
        Assert.All(batch.Azimuth.Take(batch.Count), a => Assert.InRange((int)a, 0, 35999));
    }

    internal const long N = 1_759_998_000; // whole seconds between the lidar clock and unix time in these tests

    [Theory]
    [InlineData(0)]
    [InlineData(200)]
    [InlineData(-300)]
    public void WholeSecondOffsetIsFoundFromRoughReference(int referenceErrorMs)
    {
        uint m0 = 1_234_567_000; // lidar: 1234.567 s into the hour
        long trueNs = N * 1_000_000_000L + m0 * 1000L;
        var clock = new LidarClock();
        clock.Calibrate(new[] { (m0, trueNs + referenceErrorMs * 1_000_000L) });

        Assert.Equal(N, clock.OffsetSeconds);
        Assert.Equal(referenceErrorMs, clock.ReferenceErrorMs, 3);
        Assert.Equal(trueNs, clock.ToUnixNs(m0));
        Assert.Equal(trueNs + 110_000, clock.ToUnixNs(m0 + 110)); // sub-second part comes from the lidar
    }

    [Fact]
    public void ClockUnwrapsTheHourChangeAfterCalibration()
    {
        uint late = 3_599_999_000;
        long trueNs = N * 1_000_000_000L + late * 1000L;
        var clock = new LidarClock();
        clock.Calibrate(new[] { (late, trueNs) });

        long t1 = clock.ToUnixNs(late);
        long t2 = clock.ToUnixNs(1000); // wrapped
        Assert.Equal(trueNs, t1);
        Assert.Equal(t1 + 2_000_000L, t2); // 2 ms later across the wrap
    }

    [Fact]
    public void LateInTheHourIsNotMistakenForAWrap()
    {
        // 2.5e9 us is past the point where micros + 1.8e9 overflows 32 bits
        uint start = 2_600_000_000;
        long trueNs = N * 1_000_000_000L + start * 1000L;
        var clock = new LidarClock();
        clock.Calibrate(new[] { (start, trueNs) });

        for (int i = 0; i < 1000; i++)
            Assert.Equal(trueNs + i * 1_327_000L, clock.ToUnixNs(start + (uint)(i * 1327)));
    }

    [Fact]
    public void CalibrationSamplesMayStraddleTheHourChange()
    {
        uint a = 3_599_999_000, b = 1000;
        long trueA = N * 1_000_000_000L + a * 1000L;
        var clock = new LidarClock();
        clock.Calibrate(new[] { (a, trueA), (b, trueA + 2_000_000L), (b + 1000, trueA + 3_000_000L) });

        Assert.Equal(N, clock.OffsetSeconds);
        Assert.Equal(trueA, clock.ToUnixNs(a));
        Assert.Equal(trueA + 2_000_000L, clock.ToUnixNs(b)); // still unwrapped exactly once
    }
}

public class RecorderTests
{
    [Fact]
    public async Task RecordsPacketsToChunkedArrowStreamFiles()
    {
        string dir = Path.Combine(Path.GetTempPath(), "lidar_rec_" + Guid.NewGuid().ToString("N"));
        int port = FreePort();
        uint micros = 1_234_567_000;
        long trueNs = ParserTests.N * 1_000_000_000L + micros * 1000L;

        var recorder = new LidarRecorder(new LidarRecorderOptions
        {
            ScanDirectory = dir,
            TimeReferenceUnixNs = () => trueNs + 150_000_000L, // rough reference, 150 ms ahead
            LidarAddress = IPAddress.Loopback,
            BindAddress = IPAddress.Loopback,
            Port = port,
            BatchPackets = 10,
            ChunkSeconds = 0.3,
            ExtraMetadata = new Dictionary<string, string> { ["project"] = "unit_test" },
        });
        recorder.Start();

        using var sender = new UdpClient();
        const int packets = 300;
        for (int i = 0; i < packets; i++)
        {
            var pkt = ParserTests.MakePacket(micros + (uint)(i * 1327), 100, 1000);
            sender.Send(pkt, pkt.Length, new IPEndPoint(IPAddress.Loopback, port));
            if (i % 20 == 19) await Task.Delay(30); // keep the loopback buffer happy and span a few chunks
        }
        await Task.Delay(300);
        await recorder.StopAsync();

        var files = Directory.GetFiles(dir, "lidar_*.arrow").OrderBy(f => f).ToArray();
        Assert.NotEmpty(files);
        Assert.Matches(@"lidar_0001\.arrow$", files[0]);

        long rows = 0;
        long firstTime = long.MaxValue;
        foreach (var f in files)
        {
            using var fs = File.OpenRead(f);
            using var reader = new ArrowStreamReader(fs);
            RecordBatch? b;
            while ((b = await reader.ReadNextRecordBatchAsync()) is not null)
            {
                rows += b.Length;
                var t = (TimestampArray)b.Column("time");
                firstTime = Math.Min(firstTime, t.Values[0]);
            }
            Assert.Equal("unit_test", reader.Schema.Metadata["project"]);
            Assert.Equal("strongest", reader.Schema.Metadata["return_mode"]);
        }

        // loopback can lose a packet under load, so only demand almost everything
        Assert.InRange(rows, (long)(packets * 384 * 0.9), packets * 384L);
        Assert.Equal(rows, recorder.Stats.PointsWritten);
        Assert.True(files.Length >= 2, "expected the 0.3 s chunk limit to split the recording");

        Assert.Equal(trueNs, firstTime);

        Directory.Delete(dir, true);
    }

    private static int FreePort()
    {
        using var s = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
        return ((IPEndPoint)s.Client.LocalEndPoint!).Port;
    }
}
