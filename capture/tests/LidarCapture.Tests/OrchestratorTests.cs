using System.Buffers.Binary;
using System.Reflection;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using CaveScan;
using TeensyCapture;

namespace LidarCapture.Tests;

public class TeensyParserTests
{
    internal static byte[] SamplePacket(long timeUs, int steps = 1000, byte drops = 0, byte flags = 3)
    {
        var p = new byte[57];
        BinaryPrimitives.WriteUInt64LittleEndian(p, (ulong)timeUs);
        BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(8), steps);
        BinaryPrimitives.WriteSingleLittleEndian(p.AsSpan(12), 1.0f);   // qw
        BinaryPrimitives.WriteSingleLittleEndian(p.AsSpan(16), 0.25f);  // qx
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(28), 150);    // q_acc
        p[30] = 0b1110;                                                 // cal
        BinaryPrimitives.WriteSingleLittleEndian(p.AsSpan(31), 9.5f);   // lax
        BinaryPrimitives.WriteSingleLittleEndian(p.AsSpan(43), 42.0f);  // mx
        p[55] = flags;
        p[56] = drops;
        return TeensyPacketParser.Encode('S', p);
    }

    [Fact]
    public void Crc16MatchesTheCcittFalseCheckValue()
    {
        Assert.Equal(0x29B1, TeensyPacketParser.Crc16("123456789"u8));
    }

    [Fact]
    public void DecodesSampleEdgeHeaderAndMessage()
    {
        var samples = new List<TeensySample>();
        var edges = new List<TeensyEdge>();
        var headers = new List<TeensyHeader>();
        var messages = new List<string>();
        var parser = new TeensyPacketParser { OnSample = samples.Add, OnEdge = edges.Add, OnHeader = headers.Add, OnMessage = messages.Add };

        var edge = new byte[12];
        BinaryPrimitives.WriteUInt64LittleEndian(edge, 1_760_000_000_123_456);
        BinaryPrimitives.WriteInt32LittleEndian(edge.AsSpan(8), 31999);

        var header = new byte[17];
        header[0] = 1;
        BinaryPrimitives.WriteSingleLittleEndian(header.AsSpan(1), 31999f);
        BinaryPrimitives.WriteSingleLittleEndian(header.AsSpan(5), 93f);
        BinaryPrimitives.WriteSingleLittleEndian(header.AsSpan(9), 6f);
        BinaryPrimitives.WriteUInt32LittleEndian(header.AsSpan(13), 1_760_000_001);

        parser.Feed(SamplePacket(1_760_000_000_000_000, 1234, 7));
        parser.Feed(TeensyPacketParser.Encode('E', edge));
        parser.Feed(TeensyPacketParser.Encode('H', header));
        parser.Feed(TeensyPacketParser.Encode('M', "Config ok: 6 rpm"u8));

        var s = Assert.Single(samples);
        Assert.Equal(1_760_000_000_000_000, s.TimeUs);
        Assert.Equal(1234, s.AxisSteps);
        Assert.Equal(1.0f, s.Qw);
        Assert.Equal(0.25f, s.Qx);
        Assert.Equal(150, s.QAcc);
        Assert.Equal(2, s.CalRotation);
        Assert.Equal(3, s.CalMagnet);
        Assert.Equal(9.5f, s.Lax);
        Assert.Equal(42.0f, s.Mx);
        Assert.True(s.IndexSeen && s.Running);
        Assert.Equal(7, s.Drops);
        Assert.Equal(31999, Assert.Single(edges).StepsBeforeReset);
        Assert.Equal(1_760_000_001u, Assert.Single(headers).FirstPulseUnixS);
        Assert.Equal("Config ok: 6 rpm", Assert.Single(messages));
        Assert.Equal(0, parser.CrcErrors);
    }

    [Fact]
    public void SurvivesGarbageSplitInputAndMarkerBytesInsideThePayload()
    {
        var samples = new List<TeensySample>();
        var parser = new TeensyPacketParser { OnSample = samples.Add };

        // steps = 0xA5A5A5A5 puts marker bytes inside the payload
        var stream = new List<byte> { 0x00, 0xA5, 0x53, 0xFF }; // garbage that looks like the start of a packet
        stream.AddRange(SamplePacket(1000, unchecked((int)0xA5A5A5A5)));
        var damaged = SamplePacket(2000);
        damaged[10] ^= 0xFF; // payload bit flips, CRC no longer matches
        stream.AddRange(damaged);
        stream.AddRange(SamplePacket(3000));

        foreach (byte b in stream) parser.Feed(new[] { b }); // one byte at a time

        Assert.Equal(new long[] { 1000, 3000 }, samples.Select(s => s.TimeUs).ToArray());
        Assert.True(parser.CrcErrors >= 1);
    }
}

public class ProjectStoreTests
{
    [Theory]
    [InlineData("Hoellloch 2026")]
    [InlineData("cave-1_(test)")]
    [InlineData("Maeander")]
    public void AcceptsOrdinaryNames(string name) => Assert.Null(ProjectStore.ValidateName(name));

    [Theory]
    [InlineData("")]
    [InlineData("   ")]
    [InlineData(" leading")]
    [InlineData("trailing ")]
    [InlineData("dot.")]
    [InlineData("a/b")]
    [InlineData("a\\b")]
    [InlineData("what?")]
    [InlineData("a:b")]
    [InlineData("con")]
    [InlineData("COM4.txt")]
    public void RejectsNamesThatAreNotLegalFolderNames(string name) => Assert.NotNull(ProjectStore.ValidateName(name));

    [Fact]
    public void ScanNumbersContinueAfterTheHighestExistingFolder()
    {
        string dir = Path.Combine(Path.GetTempPath(), "proj_" + Guid.NewGuid().ToString("N"));
        try
        {
            Assert.Equal(1, ProjectStore.NextScanNumber(dir)); // project folder does not exist yet
            Directory.CreateDirectory(ProjectStore.ScanDirectory(dir, 1));
            Directory.CreateDirectory(ProjectStore.ScanDirectory(dir, 3));
            Directory.CreateDirectory(Path.Combine(dir, "notes"));
            Directory.CreateDirectory(Path.Combine(dir, "scan_x"));
            Assert.Equal(new[] { 1, 3 }, ProjectStore.ExistingScans(dir));
            Assert.Equal(4, ProjectStore.NextScanNumber(dir));
            Assert.EndsWith("scan_04", ProjectStore.ScanDirectory(dir, 4));
        }
        finally { Directory.Delete(dir, true); }
    }
}

public class TeensyRecordingTests
{
    [Fact]
    public async Task WritesSamplesToAnArrowStream()
    {
        string dir = Path.Combine(Path.GetTempPath(), "rig_" + Guid.NewGuid().ToString("N"));
        try
        {
            var recording = new TeensyRecording(dir, 60, new Dictionary<string, string> { ["project"] = "unit_test" });
            var add = typeof(TeensyRecording).GetMethod("Add", BindingFlags.NonPublic | BindingFlags.Instance, new[] { typeof(TeensySample) })!;
            for (int i = 0; i < 120; i++)
            {
                TeensySample? s = null;
                new TeensyPacketParser { OnSample = x => s = x }.Feed(TeensyParserTests.SamplePacket(1_760_000_000_000_000 + i * 10_000, i));
                add.Invoke(recording, new object[] { s!.Value });
            }
            recording.Dispose();

            using var fs = File.OpenRead(Path.Combine(dir, "rig_0001.arrow"));
            using var reader = new ArrowStreamReader(fs);
            long rows = 0, firstTime = 0;
            int lastSteps = -1;
            RecordBatch? b;
            while ((b = await reader.ReadNextRecordBatchAsync()) is not null)
            {
                if (rows == 0) firstTime = ((TimestampArray)b.Column("time")).Values[0];
                rows += b.Length;
                lastSteps = ((Int32Array)b.Column("axis_steps")).Values[b.Length - 1];
            }
            Assert.Equal(120, rows);
            Assert.Equal(1_760_000_000_000_000 * 1000L, firstTime);
            Assert.Equal(119, lastSteps);
            Assert.Equal("unit_test", reader.Schema.Metadata["project"]);
            Assert.Equal(120, recording.SamplesWritten);
        }
        finally { if (Directory.Exists(dir)) Directory.Delete(dir, true); }
    }
}
