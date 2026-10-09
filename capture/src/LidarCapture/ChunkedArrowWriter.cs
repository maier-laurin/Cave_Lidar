using System.Diagnostics;
using System.Runtime.InteropServices;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using Apache.Arrow.Types;

namespace LidarCapture;

/// <summary>
/// Writes point batches as Arrow IPC streams into lidar_0001.arrow, lidar_0002.arrow, ... and starts a new file
/// after <c>chunkSeconds</c>. Every batch is flushed to disk, so a crash loses at most the batch in flight.
/// </summary>
internal sealed class ChunkedArrowWriter : IDisposable
{
    private readonly string _directory;
    private readonly double _chunkSeconds;
    private readonly IReadOnlyDictionary<string, string> _metadata;
    private readonly Stopwatch _age = new();

    private FileStream? _file;
    private ArrowStreamWriter? _writer;
    private Schema? _schema;
    private long _closedBytes;

    public int ChunkIndex { get; private set; }
    public long BytesWritten => _closedBytes + (_file?.Length ?? 0);

    public ChunkedArrowWriter(string directory, double chunkSeconds, IReadOnlyDictionary<string, string> metadata)
    {
        _directory = directory;
        _chunkSeconds = chunkSeconds;
        _metadata = metadata;
        Directory.CreateDirectory(directory);
    }

    public void Write(PointBatch b)
    {
        if (b.Count == 0) return;
        if (_writer is null || _schema is null || _age.Elapsed.TotalSeconds >= _chunkSeconds) Rotate();

        int n = b.Count;
        var batch = new RecordBatch(_schema!, new IArrowArray[]
        {
            new TimestampArray(TimeType, Buf(b.TimeNs, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt16Array(Buf(b.Azimuth, n), ArrowBuffer.Empty, n, 0, 0),
            new Int8Array(Buf(b.Elevation, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt16Array(Buf(b.Distance, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(Buf(b.Reflectivity, n), ArrowBuffer.Empty, n, 0, 0),
        }, n);

        _writer!.WriteRecordBatch(batch);
        _file!.Flush(true);
    }

    public void Close()
    {
        if (_writer is null) return;
        _writer.WriteEnd();
        _writer.Dispose();
        _file!.Flush(true);
        _closedBytes += _file.Length;
        _file.Dispose();
        _writer = null;
        _file = null;
    }

    public void Dispose() => Close();

    private void Rotate()
    {
        Close();
        ChunkIndex++;
        string path = Path.Combine(_directory, $"lidar_{ChunkIndex:D4}.arrow");
        _file = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read, 1 << 20);
        _writer = new ArrowStreamWriter(_file, _schema = BuildSchema(ChunkIndex), leaveOpen: true);
        _age.Restart();
    }

    private static readonly TimestampType TimeType = new(TimeUnit.Nanosecond, "UTC");

    private Schema BuildSchema(int chunkIndex)
    {
        var sb = new Schema.Builder()
            .Field(new Field("time", TimeType, false))
            .Field(new Field("azimuth", UInt16Type.Default, false))
            .Field(new Field("elevation", Int8Type.Default, false))
            .Field(new Field("distance", UInt16Type.Default, false))
            .Field(new Field("reflectivity", UInt8Type.Default, false));

        foreach (var (k, v) in _metadata) sb.Metadata(k, v);
        sb.Metadata("chunk_index", chunkIndex.ToString());
        sb.Metadata("chunk_created_utc", DateTime.UtcNow.ToString("O"));
        return sb.Build();
    }

    private static ArrowBuffer Buf<T>(T[] a, int n) where T : unmanaged =>
        new(MemoryMarshal.AsBytes(a.AsSpan(0, n)).ToArray());
}
