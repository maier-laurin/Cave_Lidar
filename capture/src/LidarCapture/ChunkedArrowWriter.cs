using Apache.Arrow;
using Apache.Arrow.Types;
using CaptureCore;

namespace LidarCapture;

/// <summary>Writes lidar point batches as chunked Arrow stream files lidar_0001.arrow, lidar_0002.arrow, ...</summary>
internal sealed class ChunkedArrowWriter : IDisposable
{
    private static readonly TimestampType TimeType = new(TimeUnit.Nanosecond, "UTC");

    private readonly ChunkedStreamWriter _writer;

    public int ChunkIndex => _writer.ChunkIndex;
    public long BytesWritten => _writer.BytesWritten;

    public ChunkedArrowWriter(string directory, double chunkSeconds, IReadOnlyDictionary<string, string> metadata) =>
        _writer = new ChunkedStreamWriter(directory, "lidar", chunkSeconds, index => BuildSchema(metadata, index));

    public void Write(PointBatch b)
    {
        if (b.Count == 0) return;
        int n = b.Count;
        _writer.Write(schema => new RecordBatch(schema, new IArrowArray[]
        {
            new TimestampArray(TimeType, ChunkedStreamWriter.Buffer(b.TimeNs, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt16Array(ChunkedStreamWriter.Buffer(b.Azimuth, n), ArrowBuffer.Empty, n, 0, 0),
            new Int8Array(ChunkedStreamWriter.Buffer(b.Elevation, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt16Array(ChunkedStreamWriter.Buffer(b.Distance, n), ArrowBuffer.Empty, n, 0, 0),
            new UInt8Array(ChunkedStreamWriter.Buffer(b.Reflectivity, n), ArrowBuffer.Empty, n, 0, 0),
        }, n));
    }

    public void Close() => _writer.Close();
    public void Dispose() => _writer.Dispose();

    private static Schema BuildSchema(IReadOnlyDictionary<string, string> metadata, int chunkIndex)
    {
        var sb = new Schema.Builder()
            .Field(new Field("time", TimeType, false))
            .Field(new Field("azimuth", UInt16Type.Default, false))
            .Field(new Field("elevation", Int8Type.Default, false))
            .Field(new Field("distance", UInt16Type.Default, false))
            .Field(new Field("reflectivity", UInt8Type.Default, false));

        foreach (var (k, v) in metadata) sb.Metadata(k, v);
        sb.Metadata("chunk_index", chunkIndex.ToString());
        sb.Metadata("chunk_created_utc", DateTime.UtcNow.ToString("O"));
        return sb.Build();
    }
}
