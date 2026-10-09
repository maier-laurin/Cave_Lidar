using System.Diagnostics;
using System.Runtime.InteropServices;
using Apache.Arrow;
using Apache.Arrow.Ipc;

namespace CaptureCore;

/// <summary>
/// Writes record batches as Arrow IPC streams into prefix_0001.arrow, prefix_0002.arrow, ... and starts a new file
/// after <c>chunkSeconds</c>. Every batch is flushed to disk, so a crash loses at most the batch in flight.
/// </summary>
public sealed class ChunkedStreamWriter : IDisposable
{
    private readonly string _directory;
    private readonly string _prefix;
    private readonly double _chunkSeconds;
    private readonly Func<int, Schema> _schemaFactory;
    private readonly Stopwatch _age = new();

    private FileStream? _file;
    private ArrowStreamWriter? _writer;
    private Schema? _schema;
    private long _closedBytes;

    public int ChunkIndex { get; private set; }
    public long BytesWritten => _closedBytes + (_file?.Length ?? 0);

    /// <param name="schemaFactory">Called with the chunk number whenever a new file starts, so the metadata can differ per file.</param>
    public ChunkedStreamWriter(string directory, string prefix, double chunkSeconds, Func<int, Schema> schemaFactory)
    {
        _directory = directory;
        _prefix = prefix;
        _chunkSeconds = chunkSeconds;
        _schemaFactory = schemaFactory;
        Directory.CreateDirectory(directory);
    }

    /// <summary>Starts a new file when the current one is too old, then writes the batch built from the file's schema.</summary>
    /// <returns>true if this call started a new file.</returns>
    public bool Write(Func<Schema, RecordBatch> build)
    {
        bool rotated = false;
        if (_writer is null || _schema is null || _age.Elapsed.TotalSeconds >= _chunkSeconds)
        {
            Rotate();
            rotated = true;
        }

        using var batch = build(_schema!);
        _writer!.WriteRecordBatch(batch);
        _file!.Flush(true);
        return rotated;
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
        string path = Path.Combine(_directory, $"{_prefix}_{ChunkIndex:D4}.arrow");
        _file = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read, 1 << 20);
        _schema = _schemaFactory(ChunkIndex);
        _writer = new ArrowStreamWriter(_file, _schema, leaveOpen: true);
        _age.Restart();
    }

    /// <summary>Copies the first <paramref name="count"/> values into an Arrow buffer.</summary>
    public static ArrowBuffer Buffer<T>(T[] values, int count) where T : unmanaged =>
        new(MemoryMarshal.AsBytes(values.AsSpan(0, count)).ToArray());
}
