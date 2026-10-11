using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using Apache.Arrow.Types;
using Intel.RealSense;
using System.Diagnostics;

namespace RealSense;
public class T265ArrowRecorder
{
    private readonly string _filePath;
    private readonly int _batchSize;
    private readonly Schema _schema;

    // Internal buffers
    private readonly List<double> _ts;
    private readonly List<float> _tx, _ty, _tz;
    private readonly List<float> _rx, _ry, _rz, _rw;
    private readonly List<float> _vx, _vy, _vz;
    private readonly List<float> _ax, _ay, _az;
    private readonly List<float> _avx, _avy, _avz;
    private readonly List<float> _aax, _aay, _aaz;
    private readonly List<byte> _conf;

    public T265ArrowRecorder(string filePath, int batchSize = 500)
    {
        _filePath = filePath;
        _batchSize = batchSize;
        _schema = BuildSchema();

        // Pre-allocate list capacity to avoid GC pressure during recording
        _ts = new List<double>(_batchSize);
        _tx = new List<float>(_batchSize); _ty = new List<float>(_batchSize); _tz = new List<float>(_batchSize);
        _rx = new List<float>(_batchSize); _ry = new List<float>(_batchSize); _rz = new List<float>(_batchSize); _rw = new List<float>(_batchSize);
        _vx = new List<float>(_batchSize); _vy = new List<float>(_batchSize); _vz = new List<float>(_batchSize);
        _ax = new List<float>(_batchSize); _ay = new List<float>(_batchSize); _az = new List<float>(_batchSize);
        _avx = new List<float>(_batchSize); _avy = new List<float>(_batchSize); _avz = new List<float>(_batchSize);
        _aax = new List<float>(_batchSize); _aay = new List<float>(_batchSize); _aaz = new List<float>(_batchSize);
        _conf = new List<byte>(_batchSize);
    }

    private static Schema BuildSchema()
    {
        return new Schema.Builder()
            .Field(f => f.Name("timestamp").DataType(DoubleType.Default).Nullable(false))
            .Field(f => f.Name("t_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("t_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("t_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("r_w").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("v_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("v_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("v_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("a_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("a_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("a_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("av_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("av_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("av_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("aa_x").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("aa_y").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("aa_z").DataType(FloatType.Default).Nullable(false))
            .Field(f => f.Name("confidence").DataType(UInt8Type.Default).Nullable(false))
            .Build();
    }

    public async Task RunAsync(CancellationToken token)
    {
        using var pipe = new Pipeline();
        using var cfg = new Config();
        Stopwatch diskTimer = new Stopwatch();
        cfg.EnableStream(Intel.RealSense.Stream.Pose);
        
        pipe.Start(cfg);

        using var stream = File.Create(_filePath);
        using var writer = new ArrowFileWriter(stream, _schema);

        try
        {
            // We use Task.Run because pipe.WaitForFrames is a blocking call
            await Task.Run(() =>
            {
                while (!token.IsCancellationRequested)
                {
                    using var frames = pipe.WaitForFrames();
                    using var poseFrame = frames.FirstOrDefault<PoseFrame>(Intel.RealSense.Stream.Pose);

                    if (poseFrame != null)
                    {
                        BufferData(poseFrame);

                        if (_ts.Count >= _batchSize)
                        {
                            long fileSizeBytes = new FileInfo(_filePath).Length;
                            diskTimer.Restart();
                            WriteBatch(writer);
                            diskTimer.Stop();
                            Console.WriteLine($"New T265 batch written, this write took {diskTimer.ElapsedMilliseconds} ms, the total filesice is now at {fileSizeBytes/(1024.0*1024.0*1024.0):F3} GB.");
                        }
                    }
                }
            }, token);
        }
        catch (OperationCanceledException)
        {
            Console.WriteLine("T265 Recording stopped via token.");
        }
        finally
        {
            // Flush remaining data in lists before closing
            if (_ts.Count > 0) WriteBatch(writer);
            
            writer.WriteEnd();
            pipe.Stop();
        }
    }

    //---- HELPERS -------------------------------------------------------------
    private void BufferData(PoseFrame frame)
    {
        var d = frame.PoseData;
        _ts.Add(frame.Timestamp);
        _tx.Add(d.translation.x); _ty.Add(d.translation.y); _tz.Add(d.translation.z);
        _rx.Add(d.rotation.x); _ry.Add(d.rotation.y); _rz.Add(d.rotation.z); _rw.Add(d.rotation.w);
        _vx.Add(d.velocity.x); _vy.Add(d.velocity.y); _vz.Add(d.velocity.z);
        _ax.Add(d.acceleration.x); _ay.Add(d.acceleration.y); _az.Add(d.acceleration.z);
        _avx.Add(d.angular_velocity.x); _avy.Add(d.angular_velocity.y); _avz.Add(d.angular_velocity.z);
        _aax.Add(d.angular_acceleration.x); _aay.Add(d.angular_acceleration.y); _aaz.Add(d.angular_acceleration.z);
        _conf.Add((byte)d.tracker_confidence);
    }

    private void WriteBatch(ArrowFileWriter writer)
    {
        var batch = new RecordBatch(_schema, new IArrowArray[]
        {
            new DoubleArray.Builder().AppendRange(_ts).Build(),
            new FloatArray.Builder().AppendRange(_tx).Build(),
            new FloatArray.Builder().AppendRange(_ty).Build(),
            new FloatArray.Builder().AppendRange(_tz).Build(),
            new FloatArray.Builder().AppendRange(_rx).Build(),
            new FloatArray.Builder().AppendRange(_ry).Build(),
            new FloatArray.Builder().AppendRange(_rz).Build(),
            new FloatArray.Builder().AppendRange(_rw).Build(),
            new FloatArray.Builder().AppendRange(_vx).Build(),
            new FloatArray.Builder().AppendRange(_vy).Build(),
            new FloatArray.Builder().AppendRange(_vz).Build(),
            new FloatArray.Builder().AppendRange(_ax).Build(),
            new FloatArray.Builder().AppendRange(_ay).Build(),
            new FloatArray.Builder().AppendRange(_az).Build(),
            new FloatArray.Builder().AppendRange(_avx).Build(),
            new FloatArray.Builder().AppendRange(_avy).Build(),
            new FloatArray.Builder().AppendRange(_avz).Build(),
            new FloatArray.Builder().AppendRange(_aax).Build(),
            new FloatArray.Builder().AppendRange(_aay).Build(),
            new FloatArray.Builder().AppendRange(_aaz).Build(),
            new UInt8Array.Builder().AppendRange(_conf).Build()
        }, _ts.Count);

        writer.WriteRecordBatch(batch);
        ClearBuffers();
    }

    private void ClearBuffers()
    {
        _ts.Clear(); _tx.Clear(); _ty.Clear(); _tz.Clear();
        _rx.Clear(); _ry.Clear(); _rz.Clear(); _rw.Clear();
        _vx.Clear(); _vy.Clear(); _vz.Clear();
        _ax.Clear(); _ay.Clear(); _az.Clear();
        _avx.Clear(); _avy.Clear(); _avz.Clear();
        _aax.Clear(); _aay.Clear(); _aaz.Clear();
        _conf.Clear();
    }
}