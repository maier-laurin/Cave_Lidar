using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using Apache.Arrow;
using Apache.Arrow.Ipc;
using Apache.Arrow.Types;
using System.Buffers.Binary;
using System.Threading.Channels;
using System.Diagnostics;

namespace Velodyne;

public class VelodyneStream
{
    //~ We are going to organice the logic in two threads, one is just listening on the UDPC Port for raw packages
    //~ and writes them to a que, the second one is consuming from the que transforming the data in a tabular format
    //~ and after filling a buffer writting it chunckwise to the disk. having this que allows us to still capture all 
    //~ packages even if writting to the disc takes for a long time 

    private const int Port = 3201; 
    private readonly Channel<byte[]> _packetChannel; // We use a Channel to act as our "Wait-Area" buffer
    private static readonly sbyte[] _VerticalAngles = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 }; //map of sensor id to sensor angle

    public VelodyneStream()
    {
        // We "Bound" the channel to a max capacity (e.g., 10,000 packets) 
        // to prevent RAM from exploding if the disk completely fails.
        _packetChannel = Channel.CreateBounded<byte[]>(10000);
    }

    //---- THE PRODUCER --------------------------------------------------------

    
    public async Task ProducePackets(CancellationToken ct)
    {
        using var udpClient = new UdpClient(Port);
        Console.WriteLine($"[Producer] Listening on port {Port}...");

        try
        {
            while (!ct.IsCancellationRequested)
            {
                var result = await udpClient.ReceiveAsync(ct);
                
                if (result.Buffer.Length == 1206)
                {
                    // Drop it into the channel and immediately go back to listening
                    await _packetChannel.Writer.WriteAsync(result.Buffer, ct);
                }
            }
        }
        catch (OperationCanceledException) { /* Normal exit */ }
        finally { _packetChannel.Writer.Complete(); }
    }

    //---- THE CONSUMER --------------------------------------------------------

    public async Task ConsumeAndWrite(string filePath, int batchSize, CancellationToken ct)
    {
        var (builders, schema) = SetupArrowComponents(batchSize);
        
        using var stream = File.OpenWrite(filePath);
        using var writer = new ArrowFileWriter(stream, schema);
        Stopwatch diskTimer = new Stopwatch();
        int packetsInCurrentBatch = 0;
        Console.WriteLine($"[Consumer] Writing to {filePath}...");

        try
        {
            // This loop waits for data to appear in the channel
            await foreach (var data in _packetChannel.Reader.ReadAllAsync(ct))
            {
                // Parse the data into our builders
                ParseIntoBuilders(data, builders);
                packetsInCurrentBatch++;

                // If we hit our batch size flush to disk
                if (packetsInCurrentBatch >= batchSize)
                {
                    long totalBytes = (long)builders.Time.Length * 18; // one row has exactly 18 bytes (Time: 8, Azimuth: 4, Distance: 4, Reflectivity: 1, Angle: 4) 
                    long fileSizeBytes = new FileInfo(filePath).Length;
                    diskTimer.Restart();
                    await WriteBatch(writer, schema, builders);
                    diskTimer.Stop();
                    ClearBuilders(builders);
                    packetsInCurrentBatch = 0;
                    Console.WriteLine($"New Velodine batch with {totalBytes/(1024.0*1024.0):F2} MB written to Disk, queue currently at {_packetChannel.Reader.Count} packages, this write took {diskTimer.ElapsedMilliseconds} ms, the total filesice is now at {fileSizeBytes/(1024.0*1024.0*1024.0):F3} GB.");
                }
            }
        }
        catch (OperationCanceledException) { /* Exit requested */ }
        finally
        {
            // Write any remaining points that didn't fill a full batch
            if (packetsInCurrentBatch > 0)
            {
                await WriteBatch(writer, schema, builders);
            }
            await writer.WriteEndAsync();
        }
    }

    //---- HELPERS -------------------------------------------------------------
    private (ListBuilders builders, Schema schema) SetupArrowComponents(int batchSize)
    {
        //§ returns the components that can hold the points from one vlp16 packet...
        int pointsPerBatch = batchSize * 384;

        // We bundle them into a simple record or class to keep the code clean
        var builders = new ListBuilders
        {
            Time = new DoubleArray.Builder().Reserve(pointsPerBatch),
            Azimuth = new FloatArray.Builder().Reserve(pointsPerBatch),
            Distance = new FloatArray.Builder().Reserve(pointsPerBatch),
            Reflectivity = new UInt8Array.Builder().Reserve(pointsPerBatch),
            Angle = new Int8Array.Builder().Reserve(pointsPerBatch)
        };

        var schema = new Schema.Builder()
            .Field(new Field("time", DoubleType.Default, false))
            .Field(new Field("azimuth", FloatType.Default, false))
            .Field(new Field("distance", FloatType.Default, false))
            .Field(new Field("reflectivity", UInt8Type.Default, false))
            .Field(new Field("angle", Int8Type.Default, false))
            .Build();

        return (builders, schema);
    }

    private async Task WriteBatch(ArrowFileWriter writer, Schema schema, ListBuilders b)
    {
        //§ Build the record batch from current builder states
        using var recordBatch = new RecordBatch(schema, new IArrowArray[]
        {
            b.Time.Build(),
            b.Azimuth.Build(),
            b.Distance.Build(),
            b.Reflectivity.Build(),
            b.Angle.Build()
        }, b.Time.Length);

        await writer.WriteRecordBatchAsync(recordBatch);
    }

    private void ClearBuilders(ListBuilders b)
    {
        //§ Clear the data but KEEP the allocated memory.
        b.Time.Clear();
        b.Azimuth.Clear();
        b.Distance.Clear();
        b.Reflectivity.Clear();
        b.Angle.Clear();
    }

    private void ParseIntoBuilders(byte[] data,  ListBuilders lb)
    {
        // Get the packet timestamp
        // Use BinaryPrimitives for better performance than BitConverter
        uint packetTimestampMicros = BinaryPrimitives.ReadUInt32LittleEndian(data.AsSpan(1200));

        const double SequenceDuration = 55.296;
        const double LaserInterval = 2.304;
        const double BlockDuration = 110.592;

        double azimuthSpeed = 0;

        for (int b = 0; b < 12; b++)
        {
            int blockOffset = b * 100;
            
            if (data[blockOffset] != 0xFF || data[blockOffset + 1] != 0xEE) continue;

            ushort blockAzimuth = BinaryPrimitives.ReadUInt16LittleEndian(data.AsSpan(blockOffset + 2));
            
            if (b < 11)
            {
                ushort nextAzimuth = BinaryPrimitives.ReadUInt16LittleEndian(data.AsSpan(blockOffset + 102));
                // Handle wrap-around at 360° (36000 in 0.01° units)
                int diff = nextAzimuth - blockAzimuth;
                if (diff < 0) diff += 36000;
                azimuthSpeed = diff / BlockDuration;
            }

            double blockBaseTime = packetTimestampMicros + (b * BlockDuration);

            for (int s = 0; s < 2; s++) // 2 sequences per block
            {
                double sequenceBaseTime = blockBaseTime + (s * SequenceDuration);
                int sequenceByteOffset = blockOffset + 4 + (s * 16 * 3);

                for (int l = 0; l < 16; l++) // 16 lasers per sequence
                {
                    int entryOffset = sequenceByteOffset + (l * 3);
                    ushort distRaw = BinaryPrimitives.ReadUInt16LittleEndian(data.AsSpan(entryOffset));
                    
                    // Calculations
                    double exactTime = sequenceBaseTime + (l * LaserInterval);
                    float interpolationTime = (float)(exactTime - blockBaseTime);
                    float currentAzimuth = (blockAzimuth + (float)(azimuthSpeed * interpolationTime)) / 100f;
                    if (currentAzimuth >= 360) currentAzimuth -= 360;
                    float distanceMeters = distRaw * 0.002f; // distance is in 2mm increments

                    // PUSH DIRECTLY TO ARROW BUILDERS (Zero intermediate objects)
                    lb.Time.Append(exactTime);
                    lb.Azimuth.Append(currentAzimuth);
                    lb.Distance.Append(distanceMeters);
                    lb.Reflectivity.Append(data[entryOffset + 2]);
                    lb.Angle.Append(_VerticalAngles[l]);
                }
            }
        }
    }
    public class ListBuilders
    {
        //§ a container to avoid passing 5 parameters everywhere
        public DoubleArray.Builder Time { get; set; } = null!;
        public FloatArray.Builder Azimuth { get; set; } = null!;
        public FloatArray.Builder Distance { get; set; } = null!;
        public UInt8Array.Builder Reflectivity { get; set; } = null!;
        public Int8Array.Builder Angle { get; set; } = null!;
    }
}