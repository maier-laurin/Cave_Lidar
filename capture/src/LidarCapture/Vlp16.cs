using System.Buffers.Binary;

namespace LidarCapture;

/// <summary>Constants of the VLP-16 data packet (1206 bytes, 12 blocks of 2 firing sequences x 16 lasers).</summary>
public static class Vlp16
{
    public const int PacketSize = 1206;
    public const int BlockCount = 12;
    public const int BlockBytes = 100;
    public const int PointsPerPacket = BlockCount * 2 * 16;

    public const long BlockNs = 110_592;
    public const long SequenceNs = 55_296;
    public const long LaserNs = 2_304;

    /// <summary>Factory bytes at the end of the packet.</summary>
    public const byte ModelVlp16 = 0x22;
    public const byte ReturnStrongest = 0x37;

    /// <summary>Elevation in degrees for laser id 0..15.</summary>
    public static readonly sbyte[] ElevationDeg = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };

    public static uint TimestampMicros(ReadOnlySpan<byte> packet) => BinaryPrimitives.ReadUInt32LittleEndian(packet[1200..]);
    public static byte ReturnMode(ReadOnlySpan<byte> packet) => packet[1204];
    public static byte Model(ReadOnlySpan<byte> packet) => packet[1205];
}

/// <summary>Column buffers for the points of several packets; flushed to Arrow in one go.</summary>
public sealed class PointBatch
{
    public readonly long[] TimeNs;
    public readonly ushort[] Azimuth;
    public readonly sbyte[] Elevation;
    public readonly ushort[] Distance;
    public readonly byte[] Reflectivity;
    public int Count;

    public PointBatch(int capacity)
    {
        TimeNs = new long[capacity];
        Azimuth = new ushort[capacity];
        Elevation = new sbyte[capacity];
        Distance = new ushort[capacity];
        Reflectivity = new byte[capacity];
    }

    public int Capacity => TimeNs.Length;
    public void Clear() => Count = 0;
}

public static class Vlp16Parser
{
    /// <summary>
    /// Appends the valid points of one packet to <paramref name="batch"/>. Points with distance 0 (no return) are dropped.
    /// Time is absolute UTC in ns since 1970, azimuth is in 0.01 deg (interpolated per firing), distance in 2 mm.
    /// </summary>
    /// <param name="packetStartNs">Absolute time of the first firing of the packet.</param>
    public static void Parse(ReadOnlySpan<byte> p, long packetStartNs, PointBatch batch)
    {
        int n = batch.Count;
        int diff = 0; // azimuth change per block in 0.01 deg, the last block reuses the one before

        for (int b = 0; b < Vlp16.BlockCount; b++)
        {
            int o = b * Vlp16.BlockBytes;
            if (p[o] != 0xFF || p[o + 1] != 0xEE) continue;

            int az = BinaryPrimitives.ReadUInt16LittleEndian(p[(o + 2)..]);
            if (b < Vlp16.BlockCount - 1)
            {
                int next = BinaryPrimitives.ReadUInt16LittleEndian(p[(o + 102)..]);
                int d = next - az;
                if (d < 0) d += 36000;
                if (d <= 400) diff = d; // 4 deg per block is far above the 20 rpm maximum, anything above is garbage
            }

            long blockNs = packetStartNs + b * Vlp16.BlockNs;

            for (int s = 0; s < 2; s++)
            {
                for (int l = 0; l < 16; l++)
                {
                    int e = o + 4 + (s * 16 + l) * 3;
                    ushort dist = BinaryPrimitives.ReadUInt16LittleEndian(p[e..]);
                    if (dist == 0) continue;

                    long off = s * Vlp16.SequenceNs + l * Vlp16.LaserNs;
                    int a = (int)Math.Round(az + diff * (double)off / Vlp16.BlockNs) % 36000;

                    batch.TimeNs[n] = blockNs + off;
                    batch.Azimuth[n] = (ushort)a;
                    batch.Elevation[n] = Vlp16.ElevationDeg[l];
                    batch.Distance[n] = dist;
                    batch.Reflectivity[n] = p[e + 2];
                    n++;
                }
            }
        }

        batch.Count = n;
    }
}
