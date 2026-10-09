using System.Buffers.Binary;

namespace TeensyCapture;

/// <summary>One IMU / step sample ('S' packet, 100 per second). Times are simulated UTC microseconds since 1970.</summary>
public readonly record struct TeensySample(
    long TimeUs, int AxisSteps,
    float Qw, float Qx, float Qy, float Qz, ushort QAcc, byte Cal,
    float Lax, float Lay, float Laz, float Mx, float My, float Mz,
    byte Flags, byte Drops)
{
    public bool IndexSeen => (Flags & 1) != 0;
    public bool Running => (Flags & 2) != 0;
    public int CalRotation => Cal & 3;
    public int CalMagnet => (Cal >> 2) & 3;
}

/// <summary>Index edge ('E' packet), once per revolution.</summary>
public readonly record struct TeensyEdge(long TimeUs, int StepsBeforeReset);

/// <summary>Stream header ('H' packet), at the start and every 10 s.</summary>
public readonly record struct TeensyHeader(byte Version, float StepsPerRev, float VerticalAfterEdgeDeg, float Rpm, uint FirstPulseUnixS);

/// <summary>
/// Decodes the binary stream of the Teensy: 0xA5, type, len, payload, CRC-16/CCITT-FALSE (low byte first).
/// A packet with a wrong CRC is dropped and the search for the next marker continues one byte further.
/// </summary>
public sealed class TeensyPacketParser
{
    public const int SampleLength = 57, EdgeLength = 12, HeaderLength = 17;

    public Action<TeensySample>? OnSample { get; set; }
    public Action<TeensyEdge>? OnEdge { get; set; }
    public Action<TeensyHeader>? OnHeader { get; set; }
    public Action<string>? OnMessage { get; set; }

    public long CrcErrors { get; private set; }

    private byte[] _buf = new byte[4096];
    private int _count;

    public void Feed(ReadOnlySpan<byte> data)
    {
        if (_count + data.Length > _buf.Length) Array.Resize(ref _buf, Math.Max(_buf.Length * 2, _count + data.Length));
        data.CopyTo(_buf.AsSpan(_count));
        _count += data.Length;

        int pos = 0;
        while (true)
        {
            while (pos < _count && _buf[pos] != 0xA5) pos++;
            if (_count - pos < 3) break; // marker, type, len

            byte type = _buf[pos + 1];
            int len = _buf[pos + 2];
            if (!PlausibleLength(type, len))
            {
                pos++; // a stray 0xA5, do not wait for a packet that cannot exist
                continue;
            }
            int total = 3 + len + 2;
            if (_count - pos < total) break; // wait for the rest

            var packet = _buf.AsSpan(pos, total);
            ushort crc = Crc16(packet.Slice(1, 2 + len));
            ushort sent = BinaryPrimitives.ReadUInt16LittleEndian(packet[(3 + len)..]);

            if (crc == sent && Dispatch(type, packet.Slice(3, len)))
            {
                pos += total;
            }
            else
            {
                CrcErrors++;
                pos++; // not a packet, a 0xA5 inside the payload of a damaged one, or garbage: look for the next marker
            }
        }

        if (pos > 0)
        {
            Buffer.BlockCopy(_buf, pos, _buf, 0, _count - pos);
            _count -= pos;
        }
    }

    private static bool PlausibleLength(byte type, int len) => (char)type switch
    {
        'S' => len == SampleLength,
        'E' => len == EdgeLength,
        'H' => len == HeaderLength,
        'M' => len <= 119,
        _ => false,
    };

    private bool Dispatch(byte type, ReadOnlySpan<byte> p)
    {
        switch ((char)type)
        {
            case 'S' when p.Length == SampleLength:
                OnSample?.Invoke(new TeensySample(
                    (long)BinaryPrimitives.ReadUInt64LittleEndian(p),
                    BinaryPrimitives.ReadInt32LittleEndian(p[8..]),
                    F(p, 12), F(p, 16), F(p, 20), F(p, 24),
                    BinaryPrimitives.ReadUInt16LittleEndian(p[28..]), p[30],
                    F(p, 31), F(p, 35), F(p, 39), F(p, 43), F(p, 47), F(p, 51),
                    p[55], p[56]));
                return true;
            case 'E' when p.Length == EdgeLength:
                OnEdge?.Invoke(new TeensyEdge((long)BinaryPrimitives.ReadUInt64LittleEndian(p), BinaryPrimitives.ReadInt32LittleEndian(p[8..])));
                return true;
            case 'H' when p.Length == HeaderLength:
                OnHeader?.Invoke(new TeensyHeader(p[0], F(p, 1), F(p, 5), F(p, 9), BinaryPrimitives.ReadUInt32LittleEndian(p[13..])));
                return true;
            case 'M':
                OnMessage?.Invoke(System.Text.Encoding.ASCII.GetString(p));
                return true;
            default:
                return false;
        }
    }

    private static float F(ReadOnlySpan<byte> p, int offset) => BinaryPrimitives.ReadSingleLittleEndian(p[offset..]);

    public static ushort Crc16(ReadOnlySpan<byte> data)
    {
        ushort crc = 0xFFFF;
        foreach (byte b in data)
        {
            crc ^= (ushort)(b << 8);
            for (int i = 0; i < 8; i++) crc = (crc & 0x8000) != 0 ? (ushort)((crc << 1) ^ 0x1021) : (ushort)(crc << 1);
        }
        return crc;
    }

    /// <summary>Builds a packet, used by the tests.</summary>
    public static byte[] Encode(char type, ReadOnlySpan<byte> payload)
    {
        var packet = new byte[3 + payload.Length + 2];
        packet[0] = 0xA5;
        packet[1] = (byte)type;
        packet[2] = (byte)payload.Length;
        payload.CopyTo(packet.AsSpan(3));
        BinaryPrimitives.WriteUInt16LittleEndian(packet.AsSpan(3 + payload.Length), Crc16(packet.AsSpan(1, 2 + payload.Length)));
        return packet;
    }
}
