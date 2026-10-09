using System.Net;
using System.Net.Sockets;
using System.Text;

static class Sniff
{
    /// <summary>Prints a few packets of the lidar's telemetry port (position packets with the NMEA sentence it received).</summary>
    public static int Run(int port, int count)
    {
        using var udp = new UdpClient(port);
        udp.Client.ReceiveTimeout = 5000;
        var ep = new IPEndPoint(IPAddress.Any, 0);
        try
        {
            for (int i = 0; i < count; i++)
            {
                byte[] b = udp.Receive(ref ep);
                var sb = new StringBuilder();
                foreach (byte c in b.Skip(190)) sb.Append(c is >= 0x20 and < 0x7f ? (char)c : ' ');
                string text = string.Join(' ', sb.ToString().Split(' ', StringSplitOptions.RemoveEmptyEntries));
                uint micros = b.Length >= 202 ? BitConverter.ToUInt32(b, 198) : 0;
                Console.WriteLine($"from {ep.Address}:{ep.Port}, {b.Length} bytes, timestamp {micros / 1e6:F3} s of the hour, text: {text}");
            }
        }
        catch (SocketException e)
        {
            Console.WriteLine($"nothing received: {e.SocketErrorCode}");
            return 1;
        }
        return 0;
    }
}
