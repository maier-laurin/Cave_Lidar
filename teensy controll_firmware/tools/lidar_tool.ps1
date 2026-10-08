# Test tool for lidar_ctrl.ino: sends the config line, decodes the binary stream, prints a summary.
#
#   .\lidar_tool.ps1 -Rpm 10 -Seconds 40        send config (rpm 10), watch 40 s, then send x
#   .\lidar_tool.ps1 -Watch -Seconds 20         only listen, send nothing
#   .\lidar_tool.ps1 -Stop                      only send x
#
# The config is sent exactly on a whole second: label = that second + 1 (= time of the first PPS pulse),
# so the Teensy clock is aligned with PC UTC to about a few ms.
param(
  [string]$Port = "COM4",
  [double]$Rpm = 10,
  [int]$Seconds = 30,
  [switch]$Watch,
  [switch]$Stop
)

Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Text;
public class LidarParser {
  List<byte> buf = new List<byte>();
  public Queue<string> Lines = new Queue<string>();
  public int S, E, H, M, CrcBad;            // S counts since the last Reset()
  public int SRate;                          // S packets in the last full second
  public string LastS = "";
  public double LastAxis; public ulong LastT; public int LastDrops; public byte LastFlags;
  public static ushort Crc16(byte[] b, int off, int n) {
    int crc = 0xFFFF;
    for (int k = 0; k < n; k++) {
      crc ^= b[off + k] << 8;
      for (int i = 0; i < 8; i++) crc = ((crc & 0x8000) != 0) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return (ushort)crc;
  }
  public void Rate() { SRate = S; S = 0; }
  public void Feed(byte[] d, int n) {
    for (int i = 0; i < n; i++) buf.Add(d[i]);
    int pos = 0;
    while (buf.Count - pos >= 5) {
      if (buf[pos] != 0xA5) { pos++; continue; }
      int len = buf[pos + 2];
      if (buf.Count - pos < 5 + len) break;
      byte[] p = buf.GetRange(pos, 5 + len).ToArray();
      int crcRx = p[3 + len] | (p[4 + len] << 8);
      if (Crc16(p, 1, 2 + len) != crcRx) { CrcBad++; pos++; continue; }
      pos += 5 + len;
      char t = (char)p[1];
      if (t == 'M') { M++; Lines.Enqueue("[M] " + Encoding.ASCII.GetString(p, 3, len)); }
      else if (t == 'H') {
        H++;
        Lines.Enqueue(string.Format("[H] v{0} steps/rev={1} vertical={2} rpm={3} first_pulse_unix={4}",
          p[3], BitConverter.ToSingle(p, 4), BitConverter.ToSingle(p, 8), BitConverter.ToSingle(p, 12), BitConverter.ToUInt32(p, 16)));
      }
      else if (t == 'E') {
        E++;
        ulong tu = BitConverter.ToUInt64(p, 3);
        Lines.Enqueue(string.Format("[E] edge t={0} steps_before_reset={1}", Fmt(tu), BitConverter.ToInt32(p, 11)));
      }
      else if (t == 'S') {
        S++;
        int o = 3;
        LastT = BitConverter.ToUInt64(p, o);
        int steps = BitConverter.ToInt32(p, o + 8);
        LastAxis = (((steps * 360.0 / 31999.0) - 93.0) % 360.0 + 360.0) % 360.0;
        float[] q = new float[4]; for (int k = 0; k < 4; k++) q[k] = BitConverter.ToSingle(p, o + 12 + 4 * k);
        float[] la = new float[3]; float[] mg = new float[3];
        for (int k = 0; k < 3; k++) { la[k] = BitConverter.ToSingle(p, o + 31 + 4 * k); mg[k] = BitConverter.ToSingle(p, o + 43 + 4 * k); }
        LastFlags = p[o + 55]; LastDrops = p[o + 56];
        LastS = string.Format("t={0} axis={1,6:F1}deg q=({2:F3},{3:F3},{4:F3},{5:F3}) qacc={6:F1}deg cal={7} la=({8:F2},{9:F2},{10:F2}) mag=({11:F1},{12:F1},{13:F1}) flags={14} drops={15}",
          Fmt(LastT), LastAxis, q[0], q[1], q[2], q[3], BitConverter.ToUInt16(p, o + 28) / 100.0, p[o + 30],
          la[0], la[1], la[2], mg[0], mg[1], mg[2], LastFlags, LastDrops);
      }
    }
    buf.RemoveRange(0, pos);
  }
  static string Fmt(ulong us) {
    return new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc).AddTicks((long)us * 10).ToString("HH:mm:ss.fff");
  }
}
'@

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.DtrEnable = $true
$sp.ReadTimeout = 20
$sp.Open()

if ($Stop) { $sp.Write("x`n"); Start-Sleep -Milliseconds 300; $sp.Close(); return }

if (-not $Watch) {
  $target = [DateTimeOffset]::FromUnixTimeSeconds([DateTimeOffset]::UtcNow.ToUnixTimeSeconds() + 1)
  while ([DateTimeOffset]::UtcNow -lt $target) { }
  $label = $target.ToUnixTimeSeconds() + 1
  $sp.Write(("C,{0},{1}`n" -f $Rpm.ToString([Globalization.CultureInfo]::InvariantCulture), $label))
  Write-Host ("sent config: C,{0},{1}  ({2:u})" -f $Rpm, $label, [DateTimeOffset]::FromUnixTimeSeconds($label))
}

$parser = New-Object LidarParser
$tmp = New-Object byte[] 8192
$end = (Get-Date).AddSeconds($Seconds)
$next = (Get-Date).AddSeconds(1)

while ((Get-Date) -lt $end) {
  try { $n = $sp.Read($tmp, 0, $tmp.Length) } catch { $n = 0 }
  if ($n -gt 0) { $parser.Feed($tmp, $n) }
  while ($parser.Lines.Count -gt 0) { Write-Host $parser.Lines.Dequeue() }
  if ((Get-Date) -ge $next) {
    $next = $next.AddSeconds(1)
    $parser.Rate()
    Write-Host ("S/s={0,3} crcBad={1} {2}" -f $parser.SRate, $parser.CrcBad, $parser.LastS)
  }
}

if (-not $Watch) { $sp.Write("x`n") }
$sp.Close()
