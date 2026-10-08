# Does the spinning motor disturb the BNO085 magnetometer?
# Keep the FRAME STILL (don't touch it, no tools or phones next to it) while this runs.
#
#   .\mag_test.ps1                       10 rpm, 5 revolutions
#   .\mag_test.ps1 -Rpm 5 -Revs 3 -Csv mag.csv
#
# Needs lidar_ctrl.ino on the Teensy. Sequence: config -> ~10 s still (baseline) -> spin -> x -> still again.
# Report: mean/noise of the magnetic field in the three phases, the shift while spinning, and how the
# field varies with the axis angle (36 bins of 10 deg, plus the 1x-per-revolution amplitude).
param(
  [string]$Port = "COM4",
  [double]$Rpm = 10,
  [int]$Revs = 5,
  [double]$HorizontalUT = 20,    # horizontal part of the earth field at your location (central Europe ~20 uT)
  [string]$Csv = ""
)

Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

public class MagCollector {
  List<byte> buf = new List<byte>();
  public Queue<string> Lines = new Queue<string>();
  public List<double[]> Samples = new List<double[]>();   // 0 t_s(unix) 1 axis_deg 2 run 3 mx 4 my 5 mz 6 cal
  public int CrcBad, Edges;
  public bool RunSeen, RunEnded, RunNow;
  public DateTime RunStartWall = DateTime.MaxValue, RunEndWall = DateTime.MaxValue;

  static ushort Crc16(byte[] b, int off, int n) {
    int crc = 0xFFFF;
    for (int k = 0; k < n; k++) {
      crc ^= b[off + k] << 8;
      for (int i = 0; i < 8; i++) crc = ((crc & 0x8000) != 0) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return (ushort)crc;
  }

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
      if (t == 'M') Lines.Enqueue("[M] " + Encoding.ASCII.GetString(p, 3, len));
      else if (t == 'E') Edges++;
      else if (t == 'S') {
        int o = 3;
        ulong tu = BitConverter.ToUInt64(p, o);
        int steps = BitConverter.ToInt32(p, o + 8);
        double axis = (((steps * 360.0 / 31999.0) - 93.0) % 360.0 + 360.0) % 360.0;
        byte flags = p[o + 55];
        bool run = (flags & 2) != 0;
        if (run && !RunSeen) { RunSeen = true; RunStartWall = DateTime.UtcNow; }
        if (RunSeen && !run && !RunEnded) { RunEnded = true; RunEndWall = DateTime.UtcNow; }
        RunNow = run;
        Samples.Add(new double[] { tu / 1e6, axis, run ? 1 : 0,
          BitConverter.ToSingle(p, o + 43), BitConverter.ToSingle(p, o + 47), BitConverter.ToSingle(p, o + 51), p[o + 30] });
      }
    }
    buf.RemoveRange(0, pos);
  }

  static string F(string fmt, params object[] a) { return string.Format(CultureInfo.InvariantCulture, fmt, a); }

  List<double[]> Seg(int from, int to) {
    var l = new List<double[]>();
    for (int i = Math.Max(from, 0); i <= to && i < Samples.Count; i++) {
      var s = Samples[i];
      if (!double.IsNaN(s[3]) && !double.IsNaN(s[4]) && !double.IsNaN(s[5])) l.Add(s);
    }
    return l;
  }

  static void MeanStd(List<double[]> l, int col, out double mean, out double std) {
    double sum = 0; foreach (var s in l) sum += s[col];
    mean = l.Count > 0 ? sum / l.Count : double.NaN;
    double v = 0; foreach (var s in l) v += (s[col] - mean) * (s[col] - mean);
    std = l.Count > 1 ? Math.Sqrt(v / (l.Count - 1)) : double.NaN;
  }

  static string SegLine(string name, List<double[]> l, double[] meanOut, double[] stdOut) {
    if (l.Count < 20) return F("  {0,-12} too few samples ({1})", name, l.Count);
    double mx, my, mz, sx, sy, sz;
    MeanStd(l, 3, out mx, out sx); MeanStd(l, 4, out my, out sy); MeanStd(l, 5, out mz, out sz);
    var mag = new List<double[]>();
    foreach (var s in l) mag.Add(new double[] { 0, 0, 0, Math.Sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]) });
    double mm, ms; MeanStd(mag, 3, out mm, out ms);
    meanOut[0] = mx; meanOut[1] = my; meanOut[2] = mz; stdOut[0] = sx; stdOut[1] = sy; stdOut[2] = sz;
    return F("  {0,-12} n={1,5}  mean=({2,7:F2},{3,7:F2},{4,7:F2}) uT  noise(std)=({5:F2},{6:F2},{7:F2})  |B|={8:F2} +-{9:F2}",
             name, l.Count, mx, my, mz, sx, sy, sz, mm, ms);
  }

  public string Report(double horizontalUT) {
    var sb = new StringBuilder();
    int iStart = -1, iEnd = -1;
    for (int i = 0; i < Samples.Count; i++) if (Samples[i][2] > 0) { if (iStart < 0) iStart = i; iEnd = i; }
    if (iStart < 0) return "No rotation seen in the stream, nothing to analyse.";

    var before = Seg(0, iStart - 1);
    var spin = Seg(iStart + 150, iEnd);          // skip 1.5 s of ramp-up
    var after = Seg(iEnd + 200, Samples.Count - 1);  // skip 2 s of ramp-down settling
    double[] mb = new double[3], sb_ = new double[3], ms = new double[3], ss = new double[3], ma = new double[3], sa = new double[3];
    sb.AppendLine("Phases (magnetic field in the sensor frame, frame must have stayed still):");
    sb.AppendLine(SegLine("still before", before, mb, sb_));
    sb.AppendLine(SegLine("spinning", spin, ms, ss));
    sb.AppendLine(SegLine("still after", after, ma, sa));
    if (before.Count < 20 || spin.Count < 200) return sb.ToString() + "Not enough data for the angle analysis.";

    sb.AppendLine();
    sb.AppendLine(F("Shift of the mean while spinning vs. still before: dx={0:F2} dy={1:F2} dz={2:F2} uT  (noise of the still baseline: {3:F2}/{4:F2}/{5:F2})",
      ms[0] - mb[0], ms[1] - mb[1], ms[2] - mb[2], sb_[0], sb_[1], sb_[2]));
    if (after.Count >= 20)
      sb.AppendLine(F("Drift between still before and still after:                dx={0:F2} dy={1:F2} dz={2:F2} uT  (a drift this size is NOT caused by the angle)",
        ma[0] - mb[0], ma[1] - mb[1], ma[2] - mb[2]));

    // dependence on the axis angle while spinning
    const int NB = 36;
    var sum = new double[3, NB]; var cnt = new int[NB];
    double[] a = new double[3], b = new double[3];
    foreach (var s in spin) {
      int bin = (int)(s[1] / 360.0 * NB) % NB;
      double th = s[1] * Math.PI / 180.0;
      for (int k = 0; k < 3; k++) {
        double d = s[3 + k] - ms[k];
        sum[k, bin] += s[3 + k];
        a[k] += d * Math.Cos(th); b[k] += d * Math.Sin(th);
      }
      cnt[bin]++;
    }
    sb.AppendLine();
    sb.AppendLine("Dependence on the axis angle while spinning (36 bins of 10 deg):");
    double maxDev = 0; string names = "xyz";
    for (int k = 0; k < 3; k++) {
      double lo = double.MaxValue, hi = double.MinValue;
      for (int j = 0; j < NB; j++) if (cnt[j] >= 5) { double m = sum[k, j] / cnt[j]; lo = Math.Min(lo, m); hi = Math.Max(hi, m); }
      double amp = 2.0 / spin.Count * Math.Sqrt(a[k] * a[k] + b[k] * b[k]);
      sb.AppendLine(F("  {0}: peak-to-peak of the bin means = {1:F2} uT, 1x-per-revolution amplitude = {2:F2} uT", names[k], hi - lo, amp));
    }
    for (int j = 0; j < NB; j++) if (cnt[j] >= 5) {
      double dx = sum[0, j] / cnt[j] - ms[0], dy = sum[1, j] / cnt[j] - ms[1], dz = sum[2, j] / cnt[j] - ms[2];
      maxDev = Math.Max(maxDev, Math.Sqrt(dx * dx + dy * dy + dz * dz));
    }
    double shift = Math.Sqrt(Math.Pow(ms[0] - mb[0], 2) + Math.Pow(ms[1] - mb[1], 2) + Math.Pow(ms[2] - mb[2], 2));
    double worst = Math.Max(maxDev, shift);
    double headingDeg = Math.Asin(Math.Min(1.0, worst / horizontalUT)) * 180.0 / Math.PI;
    sb.AppendLine();
    sb.AppendLine(F("Largest angle-dependent deviation from the spin mean: {0:F2} uT;  constant shift while spinning: {1:F2} uT", maxDev, shift));
    sb.AppendLine(F("Worst case heading error if that were all horizontal (earth horizontal field {0:F0} uT): about {1:F1} deg", horizontalUT, headingDeg));
    string verdict = worst < 0.5 ? "NO measurable influence of the motor on the magnetometer."
                   : worst < 2.0 ? "SMALL influence (visible, probably tolerable)."
                   : "SIGNIFICANT influence: the heading will be disturbed by the spinning motor.";
    sb.AppendLine("Verdict: " + verdict);
    return sb.ToString();
  }

  public void WriteCsv(string path) {
    using (var w = new StreamWriter(path)) {
      w.WriteLine("t_unix_s,axis_deg,running,mx_uT,my_uT,mz_uT,cal");
      foreach (var s in Samples)
        w.WriteLine(F("{0:F3},{1:F3},{2},{3:F3},{4:F3},{5:F3},{6}", s[0], s[1], s[2], s[3], s[4], s[5], s[6]));
    }
  }
}
'@

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.DtrEnable = $true
$sp.ReadTimeout = 20
$sp.Open()
$parser = New-Object MagCollector
$tmp = New-Object byte[] 8192

function Pump([double]$sec) {
  $end = (Get-Date).AddSeconds($sec)
  while ((Get-Date) -lt $end) {
    try { $n = $sp.Read($tmp, 0, $tmp.Length) } catch { $n = 0 }
    if ($n -gt 0) { $parser.Feed($tmp, $n) }
    while ($parser.Lines.Count -gt 0) { Write-Host $parser.Lines.Dequeue() }
  }
}

# make sure nothing is spinning from an earlier run
$sp.Write("x`n"); Pump 1.5
$guard = (Get-Date).AddSeconds(30)
while ($parser.RunNow -and (Get-Date) -lt $guard) { Pump 0.5 }
$parser.Samples.Clear()

$spinSec = $Revs * 60.0 / $Rpm
$target = [DateTimeOffset]::FromUnixTimeSeconds([DateTimeOffset]::UtcNow.ToUnixTimeSeconds() + 1)
while ([DateTimeOffset]::UtcNow -lt $target) { }
$label = $target.ToUnixTimeSeconds() + 1
$sp.Write(("C,{0},{1}`n" -f $Rpm.ToString([Globalization.CultureInfo]::InvariantCulture), $label))
Write-Host ("config sent: {0} rpm, {1} revolutions = {2:F0} s of spinning. Keep the frame still." -f $Rpm, $Revs, $spinSec)

# wait for the rotation to start (about 10 s after the config)
$guard = (Get-Date).AddSeconds(25)
while (-not $parser.RunSeen -and (Get-Date) -lt $guard) { Pump 0.2 }
if (-not $parser.RunSeen) { Write-Host "Rotation never started (see [M] messages above)."; $sp.Close(); return }

Pump $spinSec
$sp.Write("x`n")
Write-Host "x sent, waiting for the axis to stop at 0 deg..."
$guard = (Get-Date).AddSeconds(30)
while (-not $parser.RunEnded -and (Get-Date) -lt $guard) { Pump 0.2 }
Pump 8                                   # still-after baseline (first 2 s are skipped in the analysis)
$sp.Close()

if ($Csv) { $parser.WriteCsv($Csv); Write-Host "samples written to $Csv" }
Write-Host ""
Write-Host $parser.Report($HorizontalUT)
Write-Host ("(CRC errors: {0})" -f $parser.CrcBad)
