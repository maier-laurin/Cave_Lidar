using System.Text.RegularExpressions;

namespace CaveScan;

/// <summary>Project folders (<c>&lt;root&gt;/&lt;project name&gt;</c>) and the numbered scan folders inside them.</summary>
public static partial class ProjectStore
{
    private static readonly HashSet<string> Reserved = new(StringComparer.OrdinalIgnoreCase)
    {
        "CON", "PRN", "AUX", "NUL",
        "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9",
        "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9",
    };

    // illegal in Windows file names (the data may later be copied to a Windows machine even if it is recorded on Linux)
    private static readonly char[] IllegalOnWindows = "<>:\"/\\|?*".ToCharArray();

    /// <summary>The name is used exactly as typed for the folder, so it must be a legal folder name. Returns the problem or null.</summary>
    public static string? ValidateName(string? name)
    {
        if (string.IsNullOrWhiteSpace(name)) return "The project name is empty.";
        if (name != name.Trim()) return "The project name must not start or end with a space.";
        if (name.EndsWith('.')) return "The project name must not end with a dot.";
        if (name.Length > 100) return "The project name is longer than 100 characters.";

        var bad = name.Where(c => Path.GetInvalidFileNameChars().Contains(c) || IllegalOnWindows.Contains(c)).Distinct().ToArray();
        if (bad.Length > 0) return $"The project name contains characters that are not allowed in a folder name: {string.Join(' ', bad.Select(c => char.IsControl(c) ? $"0x{(int)c:X2}" : c.ToString()))}";

        string stem = name.Split('.')[0];
        if (Reserved.Contains(stem)) return $"\"{stem}\" is a reserved name on Windows.";
        return null;
    }

    public static string ProjectDirectory(string root, string name) => Path.Combine(root, name);

    /// <summary>Numbers of the existing scan_NN folders, ascending.</summary>
    public static IReadOnlyList<int> ExistingScans(string projectDirectory)
    {
        if (!Directory.Exists(projectDirectory)) return Array.Empty<int>();
        return Directory.GetDirectories(projectDirectory)
            .Select(d => ScanFolder().Match(Path.GetFileName(d)))
            .Where(m => m.Success)
            .Select(m => int.Parse(m.Groups[1].Value))
            .OrderBy(n => n)
            .ToArray();
    }

    public static int NextScanNumber(string projectDirectory) => ExistingScans(projectDirectory).DefaultIfEmpty(0).Max() + 1;

    public static string ScanDirectory(string projectDirectory, int number) => Path.Combine(projectDirectory, $"scan_{number:D2}");

    [GeneratedRegex(@"^scan_(\d{2,})$")]
    private static partial Regex ScanFolder();
}
