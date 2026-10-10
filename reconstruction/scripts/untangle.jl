# Untangles the lidar points of a scan folder (or of every scan folder in a project folder) into <scan>/untangled.arrow.
#
#   julia --project=reconstruction reconstruction/scripts/untangle.jl <scan_or_project_folder> [--world | --no-attitude]
#
# default        z = up from gravity (BNO tilt); the heading about the vertical stays the rig's own
# --world        full BNO attitude, East-North-Up (heading from the magnetometer)
# --no-attitude  keep the points in the rig frame (no BNO rotation)

using CaveLidarRecon, Printf

function main(args)
    attitude = "--no-attitude" in args ? :none : "--world" in args ? :world : :gravity
    dirs = filter(a -> !startswith(a, "--"), args)
    length(dirs) == 1 || (println(stderr, "usage: untangle.jl <scan_or_project_folder> [--world | --no-attitude]"); exit(2))
    scans = find_scans(dirs[1])
    isempty(scans) && (println(stderr, "no scan folder (with lidar_NNNN.arrow files) found in $(dirs[1])"); exit(1))
    for scan in scans
        out = joinpath(scan, "untangled.arrow")
        t = @elapsed c = untangle_scan(scan; out, attitude)
        @printf("%s\n  read %d, written %d  (too close %d, no rig data %d, no attitude %d)\n  -> %s  [%.1f s]\n",
            scan, c.read, c.written, c.too_close, c.no_rig, c.no_attitude, out, t)
    end
end

main(ARGS)
