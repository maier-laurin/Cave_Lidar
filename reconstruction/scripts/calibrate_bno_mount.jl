# Calibrates the mounting rotation of the BNO085 relative to the rig frame.
#
# Procedure: calibrate the magnetometer (figure-8 movements until the dashboard shows mag 3/3), put the rig on a flat
# level surface, turn it so that one horizontal rig axis points to magnetic north (compass), and record a short scan
# (20-30 s, without moving it). The rig only streams while a scan runs.
#
#   julia --project=reconstruction reconstruction/scripts/calibrate_bno_mount.jl <scan_folder> --north +X [--write] [--force] [--tilt-only]
#
#   --north  which rig axis points to magnetic north: +X, -X, +Y or -Y (X along the shaft, Z up)
#   --skip   seconds to ignore at the start (default 3)
#   --write  store the result in reconstruction/calibration/bno_mount.toml (used by RigParams from then on)
#   --force  write even if the magnetometer calibration was below 3/3 or the pose moved
#   --tilt-only  fit only the tilt of the BNO (from gravity, needs a level pose); the heading stays nominal. Use this when
#            the magnetometer is not trustworthy: a magnetic heading error does not carry over to other places.

using CaveLidarRecon, StaticArrays, Statistics, LinearAlgebra, Printf
import CaveLidarRecon: chunk_files, column, unix_ns, rotation_matrix

function option(args, name, default)
    i = findfirst(==(name), args)
    return i === nothing ? default : args[i+1]
end

function main(args)
    dirs = [a for (i, a) in enumerate(args) if !startswith(a, "--") && !(i > 1 && args[i-1] in ("--north", "--skip"))]
    north = option(args, "--north", nothing)
    (length(dirs) == 1 && north !== nothing) || (println(stderr, "usage: calibrate_bno_mount.jl <scan_folder> --north +X|-X|+Y|-Y [--skip s] [--write] [--force] [--tilt-only]"); exit(2))
    skip = parse(Float64, option(args, "--skip", "3"))
    rig = chunk_files(dirs[1], "rig")
    isempty(rig) && (println(stderr, "no rig_NNNN.arrow files in $(dirs[1])"); exit(1))

    t = unix_ns(column(rig, :time)); sec = (t .- t[1]) ./ 1e9
    keep = sec .>= skip
    q = [SVector{4,Float64}(a, b, c, d) for (a, b, c, d) in zip(column(rig, :qw), column(rig, :qx), column(rig, :qy), column(rig, :qz))]
    keep .&= [all(isfinite, v) && abs(norm(v) - 1) < 0.05 for v in q]
    q = q[keep]
    length(q) < 100 && (println(stderr, "only $(length(q)) usable samples after the first $skip s, record longer"); exit(1))
    cal = column(rig, :cal)[keep]; qacc = column(rig, :q_acc)[keep] ./ 100
    mag = [norm(SVector(Float64(a), Float64(b), Float64(c))) for (a, b, c) in
           zip(column(rig, :mx)[keep], column(rig, :my)[keep], column(rig, :mz)[keep])]
    cal_mag = minimum((cal .>> 2) .& 0x03)
    cal_rot = minimum(cal .& 0x03)

    qm, spread = mean_quaternion(q)
    tilt_only = "--tilt-only" in args
    M_full = bno_mount_from_pose(qm, north)
    M = tilt_only ? tilt_mount_from_pose(qm) : M_full
    uscatter = up_scatter(q)
    d = mount_difference(M)

    # the same pose seen at the start and the end of the window, to show drift
    h = length(q) ÷ 2
    drift = rad2deg(2acos(min(1.0, abs(dot(mean_quaternion(q[1:h])[1], mean_quaternion(q[h+1:end])[1])))))

    @printf("%s: %d samples (%.0f s), north = %s\n", dirs[1], length(q), sec[keep][end] - sec[keep][1], north)
    @printf("  BNO status:  magnetometer cal %d/3 (min), rotation cal %d/3 (min), accuracy %.1f deg (median)\n", cal_mag, cal_rot, median(qacc))
    @printf("  field |B|:   %.1f uT (median)\n", median(mag))
    @printf("  pose:        RMS scatter %.3f deg (up direction only %.3f deg), first half vs second half %.3f deg\n", spread, uscatter, drift)
    @printf("  mount vs nominal: %.2f deg in total = BNO z tilted by %.2f deg, heading %+.2f deg\n", d.total, d.tilt, d.heading)
    println("  bno_from_frame (rows, rig-frame -> BNO):")
    for i in 1:3
        @printf("    [%9.6f %9.6f %9.6f]\n", M[i, 1], M[i, 2], M[i, 3])
    end

    problems = String[]
    if tilt_only
        println("  tilt only: the heading part stays nominal, the magnetometer is not used")
        println("  (a full fit would have changed the heading by $(round(mount_difference(M_full).heading; digits = 2)) deg)")
        uscatter > 0.3 && push!(problems, "the pose moved (scatter of the up direction above 0.3 deg)")
    else
        cal_mag < 3 && push!(problems, "magnetometer calibration below 3/3: the heading part is not trustworthy")
        spread > 0.3 && push!(problems, "the pose moved (scatter above 0.3 deg)")
        drift > 0.5 && push!(problems, "the heading drifts between the halves (above 0.5 deg)")
    end
    foreach(p -> println("  WARNING: ", p), problems)

    if "--write" in args
        if !isempty(problems) && !("--force" in args)
            println("not written (use --force to write anyway)"); exit(1)
        end
        path = save_bno_mount(M, Dict("source" => abspath(dirs[1]), "north_axis" => north, "mode" => (tilt_only ? "tilt only" : "tilt and heading"),
            "heading_full_fit_deg" => mount_difference(M_full).heading, "up_scatter_deg" => uscatter, "samples" => length(q),
            "cal_mag" => Int(cal_mag), "cal_rot" => Int(cal_rot), "q_acc_deg" => median(qacc), "field_uT" => median(mag),
            "scatter_deg" => spread, "tilt_vs_nominal_deg" => d.tilt, "heading_vs_nominal_deg" => d.heading))
        println("written: ", abspath(path))
    else
        println("(dry run, nothing written; add --write to store it)")
    end
end

main(ARGS)
