# Spline odometry on a real recording (see src/spline_odometry.jl): IMU path plus a cubic B-spline correction fitted to the
# lidar points. The rig has to stand still at the start (the first --seed seconds, default 31.5).
#
#   julia --project=reconstruction -t auto reconstruction/scripts/spline_odometry.jl <scan_folder> [--knot=0.2] [--step=0.4]
#         [--refine=4] [--sigma-acc=1.0] [--sigma-yaw=0.3] [--seed=31.5] [--min-range=0] [--tag=spline] [--ply-step=20]
#         [--compare-old] [--verbose]
#
# Writes <scan>/<tag>_path.csv (t, x, y, z, yaw_deg, rms, n_used every 0.1 s) and <scan>/<tag>_cloud.ply (every N-th point
# placed with the path). With --compare-old the window odometry runs too and both are scored the same way: how far the
# moving points lie from the planes of the standstill map (an independent check, small = sharp walls).

using CaveLidarRecon, Arrow, StaticArrays, Printf, Statistics, LinearAlgebra
import CaveLidarRecon: unix_ns, scan_files

opt(args, name, default) = (v = [parse(Float64, split(a, "=")[2]) for a in args if startswith(a, "--$name=")]; isempty(v) ? default : v[1])

function write_ply(path, pts, t)
    open(path, "w") do io
        write(io, "ply\nformat binary_little_endian 1.0\nelement vertex $(length(pts))\n",
            "property float x\nproperty float y\nproperty float z\nproperty double t\nend_header\n")
        buf = IOBuffer()
        for (p, ti) in zip(pts, t)
            write(buf, Float32(p[1]), Float32(p[2]), Float32(p[3]), ti)
            buf.size > 1 << 22 && write(io, take!(buf))
        end
        write(io, take!(buf))
    end
end

# How sharp the moving points (1.5 m and farther) are against the standstill map: distances to its planes. The reference map
# only holds voxels hit in at least 3 different 0.1 s segments, and is the same for every run, so runs are comparable.
function score(posefun, pts, times, seedmap, cfg, t0, t1)
    idx = findall(t -> t0 <= t < t1, times)[1:50:end]
    idx = filter(i -> norm(pts[i]) >= 1.5, idx)
    X = [place(posefun(times[i])..., pts[i]) for i in idx]
    r = abs.(plane_residuals(seedmap, X, cfg))
    return (n = length(r), median = median(r), q90 = quantile(r, 0.9))
end

function main(args)
    dirs = filter(a -> !startswith(a, "--"), args)
    length(dirs) == 1 || (println(stderr, "usage: spline_odometry.jl <scan_folder> [options]"); exit(2))
    dir = dirs[1]
    tag = something(([split(a, "=")[2] for a in args if startswith(a, "--tag=")] |> x -> isempty(x) ? nothing : x[1]), "spline")
    cfg = OdometryConfig(; seed_s = opt(args, "seed", 31.5), min_range = opt(args, "min-range", 0.0), knot_s = opt(args, "knot", 0.2),
        step_s = opt(args, "step", 0.4), refine_knots = round(Int, opt(args, "refine", 4)), sigma_acc = opt(args, "sigma-acc", 1.0),
        sigma_yaw_acc = opt(args, "sigma-yaw", 0.3), fit_min_range = opt(args, "fit-min-range", 1.5),
        min_segments = round(Int, opt(args, "min-segments", 3)), segment_s = opt(args, "segment", 0.1))
    step = round(Int, opt(args, "ply-step", 1))

    tab = Arrow.Table(joinpath(dir, "untangled.arrow"); convert = false)
    ns = unix_ns(tab.time)
    times = (ns .- ns[1]) .* 1e-9
    pts = SVector{3,Float32}.(tab.x, tab.y, tab.z)
    imu = imu_track(scan_files(dir).rig, ns[1])
    ba, bg = standstill_bias(imu, 1.0, cfg.seed_s)
    @printf("%d points, %.1f s; knots every %.2f s, step %.2f s, %d older knots free, sigma_acc %.2f m/s2, sigma_yaw %.2f rad/s2\n",
        length(pts), times[end], cfg.knot_s, cfg.step_s, cfg.refine_knots, cfg.sigma_acc, cfg.sigma_yaw_acc)
    @printf("fit points from %.1f m, voxels need %d segments of %.2f s
", cfg.fit_min_range, cfg.min_segments, cfg.segment_s)

    t = @elapsed sp, steps = run_spline_odometry(pts, times, imu, cfg; bias_acc = ba, bias_gyro = bg, verbose = "--verbose" in args)
    t_s = times[1] + cfg.seed_s
    t_end = steps[end].t_front
    @printf("spline odometry: %.1f s, %d steps; rms point-to-plane median %.1f cm, max %.1f cm; matched points median %d\n", t, length(steps),
        100median(s.rms for s in steps), 100maximum(s.rms for s in steps), median(s.n_used for s in steps))

    ts = collect((t_s + 0.1):0.1:t_end)
    open(joinpath(dir, "$(tag)_path.csv"), "w") do io
        println(io, "t,x,y,z,yaw_deg,rms,n_used")
        for te in ts
            p, ψ = spline_pose(sp, te)
            s = steps[min(searchsortedfirst([q.t_front for q in steps], te - 1e-9), length(steps))]
            @printf(io, "%.2f,%.4f,%.4f,%.4f,%.3f,%.4f,%d\n", te, p..., rad2deg(ψ), s.rms, s.n_used)
        end
    end
    P = [spline_pose(sp, te)[1] for te in ts]
    @printf("path: x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f m; end point %s m, yaw %.1f deg\n", extrema(p[1] for p in P)..., extrema(p[2] for p in P)...,
        extrema(p[3] for p in P)..., round.(P[end]; digits = 3), rad2deg(spline_pose(sp, ts[end])[2]))
    acc = [norm(P[i+1] - 2P[i] + P[i-1]) / 0.01 for i in 2:length(P)-1]
    @printf("path acceleration (0.1 s differences): median %.2f m/s2, 95%% %.2f, max %.2f\n", median(acc), quantile(acc, 0.95), maximum(acc))

    seedmap = PointMap(cfg.map_voxel)
    iseed = 1:searchsortedfirst(times, t_s)-1
    add_points!(seedmap, (SVector{3,Float64}(pts[i]) for i in iseed), [floor(Int32, times[i] / 0.1) for i in iseed])
    rebuild!(seedmap, 3)
    sc = score(t -> spline_pose(sp, t), pts, times, seedmap, cfg, t_s, t_end)
    @printf("sharpness vs standstill map (%d points): spline path median %.1f cm, 90%% %.1f cm\n", sc.n, 100sc.median, 100sc.q90)

    if "--compare-old" in args
        old = run_odometry(pts, times, imu, cfg; bias_acc = ba, bias_gyro = bg)
        so = score(t -> pose_at(old, t), pts, times, seedmap, cfg, t_s, t_end)
        @printf("                                                       window path median %.1f cm, 90%% %.1f cm\n", 100so.median, 100so.q90)
        d = [norm(spline_pose(sp, te)[1] - pose_at(old, te - 1e-9)[1]) for te in (t_s + 1.0):1.0:t_end]
        @printf("difference spline vs window path at 1 s steps: median %.1f cm, max %.1f cm\n", 100median(d), 100maximum(d))
    end

    keep = findall(t -> t < t_end, times)[1:step:end]
    placed = [place(spline_pose(sp, times[i])..., pts[i]) for i in keep]
    write_ply(joinpath(dir, "$(tag)_cloud.ply"), placed, times[keep])
    println("wrote $(tag)_path.csv and $(tag)_cloud.ply in ", dir)
end

main(ARGS)
