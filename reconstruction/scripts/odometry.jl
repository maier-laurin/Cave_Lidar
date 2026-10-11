# Window odometry on a real recording: the untangled points of a scan folder plus the BNO085 of its rig files.
# The rig has to stand still at the start (the first --seed seconds after the first lidar point, default 31.5).
#
#   julia --project=reconstruction -t auto reconstruction/scripts/odometry.jl <scan_folder> [--seed=S] [--window=W]
#         [--min-range=R] [--sigma-p=M] [--bias-gain=G] [--ply-step=N] [--verbose]
#
# Writes <scan>/odometry_path.csv (pose per window end and the IMU-only dead reckoning) and <scan>/odometry_cloud.ply
# (every N-th point placed with the estimated path; N = --ply-step, default 20).

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

function main(args)
    dirs = filter(a -> !startswith(a, "--"), args)
    length(dirs) == 1 || (println(stderr, "usage: odometry.jl <scan_folder> [options]"); exit(2))
    dir = dirs[1]
    cfg = OdometryConfig(; seed_s = opt(args, "seed", 31.5), window_s = opt(args, "window", 1.0),
        min_range = opt(args, "min-range", 0.0), sigma_p = opt(args, "sigma-p", 0.10), bias_gain = opt(args, "bias-gain", 0.0))
    step = round(Int, opt(args, "ply-step", 20))

    tab = Arrow.Table(joinpath(dir, "untangled.arrow"); convert = false)
    ns = unix_ns(tab.time)
    times = (ns .- ns[1]) .* 1e-9
    pts = SVector{3,Float32}.(tab.x, tab.y, tab.z)
    imu = imu_track(scan_files(dir).rig, ns[1])
    ba, bg = standstill_bias(imu, 1.0, cfg.seed_s)
    @printf("%d points, %.1f s; IMU standstill bias: acc %s m/s2, gyro %.4f deg/s\n", length(pts), times[end], round.(ba; digits = 4), rad2deg(bg))

    t = @elapsed est = run_odometry(pts, times, imu, cfg; bias_acc = ba, bias_gyro = bg, verbose = "--verbose" in args)
    @printf("odometry: %.1f s, %d windows\n", t, length(est.fits))
    dr = dead_reckon(imu, cfg.seed_s + times[1], times[end], cfg; bias_acc = ba, bias_gyro = bg)

    open(joinpath(dir, "odometry_path.csv"), "w") do io
        println(io, "t,x,y,z,yaw_deg,imu_x,imu_y,imu_z,imu_yaw_deg,dp_norm,dpsi_deg,n_used,rms,info_min")
        for (k, f) in enumerate(est.fits)
            te = est.starts[k] + cfg.window_s
            pe, ψe = pose_at(est, te - 1e-9); pd, ψd = pose_at(dr, te - 1e-9)
            @printf(io, "%.2f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f,%d,%.4f,%.1f\n", te, pe..., rad2deg(ψe), pd..., rad2deg(ψd),
                norm(f.dp), rad2deg(f.dpsi), f.n_used, f.rms, f.info_min)
        end
    end
    println("\n  t[s]    x      y      z    yaw[deg] | IMU only x      y      z   yaw | |dp|  dpsi  used   rms[cm]")
    for (k, f) in enumerate(est.fits)
        te = est.starts[k] + cfg.window_s
        (k % 4 == 0 || k == length(est.fits)) || continue
        pe, ψe = pose_at(est, te - 1e-9); pd, ψd = pose_at(dr, te - 1e-9)
        @printf("%6.1f %6.2f %6.2f %6.2f %7.1f | %6.2f %6.2f %6.2f %7.1f | %.2f %5.1f %5d %6.1f\n", te, pe..., rad2deg(ψe), pd..., rad2deg(ψd),
            norm(f.dp), rad2deg(f.dpsi), f.n_used, 100f.rms)
    end
    @printf("\nrms point-to-plane over the windows: median %.1f cm, max %.1f cm; correspondences median %d; info_min median %.3g (min %.3g)\n",
        100median(f.rms for f in est.fits), 100maximum(f.rms for f in est.fits), median(f.n_used for f in est.fits),
        median(f.info_min for f in est.fits), minimum(f.info_min for f in est.fits))

    keep = findall(t -> t < est.starts[end] + cfg.window_s, times)[1:step:end]
    placed = [place(pose_at(est, times[i])..., pts[i]) for i in keep]
    write_ply(joinpath(dir, "odometry_cloud.ply"), placed, times[keep])
    println("wrote odometry_path.csv and odometry_cloud.ply in ", dir)
end

main(ARGS)
