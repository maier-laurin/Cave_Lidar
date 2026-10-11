# Test of the window odometry on a recording made at standstill: its points are moved along a known path, an IMU is
# simulated for that path, and the odometry has to find the path again from the moved points and the IMU alone.
#
#   julia --project=reconstruction -t auto reconstruction/scripts/simulate_path.jl <untangled.arrow> [--bias-gain=0.3] [--seed=8]

using CaveLidarRecon, Arrow, StaticArrays, Random, Printf, Statistics, LinearAlgebra
import CaveLidarRecon: unix_ns

function main(args)
    files = filter(a -> !startswith(a, "--"), args)
    length(files) == 1 || (println(stderr, "usage: simulate_path.jl <untangled.arrow> [--bias-gain=G] [--seed=S]"); exit(2))
    opt(name, default) = something(
        (parse(Float64, split(a, "=")[2]) for a in args if startswith(a, "--$name=")) |> collect |> x -> isempty(x) ? nothing : x[1], default)
    cfg = OdometryConfig(; bias_gain = opt("bias-gain", 0.0), seed_s = opt("seed", 8.0))

    tab = Arrow.Table(files[1]; convert = false)
    ns = unix_ns(tab.time)
    times = (ns .- ns[1]) .* 1e-9
    static = SVector{3,Float32}.(tab.x, tab.y, tab.z)
    @printf("%d points, %.1f s\n", length(static), times[end])

    path = SimPath(t_start = cfg.seed_s)
    moved = simulate_scan(static, times, path)
    imu = simulate_imu(path, 0.0, times[end], MersenneTwister(1))
    ba, bg = standstill_bias(imu, cfg.seed_s)
    @printf("standstill bias estimate: acc %s m/s2 (true 0.04 -0.03 0.02), gyro %.3f deg/s (true 0.3)\n",
        round.(ba; digits = 4), rad2deg(bg))

    t = @elapsed est = run_odometry(moved, times, imu, cfg; bias_acc = ba, bias_gyro = bg, verbose = "--verbose" in args)
    @printf("odometry: %.1f s\n", t)
    dr = dead_reckon(imu, cfg.seed_s, times[end], cfg; bias_acc = ba, bias_gyro = bg)

    println("\n  t[s]   |dp| est   |dp| IMU only   yaw err est   IMU only   (position error vs truth in cm, yaw in deg)")
    for k in 1:length(est.fits)
        te = est.starts[k] + cfg.window_s
        pt, ψt = sim_pose(path, te)
        pe, ψe = pose_at(est, te - 1e-9); pd, ψd = pose_at(dr, te - 1e-9)
        k % 3 == 0 || k == length(est.fits) || continue
        @printf("%6.1f  %8.2f  %12.2f  %12.3f  %9.3f\n", te, 100norm(pe - pt), 100norm(pd - pt),
            rad2deg(ψe - ψt), rad2deg(ψd - ψt))
    end

    # point error over the moving part: where the estimated path puts a point vs. where it really is
    idx = findall(t -> t >= cfg.seed_s + 0.0 && t < est.starts[end] + cfg.window_s, times)
    idx = idx[1:97:end]
    err = Float64[]; err_dr = Float64[]
    for i in idx
        pe, ψe = pose_at(est, times[i]); pd, ψd = pose_at(dr, times[i])
        push!(err, norm(place(pe, ψe, moved[i]) - SVector{3,Float64}(static[i])))
        push!(err_dr, norm(place(pd, ψd, moved[i]) - SVector{3,Float64}(static[i])))
    end
    @printf("\npoint error in the start frame: estimated path median %.1f cm, 95%% %.1f cm; IMU only median %.1f cm, 95%% %.1f cm\n",
        100median(err), 100quantile(err, 0.95), 100median(err_dr), 100quantile(err_dr, 0.95))
    @printf("info_min of the windows: min %.3g, median %.3g\n", minimum(f.info_min for f in est.fits), median(f.info_min for f in est.fits))
end

main(ARGS)
