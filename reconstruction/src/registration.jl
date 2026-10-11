"""
Settings of the window-by-window odometry (`run_odometry`).

The unknowns of a window are the displacement `dp` over the window (3) and the yaw offset `dpsi` at its end, both
measured against the IMU prediction and growing linearly over the window (a constant velocity and yaw rate offset).
`sigma_p` / `sigma_psi` are the widths of the Gaussian prior on them (zero = trust the IMU).
"""
Base.@kwdef struct OdometryConfig
    window_s::Float64 = 1.0
    seed_s::Float64 = 8.0               # standstill at the start: these points make the first map, the pose stays at the origin
    min_range::Float64 = 0.0            # [m] closer points (the person carrying the rig) neither enter the map nor the fit
    fit_voxel::Float64 = 0.2            # [m] points of a window are thinned to one per voxel (in the sensor frame) for the fit
    map_voxel::Float64 = 0.05           # [m] the map keeps the centroid of every voxel
    k_neighbors::Int = 8
    max_neighbor_dist::Float64 = 0.4    # [m] all neighbours of a plane must lie this close to the point
    max_residual::Float64 = 0.3         # [m] correspondences with a larger point-to-plane distance are dropped
    huber::Float64 = 0.1                # [m] residuals above this count only linearly
    planarity::Float64 = 0.2            # smallest / middle eigenvalue of the neighbourhood must be below this
    sigma_r::Float64 = 0.03             # [m] point-to-plane noise
    sigma_p::Float64 = 0.10             # [m] prior on the displacement correction of one window
    sigma_psi::Float64 = deg2rad(2.0)   # [rad] prior on the yaw correction of one window
    max_iter::Int = 15
    tol::Float64 = 1e-5
    bias_gain::Float64 = 0.0            # 0 = IMU biases stay as measured at standstill; else share of each correction fed back
    imu_dt::Float64 = 0.01

    # spline version (run_spline_odometry)
    knot_s::Float64 = 0.2               # [s] spacing of the B-spline control points
    step_s::Float64 = 0.4               # [s] new data (and new control points) per fit step
    refine_knots::Int = 4               # older control points that stay free in a step: how far a correction reaches back
    sigma_acc::Float64 = 1.0            # [m/s2] prior on the acceleration of the position correction (smoothness)
    sigma_yaw_acc::Float64 = 0.3        # [rad/s2] the same for the yaw correction
    slice_s::Float64 = 0.1              # [s] points are thinned per voxel and per slice of this length (one lidar sweep)

    # separating what stands still from the person who moves with the rig (run_spline_odometry)
    fit_min_range::Float64 = 0.0        # [m] fit points closer than this to the lidar are not used (they may still enter the map)
    segment_s::Float64 = 0.1            # [s] length of the time segments in which map voxels are counted
    min_segments::Int = 1               # a voxel is matched against only when points of this many different segments hit it
end

"""
Voxel map of the points placed so far: one centroid per voxel, in the start frame. `tree` is rebuilt by `rebuild!`
after points were added.
"""
mutable struct PointMap
    voxel::Float64
    index::Dict{NTuple{3,Int32},Int}
    sums::Vector{SVector{3,Float64}}
    counts::Vector{Int32}
    seg_last::Vector{Int32}       # time segment of the latest point in the voxel
    seg_count::Vector{Int32}      # number of different time segments that put points into the voxel (points arrive in time order)
    centroids::Vector{SVector{3,Float64}}   # of the voxels in the tree
    tree::Union{Nothing,KDTree}
end
PointMap(voxel) = PointMap(voxel, Dict{NTuple{3,Int32},Int}(), SVector{3,Float64}[], Int32[], Int32[], Int32[], SVector{3,Float64}[], nothing)

@inline voxel_key(p, inv) = (floor(Int32, p[1] * inv), floor(Int32, p[2] * inv), floor(Int32, p[3] * inv))

"Adds points (in time order) to the map. `segs` gives each point's time segment (an integer); without it all points count as one segment."
function add_points!(m::PointMap, pts, segs = nothing)
    inv = 1 / m.voxel
    for (k, p) in enumerate(pts)
        i = get!(m.index, voxel_key(p, inv)) do
            push!(m.sums, zero(SVector{3,Float64})); push!(m.counts, 0); push!(m.seg_last, -1); push!(m.seg_count, 0)
            length(m.sums)
        end
        m.sums[i] += p
        m.counts[i] += 1
        s = segs === nothing ? Int32(0) : Int32(segs[k])
        if m.seg_last[i] != s
            m.seg_last[i] = s
            m.seg_count[i] += 1
        end
    end
    return m
end

"Builds the search tree from the voxels hit by at least `min_segments` different time segments."
function rebuild!(m::PointMap, min_segments::Int = 1)
    ok = findall(>=(min_segments), m.seg_count)
    m.centroids = m.sums[ok] ./ Float64.(m.counts[ok])
    m.tree = KDTree(m.centroids)
    return m
end

"One point per voxel (the first one met), as indices into `pts`."
function voxel_thin(pts::AbstractVector, voxel::Float64)
    seen = Set{NTuple{3,Int32}}()
    inv = 1 / voxel
    keep = Int[]
    for (i, p) in enumerate(pts)
        k = voxel_key(p, inv)
        k in seen && continue
        push!(seen, k)
        push!(keep, i)
    end
    return keep
end

"What `fit_window` found: the correction and how well it is determined."
struct FitResult
    dp::SVector{3,Float64}
    dpsi::Float64
    n_used::Int
    rms::Float64
    info_min::Float64
    iterations::Int
end

"""
    fit_window(pts, tau, pred, map, cfg)

Gauss-Newton fit of the window correction `u = (dp, dpsi)` so that the sensor-frame points `pts` (taken `tau` seconds
after the window start) land on the surfaces of `map`. A point at time tau is placed at
`p_pred(tau) + tau/T dp + Rz(psi_pred(tau) + tau/T dpsi) * s`. The correspondences are point-to-plane: the plane through
the `k` nearest map points. Huber weights, plus the Gaussian prior on `u` around the IMU prediction (u = 0).
"""
function fit_window(pts::Vector{SVector{3,Float64}}, tau::Vector{Float64}, pred::WindowPrediction, map::PointMap,
                    cfg::OdometryConfig)
    n = length(pts)
    T = duration(pred)
    f = tau ./ T
    p0 = Vector{SVector{3,Float64}}(undef, n); psi0 = Vector{Float64}(undef, n)
    for i in 1:n
        p0[i], psi0[i] = predicted_pose(pred, tau[i])
    end
    prior = SVector(1 / cfg.sigma_p^2, 1 / cfg.sigma_p^2, 1 / cfg.sigma_p^2, 1 / cfg.sigma_psi^2)
    u = zero(SVector{4,Float64})
    X = Vector{SVector{3,Float64}}(undef, n); Wv = similar(X)
    inv_s2 = 1 / cfg.sigma_r^2
    n_used = 0; rms = NaN; info_min = 0.0; iters = 0
    for it in 1:cfg.max_iter
        iters = it
        dp = SVector(u[1], u[2], u[3])
        for i in 1:n
            w = rotz(psi0[i] + f[i] * u[4]) * pts[i]
            Wv[i] = w
            X[i] = p0[i] + f[i] * dp + w
        end
        idxs, dists = knn(map.tree, X, cfg.k_neighbors, false)
        H = zero(SMatrix{4,4,Float64,16}); g = zero(SVector{4,Float64})
        n_used = 0; ssq = 0.0
        for i in 1:n
            maximum(dists[i]) > cfg.max_neighbor_dist && continue
            q = map.centroids[idxs[i]]
            c = sum(q) / length(q)
            C = sum((qi - c) * (qi - c)' for qi in q) / length(q)
            E = eigen(Symmetric(C))
            E.values[1] > cfg.planarity * E.values[2] && continue
            nv = SVector{3,Float64}(E.vectors[1, 1], E.vectors[2, 1], E.vectors[3, 1])
            r = dot(nv, X[i] - c)
            abs(r) > cfg.max_residual && continue
            w = Wv[i]
            j = f[i] * SVector(nv[1], nv[2], nv[3], -nv[1] * w[2] + nv[2] * w[1])
            wt = abs(r) <= cfg.huber ? inv_s2 : inv_s2 * cfg.huber / abs(r)
            H += wt * (j * j')
            g += wt * r * j
            n_used += 1; ssq += r^2
        end
        n_used < 20 && error("window at t0 = $(pred.t0): only $n_used usable correspondences")
        rms = sqrt(ssq / n_used)
        info_min = eigmin(Symmetric(Matrix(H)))
        Hp = H + SMatrix{4,4,Float64}(Diagonal(prior))
        step = -(Hp \ (g + prior .* u))
        u += step
        maximum(abs, step) < cfg.tol && break
    end
    return FitResult(SVector(u[1], u[2], u[3]), u[4], n_used, rms, info_min, iters)
end

"""
    run_odometry(pts, times, imu, cfg; bias_acc, bias_gyro)

Moves through the recording one window at a time. The sensor-frame points `pts` (time-sorted, `times` in seconds, the
clock of `imu`) of the first `cfg.seed_s` seconds, taken at standstill, make the first map. Every following window is
predicted by the IMU from the end state of the previous one, fitted to the map (`fit_window`), and then its points,
placed with the fitted path, are added to the map. Returns the `PathEstimate`.
"""
function run_odometry(pts::AbstractVector{<:SVector{3,<:Real}}, times::Vector{Float64}, imu::ImuTrack, cfg::OdometryConfig;
                      bias_acc::SVector{3,Float64} = zero(SVector{3,Float64}), bias_gyro::Float64 = 0.0, verbose::Bool = false)
    t_seed = times[1] + cfg.seed_s
    map = PointMap(cfg.map_voxel)
    i_seed = searchsortedfirst(times, t_seed)
    add_points!(map, (SVector{3,Float64}(p) for p in view(pts, 1:i_seed-1) if norm(p) >= cfg.min_range))
    rebuild!(map)
    state = PathState(zero(SVector{3,Float64}), zero(SVector{3,Float64}), 0.0)
    fits = WindowFit[]
    T = cfg.window_s
    t0 = t_seed
    while t0 + T <= times[end]
        i0 = searchsortedfirst(times, t0); i1 = searchsortedfirst(times, t0 + T) - 1
        win = SVector{3,Float64}.(view(pts, i0:i1))
        far = findall(p -> norm(p) >= cfg.min_range, win)
        pred = predict_window(imu, state, t0, T, bias_acc, bias_gyro; dt = cfg.imu_dt)
        keep = far[voxel_thin(win[far], cfg.fit_voxel)]
        res = fit_window(win[keep], times[i0:i1][keep] .- t0, pred, map, cfg)
        fit = WindowFit(pred, res.dp, res.dpsi, res.n_used, res.rms, res.info_min, res.iterations)
        push!(fits, fit)
        state = end_state(pred, res.dp, res.dpsi)
        if cfg.bias_gain > 0
            bias_acc -= cfg.bias_gain * (rotz(-(pred.Psi[end] + 0.5 * res.dpsi)) * (res.dp / T^2))
            bias_gyro -= cfg.bias_gain * res.dpsi / T
        end
        verbose && println("t0 = ", round(t0 - times[1]; digits = 1), "  dp = ", round.(res.dp; digits = 3), "  dpsi = ",
                           round(rad2deg(res.dpsi); digits = 2), " deg  used ", res.n_used, "/", length(keep),
                           "  rms ", round(res.rms; digits = 3), "  info_min ", round(res.info_min; sigdigits = 3))
        placed = [place(corrected_pose(fit, times[i] - t0)..., pts[i]) for i in i0:i1 if norm(pts[i]) >= cfg.min_range]
        add_points!(map, placed)
        rebuild!(map)
        t0 += T
    end
    return PathEstimate(fits)
end

"Dead reckoning with the IMU alone (no lidar): the same windows as `run_odometry`, all corrections zero."
function dead_reckon(imu::ImuTrack, t_start::Float64, t_end::Float64, cfg::OdometryConfig;
                     bias_acc::SVector{3,Float64} = zero(SVector{3,Float64}), bias_gyro::Float64 = 0.0)
    state = PathState(zero(SVector{3,Float64}), zero(SVector{3,Float64}), 0.0)
    fits = WindowFit[]
    t0 = t_start
    while t0 + cfg.window_s <= t_end
        pred = predict_window(imu, state, t0, cfg.window_s, bias_acc, bias_gyro; dt = cfg.imu_dt)
        push!(fits, WindowFit(pred, zero(SVector{3,Float64}), 0.0, 0, NaN, NaN, 0))
        state = end_state(pred, zero(SVector{3,Float64}), 0.0)
        t0 += cfg.window_s
    end
    return PathEstimate(fits)
end
