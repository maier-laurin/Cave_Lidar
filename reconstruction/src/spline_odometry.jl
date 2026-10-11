# Odometry with a cubic B-spline correction on top of the IMU path.
#
# The path is `p(t) = p_imu(t) + c(t)`: the IMU integrated once over the whole moving part (fast detail: strides, turns),
# plus a smooth correction c = (x, y, z, yaw) that the lidar fits. c is a uniform cubic B-spline with control points
# every `knot_s` seconds, so a control point only moves the path over four knot intervals. The correction is found
# step by step: each step adds `step_s` seconds of new points and new control points, and re-fits only those and the
# last `refine_knots` older ones. Everything older is frozen, and only then do its points enter the map.

"Uniform cubic B-spline weights of the four control points that act in an interval, `u` in [0, 1] across it."
@inline function bspline_weights(u::Float64)
    u2 = u * u; u3 = u2 * u
    return SVector((1 - u)^3 / 6, (3u3 - 6u2 + 4) / 6, (-3u3 + 3u2 + 3u + 1) / 6, u3 / 6)
end

"""
The estimated path of a spline run: IMU baseline `base` integrated from `t0`, plus the spline with control values
`C[j+1]` = (x, y, z, yaw) of control point j; interval i (starting at `t0 + i h`) uses control points i to i + 3.
Control points 0 to 2 are zero, so the correction starts at zero with zero slope.
"""
struct SplinePath
    t0::Float64
    h::Float64
    base::WindowPrediction
    C::Vector{SVector{4,Float64}}
end

"Position and yaw of the path at time `t`; before `t0` the rig is at the origin."
function spline_pose(sp::SplinePath, t::Float64)
    tau = t - sp.t0
    tau < 0 && return zero(SVector{3,Float64}), 0.0
    p, psi = predicted_pose(sp.base, tau)
    i = clamp(floor(Int, tau / sp.h), 0, length(sp.C) - 4)
    b = bspline_weights(clamp(tau / sp.h - i, 0.0, 1.0))
    c = b[1] * sp.C[i+1] + b[2] * sp.C[i+2] + b[3] * sp.C[i+3] + b[4] * sp.C[i+4]
    return p + SVector(c[1], c[2], c[3]), psi + c[4]
end

"One point per voxel and per time slice (`slice_s` seconds, about one lidar sweep), as indices into `pts`. Slicing keeps every
sweep of a fit window represented, a plain voxel filter would keep only the earliest."
function voxel_thin_sliced(pts::AbstractVector, times::AbstractVector{Float64}, voxel::Float64, slice_s::Float64)
    seen = Set{NTuple{4,Int32}}()
    inv = 1 / voxel
    keep = Int[]
    for i in eachindex(pts)
        k = (floor(Int32, pts[i][1] * inv), floor(Int32, pts[i][2] * inv), floor(Int32, pts[i][3] * inv), floor(Int32, times[i] / slice_s))
        k in seen && continue
        push!(seen, k)
        push!(keep, i)
    end
    return keep
end

"Plane through the map points `idx` (neighbours of one query point): `(true, centroid, normal)` when they are close enough and flat enough."
function local_plane(m::PointMap, idx::AbstractVector{Int}, dist::AbstractVector{Float64}, cfg::OdometryConfig)
    maximum(dist) > cfg.max_neighbor_dist && return false, zero(SVector{3,Float64}), zero(SVector{3,Float64})
    q = m.centroids[idx]
    c = sum(q) / length(q)
    C = sum((qi - c) * (qi - c)' for qi in q) / length(q)
    E = eigen(Symmetric(C))
    E.values[1] > cfg.planarity * E.values[2] && return false, c, zero(SVector{3,Float64})
    return true, c, SVector{3,Float64}(E.vectors[1, 1], E.vectors[2, 1], E.vectors[3, 1])
end

"Point-to-plane distances of the world points `X` to the planes of `m` (only those that found a plane)."
function plane_residuals(m::PointMap, X::Vector{SVector{3,Float64}}, cfg::OdometryConfig)
    idxs, dists = knn(m.tree, X, cfg.k_neighbors, false)
    r = Float64[]
    for i in eachindex(X)
        ok, c, nv = local_plane(m, idxs[i], dists[i], cfg)
        ok && push!(r, dot(nv, X[i] - c))
    end
    return r
end

struct SplineStep
    t_front::Float64     # time up to which the correction was fitted
    n_used::Int
    rms::Float64
    iterations::Int
end

# One Gauss-Newton fit of the free control points `first_free:last` (0-based) to the points `pts` (sensor frame) at
# `tau` seconds after t0. C is updated in place. Returns (n_used, rms, iterations).
function fit_spline_step!(C::Vector{SVector{4,Float64}}, first_free::Int, last::Int, pts::Vector{SVector{3,Float64}},
                          tau::Vector{Float64}, base::WindowPrediction, h::Float64, map::PointMap, cfg::OdometryConfig)
    n = length(pts)
    iv = Vector{Int}(undef, n); bw = Vector{SVector{4,Float64}}(undef, n)
    pb = Vector{SVector{3,Float64}}(undef, n); psib = Vector{Float64}(undef, n)
    for k in 1:n
        iv[k] = floor(Int, tau[k] / h)
        bw[k] = bspline_weights(tau[k] / h - iv[k])
        pb[k], psib[k] = predicted_pose(base, tau[k])
    end
    nf = last - first_free + 1
    dim = 4nf
    inv_s2 = 1 / cfg.sigma_r^2
    sc = SVector(1 / (cfg.sigma_acc * h^2), 1 / (cfg.sigma_acc * h^2), 1 / (cfg.sigma_acc * h^2), 1 / (cfg.sigma_yaw_acc * h^2))
    X = Vector{SVector{3,Float64}}(undef, n); Wv = similar(X)
    idx = zeros(Int, 16); val = zeros(16)
    n_used = 0; rms = NaN; iters = 0
    for it in 1:cfg.max_iter
        iters = it
        for k in 1:n
            i = iv[k]; b = bw[k]
            c = b[1] * C[i+1] + b[2] * C[i+2] + b[3] * C[i+3] + b[4] * C[i+4]
            w = rotz(psib[k] + c[4]) * pts[k]
            Wv[k] = w
            X[k] = pb[k] + SVector(c[1], c[2], c[3]) + w
        end
        idxs, dists = knn(map.tree, X, cfg.k_neighbors, false)
        H = zeros(dim, dim); g = zeros(dim)
        n_used = 0; ssq = 0.0
        for k in 1:n
            ok, c, nv = local_plane(map, idxs[k], dists[k], cfg)
            ok || continue
            r = dot(nv, X[k] - c)
            abs(r) > cfg.max_residual && continue
            w = Wv[k]
            nyaw = -nv[1] * w[2] + nv[2] * w[1]
            wt = abs(r) <= cfg.huber ? inv_s2 : inv_s2 * cfg.huber / abs(r)
            m = 0
            for j in 0:3
                gi = iv[k] + j
                gi < first_free && continue
                o = 4 * (gi - first_free); bj = bw[k][j+1]
                idx[m+1] = o + 1; val[m+1] = bj * nv[1]
                idx[m+2] = o + 2; val[m+2] = bj * nv[2]
                idx[m+3] = o + 3; val[m+3] = bj * nv[3]
                idx[m+4] = o + 4; val[m+4] = bj * nyaw
                m += 4
            end
            for a in 1:m
                g[idx[a]] += wt * r * val[a]
                for bb in 1:m
                    H[idx[a], idx[bb]] += wt * val[a] * val[bb]
                end
            end
            n_used += 1; ssq += r^2
        end
        n_used < 30 && return n_used, NaN, iters
        rms = sqrt(ssq / n_used)
        # smoothness: the second difference of the control values (the acceleration of the correction) is small
        for j in max(first_free - 2, 0):(last - 2)
            d = C[j+1] - 2C[j+2] + C[j+3]
            for comp in 1:4
                wgt = sc[comp]^2
                for (off, coef) in ((0, 1.0), (1, -2.0), (2, 1.0))
                    gi = j + off
                    gi < first_free && continue
                    ia = 4 * (gi - first_free) + comp
                    g[ia] += wgt * d[comp] * coef
                    for (off2, coef2) in ((0, 1.0), (1, -2.0), (2, 1.0))
                        gj = j + off2
                        gj < first_free && continue
                        H[ia, 4 * (gj - first_free) + comp] += wgt * coef * coef2
                    end
                end
            end
        end
        for a in 1:dim
            H[a, a] += 1e-9
        end
        step = -(Symmetric(H) \ g)
        for f in 0:nf-1
            C[first_free+f+1] += SVector(step[4*f+1], step[4*f+2], step[4*f+3], step[4*f+4])
        end
        maximum(abs, step) < cfg.tol && break
    end
    return n_used, rms, iters
end

"""
    run_spline_odometry(pts, times, imu, cfg; bias_acc, bias_gyro)

Spline version of `run_odometry` (same inputs). `cfg.knot_s` is the knot spacing, `cfg.step_s` how much new data each
step adds, `cfg.refine_knots` how many older control points stay free. Returns `(path::SplinePath, steps)`.
"""
function run_spline_odometry(pts::AbstractVector{<:SVector{3,<:Real}}, times::Vector{Float64}, imu::ImuTrack, cfg::OdometryConfig;
                             bias_acc::SVector{3,Float64} = zero(SVector{3,Float64}), bias_gyro::Float64 = 0.0, verbose::Bool = false)
    t_s = times[1] + cfg.seed_s
    h = cfg.knot_s
    nnew = max(1, round(Int, cfg.step_s / h))
    R = cfg.refine_knots
    map = PointMap(cfg.map_voxel)
    i_seed = searchsortedfirst(times, t_s)
    seg(i) = floor(Int32, (times[i] - times[1]) / cfg.segment_s)
    seed_i = [i for i in 1:i_seed-1 if norm(pts[i]) >= cfg.min_range]
    add_points!(map, (SVector{3,Float64}(pts[i]) for i in seed_i), [seg(i) for i in seed_i])
    rebuild!(map, cfg.min_segments)
    base = predict_window(imu, PathState(zero(SVector{3,Float64}), zero(SVector{3,Float64}), 0.0), t_s, times[end] - t_s,
                          bias_acc, bias_gyro; dt = cfg.imu_dt)
    C = [zero(SVector{4,Float64}) for _ in 1:3]
    sp = SplinePath(t_s, h, base, C)
    steps = SplineStep[]
    N = 0
    t_ins = t_s                                     # points before this time are in the map
    while t_s + (N + nnew) * h <= times[end]
        N2 = N + nnew
        for _ in 1:nnew
            push!(C, 2C[end] - C[end-1])            # new control points continue the correction linearly
        end
        first_free = max(N + 3 - R, 3)
        t_fit0 = t_s + max(N - R, 0) * h
        # points whose control points are all frozen now go into the map
        if t_fit0 > t_ins
            a = searchsortedfirst(times, t_ins); b = searchsortedfirst(times, t_fit0) - 1
            ins = [i for i in a:b if norm(pts[i]) >= cfg.min_range]
            add_points!(map, (place(spline_pose(sp, times[i])..., pts[i]) for i in ins), [seg(i) for i in ins])
            rebuild!(map, cfg.min_segments)
            t_ins = t_fit0
        end
        a = searchsortedfirst(times, t_fit0); b = searchsortedfirst(times, t_s + N2 * h) - 1
        win = SVector{3,Float64}.(view(pts, a:b))
        tw = times[a:b]
        far = findall(p -> norm(p) >= max(cfg.min_range, cfg.fit_min_range), win)
        keep = far[voxel_thin_sliced(win[far], tw[far], cfg.fit_voxel, cfg.slice_s)]
        n_used, rms, iters = fit_spline_step!(C, first_free, N2 + 2, win[keep], tw[keep] .- t_s, base, h, map, cfg)
        push!(steps, SplineStep(t_s + N2 * h, n_used, rms, iters))
        verbose && println("t = ", round(t_s + N2 * h - times[1]; digits = 1), "  used ", n_used, "/", length(keep), "  rms ", round(rms; digits = 3),
                           "  iters ", iters, "  c = ", round.(C[end]; digits = 3))
        N = N2
    end
    return sp, steps
end
