"Where the lidar points of a run went: read, and why the others were dropped."
Base.@kwdef mutable struct UntangleCounts
    read::Int = 0
    too_close::Int = 0       # closer than min_range_m (the rig itself)
    no_rig::Int = 0          # outside the rig recording, or inside a rig dropout longer than max_rig_gap_ns
    no_attitude::Int = 0     # no usable BNO quaternion around the firing time
    written::Int = 0
end

# Index i with ts[i] <= t < ts[i+1]; `i` is the previous result, so a sorted stream costs O(1) per point.
@inline function locate(ts::Vector{Int64}, i::Int, t::Int64)
    t < ts[i] && (i = max(1, searchsortedlast(ts, t)))
    while i < length(ts) - 1 && ts[i+1] <= t
        i += 1
    end
    return i
end

# Inner loop as its own function, so that all argument types are concrete (function barrier).
# attitude_mode: 0 = none (rig frame), 1 = gravity (the rig's up direction is rotated onto +Z, heading left as it is),
# 2 = world (full BNO attitude, East-North-Up). `att_t` empty means no attitude.
function untangle_points(lt, az, el, dist, refl, rig_t, rig_theta, att_t, att_q, R0, shift, R_bno_frame,
                         sigma, min_range_m, max_gap_ns, latency_ns, attitude_mode::Int, counts::UntangleCounts)
    n = length(lt)
    use_att = attitude_mode != 0 && !isempty(att_t)
    x = Vector{Float32}(undef, n); y = similar(x); z = similar(x)
    out_refl = Vector{UInt8}(undef, n)
    out_t = Vector{Int64}(undef, n)
    m = 0
    i = 1
    ia = 1
    for k in 1:n
        t = lt[k]
        r = dist[k] * 0.002
        if r < min_range_m
            counts.too_close += 1
            continue
        end
        if t < rig_t[1] || t >= rig_t[end]
            counts.no_rig += 1
            continue
        end
        i = locate(rig_t, i, t)
        gap = rig_t[i+1] - rig_t[i]
        if gap > max_gap_ns
            counts.no_rig += 1
            continue
        end

        # shaft angle at the firing time, interpolated on the circle
        th = angle_lerp(rig_theta[i], rig_theta[i+1], (t - rig_t[i]) / gap)

        # spherical to cartesian in the lidar frame
        sa, ca = sincos(deg2rad(az[k] * 0.01))
        sw, cw = sincos(deg2rad(Float64(el[k])))
        pL = SVector{3,Float64}(r * cw * sa, r * cw * ca, r * sw)

        # mounting rotation and offset of the optical centre, then rotation about the shaft axis X
        q = R0 * pL + shift
        s, c = sincos(sigma * th)
        p = SVector{3,Float64}(q[1], c * q[2] - s * q[3], s * q[2] + c * q[3])

        if use_att
            # frame attitude at the firing time, interpolated between the two BNO samples around it
            ta = t + latency_ns
            if ta < att_t[1] || ta >= att_t[end]
                counts.no_attitude += 1
                continue
            end
            ia = locate(att_t, ia, ta)
            agap = att_t[ia+1] - att_t[ia]
            if agap > max_gap_ns
                counts.no_attitude += 1
                continue
            end
            qa = slerp(att_q[ia], att_q[ia+1], (ta - att_t[ia]) / agap)
            Rq = rotation_matrix(qa)
            if attitude_mode == 2
                p = Rq * (R_bno_frame * p)
            else
                # up in the rig frame: world up in sensor coordinates (third row of Rq), then sensor -> rig
                up = R_bno_frame' * SVector{3,Float64}(Rq[3, 1], Rq[3, 2], Rq[3, 3])
                p = rotation_between(up, SVector(0.0, 0.0, 1.0)) * p
            end
        end

        m += 1
        x[m] = p[1]; y[m] = p[2]; z[m] = p[3]
        out_refl[m] = refl[k]
        out_t[m] = t
    end
    counts.read += n
    counts.written += m
    resize!(x, m); resize!(y, m); resize!(z, m); resize!(out_refl, m); resize!(out_t, m)
    return (time = reinterpret(TimestampNs, out_t), x = x, y = y, z = z, reflectivity = out_refl)
end

"Untangles one lidar chunk file; returns the output columns `(time, x, y, z, reflectivity)`."
function untangle_chunk(lidar_file, track::AngleTrack, att::Union{AttitudeTrack,Nothing}, P::RigParams,
                        attitude_mode::Int, counts::UntangleCounts)
    lidar = Arrow.Table(lidar_file; convert = false)
    R0 = SMatrix{3,3,Float64}(RotY(deg2rad(P.tau_deg)) * RotZ(deg2rad(P.psi_deg)))
    shift = SVector{3,Float64}(P.origin_x, P.origin_y, P.origin_z)
    att_t = att === nothing ? Int64[] : att.t
    att_q = att === nothing ? SVector{4,Float64}[] : att.q
    return untangle_points(unix_ns(lidar.time), collect(lidar.azimuth), collect(lidar.elevation),
        collect(lidar.distance), collect(lidar.reflectivity), track.t, track.theta, att_t, att_q,
        R0, shift, P.bno_from_frame, P.sigma, P.min_range_m, P.max_rig_gap_ns, P.bno_latency_ns, attitude_mode, counts)
end

function git_commit()
    try
        return readchomp(pipeline(`git -C $(@__DIR__) rev-parse --short HEAD`; stderr = devnull))
    catch
        return "unknown"
    end
end

"""
    untangle_scan(scan_dir; out = joinpath(scan_dir, "untangled.arrow"), params = RigParams(), attitude = :gravity)

Reads all `lidar_*`, `rig_*` and `edges_*` Arrow files of `scan_dir` and writes one combined Arrow file with the columns
`time` (ns since 1970, UTC), `x`, `y`, `z` [m, Float32] and `reflectivity` (raw, UInt8). The lidar chunks are processed
one after the other, so memory use does not grow with the scan length.

The shaft rotation is undone with the angle track; then, depending on `attitude`:

- `:gravity` (default): the BNO085 attitude interpolated at each point's firing time gives the direction of up in the
  rig frame, and the point is rotated by the smallest rotation that takes it onto +Z. Only the tilt is corrected; the
  heading about the vertical is left as it is (the rig's own, no magnetometer), so turning of the rig stays in the cloud.
- `:world`: the full BNO attitude rotates the point into the East-North-Up world orientation (heading from the
  magnetometer, which is unreliable on this rig).
- `:none`: the points stay in the rig frame (X along the shaft, Z vertical at shaft angle 0).

Rotation only, the origin stays at the rig. Returns the `UntangleCounts`.
"""
function untangle_scan(scan_dir; out = joinpath(scan_dir, "untangled.arrow"), params::RigParams = RigParams(),
                       attitude::Symbol = :gravity)
    attitude in (:none, :gravity, :world) || error("attitude must be :none, :gravity or :world, not :$attitude")
    mode = attitude === :none ? 0 : attitude === :gravity ? 1 : 2
    use_attitude = mode != 0
    files = scan_files(scan_dir)
    params = resolve_params(params, files.rig)
    track = angle_track(files.rig, files.edges, params)
    att = use_attitude ? attitude_track(files.rig) : nothing
    use_attitude && isempty(att.t) && error("no usable BNO quaternion in $scan_dir")
    counts = UntangleCounts()

    meta = Dict(
        "software" => "CaveLidarRecon $(git_commit())",
        "frame" => attitude === :world ? "ENU world orientation (BNO rotation vector), rotation only, origin at the rig" :
                   attitude === :gravity ? "z = up from gravity (BNO tilt), heading as the rig's own, rotation only, origin at the rig" :
                   "rig frame: X along the shaft, Z vertical at shaft angle 0",
        "columns" => "time: unix ns UTC; x,y,z: m; reflectivity: raw",
        "params" => repr(params),
        "scan_dir" => abspath(scan_dir),
    )
    parts = Tables.partitioner(files.lidar) do f
        untangle_chunk(f, track, att, params, mode, counts)
    end
    Arrow.write(out, parts; ntasks = 1, metadata = meta)     # one task: the counts are shared
    return counts
end
