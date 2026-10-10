"""
Frame attitude from the BNO085 rotation vector. `q[i] = (w, x, y, z)` is a unit quaternion that rotates vectors from the
sensor frame into the world frame (East-North-Up), at unix time `t[i]` (ns).
"""
struct AttitudeTrack
    t::Vector{Int64}
    q::Vector{SVector{4,Float64}}
end

"""
    attitude_track(rig_files; norm_tol = 0.05)

Attitude track of all rig samples that carry a usable quaternion: finite and of norm within `norm_tol` of 1
(the sensor delivers zeros until its first report). The quaternions are normalised.
"""
function attitude_track(rig_files; norm_tol = 0.05)
    t = unix_ns(column(rig_files, :time))
    q = [SVector{4,Float64}(w, x, y, z) for (w, x, y, z) in
         zip(column(rig_files, :qw), column(rig_files, :qx), column(rig_files, :qy), column(rig_files, :qz))]
    issorted(t) || error("rig sample times are not sorted")
    keep = [all(isfinite, v) && abs(norm(v) - 1) <= norm_tol for v in q]
    return AttitudeTrack(t[keep], [v / norm(v) for v in q[keep]])
end

"Unit quaternion at fraction `f` on the shortest arc from `a` to `b` (both unit, (w, x, y, z))."
function slerp(a::SVector{4,Float64}, b::SVector{4,Float64}, f::Float64)
    d = dot(a, b)
    if d < 0                                     # q and -q are the same rotation: take the short way
        b = -b
        d = -d
    end
    q = if d > 0.9995                            # nearly equal: normalised linear interpolation
        a + f * (b - a)
    else
        w = acos(d)
        (sin((1 - f) * w) * a + sin(f * w) * b) / sin(w)
    end
    return q / norm(q)
end

"Rotation matrix of the unit quaternion `q = (w, x, y, z)`."
function rotation_matrix(q::SVector{4,Float64})
    w, x, y, z = q
    return @SMatrix [1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);
                     2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);
                     2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)]
end

"The smallest rotation (about the axis a x b, by the angle between them) that takes the vector `a` onto the vector `b`.
Nothing about `b` itself is added, so a heading about `b` is left as it is."
function rotation_between(a::AbstractVector, b::AbstractVector)
    a = normalize(SVector{3,Float64}(a)); b = normalize(SVector{3,Float64}(b))
    ax = cross(a, b); s = norm(ax); c = dot(a, b)
    if s < 1e-12
        c > 0 && return one(SMatrix{3,3,Float64,9})
        p = abs(a[1]) < 0.9 ? SVector(1.0, 0.0, 0.0) : SVector(0.0, 1.0, 0.0)      # opposite vectors: half turn about any axis normal to a
        k = normalize(cross(a, p))
        return SMatrix{3,3,Float64,9}(2 * k * k' - I)
    end
    k = ax / s
    K = @SMatrix [0 -k[3] k[2]; k[3] 0 -k[1]; -k[2] k[1] 0]
    return SMatrix{3,3,Float64,9}(I + s * K + (1 - c) * K * K)                    # Rodrigues
end
