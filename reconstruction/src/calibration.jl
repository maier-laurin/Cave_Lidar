# Mounting rotation of the BNO085 relative to the rig frame, from one pose with a known orientation:
# the rig stands level and one horizontal rig axis points to magnetic north (read off a compass).

"Path of the stored BNO mounting calibration (written by scripts/calibrate_bno_mount.jl)."
const BNO_MOUNT_FILE = joinpath(@__DIR__, "..", "calibration", "bno_mount.toml")

"The BNO mapping from the drawing: BNO z = rig Z, BNO y = rig X (so BNO x = rig -Y). Rig-frame coordinates to BNO coordinates."
const BNO_MOUNT_NOMINAL = SMatrix{3,3,Float64,9}(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

"The calibrated mapping from `BNO_MOUNT_FILE` when it exists, else the nominal one."
function load_bno_mount(path = BNO_MOUNT_FILE)
    isfile(path) || return BNO_MOUNT_NOMINAL
    m = TOML.parsefile(path)["matrix"]            # three rows
    return SMatrix{3,3,Float64,9}(Float64[m[i][j] for i in 1:3, j in 1:3])
end

"Mean of unit quaternions (largest eigenvector of the sum of q*q'), sign-independent. Returns the mean and the angular RMS spread [deg]."
function mean_quaternion(q::AbstractVector{SVector{4,Float64}})
    M = zeros(4, 4)
    for v in q
        M .+= v * v'
    end
    m = SVector{4,Float64}(eigen(Symmetric(M)).vectors[:, 4])
    m[1] < 0 && (m = -m)
    ang = [2acos(min(1.0, abs(dot(m, v)))) for v in q]
    return m, rad2deg(sqrt(mean(abs2, ang)))
end

"""
    frame_to_world(north)

Orientation (rig frame to ENU world) of a rig that stands level with the horizontal rig axis `north`
(`"+X"`, `"-X"`, `"+Y"` or `"-Y"`) pointing to magnetic north. Rig Z points up.
"""
function frame_to_world(north::AbstractString)
    m = match(r"^([+-])([XY])$", uppercase(strip(north)))
    m === nothing && error("north axis must be one of +X -X +Y -Y, not \"$north\"")
    s = m[1] == "+" ? 1.0 : -1.0
    N = SVector(0.0, 1.0, 0.0); U = SVector(0.0, 0.0, 1.0)
    if m[2] == "X"
        X = s * N; Y = cross(U, X)                # right-handed: Y = Z x X
    else
        Y = s * N; X = cross(Y, U)                # X = Y x Z
    end
    return SMatrix{3,3,Float64,9}(X..., Y..., U...)
end

"""
    bno_mount_from_pose(q, north)

`q`: mean BNO quaternion (sensor to world) while the rig stands level with rig axis `north` pointing to magnetic north.
Returns the rotation from rig-frame to BNO coordinates, `M = R(q)' * R_true`, so that `R(q) * M` is the true attitude.
"""
bno_mount_from_pose(q::SVector{4,Float64}, north) = SMatrix{3,3,Float64,9}(rotation_matrix(q)' * frame_to_world(north))

"Writes the mounting matrix with its provenance. `info` holds plain values (strings, numbers) that are stored along."
function save_bno_mount(M::SMatrix{3,3,Float64,9}, info::AbstractDict; path = BNO_MOUNT_FILE)
    mkpath(dirname(path))
    doc = Dict{String,Any}(
        "written" => string(Dates.now()),
        "matrix" => [[M[i, j] for j in 1:3] for i in 1:3],
        "info" => Dict{String,Any}(string(k) => v for (k, v) in info))
    open(path, "w") do io
        println(io, "# BNO085 mounting: rotation from rig-frame to BNO-sensor coordinates (rows). Written by scripts/calibrate_bno_mount.jl")
        TOML.print(io, doc)
    end
    return path
end

"The rotation angle [deg] of `M` relative to the nominal mounting, split as (total, tilt of the BNO z axis, heading)."
function mount_difference(M)
    D = M * BNO_MOUNT_NOMINAL'                   # correction applied on the BNO side
    total = rad2deg(acos(clamp((tr(D) - 1) / 2, -1, 1)))
    tilt = rad2deg(acos(clamp(D[3, 3], -1, 1)))
    heading = rad2deg(atan(D[2, 1], D[1, 1]))
    return (total = total, tilt = tilt, heading = heading)
end

"""
    tilt_mount_from_pose(q)

Tilt-only mounting rotation from a level pose, independent of the heading (so independent of the magnetometer): the
smallest rotation that makes the sensor's idea of "up" (`R(q)' * z`) the rig Z axis. The heading part stays nominal.
"""
function tilt_mount_from_pose(q::SVector{4,Float64})
    u = SVector{3,Float64}(rotation_matrix(q)' * SVector(0.0, 0.0, 1.0))   # world up in sensor coordinates
    T = rotation_between(SVector(0.0, 0.0, 1.0), u)                        # rotates z onto u
    return SMatrix{3,3,Float64,9}(T * BNO_MOUNT_NOMINAL)
end

"RMS angle [deg] of the sensor's up direction around its mean over the quaternions `q` (heading does not matter)."
function up_scatter(q::AbstractVector{SVector{4,Float64}})
    ups = [rotation_matrix(v)' * SVector(0.0, 0.0, 1.0) for v in q]
    m = normalize(sum(ups))
    return rad2deg(sqrt(mean(acos(clamp(dot(m, u), -1, 1))^2 for u in ups)))
end
