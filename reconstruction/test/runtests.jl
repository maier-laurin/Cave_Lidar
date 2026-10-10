using Test, Arrow, StaticArrays, LinearAlgebra
using CaveLidarRecon
import CaveLidarRecon: angle_lerp, wrap_pi, slerp, rotation_matrix, TimestampNs, unix_ns

ts(v) = reinterpret(TimestampNs, Int64.(v))

@testset "angles" begin
    @test wrap_pi(3pi / 2) ≈ -pi / 2
    # shortest arc across the 0 / 2pi seam, never the long way round
    @test wrap_pi(angle_lerp(deg2rad(350), deg2rad(10), 0.5)) ≈ 0.0 atol = 1e-12
    @test angle_lerp(deg2rad(10), deg2rad(350), 0.25) ≈ deg2rad(5)
    @test 0 <= angle_lerp(deg2rad(359), deg2rad(1), 0.1) < 2pi
end

@testset "attitude" begin
    qz(a) = SVector(cos(a / 2), 0.0, 0.0, sin(a / 2))
    @test rotation_matrix(qz(pi / 2)) * SVector(1.0, 0, 0) ≈ SVector(0.0, 1, 0) atol = 1e-12
    @test slerp(qz(0.0), qz(pi / 2), 0.5) ≈ qz(pi / 4)
    # q and -q are the same rotation: interpolation must not go the long way
    @test slerp(qz(0.0), -qz(pi / 2), 0.5) ≈ qz(pi / 4)
    @test slerp(qz(0.3), qz(0.3), 0.7) ≈ qz(0.3)
end

# Synthetic scan: shaft angle 0 the whole time, the frame turns about world z at a constant rate, one lidar point
# straight ahead (az 0, el 0, mount rotations 0). The expected world position is known in closed form.
@testset "untangle_scan end to end" begin
    dir = mktempdir()
    t0 = 1_700_000_000_000_000_000
    ω = 0.5                                    # rad/s about world z
    dt = 10_000_000                            # 100 Hz rig samples
    nrig = 101
    rig_t = [t0 + (k - 1) * dt for k in 1:nrig]
    ang(t) = ω * (t - t0) * 1e-9
    qs = [(cos(ang(t) / 2), 0.0, 0.0, sin(ang(t) / 2)) for t in rig_t]
    Arrow.write(joinpath(dir, "rig_0001.arrow"), (
        time = ts(rig_t), axis_steps = zeros(Int32, nrig),
        qw = Float32[q[1] for q in qs], qx = Float32[q[2] for q in qs],
        qy = Float32[q[3] for q in qs], qz = Float32[q[4] for q in qs],
        flags = fill(0x01, nrig)))
    Arrow.write(joinpath(dir, "edges_0001.arrow"), (time = ts([t0 - dt]), steps_before_reset = Int32[31999]))

    lt = [t0 + 3_300_000, t0 + 123_456_789, t0 + 987_654_321]      # between rig samples
    n = length(lt)
    Arrow.write(joinpath(dir, "lidar_0001.arrow"), (
        time = ts(lt), azimuth = zeros(UInt16, n), elevation = zeros(Int8, n),
        distance = fill(UInt16(5000), n), reflectivity = UInt8[10, 20, 30]))   # 5000 * 2 mm = 10 m

    P = RigParams(bno_from_frame = CaveLidarRecon.BNO_MOUNT_NOMINAL, vertical_after_edge_deg = 0.0, tau_deg = 0.0, psi_deg = 0.0,
        origin_x = 0.0, origin_y = 0.0, origin_z = 0.0)
    out = joinpath(dir, "untangled.arrow")
    c = untangle_scan(dir; out, params = P, attitude = :world)
    @test (c.read, c.written, c.no_rig, c.no_attitude) == (3, 3, 0, 0)

    res = Arrow.Table(out; convert = false)
    @test unix_ns(res.time) == lt
    @test collect(res.reflectivity) == UInt8[10, 20, 30]
    # rig frame point (0, r, 0); the BNO sits turned by 90 deg: bno = Rz(90) * frame = (-r, 0, 0); then Rz(ang)
    for (k, t) in enumerate(lt)
        φ = ang(t)
        @test [res.x[k], res.y[k], res.z[k]] ≈ [-10cos(φ), -10sin(φ), 0] atol = 2e-4
    end

    # without the attitude the points stay in the rig frame
    out2 = joinpath(dir, "rigframe.arrow")   # not `out`: that file is memory-mapped by `res` (Windows)
    c2 = untangle_scan(dir; out = out2, params = P, attitude = :none)
    res2 = Arrow.Table(out2; convert = false)
    @test [res2.x[1], res2.y[1], res2.z[1]] ≈ [0, 10, 0] atol = 1e-5

    # gravity mode: the frame only turns about the vertical, so nothing is corrected and the rig-frame point stays
    out3 = joinpath(dir, "gravity.arrow")
    c4 = untangle_scan(dir; out = out3, params = P)
    res3 = Arrow.Table(out3; convert = false)
    @test c4.written == 3
    @test [res3.x[2], res3.y[2], res3.z[2]] ≈ [0, 10, 0] atol = 1e-4

    # a BNO dropout longer than max_rig_gap_ns drops the points inside it
    P2 = RigParams(bno_from_frame = CaveLidarRecon.BNO_MOUNT_NOMINAL, vertical_after_edge_deg = 0.0, tau_deg = 0.0, psi_deg = 0.0, origin_x = 0.0, origin_y = 0.0,
        origin_z = 0.0, max_rig_gap_ns = 5_000_000)
    c3 = untangle_scan(dir; out = joinpath(dir, "gap.arrow"), params = P2, attitude = :world)
    @test c3.written == 0
end

@testset "BNO mount calibration" begin
    import CaveLidarRecon: BNO_MOUNT_NOMINAL
    # a rig that stands level with +X to the north, and a BNO that is mounted like the nominal mapping but tilted 4 deg about rig Y
    Rtrue = frame_to_world("+X")
    @test Rtrue * SVector(1.0, 0, 0) ≈ SVector(0.0, 1, 0)     # rig X -> north
    @test Rtrue * SVector(0.0, 0, 1) ≈ SVector(0.0, 0, 1)     # rig Z -> up
    @test Rtrue * SVector(0.0, 1, 0) ≈ SVector(-1.0, 0, 0)    # rig Y -> west (right-handed)
    @test frame_to_world("+Y") * SVector(0.0, 1, 0) ≈ SVector(0.0, 1, 0)
    @test det(frame_to_world("-X")) ≈ 1

    a = deg2rad(4)
    tilt = @SMatrix [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]
    Mtrue = SMatrix{3,3,Float64,9}(tilt * BNO_MOUNT_NOMINAL)    # what the calibration has to find
    # attitude the sensor would report: R(q) * Mtrue = Rtrue  =>  R(q) = Rtrue * Mtrue'
    Rq = Rtrue * Mtrue'
    w = sqrt(1 + Rq[1, 1] + Rq[2, 2] + Rq[3, 3]) / 2
    q = SVector(w, (Rq[3, 2] - Rq[2, 3]) / 4w, (Rq[1, 3] - Rq[3, 1]) / 4w, (Rq[2, 1] - Rq[1, 2]) / 4w)
    @test CaveLidarRecon.rotation_matrix(q) ≈ Rq
    qm, spread = mean_quaternion([q, -q, q])                    # the sign of a quaternion is arbitrary
    @test spread < 1e-6
    M = bno_mount_from_pose(qm, "+X")
    @test M ≈ Mtrue atol = 1e-9
    @test mount_difference(M).tilt ≈ 4 atol = 1e-6
    @test mount_difference(BNO_MOUNT_NOMINAL).total ≈ 0 atol = 1e-6

    path = joinpath(mktempdir(), "bno_mount.toml")
    @test load_bno_mount(path) == BNO_MOUNT_NOMINAL              # no file: nominal
    save_bno_mount(M, Dict("north_axis" => "+X"); path)
    @test load_bno_mount(path) ≈ M
end

@testset "tilt-only mount" begin
    import CaveLidarRecon: BNO_MOUNT_NOMINAL
    a = deg2rad(4)
    tilt = @SMatrix [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]
    # any heading must give the same tilt result: the sensor reports Rq = Rz(h) * (tilt * nominal)' ... for a level rig
    for h in (0.0, 0.7, 2.5, -1.9)
        Rz = @SMatrix [cos(h) -sin(h) 0; sin(h) cos(h) 0; 0 0 1]
        Rq = Rz * (tilt * BNO_MOUNT_NOMINAL)'
        w = sqrt(1 + Rq[1, 1] + Rq[2, 2] + Rq[3, 3]) / 2
        q = SVector(w, (Rq[3, 2] - Rq[2, 3]) / 4w, (Rq[1, 3] - Rq[3, 1]) / 4w, (Rq[2, 1] - Rq[1, 2]) / 4w)
        M = tilt_mount_from_pose(q)
        @test CaveLidarRecon.rotation_matrix(q) * M * SVector(0.0, 0, 1) ≈ SVector(0.0, 0, 1) atol = 1e-9   # rig Z is up
        @test up_scatter([q, q]) < 1e-5
    end
    @test tilt_mount_from_pose(SVector(1.0, 0, 0, 0)) ≈ BNO_MOUNT_NOMINAL                                   # level BNO: nominal
end

@testset "rotation_between and gravity mode" begin
    import CaveLidarRecon: rotation_between
    z = SVector(0.0, 0.0, 1.0)
    for a in (SVector(0.0, 0.087, 0.996), SVector(0.3, -0.5, 0.8), SVector(0.0, 0.0, 1.0), SVector(1.0, 0.0, 0.0), SVector(0.0, 0.0, -1.0))
        R = rotation_between(a, z)
        @test R * normalize(a) ≈ z atol = 1e-12
        @test det(R) ≈ 1 atol = 1e-12
        @test R * R' ≈ I atol = 1e-12
    end
    # minimal: the axis perpendicular to both vectors is untouched, the angle is the angle between them
    a = normalize(SVector(0.3, -0.5, 0.8)); R = rotation_between(a, z)
    @test R * normalize(cross(a, z)) ≈ normalize(cross(a, z)) atol = 1e-12
    @test acos((tr(R) - 1) / 2) ≈ acos(dot(a, z)) atol = 1e-9
end

@testset "vertical offset from the recording" begin
    import CaveLidarRecon: LEGACY_VERTICAL_AFTER_EDGE_DEG
    dir = mktempdir()
    tbl = (time = ts([1, 2]), axis_steps = Int32[0, 1])
    with_meta = joinpath(dir, "rig_0001.arrow"); no_meta = joinpath(dir, "old_0001.arrow")
    Arrow.write(with_meta, tbl; metadata = ["vertical_after_edge_deg" => "90.3"])
    Arrow.write(no_meta, tbl)
    @test isnan(RigParams().vertical_after_edge_deg)                                          # default: ask the recording
    @test resolve_params(RigParams(), with_meta).vertical_after_edge_deg ≈ 90.3
    @test resolve_params(RigParams(), [no_meta, with_meta]).vertical_after_edge_deg ≈ 90.3   # first file that has it
    @test (@test_logs (:warn, r"legacy") resolve_params(RigParams(), no_meta)).vertical_after_edge_deg == LEGACY_VERTICAL_AFTER_EDGE_DEG
    @test resolve_params(with_params(RigParams(); vertical_after_edge_deg = 95.0), with_meta).vertical_after_edge_deg == 95.0   # explicit wins
end
