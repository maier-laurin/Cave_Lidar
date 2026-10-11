"""
    imu_track(rig_files, t_ref_ns; params = RigParams())

The BNO085 linear acceleration and gyroscope of the rig files as an `ImuTrack` in the levelled rig frame of
`untangled.arrow` (`attitude = :gravity`): each sample is taken from the sensor into the rig frame and then turned by the
smallest rotation that takes the rig's up direction (from the BNO attitude) onto +Z, exactly like the lidar points. So
`acc` is the acceleration without gravity and `gyro_z` the yaw rate about the vertical. Times are seconds after
`t_ref_ns` (unix ns), samples without a quaternion, acceleration or gyroscope value are dropped.

The offset between the BNO and the optical centre of the lidar (about 0.3 m) is not compensated: for a slow rig it adds
`w^2 r` + `alpha x r`, a few cm/s^2.
"""
function imu_track(rig_files, t_ref_ns::Integer; params::RigParams = RigParams())
    t = unix_ns(column(rig_files, :time))
    q = [SVector{4,Float64}(w, x, y, z) for (w, x, y, z) in
         zip(column(rig_files, :qw), column(rig_files, :qx), column(rig_files, :qy), column(rig_files, :qz))]
    la = [SVector{3,Float64}(x, y, z) for (x, y, z) in zip(column(rig_files, :lax), column(rig_files, :lay), column(rig_files, :laz))]
    gy = [SVector{3,Float64}(x, y, z) for (x, y, z) in zip(column(rig_files, :gx), column(rig_files, :gy), column(rig_files, :gz))]
    Rbf = params.bno_from_frame'                     # sensor -> rig frame
    ts = Float64[]; acc = SVector{3,Float64}[]; gz = Float64[]
    for i in eachindex(t)
        (all(isfinite, q[i]) && abs(norm(q[i]) - 1) <= 0.05 && all(isfinite, la[i]) && all(isfinite, gy[i])) || continue
        Rq = rotation_matrix(q[i] / norm(q[i]))
        up = Rbf * SVector{3,Float64}(Rq[3, 1], Rq[3, 2], Rq[3, 3])
        Rl = rotation_between(up, SVector(0.0, 0.0, 1.0))
        push!(ts, (t[i] - t_ref_ns) * 1e-9)
        push!(acc, Rl * (Rbf * la[i]))
        push!(gz, (Rl * (Rbf * gy[i]))[3])
    end
    return ImuTrack(ts, acc, gz)
end
