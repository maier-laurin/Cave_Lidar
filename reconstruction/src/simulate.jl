"""
A known test path of the rig: standstill until `t_start`, then a slow walk with a stride bounce, a sideways sway and a
yaw swing. Times in seconds from the start of the recording. Analytic position and yaw; velocity and acceleration by
central differences (the error is far below the IMU noise).
"""
Base.@kwdef struct SimPath
    t_start::Float64 = 8.0
    ramp_s::Float64 = 3.0               # the stride and sway fade in over this time
    amp::SVector{3,Float64} = SVector(1.5, 0.4, 0.1)        # [m] slow motion in x, y, z
    freq::SVector{3,Float64} = SVector(0.2, 0.35, 0.5)      # [rad/s]
    stride_hz::Float64 = 1.8
    stride_amp::SVector{3,Float64} = SVector(0.02, 0.015, 0.03)   # [m]
    yaw_amp::Float64 = 0.5              # [rad]
    yaw_freq::Float64 = 0.2             # [rad/s]
    sway_yaw::Float64 = 0.03            # [rad] at half the stride frequency
end

function sim_pose(sp::SimPath, t::Float64)
    u = max(t - sp.t_start, 0.0)
    s = clamp(u / sp.ramp_s, 0.0, 1.0)
    r = 3s^2 - 2s^3
    w = 2pi * sp.stride_hz
    p = SVector(sp.amp[1] * (1 - cos(sp.freq[1] * u)) + r * sp.stride_amp[1] * sin(w * u),
                sp.amp[2] * (1 - cos(sp.freq[2] * u)) + r * sp.stride_amp[2] * sin(w * u + 1.0),
                sp.amp[3] * (1 - cos(sp.freq[3] * u)) + r * sp.stride_amp[3] * sin(w * u + 2.0))
    psi = sp.yaw_amp * (1 - cos(sp.yaw_freq * u)) + r * sp.sway_yaw * sin(0.5 * w * u)
    return p, psi
end

"Position, acceleration, yaw and yaw rate of the test path at time `t`."
function sim_motion(sp::SimPath, t::Float64; h::Float64 = 1e-3)
    p, psi = sim_pose(sp, t)
    pp, ppsi = sim_pose(sp, t + h); pm, pmpsi = sim_pose(sp, t - h)
    return p, (pp - 2p + pm) / h^2, psi, (ppsi - pmpsi) / (2h)
end

"""
    simulate_scan(static_pts, times, path)

What a rig moving along `path` would have recorded of a static scene: the points of a standstill scan (`static_pts`,
in the frame of the rig at rest, scan times `times`) seen from the sensor frame at the pose of the path at each point's
time. Only the geometry is moved; occlusion and the changed sampling density at the new viewpoint are not modelled.
"""
function simulate_scan(static_pts::AbstractVector{<:SVector{3,<:Real}}, times::Vector{Float64}, path::SimPath)
    out = Vector{SVector{3,Float32}}(undef, length(static_pts))
    for i in eachindex(static_pts)
        p, psi = sim_pose(path, times[i])
        out[i] = SVector{3,Float32}(rotz(-psi) * (SVector{3,Float64}(static_pts[i]) - p))
    end
    return out
end

"""
    simulate_imu(path, t0, t1, rng; rate_hz, acc_noise, gyro_noise, acc_bias, gyro_bias)

IMU samples of the rig on `path`: acceleration in the levelled rig frame (no gravity, tilt is assumed perfect) and yaw
rate, with constant biases and white noise. Defaults are a rough guess for the BNO085, not a measurement.
"""
function simulate_imu(path::SimPath, t0::Float64, t1::Float64, rng::AbstractRNG; rate_hz::Float64 = 100.0,
                      acc_noise::Float64 = 0.02, gyro_noise::Float64 = deg2rad(0.1),
                      acc_bias::SVector{3,Float64} = SVector(0.04, -0.03, 0.02), gyro_bias::Float64 = deg2rad(0.3))
    t = collect(t0:(1 / rate_hz):t1)
    acc = Vector{SVector{3,Float64}}(undef, length(t)); gz = similar(t)
    for (i, ti) in enumerate(t)
        _, a, psi, w = sim_motion(path, ti)
        acc[i] = rotz(-psi) * a + acc_bias + acc_noise * SVector(randn(rng), randn(rng), randn(rng))
        gz[i] = w + gyro_bias + gyro_noise * randn(rng)
    end
    return ImuTrack(t, acc, gz)
end

"""
Biases as the rig would estimate them at standstill: the mean IMU reading between `t_from` and `t_to`, where the true
acceleration and yaw rate are zero.
"""
function standstill_bias(imu::ImuTrack, t_from::Float64, t_to::Float64)
    m = (imu.t .>= t_from) .& (imu.t .<= t_to)
    return sum(imu.acc[m]) / count(m), sum(imu.gyro_z[m]) / count(m)
end
standstill_bias(imu::ImuTrack, seed_s::Float64) = standstill_bias(imu, imu.t[1], imu.t[1] + seed_s)
