"Rotation about +Z by `psi` [rad]."
@inline function rotz(psi::Real)
    s, c = sincos(psi)
    return SMatrix{3,3,Float64,9}(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0)
end

"""
IMU samples in the levelled rig frame (the frame of `untangled.arrow`: z up, heading the rig's own). Times in seconds
(any origin, the same one as the lidar points). `acc` is the acceleration of the sensor without gravity, `gyro_z` the
yaw rate about the vertical.
"""
struct ImuTrack
    t::Vector{Float64}
    acc::Vector{SVector{3,Float64}}
    gyro_z::Vector{Float64}
end

"Linear interpolation of the IMU at time `t` (held constant outside the recording)."
function sample_imu(imu::ImuTrack, t::Float64)
    n = length(imu.t)
    i = clamp(searchsortedlast(imu.t, t), 1, n - 1)
    f = clamp((t - imu.t[i]) / (imu.t[i+1] - imu.t[i]), 0.0, 1.0)
    return imu.acc[i] + f * (imu.acc[i+1] - imu.acc[i]), imu.gyro_z[i] + f * (imu.gyro_z[i+1] - imu.gyro_z[i])
end

"Pose and velocity of the rig at one instant: position and velocity in the start frame, yaw about the vertical."
struct PathState
    p::SVector{3,Float64}
    v::SVector{3,Float64}
    psi::Float64
end

"""
The IMU prediction of one window `[t0, t0 + T]`, integrated from `PathState` with a fixed step `dt`: constant
acceleration `A[j]` and yaw rate `G[j]` in step j, `P`, `V`, `Psi` at the step boundaries (one more entry than steps).
The accelerometer is rotated into the start frame with the predicted yaw, so a yaw error leaks into the position.
"""
struct WindowPrediction
    t0::Float64
    dt::Float64
    P::Vector{SVector{3,Float64}}
    V::Vector{SVector{3,Float64}}
    Psi::Vector{Float64}
    A::Vector{SVector{3,Float64}}
    G::Vector{Float64}
end

Base.length(w::WindowPrediction) = length(w.A)
duration(w::WindowPrediction) = length(w) * w.dt

"Integrates the IMU over one window; `bias_acc` (levelled rig frame) and `bias_gyro` are subtracted from the samples."
function predict_window(imu::ImuTrack, s::PathState, t0::Float64, T::Float64, bias_acc::SVector{3,Float64},
                        bias_gyro::Float64; dt::Float64 = 0.01)
    n = max(1, round(Int, T / dt))
    P = Vector{SVector{3,Float64}}(undef, n + 1); V = similar(P); Psi = Vector{Float64}(undef, n + 1)
    A = Vector{SVector{3,Float64}}(undef, n); G = Vector{Float64}(undef, n)
    P[1] = s.p; V[1] = s.v; Psi[1] = s.psi
    for j in 1:n
        a_b, g = sample_imu(imu, t0 + (j - 0.5) * dt)
        g -= bias_gyro
        a = rotz(Psi[j] + 0.5 * g * dt) * (a_b - bias_acc)
        A[j] = a; G[j] = g
        P[j+1] = P[j] + V[j] * dt + 0.5 * a * dt^2
        V[j+1] = V[j] + a * dt
        Psi[j+1] = Psi[j] + g * dt
    end
    return WindowPrediction(t0, dt, P, V, Psi, A, G)
end

"Predicted position and yaw `tau` seconds after the window start (exact for the piecewise constant acceleration)."
@inline function predicted_pose(w::WindowPrediction, tau::Float64)
    j = clamp(floor(Int, tau / w.dt) + 1, 1, length(w))
    s = tau - (j - 1) * w.dt
    return w.P[j] + w.V[j] * s + (0.5 * s^2) * w.A[j], w.Psi[j] + w.G[j] * s
end

"The end state of a window prediction plus the correction `dp` (displacement over the window) and `dpsi` (yaw offset
at the window end) found by the registration. The velocity offset `dp / T` stays in the state for all later windows."
function end_state(w::WindowPrediction, dp::SVector{3,Float64}, dpsi::Float64)
    T = duration(w)
    return PathState(w.P[end] + dp, w.V[end] + dp / T, w.Psi[end] + dpsi)
end

"One fitted window: the IMU prediction and the correction. Together with the others it is the estimated path."
struct WindowFit
    pred::WindowPrediction
    dp::SVector{3,Float64}      # corrected displacement at the window end [m]; the correction grows linearly in time
    dpsi::Float64               # corrected yaw offset at the window end [rad]
    n_used::Int                 # points with a plane correspondence in the last iteration
    rms::Float64                # rms point-to-plane residual of those [m]
    info_min::Float64           # smallest eigenvalue of the data information matrix (degenerate direction when small)
    iterations::Int
end

"The estimated path: all fitted windows in time order and their start times."
struct PathEstimate
    fits::Vector{WindowFit}
    starts::Vector{Float64}
end
PathEstimate(fits::Vector{WindowFit}) = PathEstimate(fits, [f.pred.t0 for f in fits])

"Pose (position, yaw) of the estimated path at time `t`. Before the first window the rig is at the origin."
function pose_at(est::PathEstimate, t::Float64)
    (isempty(est.fits) || t < est.starts[1]) && return SVector(0.0, 0.0, 0.0), 0.0
    k = clamp(searchsortedlast(est.starts, t), 1, length(est.fits))
    return corrected_pose(est.fits[k], t - est.starts[k])
end

@inline function corrected_pose(f::WindowFit, tau::Float64)
    p, psi = predicted_pose(f.pred, tau)
    r = clamp(tau / duration(f.pred), 0.0, 1.0)
    return p + r * f.dp, psi + r * f.dpsi
end

"A sensor-frame point `s` placed into the start frame by the pose `(p, psi)`."
@inline place(p::SVector{3,Float64}, psi::Float64, s) = p + rotz(psi) * SVector{3,Float64}(s)
