"Wraps an angle difference to [-pi, pi]."
wrap_pi(d) = rem2pi(d, RoundNearest)

"Point at fraction `f` on the shortest arc from angle `a` to angle `b` (radians), result in [0, 2pi)."
angle_lerp(a, b, f) = mod2pi(a + f * wrap_pi(b - a))

"Shaft angle [rad, in [0, 2pi)] at the unix times `t` (sorted, ns); 0 is vertical."
struct AngleTrack
    t::Vector{Int64}
    theta::Vector{Float64}
end

"""
    sample_angles(rig_files, edges_files, P)

Shaft angle of every valid 100 Hz rig sample, and the edge times. The angle of a sample is
`2pi * steps / N - vertical_after_edge`, where N is the step count of the revolution the sample belongs to,
known from the edge that closes it. A sample after the last edge uses the mean of the known counts.
"""
function sample_angles(rig_files, edges_files, P::RigParams)
    valid = (column(rig_files, :flags) .& 0x01) .!= 0
    t_s = unix_ns(column(rig_files, :time))[valid]
    n_s = Float64.(column(rig_files, :axis_steps)[valid])
    t_e = unix_ns(column(edges_files, :time))
    N_e = Float64.(column(edges_files, :steps_before_reset))
    N_mean = isempty(N_e) ? P.steps_per_rev : mean(N_e)
    offset = deg2rad(P.vertical_after_edge_deg)

    theta_s = similar(n_s)
    for j in eachindex(n_s)
        k = searchsortedlast(t_e, t_s[j])        # edges at or before the sample, so edge k+1 closes its revolution
        N = k < length(t_e) ? N_e[k+1] : N_mean
        theta_s[j] = mod2pi(2pi * n_s[j] / N - offset)
    end
    return t_s, theta_s, t_e
end

"Angle track of the rig samples; the index edges are added as exact nodes (angle = -vertical_after_edge)."
function angle_track(rig_files, edges_files, P::RigParams = RigParams())
    P = resolve_params(P, rig_files)
    t_s, theta_s, t_e = sample_angles(rig_files, edges_files, P)
    theta_edge = mod2pi(-deg2rad(P.vertical_after_edge_deg))
    t = [t_s; t_e]
    theta = [theta_s; fill(theta_edge, length(t_e))]
    p = sortperm(t)
    return AngleTrack(t[p], theta[p])
end
