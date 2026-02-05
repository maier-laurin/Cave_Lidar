include("../DualQuaternions/DQ/src/DQ.jl")
using .DQ

using GLMakie
using Arrow
using Tables
using DataFrames

cd("PostProcessing")
t265 = Arrow.Table("t265_data.arrow")
vlp16 = Arrow.Table("vlp16_data.arrow")

# we know, that the timestamp of the t265 is in milliseconds, andd the one of the t265 in microseconds, we use seconds as a common timeframe
# we further asume (for now) that the last timestamp happend at the same time and we use the last 60 seconds of the recording as the possitive time frame

time_vlp16 = vlp16.time ./ 1000000 .- vlp16.time[end] ./ 1000000 .+ 60
time_t265 = t265.timestamp ./ 1000 .- t265.timestamp[end] ./1000 .+ 60

# we first convert the points of the vlp in local euclidian coordinates
function vlp_to_eucl(azi, ang, dist)
    sin_e, cos_e = sincosd(ang)
    h_dist = dist * cos_e
    sin_a, cos_a = sincosd(azi)
    return DQ.Point(h_dist * sin_a, h_dist * cos_a, dist * sin_e)
end

points_local = vlp_to_eucl.(vlp16.azimuth, vlp16.angle, vlp16.distance)
# now look only at the last 60 seconds of the recording
points_local = points_local[time_vlp16 .>= 0]
time_vlp16 = time_vlp16[time_vlp16 .>= 0]

function sync_times(time_vlp, time_ref)
    n = length(time_vlp)
    interp_idx = zeros(Int, n)
    interp_ofs = zeros(Float64, n)
    
    cur_ref = 2 # Start at the second element of reference
    max_ref = length(time_ref)
    
    for i in eachindex(time_vlp)
        t_target = time_vlp[i]
        
        # Advance the reference pointer until t_target is between ref[cur_ref-1] and ref[cur_ref]
        while cur_ref < max_ref && time_ref[cur_ref] < t_target
            cur_ref += 1
        end
        
        # Store the lower index
        idx_low = cur_ref - 1
        interp_idx[i] = idx_low
        
        # Calculate the percentage (interpolation factor)
        t_low = time_ref[idx_low]
        t_high = time_ref[cur_ref]
        
        # Linear interpolation formula: (t - t_low) / (t_high - t_low)
        interp_ofs[i] = (t_target - t_low) / (t_high - t_low)
    end
    
    return interp_idx, interp_ofs
end

interp_idx, interp_ofs = sync_times(time_vlp16, time_t265)

interp_idx
interp_ofs

# for now we will not interpolate the sensor positions, we just use the closest one
vlp_transforms = UnitDualQuaternion.(UnitQuaternion.(t265.r_w, t265.r_x, t265.r_y, t265.r_z), t265.t_x, t265.t_y, t265.t_z)

offsets = vlp_transforms[interp_idx]

points_global = DQ.transform.(offsets, points_local)

xs = getfield.(points_global, :x)
ys = getfield.(points_global, :y)
zs = getfield.(points_global, :z)

points = Point3f.(xs, ys, zs)

fig = Figure(resolution = (1200, 800))
ax = Axis3(fig[1, 1], title = "Point Cloud")
scatter!(ax, points, 
    markersize = 1,
    markerspace = :pixel,
    color = zs,
    colormap = :turbo,
    alpha = 0.5
)
xlims!(ax, -10, 10)
ylims!(ax, -10, 10)
zlims!(ax, -3, 3)
display(fig)