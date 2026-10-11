using Arrow, DataFrames
using GLMakie

cd("Cave_Lidar")

Tracking = Arrow.Table("t265_data.arrow")
Tracking_df = DataFrame(Tracking)

show(first(Tracking_df, 10))

tr_ts = Tracking_df.timestamp

diff(tr_ts)


fig = Figure(resolution = (1920, 1080))
ax = Axis3(fig[1, 1], title = "3D Trajectory", xlabel = "t_x", ylabel = "t_y", zlabel = "t_z")

# Use 'lines' for a continuous colored line
# colorrange ensures 0-3 mapping stays consistent
plt = lines!(ax, Tracking_df.t_x, Tracking_df.t_y, -Tracking_df.t_z, 
             color = Tracking_df.r_w, 
             colormap = :viridis,
             linewidth = 4)

             # Add a colorbar
Colorbar(fig[1, 2], plt, label = "rotation")

display(plt)