using Arrow, DataFrames
using GLMakie

cd("Cave_Lidar")

table = Arrow.Table("t265_data.arrow")
df = DataFrame(table)

fig = Figure(resolution = (1920, 1080))
ax = Axis3(fig[1, 1], title = "3D Trajectory", xlabel = "t_x", ylabel = "t_y", zlabel = "t_z")

# Use 'lines' for a continuous colored line
# colorrange ensures 0-3 mapping stays consistent
plt = lines!(ax, df.t_x, df.t_y, df.t_z, 
             color = df.confidence, 
             colormap = :viridis, 
             colorrange = (0, 3), 
             linewidth = 4)

             # Add a colorbar
Colorbar(fig[1, 2], plt, label = "Confidence", ticks = 0:3)

display(plt)