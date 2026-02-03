using Arrow
using DataFrames
using GLMakie

# Specify the path to your .arrow or .feather file
file_path = "vlp16/output.arrow"

# Read the file and convert it into a DataFrame
df = Arrow.Table(file_path) |> DataFrame

# Display the first few rows
show(first(df, 5))

maximum(df.distance)

z = sind.(df.angle) .* df.distance
vd = cosd.(df.angle) .* df.distance
x = sind.(df.azimuth) .* vd
y = cosd.(df.azimuth) .* vd

fig= Figure(resolution = (2000, 1200))
ax = Axis3(fig[1, 1], title = "3D Cloud", xlabel = "x", ylabel = "y", zlabel = "z", aspect = :data)
plt = scatter!(ax, x, y, z)