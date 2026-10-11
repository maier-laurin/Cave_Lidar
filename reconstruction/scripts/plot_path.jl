# Self-contained HTML viewer of an odometry result: the fitted path in 3D (optionally with the standstill map and the moving
# points) and charts of yaw and fit quality. Other paths can be overlaid. Open the file in any browser (plotly.js comes from a CDN).
#
#   julia --project=reconstruction reconstruction/scripts/plot_path.jl <scan_folder> [--main=spline_h01] [--compare=spline_h02,odometry]
#         [--points] [--seed=31.5] [--title="First Move Path"] [--out=odometry_view.html]
#
# `--main=NAME` reads <scan>/NAME_path.csv (and NAME_cloud.ply with --points); `--compare` lists more paths to overlay, written by
# spline_odometry.jl or odometry.jl (columns are found by name: t, x, y, z, yaw_deg, rms, n_used).

using Printf

function read_ply(path)
    bytes = read(path)
    hdr_end = findfirst(codeunits("end_header\n"), bytes)[end]
    n = parse(Int, match(r"element vertex (\d+)", String(bytes[1:hdr_end])).captures[1])
    io = IOBuffer(bytes[hdr_end+1:end])
    xyz = Matrix{Float32}(undef, 3, n); t = Vector{Float64}(undef, n)
    for i in 1:n
        xyz[1, i] = read(io, Float32); xyz[2, i] = read(io, Float32); xyz[3, i] = read(io, Float32)
        t[i] = read(io, Float64)
    end
    return xyz, t
end

arr(v, d) = "[" * join((@sprintf("%.*f", d, x) for x in v), ",") * "]"

# columns of a path csv by header name
function read_path(file)
    lines = readlines(file)
    names = split(lines[1], ",")
    rows = [parse.(Float64, split(l, ",")) for l in lines[2:end]]
    return Dict(String(n) => [r[i] for r in rows] for (i, n) in enumerate(names))
end

path_json(P, d = 3) = """{"t":$(arr(P["t"],2)),"x":$(arr(P["x"],d)),"y":$(arr(P["y"],d)),"z":$(arr(P["z"],d)),"yaw":$(arr(P["yaw_deg"],2)),"rms":$(arr(P["rms"],4)),"n":$(arr(P["n_used"],0))}"""

function main(args)
    dirs = filter(a -> !startswith(a, "--"), args)
    length(dirs) == 1 || (println(stderr, "usage: plot_path.jl <scan_folder> [options]"); exit(2))
    dir = dirs[1]
    opt(name, default) = (v = [split(a, "=", limit = 2)[2] for a in args if startswith(a, "--$name=")]; isempty(v) ? default : v[1])
    seed = parse(Float64, opt("seed", "31.5"))
    main_name = opt("main", "odometry")
    title = opt("title", "Rig Path " * basename(dirname(abspath(dir))))
    scan = basename(dirname(abspath(dir))) * "/" * basename(abspath(dir))

    P = read_path(joinpath(dir, "$(main_name)_path.csv"))
    cmp = String[]
    for c in split(opt("compare", ""), ",", keepempty = false)
        push!(cmp, """{"name":"$c","path":$(path_json(read_path(joinpath(dir, "$(c)_path.csv"))))}""")
    end

    seedj = "{\"x\":[],\"y\":[],\"z\":[]}"; movej = "{\"t\":[],\"x\":[],\"y\":[],\"z\":[]}"
    if "--points" in args
        xyz, t = read_ply(joinpath(dir, "$(main_name)_cloud.ply"))
        thin(idx, target) = idx[1:max(1, length(idx) ÷ target):end]
        si = thin(findall(<(seed), t), 14000); mi = thin(findall(>=(seed), t), 22000)
        seedj = """{"x":$(arr(xyz[1,si],3)),"y":$(arr(xyz[2,si],3)),"z":$(arr(xyz[3,si],3))}"""
        movej = """{"t":$(arr(t[mi],2)),"x":$(arr(xyz[1,mi],3)),"y":$(arr(xyz[2,mi],3)),"z":$(arr(xyz[3,mi],3))}"""
    end
    data = """{"scan":"$scan","main":"$main_name","path":$(path_json(P)),"cmp":[$(join(cmp, ","))],"seed":$seedj,"move":$movej}"""
    html = replace(read(joinpath(@__DIR__, "path_view_template.html"), String), "__TITLE__" => title, "__DATA__" => data)
    out = joinpath(dir, opt("out", "odometry_view.html"))
    write(out, html)
    @printf("wrote %s (%.1f MB): %d path poses, %d overlays, points %s\n", out, filesize(out) / 1e6, length(P["t"]), length(cmp), "--points" in args)
end

main(ARGS)
