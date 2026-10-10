"Arrow timestamp column type used by the recorders: nanoseconds since 1970, UTC."
const TimestampNs = Arrow.Timestamp{Arrow.Flatbuf.TimeUnit.NANOSECOND,:UTC}

unix_ns(col) = Int64[getfield(t, 1) for t in col]

"One column of an Arrow stream file, or of several chunk files (lidar_0001.arrow, lidar_0002.arrow, ...) joined."
function column(files, name::Symbol)
    files = files isa AbstractString ? [files] : files
    return reduce(vcat, (collect(getproperty(Arrow.Table(f; convert = false), name)) for f in files))
end

"The chunk files `<prefix>_0001.arrow`, `<prefix>_0002.arrow`, ... of a scan folder, in order."
chunk_files(dir, prefix) = [joinpath(dir, f) for f in sort(readdir(dir)) if occursin(Regex("^$(prefix)_\\d+\\.arrow\$"), f)]

"The lidar, rig and edges chunk files of a scan folder. Throws when one kind is missing."
function scan_files(dir)
    isdir(dir) || error("not a folder: $dir")
    files = (lidar = chunk_files(dir, "lidar"), rig = chunk_files(dir, "rig"), edges = chunk_files(dir, "edges"))
    for k in keys(files)
        isempty(files[k]) && error("no $(k)_NNNN.arrow files in $dir")
    end
    return files
end

"The scan folders at or directly below `dir` (a scan folder itself, or a project folder holding scan_NN folders)."
function find_scans(dir)
    isempty(chunk_files(dir, "lidar")) || return [dir]
    subs = [joinpath(dir, d) for d in sort(readdir(dir)) if isdir(joinpath(dir, d))]
    return filter(d -> !isempty(chunk_files(d, "lidar")), subs)
end
