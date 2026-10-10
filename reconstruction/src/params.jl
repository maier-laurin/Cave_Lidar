Base.@kwdef struct RigParams
    steps_per_rev::Float64 = 31999.0         # docs, measured; only a fallback when a revolution has no closing edge
    vertical_after_edge_deg::Float64 = NaN   # angle from the index edge to vertical; NaN = the value the firmware wrote into the
                                             # recording (rig file metadata, 90.3 since 2026-10-10), else 93.0 (the value before)
    sigma::Int = -1                          # turning direction of the shaft angle about +X
    tau_deg::Float64 = 15.960                # tilt of the lidar spin axis; drawing says 15 (mount at 165 deg), fitted 15.960
    psi_deg::Float64 = 89.855                # yaw of the lidar on its mount; fitted 89.855
    origin_x::Float64 = 0.0                  # [m] optical centre in the rotating frame (X along the shaft); X is a pure
    origin_y::Float64 = -1.111e-3            #     translation of the cloud, Y and Z move the lidar on a circle around the
    origin_z::Float64 = -0.482e-3            #     axis; fitted with calibrate_mount.jl on data/scans/optim_run (2026-10-09)
    min_range_m::Float64 = 0.5               # drops the rig itself
    max_rig_gap_ns::Int64 = 50_000_000       # no interpolation across a rig dropout longer than this

    # BNO085 sits rigidly on the frame. Nominal (drawing): its z axis is the rig Z axis, its y axis is the rig X axis (so its x axis is
    # the rig -Y axis). This maps a vector from rig-frame coordinates to BNO-sensor coordinates (a rotation of +90 deg
    # about z); the columns are the images of rig X, Y, Z.
    bno_from_frame::SMatrix{3,3,Float64,9} = load_bno_mount()   # calibration/bno_mount.toml if present, else the nominal mapping
    bno_latency_ns::Int64 = 0                # how much later than the measurement the BNO sample is stamped
end

"Copy of `P` with the given fields replaced."
function with_params(P::RigParams; kw...)
    nt = NamedTuple{fieldnames(RigParams)}(map(f -> getfield(P, f), fieldnames(RigParams)))
    return RigParams(; merge(nt, (; kw...))...)
end

"Offset to vertical of all recordings made before the firmware wrote it into the rig files (until 2026-10-10)."
const LEGACY_VERTICAL_AFTER_EDGE_DEG = 93.0

"`P` with `vertical_after_edge_deg` filled in from the schema metadata of the rig files (written by the recorder from the
firmware's stream header) when it is NaN. Recordings without that entry were made with the old firmware: 93.0."
function resolve_params(P::RigParams, rig_files)
    isnan(P.vertical_after_edge_deg) || return P
    files = rig_files isa AbstractString ? [rig_files] : rig_files
    v = NaN
    for f in files
        m = Arrow.getmetadata(Arrow.Table(f))
        if m !== nothing && haskey(m, "vertical_after_edge_deg")
            v = parse(Float64, m["vertical_after_edge_deg"])
            break
        end
    end
    if isnan(v)
        @warn "no vertical_after_edge_deg in the rig files, using the legacy value $LEGACY_VERTICAL_AFTER_EDGE_DEG deg"
        v = LEGACY_VERTICAL_AFTER_EDGE_DEG
    end
    return with_params(P; vertical_after_edge_deg = v)
end
