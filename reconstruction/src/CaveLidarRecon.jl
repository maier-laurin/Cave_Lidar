"""
Offline reconstruction of the cave lidar rig recordings.

First stage, `untangle_scan`: turns the raw lidar returns of a scan folder into xyz points. The shaft rotation is
undone with the angle track from the rig samples and the index edges, then the frame attitude of the BNO085 is
applied to every point, so the points end up in an East-North-Up world orientation (rotation only, no translation).
"""
module CaveLidarRecon

using Arrow, StaticArrays, Rotations, Statistics, LinearAlgebra, TOML, Dates
import Tables

include("params.jl")
include("io.jl")
include("angles.jl")
include("attitude.jl")
include("calibration.jl")
include("untangle.jl")

export RigParams, with_params, resolve_params, AngleTrack, AttitudeTrack, UntangleCounts
export scan_files, angle_track, attitude_track, untangle_scan, find_scans
export load_bno_mount, mean_quaternion, frame_to_world, bno_mount_from_pose, tilt_mount_from_pose, up_scatter, save_bno_mount, mount_difference, BNO_MOUNT_FILE

end
