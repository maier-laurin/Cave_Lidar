"""
Offline reconstruction of the cave lidar rig recordings.

First stage, `untangle_scan`: turns the raw lidar returns of a scan folder into xyz points. The shaft rotation is
undone with the angle track from the rig samples and the index edges, then the frame attitude of the BNO085 is
applied to every point, so the points end up in an East-North-Up world orientation (rotation only, no translation).

Second stage (in progress), `run_odometry`: moves through the untangled points one window at a time and fits the path of
the rig (position and yaw) to the points seen so far, with the IMU as the prior.
"""
module CaveLidarRecon

using Arrow, StaticArrays, Rotations, Statistics, LinearAlgebra, TOML, Dates, Random
using NearestNeighbors
import Tables

include("params.jl")
include("io.jl")
include("angles.jl")
include("attitude.jl")
include("calibration.jl")
include("untangle.jl")
include("trajectory.jl")
include("registration.jl")
include("spline_odometry.jl")
include("imu.jl")
include("simulate.jl")

export RigParams, with_params, resolve_params, AngleTrack, AttitudeTrack, UntangleCounts
export scan_files, angle_track, attitude_track, untangle_scan, find_scans
export load_bno_mount, mean_quaternion, frame_to_world, bno_mount_from_pose, tilt_mount_from_pose, up_scatter, save_bno_mount, mount_difference, BNO_MOUNT_FILE
export ImuTrack, PathState, WindowPrediction, WindowFit, PathEstimate, predict_window, pose_at, rotz, place
export OdometryConfig, PointMap, fit_window, run_odometry, dead_reckon
export SplinePath, run_spline_odometry, spline_pose, plane_residuals, add_points!, rebuild!
export imu_track
export SimPath, sim_pose, sim_motion, simulate_scan, simulate_imu, standstill_bias

end
