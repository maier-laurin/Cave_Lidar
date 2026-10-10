# Cave_Lidar
A system built around a Velodyne VLP-16 to assist in the mapping of large indoor cavity systems like caves.

## Layout
| Folder | Content |
|---|---|
| `cad/` | AutoCAD rig: `assembly.dwg` (full rig), `drawings.dwg` (manufacturing drawings), `lidar_vlp16.stl` (VLP-16 reference model) |
| `firmware/` | Teensy control firmware (`lidar_ctrl/`) and the BNO085 calibration helper (`bno_calibration/`) |
| `capture/` | C# acquisition software (.NET): lidar and rig recorders, `CaveScan` orchestrator |
| `reconstruction/` | Julia package `CaveLidarRecon`: untangling of the shaft rotation and the IMU attitude, BNO calibration |
| `docs/` | Typst engineering document (`cave_lidar.typ`, figures in `figures/`). Build: `typst compile docs/cave_lidar.typ` |
| `data/` | Local recorded sensor data, not tracked by git |

The viewing software will be added as a separate top-level folder.

Note: CAD drawings in this repository were created using an Autodesk Student Version and are intended for educational/non-commercial use only.
