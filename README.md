# Cave_Lidar
A system built around a Velodyne VLP-16 to assist in the mapping of large indoor cavity systems like caves.

## Layout
| Folder | Content |
|---|---|
| `cad/` | AutoCAD rig: `assembly.dwg` (full rig), `drawings.dwg` (manufacturing drawings), `lidar_vlp16.stl` (VLP-16 reference model) |
| `firmware/` | Teensy control firmware (`lidar_ctrl/`) |
| `docs/` | Typst engineering document (`cave_lidar.typ`, figures in `figures/`). Build: `typst compile docs/cave_lidar.typ` |
| `data/` | Local recorded sensor data, not tracked by git |

Acquisition, reconstruction and viewing software will be added as separate top-level folders.

Note: CAD drawings in this repository were created using an Autodesk Student Version and are intended for educational/non-commercial use only.
