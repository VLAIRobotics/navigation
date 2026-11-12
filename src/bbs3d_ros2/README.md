# 3D-BBS global localization

This package vendors the CPU implementation of [3D-BBS](https://github.com/KOKIAOKI/3d_bbs) and its ROS 2 wrapper from [bbs3d_ros2](https://github.com/abudori3939/bbs3d_ros2). Upstream revisions are recorded in `UPSTREAM.md`; both MIT license files are retained.

`tool/start_map_navigation.sh` creates a cached binary 0.20 m voxel PCD from the original map during terrain conversion and passes that same map to 3D-BBS and `small_gicp_relocalization`. Voxel centroids stay in the source map frame; the original PCD remains the input to 2D/2.5D navigation map conversion. Direct launch users can still pass any PCD through `pcd_file`.

3D-BBS searches the shared map using `/cloud_registered_body_downsampled`; its map-to-body pose initializes GICP. GICP continues to apply its existing geometry, stability, health, and handoff checks.

The BBS source cloud is already deskewed and gravity calibrated by FAST-LIO, so the integration config disables a second IMU gravity alignment. Automatic searches wait until a source cloud arrives, retry every 1.0 s, and pause only while GICP is `LOCALIZED`. Searches continue during `VERIFYING` so GICP can receive replacement BBS seeds at its configured retry interval.

## Build and launch

Build the workspace using the repository's normal ROS 2 Humble setup. The CPU library builds as a local CMake target and does not need a system install step.

```bash
ros2 launch bringup livox_fastlio_gicp_relocalization.launch.py pcd_file:=/absolute/path/to/map.pcd
```

The `pcd_file` argument accepts one PCD file and is shared by BBS and GICP. CPU searches have a 30-second timeout and retry automatically. The default BBS score threshold is 0.9; performance and threshold suitability depend on map size and sensor coverage, so inspect BBS scores and GICP status on the target robot before relying on automatic handoff.
