# hma_pcl_reconst2

ROS 2 (Jazzy) package that builds an organized XYZRGB `PointCloud2` from a registered RGB image,
a depth image, and camera info. It can optionally denoise active-stereo depth first.

- `hma_pcl_reconst2::PointCloudXyzrgb`: composable node (raw or `compressed`/`compressedDepth` input,
  raw or `point_cloud_transport` output)
- `hma_pcl_reconst2/depth_denoise.hpp`: depth denoiser for active-stereo sensors
- `pcl_transport_viewer`: decompresses a compressed cloud and republishes it raw
- `hma_pcl_reconst2/pcl_transport_decompressor.hpp`: header-only helper for subscribing to compressed clouds

## Build

```bash
sudo apt install ros-jazzy-{image-transport,compressed-image-transport,compressed-depth-image-transport,point-cloud-transport,zstd-point-cloud-transport}
colcon build --packages-select hma_pcl_reconst2 --symlink-install
```

## Usage

```bash
# HSR (real-robot topics; use_sim_time:=true or USE_SIM_TIME=true for the simulator)
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py

# other cameras
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info
  # + use_compressed:=true             subscribe compressed / compressedDepth (pass the base topic)
  # + use_pointcloud_compressed:=true  publish on <output_topic>/zstd
  # + denoise:=true                    for active-stereo sensors (RealSense, Orbbec Gemini)

# Orbbec Gemini 336: OrbbecSDK_ROS2 (v2-main) driver + pcl_reconst with denoise:=true
ros2 launch hma_pcl_reconst2 gemini336.launch.py rviz:=true

# decompress <output_topic>/zstd for RViz (republished on .../zstd_decompressed)
ros2 launch hma_pcl_reconst2 pcl_transport_viewer.launch.py
```

`use_sim_time` switches the default RGB input between `/head_rgbd_sensor/rgb/image_rect_raw` (simulator)
and `/head_rgbd_sensor/rgb/image_rect_color` (real robot). Depth is `/head_rgbd_sensor/depth_registered/image_rect_raw`
in both. Explicit `topic_*` arguments always win.

`gemini336.launch.py` registers depth to color and turns the camera's own post filters off. Its resolution defaults
to 1280x720 (change it with `color_width:=848` and the other size arguments). It also accepts every `denoise_*` argument below.

## Denoising (`denoise:=true`)

Stereo depth error grows with the square of the range (`sigma_z = z^2 * sigma_d / (f * b)`),
so every threshold is `base + quad * z^2`. With `denoise:=false` the depth passes through untouched,
which is the right choice for ToF sensors such as the Xtion. The stages run in this order:

1. **Clip**: drops depth outside `[min_depth, max_depth]`.
2. **Speckle**: drops small depth-continuous components (flying pixels).
3. **Bilateral**: NaN-aware edge-preserving smoothing. Removes the pixel-level grain.
4. **Temporal**: EMA with a motion gate.
5. **Plane snap**: RANSAC finds the dominant planes. Pixels within `tol(z)` of a plane are pulled onto it
   along their ray, with a soft blend. This removes the low-frequency waviness that local filters cannot.
   Anything thinner than `tol` lying on a plane gets flattened.

Plane RMS on a live Gemini 336 at 1280x720:

| | floor 0.8 m | wall 1.9 m |
| --- | --- | --- |
| raw | 1.75 mm | 16.1 mm |
| stages 1-4 | 1.55 mm | 14.2 mm |
| + snap, `diff_quad` 0.008 (default) | 1.13 mm | 10.0 mm |
| + snap, `diff_quad` 0.012 | 0.97 mm | 5.7 mm |

Cost on a 20-core PC: denoise about 14 ms (bilateral 7, snap 4), reprojection 5 ms, publish 11 ms.
That is about 30 Hz, with about 110 ms from the sensor stamp to publish.

Every `denoise.*` node parameter can be changed at runtime (`ros2 param set /pcl_reconst ...` or `rqt_reconfigure`).
The full list is in `DepthDenoiser::declareParameters` (`src/depth_denoise.cpp`).

## Parameters (`pcl_reconst.launch.py`)

| Name | Default | Description |
| --- | --- | --- |
| `topic_rgb` / `topic_depth` / `topic_camera_info` | HSR topics (see above) | Inputs. RGB and depth must be registered |
| `output_topic` | `/hma_pcl_reconst/depth_registered/points` | Output base topic |
| `use_compressed` | `false` | Subscribe `compressed` / `compressedDepth` |
| `use_pointcloud_compressed` / `compressed_transport` | `false` / `zstd` | Publish through `point_cloud_transport` |
| `queue_size` / `exact_sync` | `5` / `false` | RGB-depth synchronizer |
| `use_sim_time` | `$USE_SIM_TIME` | Simulation clock and simulator topics |
| `denoise` | `false` | Enable the denoiser |
| `denoise_bilateral_radius` / `denoise_bilateral_sigma_quad` | `2` / `0.004` | Bilateral window radius and range-sigma z² term [1/m] |
| `denoise_snap` / `denoise_snap_diff_quad` | `true` / `0.008` | Plane snap and its tolerance z² term [1/m] (about 9 mm at 0.8 m) |
| `denoise_num_threads` | `0` | OpenCV thread count (process-global); `0` leaves it alone |
| `denoise_clip_{min,max}_depth`, `denoise_speckle_{max_size,diff_quad}`, `denoise_temporal_alpha` | node default | Passed only when set |

Node-only parameter: `input_queue_depth` (default `1`: always process the newest frame, for the lowest latency).

`pcl_transport_viewer.launch.py` takes `input_topic`, `output_topic`, `transport` (`zstd`), `republish` (`true`),
`log_interval_sec` (`1.0`), and `use_sim_time`.

## C++ decompressor helper

```cpp
#include "hma_pcl_reconst2/pcl_transport_decompressor.hpp"

auto decompressor = std::make_unique<hma_pcl_reconst2::PointCloudTransportDecompressor>(
  node, "/hma_pcl_reconst/depth_registered/points/zstd", "zstd", rclcpp::SensorDataQoS(),
  [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { /* already decompressed */ });
```

## Notes

- The output point is packed into 16 bytes (`x`, `y`, `z`, `rgb` as FLOAT32). A raw 1280x720 cloud is about 14.7 MB.
- 16UC1 depth is read as 1 count = 1 mm.
- If distant surfaces vanish with denoising on, raise `denoise_speckle_diff_quad`. Keep `denoise_clip_max_depth`
  well beyond the furthest surface you care about.
- Gemini 336 with old firmware: if the driver fails with `propertyId: 2052 status:1005`, update the firmware
  to 1.8.10 or guard the auto-exposure-priority readback in `ob_camera_node.cpp`.

## Author / License

Ryohei Kobayashi (<kobayashi.ryohei621@mail.kyutech.jp>), [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
