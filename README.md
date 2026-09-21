# hma_pcl_reconst2

A ROS 2 (Jazzy) package that builds an XYZRGB `PointCloud2` from an RGB image, a depth image, and camera info.
Input images can be raw, or `compressed` / `compressedDepth` via `image_transport`.
The output point cloud can be published as a raw `PointCloud2`, or through a compressed `point_cloud_transport` transport.

The package provides:

- `hma_pcl_reconst2::PointCloudXyzrgb`: a composable node that reconstructs the point cloud
- `hma_pcl_reconst2/depth_denoise.hpp`: an optional depth-image denoiser for active-stereo sensors
- `pcl_transport_viewer`: a node that decompresses a compressed point cloud and republishes it as a raw one
- `hma_pcl_reconst2/pcl_transport_decompressor.hpp`: a header-only helper for subscribing to compressed point clouds from C++

## Install

Install the dependencies:

```bash
sudo apt install \
  ros-jazzy-image-transport \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport \
  ros-jazzy-point-cloud-transport \
  ros-jazzy-zstd-point-cloud-transport
```

To check that the `zstd` transport is available, run:

```bash
ros2 pkg prefix zstd_point_cloud_transport
```

Build:

```bash
cd /hsr_ros2_ws
colcon build --packages-select hma_pcl_reconst2 --symlink-install
source install/setup.bash
```

## Usage

### Raw image input, raw point cloud output

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info
```

### Compressed image input

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info \
  use_compressed:=true
```

With `use_compressed:=true`, the RGB image is subscribed with the `compressed` transport and the depth image with `compressedDepth`. Pass the base topic, without the transport suffix.

### Compressed point cloud output

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info \
  use_compressed:=true \
  use_pointcloud_compressed:=true
```

The compressed cloud is published on `<output_topic>/<compressed_transport>`, which is `/hma_pcl_reconst/depth_registered/points/zstd` by default.

### Denoising an active-stereo depth image

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py denoise:=true
```

Active-stereo sensors such as the RealSense and the Orbbec Gemini produce visibly
noisy planes, unlike ToF or structured-light sensors such as the Xtion. Stereo
triangulates `z = f*b/d`, so the depth error grows with the square of the range:

```
sigma_z(z) = z^2 * sigma_d / (f * b)
```

For a Gemini 336L (`b` ~= 0.095 m, `f` ~= 640 px, `sigma_d` ~= 0.15 px) that is about
2.5 mm at 1 m but 25 cm at 10 m. Every threshold in the denoiser therefore has the
form `base + quad * z^2`, where the quadratic coefficient is the standard deviation
expressed in inverse depth (1/m) — equivalently, a constant sigma in disparity space.
A fixed threshold cannot serve a 0.3-10 m range: tuned for the near field it shreds
distant surfaces, tuned for the far field it destroys near geometry.

With `denoise:=false` (the default) the depth image is passed through untouched, so
ToF sensors are unaffected.

The four stages run in this order, and each can be switched off on its own:

1. **Range clip** — drops depth outside `[min_depth, max_depth]`.
2. **Speckle removal** — builds depth-continuous connected components (8-connected,
   with the depth-dependent tolerance above) and deletes those below
   `speckle.max_size` pixels. This is what removes flying pixels.
3. **Bilateral smoothing** — NaN-aware, so holes are never filled in and never bleed
   into their surroundings. This is the main fix for noisy planes.
4. **Temporal EMA** — stereo noise is nearly uncorrelated between frames, so this is
   the strongest lever at long range. A depth-dependent gate keeps moving objects
   from ghosting.

Measured on synthetic Gemini-336L-like data (plane RMS, 5x5 window, defaults):

| Scene | Before | After |
| --- | --- | --- |
| Plane at 1 m + 3 % speckle | 85.7 mm | 0.7 mm |
| Plane at 5 m + 3 % speckle | 105.7 mm | 24.5 mm |
| Plane at 10 m | 259.6 mm | 127.6 mm |
| Plane at 10 m, 10 frames of EMA | 246.2 mm | 58.7 mm |
| Plane at 10 m with 40 % dropout | 246.4 mm | 117.0 mm (0.1 pt of pixels lost) |

A 2.00 m / 2.50 m step edge stays sharp, with no pixels pulled across it.

Cost, all four stages, on a 20-core machine: about 15 ms per frame at 1280x720,
7 ms at 848x480 and 5 ms at 640x480.

### Simulation (`use_sim_time`)

`pcl_reconst.launch.py` picks its default input topics based on `use_sim_time`.

- The `use_sim_time` argument defaults to the `USE_SIM_TIME` environment variable. The values `true`, `1`, `yes`, and `on` enable it.
- With `use_sim_time:=true`, the node subscribes to the simulator topics (`hsrb_gazebo_bringup`). Otherwise, it subscribes to the real robot topics.
- Only the input topics change. To subscribe to compressed images, you still have to set `use_compressed:=true`.

```bash
USE_SIM_TIME=true ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py
# or
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py use_sim_time:=true
```

| `use_sim_time` | `topic_rgb`                              | `topic_depth`                                       | `topic_camera_info`                 |
| -------------- | ---------------------------------------- | --------------------------------------------------- | ----------------------------------- |
| `true`         | `/head_rgbd_sensor/rgb/image_rect_raw`   | `/head_rgbd_sensor/depth_registered/image_rect_raw` | `/head_rgbd_sensor/rgb/camera_info` |
| `false`        | `/head_rgbd_sensor/rgb/image_rect_color` | `/head_rgbd_sensor/depth_registered/image_rect_raw` | `/head_rgbd_sensor/rgb/camera_info` |

You can still override any of these on the command line, for example with `topic_rgb:=...`.

### Viewing a compressed point cloud

To decompress the point cloud and view it in RViz or another tool:

```bash
ros2 launch hma_pcl_reconst2 pcl_transport_viewer.launch.py
```

By default, the viewer subscribes to `/hma_pcl_reconst/depth_registered/points/zstd`
and republishes the decompressed raw `PointCloud2` on `/hma_pcl_reconst/depth_registered/points/zstd_decompressed`.
The viewer also accepts a `use_sim_time` argument, which defaults to the `USE_SIM_TIME` environment variable. It does not change the viewer's topics.

## Parameters

### `pcl_reconst.launch.py`

| Name                        | Default                                             | Description                                                     |
| --------------------------- | --------------------------------------------------- | --------------------------------------------------------------- |
| `topic_rgb`                 | `/head_rgbd_sensor/rgb/image_rect_color`            | RGB image base topic                                            |
| `topic_depth`               | `/head_rgbd_sensor/depth_registered/image_rect_raw` | Depth image base topic                                          |
| `topic_camera_info`         | `/head_rgbd_sensor/rgb/camera_info`                 | Camera info topic                                               |
| `output_topic`              | `/hma_pcl_reconst/depth_registered/points`          | Output point cloud base topic                                   |
| `use_compressed`            | `false`                                             | Subscribe to RGB as `compressed` and depth as `compressedDepth` |
| `use_pointcloud_compressed` | `false`                                             | Publish through `point_cloud_transport`                         |
| `compressed_transport`      | `zstd`                                              | Point cloud transport name                                      |
| `queue_size`                | `5`                                                 | Sync queue size                                                 |
| `exact_sync`                | `false`                                             | Use exact timestamp sync instead of approximate sync            |
| `denoise`                   | `false`                                             | Denoise the depth image before reprojecting it                  |
| `denoise_clip_min_depth`    | node default (`0.3`)                                | [m] Discard depth below this                                    |
| `denoise_clip_max_depth`    | node default (`10.0`)                               | [m] Discard depth above this                                    |
| `denoise_speckle_max_size`  | node default (`200`)                                | [px] Drop connected components smaller than this                |
| `denoise_speckle_diff_quad` | node default (`0.011`)                              | [1/m] z² term of the speckle connectivity tolerance             |
| `denoise_bilateral_radius`  | node default (`2`)                                  | Bilateral window radius; `2` is 5x5, `0` disables the stage     |
| `denoise_bilateral_sigma_quad` | node default (`0.004`)                           | [1/m] z² term of the bilateral range sigma                      |
| `denoise_temporal_alpha`    | node default (`0.5`)                                | Weight of the new frame in the temporal EMA; `0` disables it    |
| `use_sim_time`              | `$USE_SIM_TIME` (`false` if unset)                  | Use the simulation clock and the simulation input topics        |

The `denoise_*` tuning arguments are left out of the node's parameters when unset, so
the node's own defaults apply. Every denoise parameter is also settable directly, under
the dotted names `denoise.enable`, `denoise.clip.*`, `denoise.speckle.*`,
`denoise.bilateral.*` and `denoise.temporal.*` — see `DepthDenoiser::declareParameters`
in `src/depth_denoise.cpp` for the full list, including the per-stage `enable` flags.

The defaults of `topic_rgb`, `topic_depth`, and `topic_camera_info` depend on `use_sim_time` (see [Simulation](#simulation-use_sim_time)).
The table shows the values for the real robot (`use_sim_time:=false`).

### `pcl_transport_viewer.launch.py`

| Name               | Default                                                      | Description                                            |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------------------ |
| `input_topic`      | `/hma_pcl_reconst/depth_registered/points/zstd`              | Compressed transport topic, or the base topic          |
| `output_topic`     | `/hma_pcl_reconst/depth_registered/points/zstd_decompressed` | Raw `PointCloud2` topic for the decompressed clouds    |
| `transport`        | `zstd`                                                       | `point_cloud_transport` subscriber transport           |
| `republish`        | `true`                                                       | Republish the decompressed clouds as raw `PointCloud2` |
| `log_interval_sec` | `1.0`                                                        | Interval between terminal status logs, in seconds      |
| `use_sim_time`     | `$USE_SIM_TIME` (`false` if unset)                           | Use the simulation clock                               |

## C++ Decompressor Helper

`pcl_transport_decompressor.hpp` lets other nodes subscribe to a compressed `point_cloud_transport` point cloud with a few lines of code:

```cpp
#include "hma_pcl_reconst2/pcl_transport_decompressor.hpp"

auto decompressor =
  std::make_unique<hma_pcl_reconst2::PointCloudTransportDecompressor>(
    node,
    "/hma_pcl_reconst/depth_registered/points/zstd",
    "zstd",
    rclcpp::SensorDataQoS(),
    [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      // msg is already a decompressed PointCloud2.
    });
```

`input_topic` accepts either the base topic or a topic with a transport suffix such as `/zstd`.

## Notes

- The RGB and depth topics must have the same resolution and the same frame, that is, the depth image must be registered to the RGB image.
  On a RealSense camera, `/camera/aligned_depth_to_color/image_raw` is a suitable depth topic.
- `denoise:=true` is for active-stereo sensors. ToF and structured-light sensors such
  as the Xtion do not need it; leave it at the default.
- Set `denoise_clip_max_depth` comfortably beyond the furthest surface you care about.
  A surface sitting exactly on the limit loses roughly half its pixels to the clip,
  because its noise straddles the threshold.
- If distant surfaces disappear when denoising is on, the speckle tolerance is too
  tight for the noise at that range: raise `denoise_speckle_diff_quad`, or lower
  `denoise_speckle_max_size`.
- 16UC1 depth input is assumed to be 1 count = 1 mm (see `depth_traits.hpp`). Orbbec
  drivers can be configured with a different depth unit; at a 0.1 mm unit a 10 m
  reading overflows uint16 and the scale is wrong regardless of denoising.
- A 1280x720 organized XYZRGB point cloud is about 29.5 MB per frame when raw.
  To reach 30 Hz, consider lowering the resolution or the number of points as well as compressing the cloud.

## Author

- Ryohei Kobayashi (<kobayashi.ryohei621@mail.kyutech.jp>)

## License

This package is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
