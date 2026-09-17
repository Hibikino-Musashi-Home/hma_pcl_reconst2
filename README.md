# hma_pcl_reconst2

A ROS 2 (Jazzy) package that builds an XYZRGB `PointCloud2` from an RGB image, a depth image, and camera info.
Input images can be raw, or `compressed` / `compressedDepth` via `image_transport`.
The output point cloud can be published as a raw `PointCloud2`, or through a compressed `point_cloud_transport` transport.

The package provides:

- `hma_pcl_reconst2::PointCloudXyzrgb`: a composable node that reconstructs the point cloud
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
| `use_sim_time`              | `$USE_SIM_TIME` (`false` if unset)                  | Use the simulation clock and the simulation input topics        |

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
- A 1280x720 organized XYZRGB point cloud is about 29.5 MB per frame when raw.
  To reach 30 Hz, consider lowering the resolution or the number of points as well as compressing the cloud.

## Author

- Ryohei Kobayashi (<kobayashi.ryohei621@mail.kyutech.jp>)

## License

This package is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
