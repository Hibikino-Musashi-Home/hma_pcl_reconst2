# hma_pcl_reconst2

RGB image + depth image + camera infoから、XYZRGBの`PointCloud2`を生成するROS 2パッケージです。
入力画像はrawまたは`image_transport`の`compressed` / `compressedDepth`を使えます。
出力点群はraw `PointCloud2`または`point_cloud_transport`の圧縮transportでpublishできます。

## Install

依存パッケージを用意します。

```bash
sudo apt install \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-compressed-depth-image-transport \
  ros-humble-point-cloud-transport
```

`use_pointcloud_compressed:=true`で`zstd`を使う場合は、`zstd_point_cloud_transport`も環境に入れてください。

```bash
ros2 pkg prefix zstd_point_cloud_transport
```

が通ればOKです。

ビルド:

```bash
cd ~/hma2_ws
colcon build --packages-select hma_pcl_reconst2 --symlink-install
source install/setup.bash
```

## Usage

raw画像入力、raw点群出力:

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info
```

圧縮画像入力:

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info \
  use_compressed:=true
```

圧縮点群出力:

```bash
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  topic_rgb:=/camera/color/image_raw \
  topic_depth:=/camera/aligned_depth_to_color/image_raw \
  topic_camera_info:=/camera/color/camera_info \
  use_compressed:=true \
  use_pointcloud_compressed:=true
```

圧縮点群を解凍してRVizなどで見る:

```bash
ros2 launch hma_pcl_reconst2 pcl_transport_viewer.launch.py
```

デフォルトでは`/hma_pcl_reconst/depth_registered/points/zstd`を購読し、
解凍したraw `PointCloud2`を`/hma_pcl_reconst/depth_registered/points/zstd_decompressed`に再publishします。

## Parameters

`pcl_reconst.launch.py`:

| Name | Default | Description |
| --- | --- | --- |
| `topic_rgb` | `/head_rgbd_sensor/rgb/image_rect_color` | RGB image base topic |
| `topic_depth` | `/head_rgbd_sensor/depth_registered/image_rect_raw` | Depth image base topic |
| `topic_camera_info` | `/head_rgbd_sensor/rgb/camera_info` | Camera info topic |
| `output_topic` | `/hma_pcl_reconst/depth_registered/points` | Output point cloud base topic |
| `use_compressed` | `false` | Subscribe RGB as `compressed` and depth as `compressedDepth` |
| `use_pointcloud_compressed` | `false` | Publish through `point_cloud_transport` |
| `compressed_transport` | `zstd` | Point cloud transport name |
| `queue_size` | `5` | Sync queue size |
| `exact_sync` | `false` | Use exact timestamp sync |

## C++ Decompressor Helper

圧縮された`point_cloud_transport`の点群を、ほかのノードから簡単に購読するためのヘッダを用意しています。

```cpp
#include "hma_pcl_reconst2/pcl_transport_decompressor.hpp"

auto decompressor =
  std::make_unique<hma_pcl_reconst2::PointCloudTransportDecompressor>(
    node,
    "/hma_pcl_reconst/depth_registered/points/zstd",
    "zstd",
    rclcpp::SensorDataQoS(),
    [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      // msg is already decompressed PointCloud2.
    });
```

`input_topic`にはbase topicでも、`/zstd`などのtransport suffix付きtopicでも渡せます。

## Notes

- RGBとdepthは同じ解像度・同じ座標系に揃えたtopicを使ってください。
  RealSenseでは`/camera/aligned_depth_to_color/image_raw`が候補です。
- 1280x720のorganized XYZRGB点群はrawで約29.5MB/frameになります。
  30Hzを狙う場合、圧縮率だけでなく解像度や点数削減も検討してください。
