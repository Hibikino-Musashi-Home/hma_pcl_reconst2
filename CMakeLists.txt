cmake_minimum_required(VERSION 3.8)
project(hma_pcl_reconst2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)

include_directories(include)

# for compopnent
add_library(${PROJECT_NAME} SHARED
  src/pcl_xyzrgb_component.cpp
)

add_executable(pcl_transport_viewer
  src/pcl_transport_viewer.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_features(pcl_transport_viewer PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  point_cloud_transport
  cv_bridge
  image_geometry
  depth_image_proc
)

ament_target_dependencies(pcl_transport_viewer
  rclcpp
  sensor_msgs
  point_cloud_transport
)

# register component
rclcpp_components_register_nodes(${PROJECT_NAME}
  "hma_pcl_reconst2::PointCloudXyzrgb")

install(TARGETS ${PROJECT_NAME} pcl_transport_viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
