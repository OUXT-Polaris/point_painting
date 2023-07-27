// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <cstdint>

#include <point_painting/point_painting_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

//ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "segmentation_msg/msg/segmentation_info.hpp"


namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options), tfBuffer(get_clock()) 
{  
  //param
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  pointcloud_range_ = this->declare_parameter<std::vector<double>>("point_cloud_range");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");

  using namespace std::chrono_literals;
  //timer_ = create_wall_timer(10ms, [this]() { timer_callback(); });  
  segmentation_sub_ = create_subscription<segmentation_msg::msg::SegmentationInfo>(
    "/SegmentationInfo", 1, [this](const segmentation_msg::msg::SegmentationInfo seg_msg) { segmentation_callback(seg_msg); });
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud",10, [this](const sensor_msgs::msg::PointCloud2 & point){pointcloud_callback(point);});
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info_topic", 10, [this](const sensor_msgs::msg::CameraInfo & camera_info_msg){camera_info_callback(camera_info_msg);});
}

void PointPaintingFusionComponent::segmentation_callback(const segmentation_msg::msg::SegmentationInfo & seg_msg)
{
  segmentationinfo_ = seg_msg;
} 
void PointPaintingFusionComponent::camera_info_callback(const sensor_msgs::msg::CameraInfo  & camera_info_msg)
{
  camera_info_ = camera_info_msg;
}
void PointPaintingFusionComponent::pointcloud_callback(const sensor_msgs::msg::PointCloud2 &  pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 pointcloud = pointcloud_msg;
  preprocess(pointcloud);
  fuseOnSingleImage(segmentationinfo_,pointcloud,camera_info_);
}

//カメラの視野内に点群を制限
void PointPaintingFusionComponent::preprocess(sensor_msgs::msg::PointCloud2 & sensor_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp;
  tmp = sensor_pointcloud_msg;
  sensor_pointcloud_msg.width = tmp.width;
  sensor_msgs::PointCloud2Modifier pcd_modifier(sensor_pointcloud_msg); 
  pcd_modifier.clear(); 
  int num_fields = 7;
  //ここで新しいポイントクラウドのフィールドを設定する
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "RED_BUOY ", 1,sensor_msgs::msg::PointField::FLOAT32, 
    "YELLOW_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "BLACK_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "DOCK", 1, sensor_msgs::msg::PointField::FLOAT32
    );
  sensor_pointcloud_msg.point_step = num_fields * sizeof(float);
  const auto painted_point_step = sensor_pointcloud_msg.point_step;
  size_t j = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_painted_x(sensor_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_y(sensor_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_z(sensor_pointcloud_msg, "z");
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(tmp, "x"), iter_y(tmp, "y"),
       iter_z(tmp, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_painted_x, ++iter_painted_y, ++iter_painted_z) {
    if (
      *iter_x <= pointcloud_range_.at(0) || *iter_x >= pointcloud_range_.at(3) ||
      *iter_y <= pointcloud_range_.at(1) || *iter_y >= pointcloud_range_.at(4)) {
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      j += painted_point_step;
    }
  }
  sensor_pointcloud_msg.data.resize(j);
  sensor_pointcloud_msg.width = static_cast<uint32_t>(sensor_pointcloud_msg.data.size() / sensor_pointcloud_msg.height /sensor_pointcloud_msg.point_step);
  sensor_pointcloud_msg.row_step =static_cast<uint32_t>(sensor_pointcloud_msg.data.size() / sensor_pointcloud_msg.height);
}


void PointPaintingFusionComponent::fuseOnSingleImage(
  const segmentation_msg::msg::SegmentationInfo & SegmentationInfo,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
  const sensor_msgs::msg::CameraInfo & camera_info
)
{
  uint32_t width = SegmentationInfo.segmentation.width;
  uint32_t height = SegmentationInfo.segmentation.height;

  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tfBuffer, camera_info.header.frame_id,painted_pointcloud_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  Eigen::Matrix4d camera_projection;
  camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
    camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
    camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
    camera_info.p.at(11);
  
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  //点群::LiDAR座標系⇨カメラ行列
  tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);
  std::vector<std::string> seg_map = SegmentationInfo.detected_classes;


  sensor_msgs::PointCloud2Iterator<float> iter_red_buoy(painted_pointcloud_msg, "RED_BUOY");
  sensor_msgs::PointCloud2Iterator<float> iter_yellow_buoy(painted_pointcloud_msg, "YELLOW_BUOY");
  sensor_msgs::PointCloud2Iterator<float> iter_black_buoy(painted_pointcloud_msg, "BLACK_BUOY");
  sensor_msgs::PointCloud2Iterator<float> iter_dock(painted_pointcloud_msg, "DOCK");

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
       iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, 
       ++iter_red_buoy, ++iter_yellow_buoy, ++iter_black_buoy, ++iter_dock) {
    //カメラ座標⇨画像座標系
    Eigen::Vector4d projected_point = camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
    
    int target_row = int(normalized_projected_point.y()) ;
    int target_col = int(normalized_projected_point.x()) ; 
    const size_t target_index = target_row * width + target_col ;
    if (
      target_index <= 0 || target_index >= width*height
     ) {
      continue;
    } else {
      std::string class_name = seg_map[target_index] ;
      if (class_name == "RED_BUOY") {
        *iter_red_buoy = 1.0 ; 
      } else if (class_name == "YELLOW_BUOY") {
        *iter_yellow_buoy = 1.0 ;
      } else if (class_name == "BLACK_BUOY") {
        *iter_black_buoy = 1.0 ;
      } else if (class_name == "DOCK") {
        *iter_dock = 1.0 ;
      } else {
      }
    }
  }
}


std::optional<geometry_msgs::msg::TransformStamped> PointPaintingFusionComponent::getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time)
  {
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return transform_stamped;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
    return std::nullopt;
  }
}
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)
