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

#include <point_painting/point_painting_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

//ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/camera_info.hpp>


#include "segmentation_msg/msg/segmentation_info.hpp"

namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  tf2_ros::Buffer tf_buffer_;

  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  pointcloud_range = this->declare_parameter<std::vector<double>>("point_cloud_range");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(10ms, [this]() { timer_callback(); }); //ラムダ式　thisは周りの変数を渡す(通常はスコープ範囲は外見れない) 
  segmentation_sub_ = create_subscription<point_painting::msg::SegmentationInfo>(
    "/SegmentationInfo", 1, [this](const  seg_msg) { segmentation_callback(seg_msg); });
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pointcloud",10, [this](const sensor_msgs::msg::PointCloud2 & point){pointcloud_callback(point);});
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info_topic", 10, [this](const sensor_msgs::msg::CameraInfo & camera_info){camera_info_callback(camera_info_);});
}

PointPaintingFusionComponent::~PointPaintingFusionComponent() {}



void PointPaintingFusionComponent::segmentation_callback(
point_painting::msg::SegmentationInfo &  segmentationinfo
)
{
  RCLCPP_INFO(node->get_logger(),"Pub:%d, %d",segmentationinfo->scores);
}

void PointPaintingFusionComponent::camera_info_callback(sensor_msgs::msg::CameraInfo  & camera_info)
{
  RCLCPP_INFO(node->get_logger(),"Pub:%d, %d",camera_info);
}

void PointPaintingFusionComponent::pointcloud_callback(sensor_msgs::msg::PointCloud2 &  pointcloud)
{
  preprocess(pointcloud);
  fuseOnSingleImage(segmentationinfo,camera_info,pointcloud);
}



void PointPaintingFusionComponent::preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp;
  tmp = painted_pointcloud_msg;
  painted_pointcloud_msg.width = tmp.width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg); //Pointcloud2に変更を加えるクラス
  pcd_modifier.clear(); 
  int num_fields = 7;
  //ここで新しいポイントクラウドのフィールドを設定する
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "RED_BUOY ", 1,sensor_msgs::msg::PointField::FLOAT32, 
    "YELLO_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "BLOCK_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "DOCK", 1, sensor_msgs::msg::PointField::FLOAT32
    );
  painted_pointcloud_msg.point_step = num_fields * sizeof(float); //長さが変わるので

  // filter points out of range
  const auto painted_point_step = painted_pointcloud_msg.point_step;
  size_t j = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_painted_x(painted_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_y(painted_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_z(painted_pointcloud_msg, "z");

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(tmp, "x"), iter_y(tmp, "y"),
       iter_z(tmp, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_painted_x, ++iter_painted_y, ++iter_painted_z) {
    if (
      *iter_x <= pointcloud_range.at(0) || *iter_x >= pointcloud_range.at(3) ||
      *iter_y <= pointcloud_range.at(1) || *iter_y >= pointcloud_range.at(4)) {
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      j += painted_point_step;
    }
  }
  painted_pointcloud_msg.data.resize(j);
  painted_pointcloud_msg.width = static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height /painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step =static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
}

  
void PointPaintingFusionComponent::fuseOnSingleImage(
  const SegmentationInfo & SegmentationInfo,
  const sensor_msgs::msg::CameraInfo & camera_info,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg
)
{
  std::vector<point_painting::msg::SegmentationInfo> segmentationInfo; //segmantationメッセージに書き換え
  std::vector<Eigen::Vector2d> debug_image_points; 
  

  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, camera_info.header.frame_id, painted_pointcloud_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  //カメラの外パラを取得　Matrix4dは4✕4の行列を作る関数 
  Eigen::Matrix4d camera_projection;
  camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
    camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
    camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
    camera_info.p.at(11);
  
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);
  }
}



std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
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

RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)
