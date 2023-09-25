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

#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <point_painting/point_painting_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

//ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "detic_onnx_ros2_msg/msg/segmentation_info.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

//opencv
#include <sensor_msgs/image_encodings.hpp>
#ifdef ROS_DISTRO_ROLLING
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options), buffer_(get_clock()), listener_(buffer_)
{
  //param
  declare_parameter<std::vector<double>>("point_cloud_range");
  declare_parameter("segmentation_topic", "/SegmentationInfo");
  declare_parameter("camera_info_topic", "/CameraInfo");
  declare_parameter("point_cloud_topic", "/point_cloud");
  declare_parameter("debug", false);

  pointcloud_range_ = get_parameter("point_cloud_range").as_double_array();
  const auto segmentation_topic = get_parameter("segmentation_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto point_cloud_topic = get_parameter("point_cloud_topic").as_string();
  debug_ = get_parameter("debug").as_bool();

  //piblisher
  using namespace std::chrono_literals;
  point_painting_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_painting", 10);
  if (debug_) {
    preprocess_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("preprocess_debug", 10);
  }
  //subscriber
  segmentation_sub_ = create_subscription<detic_onnx_ros2_msg::msg::SegmentationInfo>(
    segmentation_topic, 1, [this](const detic_onnx_ros2_msg::msg::SegmentationInfo seg_msg) {
      segmentation_callback(seg_msg);
    });
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo & camera_info_msg) {
      camera_info_callback(camera_info_msg);
    });
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic, 10,
    [this](const sensor_msgs::msg::PointCloud2 & point) { pointcloud_callback(point); });
}

void PointPaintingFusionComponent::segmentation_callback(
  const detic_onnx_ros2_msg::msg::SegmentationInfo & seg_msg)
{
  segmentationinfo_ = seg_msg;
}
void PointPaintingFusionComponent::camera_info_callback(
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  camera_info_ = camera_info_msg;
}
void PointPaintingFusionComponent::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 pointcloud = pointcloud_msg;
  preprocess(pointcloud);
  fuseOnSingleImage(segmentationinfo_,pointcloud,camera_info_);
}

void PointPaintingFusionComponent::preprocess(
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp_point;
  tmp_point = painted_pointcloud_msg;
  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
  pcd_modifier.clear();
  pcd_modifier.reserve(tmp_point.width);
  painted_pointcloud_msg.width = tmp_point.width;
  painted_pointcloud_msg.height = tmp_point.height;
  int num_fields = 6;

  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "y", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "class", 1,sensor_msgs::msg::PointField::FLOAT32, 
    "scores", 1, sensor_msgs::msg::PointField::FLOAT32);

  painted_pointcloud_msg.point_step = num_fields * sizeof(float);
  const auto painted_point_step = painted_pointcloud_msg.point_step;
  size_t j = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_painted_x(painted_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_y(painted_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_z(painted_pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_intensity(painted_pointcloud_msg, "intensity");

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(tmp_point, "x"), iter_y(tmp_point, "y"),
       iter_z(tmp_point, "z"), iter_intensity(tmp_point, "intensity");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_painted_x,
                               ++iter_painted_y, ++iter_painted_z, ++iter_painted_intensity) {
    if (
      *iter_x <= pointcloud_range_.at(0) || *iter_x >= pointcloud_range_.at(3) ||
      *iter_y <= pointcloud_range_.at(1) || *iter_y >= pointcloud_range_.at(4)) {
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      *iter_painted_intensity = *iter_intensity;
      j += painted_point_step;
    }
  }
  painted_pointcloud_msg.data.resize(j);
  painted_pointcloud_msg.width = static_cast<uint32_t>(
    painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height /
    painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step =
    static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
}

void PointPaintingFusionComponent::fuseOnSingleImage(
  const detic_onnx_ros2_msg::msg::SegmentationInfo & SegmentationInfo,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat seg_img;
  try {
    cv_ptr = cv_bridge::toCvCopy(SegmentationInfo.segmentation,sensor_msgs::image_encodings::MONO8);
    seg_img = cv_ptr->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  uint32_t width = seg_img.cols;
  uint32_t height = seg_img.rows;

  geometry_msgs::msg::TransformStamped transform_stamped;
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  try {
    transform_stamped = buffer_.lookupTransform(
      camera_info.header.frame_id, painted_pointcloud_msg.header.frame_id, camera_info.header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    //点群::LiDAR座標系⇨カメラ行列
    tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
    return;
  }

  Eigen::Matrix4d camera_projection;  //Homogeneous Coordinates
  camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
    camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
    camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
    camera_info.p.at(11), 0, 0, 1, 0;

  sensor_msgs::PointCloud2Iterator<float> iter_class(transformed_pointcloud, "class");
  sensor_msgs::PointCloud2Iterator<float> iter_scores(transformed_pointcloud, "scores");

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
       iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_class, ++iter_scores) {
    //Camera Coordinates⇨ Image Coordinates
    Eigen::Vector4d projected_point =
      camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
    if(projected_point.z() < 0.0){
      continue;
    }
    int img_point_x = int(normalized_projected_point.x());
    int img_point_y = int(normalized_projected_point.y());
        

    if (0 <= img_point_x && img_point_x <= int(width) && 0 <= img_point_y &&
      img_point_y <= int(height)) 
    { 
      int mask_idx = static_cast<int>(seg_img.at<unsigned char>(img_point_y,img_point_x));
      std::vector<std::string> task_obj = {"airplane","sail"}; 
      std::vector<std::string> obst_obj = {"ball","cone","beachball"}; 
      if (!SegmentationInfo.detected_classes.empty()){
        if (mask_idx < int(sizeof(SegmentationInfo.detected_classes) / sizeof(int))){
          std::string seg_class = SegmentationInfo.detected_classes[mask_idx];
          for (auto &obj : task_obj) {
              if (seg_class == obj) {
                  *iter_class = 50.0;
                  break;
              }
          }
          for (auto &obj : obst_obj) {
              if (seg_class == obj) {
                  *iter_class =150.0;
                  break;
              }
          }
          if(seg_class == "background"){
            *iter_class = 300.0;     
          }
        }
      }
    }
    Eigen::Vector4d reprojection_cloud;
    reprojection_cloud << *iter_x, *iter_y, *iter_z, 1.0;
    reprojection_cloud = camera_projection.reverse() * projected_point;
  }
              
  sensor_msgs::msg::PointCloud2 paint_pointcloud;
  geometry_msgs::msg::TransformStamped reverse_transform_stamped;
  try {
  reverse_transform_stamped = buffer_.lookupTransform(
    painted_pointcloud_msg.header.frame_id, camera_info.header.frame_id, camera_info.header.stamp,
    rclcpp::Duration::from_seconds(0.1));
  //点群::LiDAR座標系⇨カメラ行列
  tf2::doTransform(transformed_pointcloud, paint_pointcloud , reverse_transform_stamped);
  } catch (tf2::TransformException & ex) {
  RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
  return;
  }
  if(debug_){
    preprocess_debug_pub_->publish(transformed_pointcloud);
  }
  point_painting_pub_->publish(paint_pointcloud);
}
}  // namespace point_painting

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)