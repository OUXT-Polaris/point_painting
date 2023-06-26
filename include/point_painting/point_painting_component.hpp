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

#ifndef POINTPAINTING_FUSIONCOMPONENT_
#define POINTPAINTING_FUSIONCOMPONENT_


#pragma once

#include <point_painting/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>  
#include <optional>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "segmentation_msg/msg/segmentation_info.hpp"


namespace point_painting
{
class PointPaintingFusionComponent : public rclcpp::Node
{
public:
  POINTPAINTING_FUSIONCOMPONENT_PUBLIC
  explicit PointPaintingFusionComponent(const rclcpp::NodeOptions & options);
 
private:
  tf2_ros::Buffer tfBuffer;

  segmentation_msg::msg::SegmentationInfo segmentationinfo_; 
  sensor_msgs::msg::CameraInfo  camera_info_;

  void preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg);
  void fuseOnSingleImage(
  const segmentation_msg::msg::SegmentationInfo & SegmentationInfo,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
  const sensor_msgs::msg::CameraInfo & camera_info
  );
  
  std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time);

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
 
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<segmentation_msg::msg::SegmentationInfo>::SharedPtr segmentation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  std::vector<std::string> class_names_;
  std::vector<double> pointcloud_range;

  void segmentation_callback(const segmentation_msg::msg::SegmentationInfo &  segmentationinfo);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2 &  pointcloud);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo  & camera_info);
};
}  
#endif 