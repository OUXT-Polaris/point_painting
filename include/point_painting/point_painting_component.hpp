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
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <perception_msgs/msg/tracking2_d.hpp> // Tracking2D in pub_

namespace point_painting
{
class PointPaintingFusionComponent : public rclcpp::Node
{
public:
  POINTPAINTING_FUSIONCOMPONENT_PUBLIC
  explicit PointPaintingFusionComponent(const rclcpp::NodeOptions & options);
  virtual ~PointPaintingFusionComponent();

private:
  void preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg);
  void fuseOnSingleImage(
  //const SegmentationInfo & SegmentationInfo,
  // const sensor_msgs::msg::CameraInfo & camera_info,
  // sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg
  );
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};
}  // namespace point_segmentation_fusion 
#endif 