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
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

//ros2
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/assign.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "detic_onnx_ros2_msg/msg/segmentation_info.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace point_painting
{
class PointPaintingFusionComponent : public rclcpp::Node
{
public:
  POINTPAINTING_FUSIONCOMPONENT_PUBLIC
  explicit PointPaintingFusionComponent(const rclcpp::NodeOptions & options);

private:
  std::vector<std::string> task_obj;
  std::vector<std::string> obst_obj;
  bool debug;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::vector<std::string> class_names_;
  std::vector<double> pointcloud_range_;

  void preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg);
  void fuseOnSingleImage(
    const detic_onnx_ros2_msg::msg::SegmentationInfo & SegmentationInfo,
    sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
    const sensor_msgs::msg::CameraInfo & camera_info);

  message_filters::Subscriber<detic_onnx_ros2_msg::msg::SegmentationInfo> segmentation_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    detic_onnx_ros2_msg::msg::SegmentationInfo, sensor_msgs::msg::PointCloud2>
    approximate_policy;
  message_filters::Synchronizer<approximate_policy> sync_;

  void topic_callback(
    const detic_onnx_ros2_msg::msg::SegmentationInfo & seg_msg,
    const sensor_msgs::msg::PointCloud2 & pointcloud_msg);

  //timer
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  //ros2 message
  detic_onnx_ros2_msg::msg::SegmentationInfo segmentationinfo_;
  sensor_msgs::msg::CameraInfo camera_info_;
  //Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_painting_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preprocess_debug_pub_;

  //Subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  //callback
  void camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info);
};
}  // namespace point_painting
#endif
