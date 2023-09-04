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

//opencv
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options), buffer_(get_clock()),listener_(buffer_)
{  
  //param
  declare_parameter<std::vector<std::string>>("class_names");
  declare_parameter<std::vector<double>>("point_cloud_range");
  declare_parameter<std::vector<double>>("min_area_matrix");
  declare_parameter<std::vector<double>>("max_area_matrix");
  declare_parameter("segmentation_topic","/SegmentationInfo");
  declare_parameter("camera_info_topic","/CameraInfo");
  declare_parameter("point_cloud_topic","/point_cloud");
  declare_parameter("debug",false);

  debug = get_parameter("debug").as_bool();
  class_names_ = get_parameter("class_names").as_string_array();
  pointcloud_range_ = get_parameter("point_cloud_range").as_double_array();
  const auto min_area_matrix = get_parameter("min_area_matrix").as_double_array();
  const auto max_area_matrix = get_parameter("max_area_matrix").as_double_array();
  const auto segmentation_topic = get_parameter("segmentation_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto point_cloud_topic = get_parameter("point_cloud_topic").as_string();

  //piblisher
  using namespace std::chrono_literals;
  point_painting_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_painting", 10);
  if (debug){
    preprocess_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("preprocess_debug", 10);
  }
  //subscriber
  segmentation_sub_ = create_subscription<segmentation_msg::msg::SegmentationInfo>(
    segmentation_topic, 1, [this](const segmentation_msg::msg::SegmentationInfo seg_msg) { segmentation_callback(seg_msg); });
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo & camera_info_msg){camera_info_callback(camera_info_msg);});
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic,10, [this](const sensor_msgs::msg::PointCloud2 & point){pointcloud_callback(point);});
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
void PointPaintingFusionComponent::preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp;
  tmp = painted_pointcloud_msg;
  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
  pcd_modifier.clear();
  pcd_modifier.reserve(tmp.width);
  painted_pointcloud_msg.width = tmp.width;
  painted_pointcloud_msg.height = tmp.height;
  constexpr int num_fields = 7;
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, 
    "RED_BUOY ", 1,sensor_msgs::msg::PointField::FLOAT32, 
    "YELLOW_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "BLACK_BUOY", 1, sensor_msgs::msg::PointField::FLOAT32,
    "DOCK", 1, sensor_msgs::msg::PointField::FLOAT32
    );
  painted_pointcloud_msg.point_step = num_fields * sizeof(float);
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
      *iter_x <= pointcloud_range_.at(0) || *iter_x >= pointcloud_range_.at(3) ||
      *iter_y <= pointcloud_range_.at(1) || *iter_y >= pointcloud_range_.at(4)) {
  // 点群の行数を再定義
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      j += painted_point_step;
    }
  }
  painted_pointcloud_msg.data.resize(j); //バッファのサイズを最適化
  painted_pointcloud_msg.width = static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height / painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step = static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
  if (debug){
    preprocess_debug_pub_->publish(painted_pointcloud_msg);
  }
}

void PointPaintingFusionComponent::fuseOnSingleImage(
  const segmentation_msg::msg::SegmentationInfo & SegmentationInfo,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
  const sensor_msgs::msg::CameraInfo & camera_info
)
{ 
  uint32_t width = SegmentationInfo.segmentation.width;
  uint32_t height = SegmentationInfo.segmentation.height;
  // cv_bridge::CvImagePtr cv_ptr;
  // cv::Mat seg_map ;
  // try
  // {  
  // cv_ptr = cv_bridge::toCvCopy(SegmentationInfo.segmentation,sensor_msgs::image_encodings::MONO8);
  // seg_map = cv_ptr->image;
  // if (debug){
  //   cv::imshow("Received Image", seg_map);
  //   cv::waitKey(1);
  //   uchar pixel_value = seg_map.at<uchar>(2,1);
  //   RCLCPP_INFO(this->get_logger(), "Pixel value at %d",pixel_value);
  // }
  // }
  // catch (cv_bridge::Exception& e)
  // {
  //     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  //     return ;
  // }
  

  // geometry_msgs::msg::TransformStamped transform_stamped;
  // {
  //   const auto transform_stamped_optional = getTransformStamped(
  //     tfBuffer, camera_info.header.frame_id,painted_pointcloud_msg.header.frame_id,camera_info.header.stamp);
  //   if (!transform_stamped_optional) {
  //     return;
  //   }
  //   transform_stamped = transform_stamped_optional.value();    
  // }
  geometry_msgs::msg::TransformStamped transform_stamped;
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  try {  
    transform_stamped = buffer_.lookupTransform(camera_info.header.frame_id,painted_pointcloud_msg.header.frame_id,camera_info.header.stamp, rclcpp::Duration::from_seconds(0.5));
    //点群::LiDAR座標系⇨カメラ行列
    tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);
    point_painting_pub_->publish(transformed_pointcloud);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
    // return std::nullopt;
  }




  // Eigen::Matrix4d camera_projection;
  // camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
  //   camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
  //   camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
  //   camera_info.p.at(11);
  
  // sensor_msgs::PointCloud2Iterator<float> iter_red_buoy(painted_pointcloud_msg, "");
  // sensor_msgs::PointCloud2Iterator<float> iter_yellow_buoy(painted_pointcloud_msg, "YELLOW_BUOY");
  // sensor_msgs::PointCloud2Iterator<float> iter_black_buoy(painted_pointcloud_msg, "BLACK_BUOY");
  // sensor_msgs::PointCloud2Iterator<float> iter_dock(painted_pointcloud_msg, "DOCK");

  // for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
  //      iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
  //      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, 
  //      ++iter_red_buoy, ++iter_yellow_buoy, ++iter_black_buoy, ++iter_dock) {
  //   //カメラ座標⇨画像座標系
  //   Eigen::Vector4d projected_point = camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
  //   Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
    
  //   int target_row = int(normalized_projected_point.y()) ;
  //   int target_col = int(normalized_projected_point.x()) ; 
  //   const size_t target_index = target_row * width + target_col ;
  //   //*iter_red_buoy = 1.0 ; 
  //   if (
  //     target_index <= 0 || target_index >= width*height
  //    ) {
  //     continue;
  //   } else {
  //     uchar class_name = seg_map.at<uchar>(target_row,target_col);
  //     if (class_name == 0) {
  //       *iter_red_buoy = 1.0 ; 
  //     } else if (class_name == 1) {
  //       *iter_yellow_buoy = 1.0 ;
  //     } else if (class_name == 2) {
  //       *iter_black_buoy = 1.0 ;
  //     } else if (class_name == 3) {
  //       *iter_dock = 1.0 ;
  //     } else {
  //     }
  //   }
 
  // }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)