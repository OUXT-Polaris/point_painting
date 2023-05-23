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

// Headers in this package
#include "point_painting/point_painting_component.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component
#include <sensor_msgs/msg/point_cloud2.hpp>



namespace point_segmentation_fusion 
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options)
{
  // //subscriber
  // scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "/scan", 1, [this](const sensor_msgs::msg::PointCloud2::SharedPtr scan) { sub_callback(scan); });
  // segmentation_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "/SegmentationInfo", 1, [this](const  seg_msg) { segmentation_callback(scan); });
  // timer_ = create_wall_timer(10ms, [this]() { timer_callback(); }); //ラムダ式　thisは周りの変数を渡す(通常はスコープ範囲は外見れない) 
}


// void PointPaintingFusionComponent::timer_callback()
// {

// }


// void PointPaintingFusionComponent::sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point)
// {

// }

// void PointPaintingFusionComponent::segmentation()
// {
  
// }

// void PointPaintingFusionComponent::preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
// {
//   sensor_msgs::msg::PointCloud2 tmp;
//   tmp = painted_pointcloud_msg;
//   painted_pointcloud_msg.width = tmp.width;

//   sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
//   pcd_modifier.clear();

//   //ここで新しいポイントクラウドのフィールドを設定する
//   pcd_modifier.setPointCloud2Fields(
//     num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
//     sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
//     "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "RED", 1,
//     sensor_msgs::msg::PointField::FLOAT32, "PEDESTRIAN", 1, sensor_msgs::msg::PointField::FLOAT32,
//     "BICYCLE", 1, sensor_msgs::msg::PointField::FLOAT32);


//   // transform
//   sensor_msgs::msg::PointCloud2 transformed_pointcloud;


  
//   // iterate points
//   sensor_msgs::PointCloud2Iterator<float> iter_car(painted_pointcloud_msg, "CAR");
//   sensor_msgs::PointCloud2Iterator<float> iter_ped(painted_pointcloud_msg, "PEDESTRIAN");
//   sensor_msgs::PointCloud2Iterator<float> iter_bic(painted_pointcloud_msg, "BICYCLE");

//   for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
//        iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
//        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_car, ++iter_ped, ++iter_bic) {
//     // filter the points outside of the horizontal field of view
//     if (
//       *iter_z <= 0.0 || (*iter_x / *iter_z) > tan_h_.at(image_id) ||
//       (*iter_x / *iter_z) < -tan_h_.at(image_id)) {
//       continue;
//     }
// //   }

// }
}  // namespace pkgname

RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)
