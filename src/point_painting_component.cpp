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

namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options)
{
  // const float score_threshold =static_cast<float>(this->declare_parameter<double>("score_threshold", 0.4));
  // class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  // pointcloud_range = this->declare_parameter<std::vector<double>>("point_cloud_range");
  // const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  // const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(10ms, [this]() { timer_callback(); }); //ラムダ式　thisは周りの変数を渡す(通常はスコープ範囲は外見れない) 
}

PointPaintingFusionComponent::~PointPaintingFusionComponent() {}

void PointPaintingFusionComponent::timer_callback()
{
  //subscriber
  // pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "/pointcloud",10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr & point){
  //     // ROS2のロギング用マクロ、ターミナルにログ出力しつつログファイルにも吐き出してくれる便利なやつ
  //     preprocess(point);});
  //segmentation_sub_ = create_subscription<SegmentationInfo>(
    //"/SegmentationInfo", 1, [this](const  seg_msg) { segmentation_callback(seg_msg); });
}
// void PointPaintingFusionComponent::segmentation(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
// { 
//   preprocess(painted_pointcloud_msg)
//   //fuseOnSingleImage(painted_pointcloud_msg)
// }


// void PointPaintingFusionComponent::segmentation()
// {
  
// }

void PointPaintingFusionComponent::preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp;
  tmp = painted_pointcloud_msg;
  painted_pointcloud_msg.width = tmp.width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
  pcd_modifier.clear(); //なぜここでclearしてるのか
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
  painted_pointcloud_msg.point_step = num_fields * sizeof(float); //バイト単位のポイントの長さ
  

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
    // if (
    //   *iter_x <= pointcloud_range.at(0) || *iter_x >= pointcloud_range.at(3) ||
    //   *iter_y <= pointcloud_range.at(1) || *iter_y >= pointcloud_range.at(4)) {
    //   continue;
    // } else {
    //   *iter_painted_x = *iter_x;
    //   *iter_painted_y = *iter_y;
    //   *iter_painted_z = *iter_z;
    //   j += painted_point_step;
    // }
    
  }
  painted_pointcloud_msg.data.resize(j);
  painted_pointcloud_msg.width = static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height /painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step =static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
}

  
// void PointPaintingFusionComponent::fuseOnSingleImage(
//   const SegmentationInfo & SegmentationInfo,
//   const sensor_msgs::msg::CameraInfo & camera_info,
//   sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg
// )
// {
//   std::vector<sensor_msgs::msg::RegionOfInterest> segmentationInfo; //segmantationメッセージに書き換え
//   std::vector<Eigen::Vector2d> debug_image_points;

//   geometry_msgs::msg::TransformStamped transform_stamped;
//   {
//     //https://docs.ros.org/en/jade/api/tf/html/c++/classtf_1_1StampedTransform.html
//     const auto transform_stamped_optional = getTransformStamped(
//       tf_buffer_, /*target*/ camera_info.header.frame_id,
//       /*source*/ painted_pointcloud_msg.header.frame_id, camera_info.header.stamp);
//     if (!transform_stamped_optional) {
//       return;
//     }
//     transform_stamped = transform_stamped_optional.value();
//   }

//   geometry_msgs::msg::TransformStamped transform_stamped;

//   //カメラの外パラを取得　Matrix4dは4✕4の行列を作る関数 
//   Eigen::Matrix4d camera_projection;
//   camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
//     camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
//     camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
//     camera_info.p.at(11);
  
//   sensor_msgs::msg::PointCloud2 transformed_pointcloud;
//   tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);
//  // iterate points
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
//     // 外パラと世界座標系かけたらカメラ座標系に戻る
//     Eigen::Vector4d projected_point =camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
//     //カメラ座標系から画像座標系
//     Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

//   //   // iterate 2d bbox
//   //   for (const auto & feature_object : input_roi_msg.feature_objects) {
//   //     sensor_msgs::msg::RegionOfInterest roi = feature_object.feature.roi;
//   //     // paint current point if it is inside bbox
//   //     if (
//   //       normalized_projected_point.x() >= roi.x_offset &&
//   //       normalized_projected_point.x() <= roi.x_offset + roi.width &&
//   //       normalized_projected_point.y() >= roi.y_offset &&
//   //       normalized_projected_point.y() <= roi.y_offset + roi.height &&
//   //       feature_object.object.classification.front().label !=
//   //         autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
//   //       switch (feature_object.object.classification.front().label) {
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::CAR:
//   //           *iter_car = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK:
//   //           *iter_car = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER:
//   //           *iter_car = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::BUS:
//   //           *iter_car = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
//   //           *iter_ped = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE:
//   //           *iter_bic = 1.0;
//   //           break;
//   //         case autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
//   //           *iter_bic = 1.0;
//   //           break;
//   //       }
//   //   }
//   // }  // namespace pkgname
// }
// }
}

RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)
