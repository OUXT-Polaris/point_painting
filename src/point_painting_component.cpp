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

namespace bg = boost::geometry;
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

namespace point_painting
{
PointPaintingFusionComponent::PointPaintingFusionComponent(const rclcpp::NodeOptions & options)
: Node("pointpainting_fusion", options), 
  buffer_(get_clock()),
  listener_(buffer_),
  sync_(approximate_policy(10),segmentation_sub_,pointcloud_sub_)
{  
  //param
  declare_parameter<std::vector<double>>("point_cloud_range");
  declare_parameter("segmentation_topic","/detic_node/detic_result/segmentation_info");
  declare_parameter("camera_info_topic","/CameraInfo");
  declare_parameter("point_cloud_topic","/point_cloud");
  declare_parameter("task_obj",rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("obst_obj",rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("debug",false);

  pointcloud_range_ = get_parameter("point_cloud_range").as_double_array();
  const auto segmentation_topic = get_parameter("segmentation_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto point_cloud_topic = get_parameter("point_cloud_topic").as_string();
  task_obj = get_parameter("task_obj").as_string_array();
  obst_obj = get_parameter("obst_obj").as_string_array();
  debug = get_parameter("debug").as_bool();

  //publisher
  using namespace std::chrono_literals;
  point_painting_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_painting", 10);
  if (debug){
    preprocess_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("preprocess_debug", 10);
  }

  //subscriber
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo & camera_info_msg){camera_info_callback(camera_info_msg);});

  //追加
  segmentation_sub_.subscribe(this,segmentation_topic);
  pointcloud_sub_.subscribe(this,point_cloud_topic);
  sync_.registerCallback(&PointPaintingFusionComponent::topic_callback, this);
}

void PointPaintingFusionComponent::topic_callback(
  const detic_onnx_ros2_msg::msg::SegmentationInfo & seg_msg,
  const sensor_msgs::msg::PointCloud2 &pointcloud_msg) {
    segmentationinfo_ = seg_msg;
    sensor_msgs::msg::PointCloud2 pointcloud = pointcloud_msg;
    preprocess(pointcloud);
    fuseOnSingleImage(segmentationinfo_,pointcloud,camera_info_);
}

void PointPaintingFusionComponent::camera_info_callback(const sensor_msgs::msg::CameraInfo  & camera_info_msg)
{
  camera_info_ = camera_info_msg;
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
  const detic_onnx_ros2_msg::msg::SegmentationInfo & seg_msg,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg,
  const sensor_msgs::msg::CameraInfo & camera_info
)
{ 
  typedef bg::model::polygon<bg::model::d2::point_xy<int>> polygon;
  std::vector<polygon> polygons;
  for (const auto& seg_info : seg_msg.segmentations){  
    polygon seg_polygon;
    for (const auto& img_point : seg_info.polygons[0].points){
      seg_polygon.outer().emplace_back(bg::make<bg::model::d2::point_xy<int>>(img_point.x, img_point.y));
    }
    seg_polygon.outer().emplace_back(seg_polygon.outer().front()); //ポリゴンを閉じる作業
    polygons.emplace_back(seg_polygon);
  }

  /*
  点群::LiDAR座標系⇨カメラ座標系⇨画像座標系
  */
  //点群::LiDAR座標系⇨カメラ座標系
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
    camera_info.p.at(7),camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), 
    camera_info.p.at(11), 0.0, 0.0, 0.0 ,1.0;

  sensor_msgs::PointCloud2Iterator<float> iter_class(transformed_pointcloud, "class");
  sensor_msgs::PointCloud2Iterator<float> iter_scores(transformed_pointcloud, "scores");
    
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
       iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
       iter_x != iter_x.end(); 
       ++iter_x, ++iter_y, ++iter_z,
       ++iter_class, ++iter_scores) {
    Eigen::Vector4d projected_point =
      camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
    if(projected_point.z() < 0.0){
      continue;
    }
    int img_point_x = int(normalized_projected_point.x());
    int img_point_y = int(normalized_projected_point.y());

    int width = camera_info.width ;
    int height = camera_info.height ;
    *iter_class = 0.0; 
    *iter_scores = 0.0;
    if (0 <= img_point_x && img_point_x <= int(width) && 0 <= img_point_y && img_point_y <= int(height)) {   
      for (int i = 0; i < static_cast<int>(polygons.size()); ++i)
      {
        typedef bg::model::d2::point_xy<double> img_point;
        bool is_covered = boost::geometry::covered_by(img_point(img_point_x,img_point_y),polygons[i]);
        if (is_covered) {
          auto temp_seg_info = seg_msg.segmentations[i]; 
          std::string seg_class = temp_seg_info.object_class;
          //RCLCPP_INFO(this->get_logger(),seg_class.c_str());
          for (auto &obj : task_obj) {
              if (seg_class == obj) {
                  *iter_class = 200.0;
                  *iter_scores = temp_seg_info.score;
                  //RCLCPP_INFO(this->get_logger(),"task_obj");
                  break;
              }
          }
          for (auto &obj : obst_obj) {
              if (seg_class == obj) {
                  *iter_class =100.0;
                  *iter_scores = temp_seg_info.score;
                  //RCLCPP_INFO(this->get_logger(),"obst_obj");
                  break;
              }
          }
        }else{
          // *iter_class = 50.0;     
          // *iter_scores = 100.0;
          //RCLCPP_INFO(this->get_logger(),"don't coverd");
        }
      }
    }
  }
  sensor_msgs::msg::PointCloud2 paint_pointcloud;
  geometry_msgs::msg::TransformStamped reverse_transform_stamped;
  try {
  reverse_transform_stamped = buffer_.lookupTransform(
    painted_pointcloud_msg.header.frame_id, camera_info.header.frame_id, camera_info.header.stamp,
    rclcpp::Duration::from_seconds(0.1));
  tf2::doTransform(transformed_pointcloud, paint_pointcloud , reverse_transform_stamped);
  } catch (tf2::TransformException & ex) {
  RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
  return;
  }
  point_painting_pub_->publish(paint_pointcloud);
   
}
}  // namespace point_painting

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_painting::PointPaintingFusionComponent)
