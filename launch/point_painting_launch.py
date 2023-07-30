from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os
import yaml

    
def getPointPaintingComponent():
    config = os.path.join(
      get_package_share_directory("point_painting"),
      'config',
      'pointpainting.param.yaml')
    component = ComposableNode(
        package="point_painting", 
        plugin="point_painting::PointPaintingFusionComponent", 
        name="point_painting_node",
        namespace='point_painting_node',
        parameters=[config]
    )
    return component


def generate_launch_description():
    


    container = ComposableNodeContainer(
        name="point_painting",
        namespace="point_painting",
        package="point_painting",
        executable="point_painting_component",
        composable_node_descriptions=[
            getPointPaintingComponent()
        ],
        output="screen",
    )

    description = LaunchDescription(
        [
            container,
        ]
    )  
    return description
