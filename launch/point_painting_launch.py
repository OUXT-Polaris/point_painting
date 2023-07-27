# from launch import LaunchDescription
# from launch_ros.actions import Node
# #どっちもshareから引っ張ってきているだけっぽい
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.substitutions import FindPackageShare

# #パラメータ宣言用のやつ
# from launch.actions.declare_launch_argument import DeclareLaunchArgument

# #
# from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes

# import os
# import yaml

    



# def getPointPaintingComponent():
#     component = ComposableNode(
#         package="point_painting", plugin="navi_sim::NaviSimComponent", name="navi_sim_node"
#     )
#     return component


# def generate_launch_description():
        
#     config_directory = os.path.join(get_package_share_directory("point_painting"), "config")
#     param_config = os.path.join(config_directory,"/*.yaml")
#     params = {}

#     #LaunchConfigurationを使うことで第一引数でしていしたparameterを変数のように扱える
#     camera = LaunchConfiguration("camera_num", default=0)
#     #descriptionにこの変数を渡すことで実行時にコマンド引数として扱える
#     camera_ns = DeclareLaunchArgument('camera_num',default_value=0)
    
#     LoadComposableNodes(
#         composable_node_descriptions=[getScenarioTestComponent(scenario_filename)],
#         target_container=simulator,
#         condition=IfCondition(scenario_mode),
#     )
    

#     RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=simulator, on_exit=[EmitEvent(event=Shutdown())]
#         )
#     ),
#     IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [description_dir, "/wamv_description.launch.py"]
#         )
#     ),

#     container = ComposableNodeContainer(
#         name="point_painting",
#         namespace="point_painting",
#         package="point_painting",
#         executable="point_painting_component",
#         composable_node_descriptions=[
#             getPointPaintingComponent(),
#         ],
#         output="screen",
#     )

#     description = LaunchDescription(
#         [
#             camera_ns,
#             container,
#         ]
#     )  
#     return description
