cmake_minimum_required(VERSION 3.8)
project(point_painting)
set(COMP_NAME "point_painting")
string(TOLOWER ${COMP_NAME} COMP_NAME_LOWER)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(segmentation_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


# Create the component target
ament_auto_add_library(${PROJECT_NAME}-${COMP_NAME_LOWER}_component SHARED
    src/${COMP_NAME_LOWER}_component.cpp
)
target_compile_definitions(${PROJECT_NAME}-${COMP_NAME_LOWER}_component
    PRIVATE ${PROJECT_NAME}_BUILDING_DLL)

rclcpp_components_register_nodes(${PROJECT_NAME}-${COMP_NAME_LOWER}_component
    "point_painting::PointPaintingFusionComponent")


# Create the executor target
ament_auto_add_executable(${PROJECT_NAME}-${COMP_NAME_LOWER}_node
    NO_TARGET_LINK_LIBRARIES
    src/${COMP_NAME_LOWER}_node.cpp
)
target_link_libraries(${PROJECT_NAME}-${COMP_NAME_LOWER}_node
    ${PROJECT_NAME}-${COMP_NAME_LOWER}_component)

ament_auto_package()

