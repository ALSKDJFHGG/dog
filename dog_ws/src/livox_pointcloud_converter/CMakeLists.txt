cmake_minimum_required(VERSION 3.8)
project(livox_pointcloud_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Create the converter node
add_executable(livox_to_pointcloud2_node 
  src/livox_to_pointcloud2_node.cpp
  src/livox_to_pointcloud2_component.cpp
)

ament_target_dependencies(livox_to_pointcloud2_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
)

target_link_libraries(livox_to_pointcloud2_node ${PCL_LIBRARIES})

# Create the converter component
add_library(livox_to_pointcloud2_component SHARED
  src/livox_to_pointcloud2_component.cpp
)

ament_target_dependencies(livox_to_pointcloud2_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
)

target_link_libraries(livox_to_pointcloud2_component ${PCL_LIBRARIES})

rclcpp_components_register_nodes(livox_to_pointcloud2_component "livox_pointcloud_converter::LivoxToPointCloud2Component")

# Install
install(TARGETS
  livox_to_pointcloud2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  livox_to_pointcloud2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
