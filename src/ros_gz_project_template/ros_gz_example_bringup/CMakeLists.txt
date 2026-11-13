cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_bringup)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_example_description REQUIRED)
find_package(ros_gz_example_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# urdf_to_rerun: streams the robot to a Rerun viewer. Built only when the Rerun C++ SDK
# is available (the Docker image installs it to /opt/rerun_cpp_sdk).
find_package(rerun_sdk QUIET PATHS /opt/rerun_cpp_sdk)
if(rerun_sdk_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(urdf REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  add_executable(urdf_to_rerun src/urdf_to_rerun.cpp)
  target_compile_features(urdf_to_rerun PRIVATE cxx_std_17)
  ament_target_dependencies(urdf_to_rerun
    rclcpp std_msgs tf2 tf2_ros urdf ament_index_cpp)
  target_link_libraries(urdf_to_rerun rerun_sdk)
  install(TARGETS urdf_to_rerun DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "Rerun C++ SDK not found; skipping urdf_to_rerun. "
    "Set CMAKE_PREFIX_PATH to a rerun_cpp_sdk install to build it.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
