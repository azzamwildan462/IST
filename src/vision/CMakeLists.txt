cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(var_dependencies
    ament_cmake
    cv_bridge
    OpenCV
    rclcpp
    std_msgs
    nav_msgs
    ros2_utils
    ros2_interface
    yaml-cpp
    tf2_geometry_msgs
    tf2_ros)
    
foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

add_executable(vision_capture src/vision_capture.cpp)
ament_target_dependencies(vision_capture ${var_dependencies})

add_executable(lane_detection src/lane_detection.cpp)
ament_target_dependencies(lane_detection ${var_dependencies})
target_link_libraries(lane_detection ${YAML_CPP_LIBRARIES})

install(TARGETS
  vision_capture
  lane_detection
  DESTINATION lib/${PROJECT_NAME}
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
