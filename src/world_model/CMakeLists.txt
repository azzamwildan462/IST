cmake_minimum_required(VERSION 3.8)
project(world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add -Wno-dev to suppress warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-dev")

# find dependencies
set(var_dependencies
    ament_cmake
    rclcpp
    std_msgs
    nav_msgs
    ros2_utils
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_sensor_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    tf2_eigen
    OpenCV
    laser_geometry
    yaml-cpp
    tf2_ros)

foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pose_estimator src/pose_estimator.cpp)
ament_target_dependencies(pose_estimator ${var_dependencies})

add_executable(obstacle_filter src/obstacle_filter.cpp)
ament_target_dependencies(obstacle_filter ${var_dependencies})
target_link_libraries(obstacle_filter ${PCL_LIBRARIES})

add_executable(all_obstacle_filter src/all_obstacle_filter.cpp)
ament_target_dependencies(all_obstacle_filter ${var_dependencies})
target_link_libraries(all_obstacle_filter ${PCL_LIBRARIES})

add_executable(icp src/icp.cpp)
ament_target_dependencies(icp ${var_dependencies})
target_link_libraries(icp ${PCL_LIBRARIES})

# add_executable(pose_estimator_icp src/pose_estimator_icp.cpp)
# ament_target_dependencies(pose_estimator_icp ${var_dependencies})
# target_link_libraries(pose_estimator_icp ${PCL_LIBRARIES})

add_executable(camera_obstacle_detector src/camera_obstacle_detector.cpp)
ament_target_dependencies(camera_obstacle_detector ${var_dependencies})
target_link_libraries(camera_obstacle_detector ${PCL_LIBRARIES})

add_executable(lidar_obstacle_filter src/lidar_obstacle_filter.cpp)
ament_target_dependencies(lidar_obstacle_filter ${var_dependencies})
target_link_libraries(lidar_obstacle_filter ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})

install(
  TARGETS pose_estimator
          obstacle_filter
          all_obstacle_filter
          # pose_estimator_icp
          camera_obstacle_detector
          lidar_obstacle_filter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
