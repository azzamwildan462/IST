cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CURRENT_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  /usr/local/include
  ${CURRENT_WORKING_DIRECTORY}/../../SOEM_local/install/include
)

add_executable(beckhoff src/beckhoff.cpp)
add_executable(CANbus_HAL src/CANbus_HAL.cpp)
add_executable(serial_imu src/serial_imu.cpp)
add_executable(keyboard_input src/keyboard_input.cpp)

ament_target_dependencies(beckhoff rclcpp std_msgs ros2_utils)
ament_target_dependencies(CANbus_HAL rclcpp std_msgs ros2_utils)
ament_target_dependencies(serial_imu rclcpp std_msgs sensor_msgs ros2_utils tf2_ros)
ament_target_dependencies(keyboard_input rclcpp std_msgs)

target_link_libraries(beckhoff ${CURRENT_WORKING_DIRECTORY}/../../SOEM_local/install/lib/libsoem.a)

install(TARGETS
  beckhoff
  CANbus_HAL
  serial_imu
  keyboard_input
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
