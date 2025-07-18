cmake_minimum_required(VERSION 3.8)
project(master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(var_dependencies
    std_srvs
    ament_cmake
    rclcpp
    std_msgs
    nav_msgs
    ros2_utils
    ros2_interface
    tf2_geometry_msgs
    rtabmap_msgs
    tf2_ros
    tf2_sensor_msgs
    pcl_conversions
    pcl_ros
    tf2_eigen
    )
    
foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS filesystem)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_executable(master src/master.cpp src/master_definition.cpp src/motion.cpp)
ament_target_dependencies(master ${var_dependencies})
target_link_libraries(master Boost::filesystem ${PCL_LIBRARIES})

install(TARGETS
  master
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
