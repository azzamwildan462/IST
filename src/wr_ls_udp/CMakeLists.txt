cmake_minimum_required(VERSION 3.5)
project(wr_ls_udp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED system)

include_directories(
  include
  )

add_library(${PROJECT_NAME}_lib
  "src/wr_ls_common_udp.cpp"
  "src/wr_ls_common.cpp"
  "src/wr_ls_sensor_frame.cpp"
  "src/wr_ls1207de_parser.cpp"
  "src/wr_ls1207de.cpp"
  "src/parser_base.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})
set(EXECUTABLE_NAME "wr_ls_udp")

add_executable(${EXECUTABLE_NAME} src/wr_ls1207de.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})


# generate_dynamic_reconfigure_options(cfg/WrLs.cfg)



install(
    TARGETS ${EXECUTABLE_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY include/
    DESTINATION include
    )
    
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

#install(DIRECTORY urdf
  #DESTINATION share/${PROJECT_NAME})

#install(DIRECTORY meshes
  #DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY config
  #DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()

