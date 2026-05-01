cmake_minimum_required(VERSION 3.8)
project(rebob_map_builder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePCD.srv"
)

add_executable(build_map src/build_map.cpp)

rosidl_target_interfaces(build_map ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(build_map 
  rclcpp 
  sensor_msgs 
  tf2_ros 
  tf2_sensor_msgs 
  pcl_conversions
  grid_map_core
  grid_map_ros
  grid_map_msgs
)
target_link_libraries(build_map ${PCL_LIBRARIES})

install(TARGETS build_map
  DESTINATION lib/${PROJECT_NAME})

ament_package()
