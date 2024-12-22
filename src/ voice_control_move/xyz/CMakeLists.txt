cmake_minimum_required(VERSION 3.8)
project(xyz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(xyz_node src/xyz_node.cpp)
ament_target_dependencies(xyz_node rclcpp std_msgs geometry_msgs)

install(TARGETS xyz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()