cmake_minimum_required(VERSION 3.8)
project(fastlio2_px4_odom_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fastlio2_px4_odom_bridge_node src/fastlio2_px4_odom_bridge_node.cpp)
target_compile_features(fastlio2_px4_odom_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(fastlio2_px4_odom_bridge_node rclcpp nav_msgs px4_msgs std_msgs)
target_link_libraries(fastlio2_px4_odom_bridge_node Eigen3::Eigen)

install(TARGETS fastlio2_px4_odom_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
