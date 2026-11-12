cmake_minimum_required(VERSION 3.16)
project(px4ctrl)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DesiredState.msg"
  "msg/ControlDebug.msg"
  DEPENDENCIES geometry_msgs std_msgs)

add_executable(px4ctrl_node src/px4ctrl_node.cpp)
target_compile_features(px4ctrl_node PUBLIC cxx_std_17)
target_link_libraries(px4ctrl_node Eigen3::Eigen)
ament_target_dependencies(px4ctrl_node
  geometry_msgs px4_msgs rclcpp std_msgs)
rosidl_get_typesupport_target(px4ctrl_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(px4ctrl_node "${px4ctrl_typesupport_target}")

install(TARGETS px4ctrl_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ego_position_command_adapter.py
  DESTINATION lib/${PROJECT_NAME} RENAME ego_position_command_adapter)
install(PROGRAMS scripts/manual_goal.py
  DESTINATION lib/${PROJECT_NAME} RENAME manual_goal)
install(PROGRAMS scripts/takeoff.py
  DESTINATION lib/${PROJECT_NAME} RENAME takeoff)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_px4ctrl_integration.py")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_px4ctrl_integration
    test/test_px4ctrl_integration.py TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
