cmake_minimum_required(VERSION 3.8)
project(plan_env)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
set(dependencies
        rclcpp
        std_msgs
        nav_msgs
        sensor_msgs
        Eigen3
        PCL
        pcl_conversions
)
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()


# Plan_env Library
add_library(${PROJECT_NAME} SHARED
        src/library/grid_map.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${dependencies}
)


install(
        TARGETS plan_env
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
