cmake_minimum_required(VERSION 3.12)
project(legged_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Find ROS 2 package dependencies individually
find_package(rclcpp REQUIRED)
find_package(legged_common REQUIRED)
find_package(simulation_interfaces REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

# UPDATED: Replaced Gazebo Classic packages with modern Gazebo Harmonic wrappers
find_package(ros_gz_sim REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

add_library(legged_hw_sim SHARED
        src/LeggedHWSim.cpp
        )

# Configure include directory interfaces for the library target
target_include_directories(legged_hw_sim PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Fixed: Added the explicit PUBLIC keyword to match the signature above
# UPDATED: Dependencies updated to target the new modern Gazebo components
ament_target_dependencies(legged_hw_sim PUBLIC
        rclcpp
        legged_common
        ros_gz_sim
        gz_ros2_control
        )

# Register the plugin description file for pluginlib in ROS 2
# UPDATED: Target package changed to gz_ros2_control
pluginlib_export_plugin_description_file(gz_ros2_control legged_hw_sim_plugins.xml)

# rslidar_sdk-style point cloud for the simulated RoboSense Airy (legged_unitree_description/urdf/common/robosense_airy.xacro)
add_library(airy_points_conversion STATIC
        src/AiryPointsConversion.cpp
        )
target_include_directories(airy_points_conversion PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
target_link_libraries(airy_points_conversion PUBLIC Eigen3::Eigen)
ament_target_dependencies(airy_points_conversion PUBLIC sensor_msgs)

add_executable(airy_points_relay
        src/AiryPointsRelay.cpp
        )
target_link_libraries(airy_points_relay airy_points_conversion)
ament_target_dependencies(airy_points_relay rclcpp sensor_msgs tf2_ros tf2_eigen)

# This package is built without a build type (-O0), where the Eigen per-point loop takes ~100 ms for a 900 x 96 cloud
target_compile_options(airy_points_conversion PRIVATE -O2)
target_compile_options(airy_points_relay PRIVATE -O2)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_airy_points_conversion test/test_airy_points_conversion.cpp)
    target_link_libraries(test_airy_points_conversion airy_points_conversion)
endif ()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target legged_hw_sim")
    add_clang_tooling(
            TARGETS legged_hw_sim
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark library for installation into standard workspace paths
install(TARGETS legged_hw_sim
        EXPORT legged_hw_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS airy_points_relay
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# Mark headers for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

# Install resource config directory into share
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )
        
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

#####################
## Ament Exporting ##
#####################

# Export interfaces so downstream packages can find this library via find_package()
ament_export_include_directories(include)
ament_export_libraries(legged_hw_sim)
ament_export_targets(legged_hw_sim)

# UPDATED: Updated exported dependencies to match the modern wrappers
ament_export_dependencies(rclcpp legged_common ros_gz_sim gz_ros2_control)

ament_package()