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