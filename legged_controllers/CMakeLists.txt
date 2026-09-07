cmake_minimum_required(VERSION 3.12)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Python3 and set PYTHON_EXECUTABLE to avoid conflicts between eigenpy/Boost.Python and rosidl_generate_interfaces
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python interpreter" FORCE)


# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find ROS 2 package dependencies individually
find_package(rclcpp REQUIRED)
find_package(legged_common REQUIRED)
find_package(legged_interface REQUIRED)
find_package(legged_wbc REQUIRED)
find_package(legged_estimation REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(opti_pessi_interface REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)

# Generate custom interfaces/services in ROS 2 instead of add_service_files()
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KillNode.srv"
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########

# Declare a shared controller plugin library target
add_library(${PROJECT_NAME}_lib SHARED
        src/LeggedController.cpp
        )

# Set the output name of the library to match the original package name
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Configure include directory interfaces for the library target
target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Use ROS 2 recommended macro to link and include ament dependencies
#
# NOTE: as with pinocchio (see target_link_libraries below), opti_pessi_wbc_bridge is passed
# through ament_target_dependencies() -- unlike pinocchio it has no legacy *_LIBRARIES fallback
# issue of its own, but it links pinocchio::pinocchio (the modern imported target) internally and
# exports that transitively, so consuming it here does not reintroduce the dual-pinocchio conflict
# documented in opti_pessi_wbc_bridge/CMakeLists.txt -- only calling ament_target_dependencies()
# with "pinocchio" itself (which this file deliberately never does) would do that.
ament_target_dependencies(${PROJECT_NAME}_lib
        rclcpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        pluginlib
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        std_msgs
        opti_pessi_interface
        )

# Link standard non-ament external libraries using namespaced targets
target_link_libraries(${PROJECT_NAME}_lib
        pinocchio::pinocchio
        )

# Link the generated interface typesupport to the library so it can use KillNode.srv internally
rosidl_get_typesupport_target(typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${typesupport_target})

target_compile_options(${PROJECT_NAME}_lib PUBLIC ${OCS2_CXX_FLAGS})

# Target trajectories publish node executable
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )

ament_target_dependencies(legged_target_trajectories_publisher
        rclcpp
        ocs2_legged_robot_ros
        )

target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}_lib
        )

target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

# Register the plugin configuration file for ros2_control/pluginlib loader
pluginlib_export_plugin_description_file(controller_interface legged_controllers_plugins.xml)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_lib legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_lib legged_target_trajectories_publisher
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

# Install executable scripts to standard ROS 2 workspace layout
install(PROGRAMS scripts/kill_node_sigint_service.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )

#####################
## Ament Exporting ##
#####################

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp legged_common legged_interface legged_wbc legged_estimation controller_interface pluginlib ocs2_legged_robot_ros ocs2_self_collision_visualization angles std_msgs pinocchio opti_pessi_interface)

ament_package()