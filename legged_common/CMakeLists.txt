cmake_minimum_required(VERSION 3.12)
project(legged_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Find ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

# Define the interface library
add_library(${PROJECT_NAME} INTERFACE)

# Define include directories for the interface library
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies to the interface library using modern target-based CMake
target_link_libraries(${PROJECT_NAME} INTERFACE
    rclcpp::rclcpp
    hardware_interface::hardware_interface
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Interface targets do not produce binary files, so ARCHIVE/LIBRARY/RUNTIME are omitted
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

# Install executable scripts to the standard ROS 2 package lib layout
install(PROGRAMS scripts/generate_urdf.sh
        DESTINATION lib/${PROJECT_NAME}
)

#####################
## Ament Exporting ##
#####################

# Export targets and dependencies for downstream packages
ament_export_include_directories(include)

# NOTE: ament_export_libraries() is removed here because this is a pure INTERFACE target.
# Downstream packages will still get the right configurations via ament_export_targets.
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface)

ament_package()