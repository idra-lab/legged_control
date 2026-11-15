cmake_minimum_required(VERSION 3.12)
project(legged_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Find ROS 2 package dependencies individually
find_package(rclcpp REQUIRED)
find_package(legged_common REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)

###########
## Build ##
###########

# LeggedHW: base class of the hardware plugins (legged_unitree_hw), not a plugin itself
add_library(${PROJECT_NAME} SHARED
        src/LeggedHW.cpp
        )

# Configure include directory interfaces for the library target
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Link and include ament dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        legged_common
        controller_manager
        hardware_interface
        urdf
        )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

#####################
## Ament Exporting ##
#####################

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp legged_common controller_manager hardware_interface urdf)

ament_package()