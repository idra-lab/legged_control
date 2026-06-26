cmake_minimum_required(VERSION 3.12)
project(legged_estimation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Find ROS 2 package dependencies individually
find_package(rclcpp REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(legged_common REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_ros REQUIRED)
# FIXED: Explicitly discover tf2_geometry_msgs
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find standard external CMake libraries directly
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log log_setup)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
        src/StateEstimateBase.cpp
        src/FromTopicEstimate.cpp
        src/LinearKalmanFilter.cpp
        )

# Configure include directory interfaces for the library target
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# Use ROS 2 recommended macro to link and include ament dependencies
# FIXED: Appended tf2_geometry_msgs
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        ocs2_legged_robot
        legged_common
        realtime_tools
        tf2_ros
        tf2_geometry_msgs
        nav_msgs
        geometry_msgs
        )

# Link standard non-ament external libraries using namespaced targets
target_link_libraries(${PROJECT_NAME} PUBLIC
        pinocchio::pinocchio
        Boost::system
        Boost::log
        Boost::log_setup
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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

# Mark library target for installation into the standard workspace layout
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

# Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

#####################
## Ament Exporting ##
#####################

# Export interfaces so downstream packages can find this library via find_package()
# FIXED: Appended tf2_geometry_msgs
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp ocs2_legged_robot legged_common realtime_tools tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs pinocchio Boost)

ament_package()