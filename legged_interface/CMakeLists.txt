cmake_minimum_required(VERSION 3.12)
project(legged_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

# Find ROS 2 packages individually
find_package(rclcpp REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_self_collision REQUIRED)

# Find plain CMake external dependencies
find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log
        log_setup
        )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

message(STATUS "pinocchio_FOUND = ${pinocchio_FOUND}")
message(STATUS "pinocchio_INCLUDE_DIRS = ${pinocchio_INCLUDE_DIRS}")
message(STATUS "pinocchio_LIBRARIES = ${pinocchio_LIBRARIES}")
message(STATUS "pinocchio_DIR = ${pinocchio_DIR}")

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/constraint/SwingTrajectoryPlanner.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/SwitchedModelReferenceManager.cpp
        src/LeggedRobotPreComputation.cpp
        src/LeggedInterface.cpp
        )

# Use ROS 2 recommended macro to link and include ament dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        rclcpp
        ocs2_legged_robot
        ocs2_self_collision
        )

# Link standard CMake external dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
        pinocchio::pinocchio
        Eigen3::Eigen
        Boost::system
        Boost::filesystem
        )

# Configure include directory interfaces for the library target
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
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

add_executable(legged_precompile src/precompile.cpp)
target_link_libraries(legged_precompile PUBLIC ${PROJECT_NAME})

#############
## Install ##
#############

# Mark targets for installation into the local workspace layout
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS legged_precompile
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
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp ocs2_legged_robot ocs2_self_collision pinocchio Eigen3 Boost)

ament_package()