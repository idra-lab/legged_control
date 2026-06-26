cmake_minimum_required(VERSION 3.12)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

# Find ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(legged_hw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########

set(SOURCE src/UnitreeHW.cpp)

# --- 1. Shared Plugin Library for Unitree SDK 3.3.1 ---
add_library(${PROJECT_NAME}_3_3_1 SHARED ${SOURCE})
target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)
target_include_directories(${PROJECT_NAME}_3_3_1 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_3_3_1 PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64_3_3_1.so
        lcm
)
ament_target_dependencies(${PROJECT_NAME}_3_3_1 PUBLIC
        rclcpp hardware_interface pluginlib legged_hw sensor_msgs std_msgs
)

# --- 2. Shared Plugin Library for Unitree SDK 3.8.0 ---
add_library(${PROJECT_NAME}_3_8_0 SHARED ${SOURCE})
target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)
target_include_directories(${PROJECT_NAME}_3_8_0 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_3_8_0 PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libunitree_legged_sdk_amd64_3_8_0.a
)
ament_target_dependencies(${PROJECT_NAME}_3_8_0 PUBLIC
        rclcpp hardware_interface pluginlib legged_hw sensor_msgs std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface legged_unitree_hw_plugins.xml)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_3_3_1 TARGETS ${PROJECT_NAME}_3_8_0
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_3_3_1 ${PROJECT_NAME}_3_8_0
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY config launch lib
        DESTINATION share/${PROJECT_NAME}
        )

#####################
## Ament Exporting ##
#####################

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_3_3_1 ${PROJECT_NAME}_3_8_0)
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib legged_hw sensor_msgs std_msgs)

ament_package()