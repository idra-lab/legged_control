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

find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)


# acados Configuration
set(ACADOS_SOURCE_DIR "$ENV{HOME}/acados")


link_directories(
        ${ACADOS_SOURCE_DIR}/lib
)

file(GLOB_RECURSE ACADOS_C_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/*.c"
)



# Generate custom interfaces/services in ROS 2 instead of add_service_files()
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KillNode.srv"
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########


# 1. Definizione target libreria shared (C++ + sorgenti C Acados)
add_library(${PROJECT_NAME}_lib SHARED
        src/LeggedController.cpp
        ${ACADOS_C_SOURCES}
        )

set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Include directories
target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code>
        $<BUILD_INTERFACE:${ACADOS_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ACADOS_SOURCE_DIR}/include/acados>
        $<BUILD_INTERFACE:${ACADOS_SOURCE_DIR}/include/hpipm/include>
        $<BUILD_INTERFACE:${ACADOS_SOURCE_DIR}/include/blasfeo/include>
        $<INSTALL_INTERFACE:include>
        )

# 2. Dipendenze ROS 2 (Macro Plain)
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
        )

# 3. Linking Librerie Esterne usando i percorsi ASSOLUTI ai file .so di Acados (senza la parola PUBLIC!)
target_link_libraries(${PROJECT_NAME}_lib
        pinocchio::pinocchio
        "${ACADOS_SOURCE_DIR}/lib/libacados.so"
        "${ACADOS_SOURCE_DIR}/lib/libhpipm.so"
        "${ACADOS_SOURCE_DIR}/lib/libblasfeo.so"
        m
        )

# Link del servizio generato KillNode.srv
rosidl_get_typesupport_target(typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${typesupport_target})

target_compile_options(${PROJECT_NAME}_lib PUBLIC ${OCS2_CXX_FLAGS})

# 4. Eseguibile Target Trajectories Publisher
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )

ament_target_dependencies(legged_target_trajectories_publisher
        rclcpp
        ocs2_legged_robot_ros
        )

# Link esplicito anche per l'eseguibile publisher
target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}_lib
        "${ACADOS_SOURCE_DIR}/lib/libacados.so"
        "${ACADOS_SOURCE_DIR}/lib/libhpipm.so"
        "${ACADOS_SOURCE_DIR}/lib/libblasfeo.so"
        )

target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

# Register plugin
pluginlib_export_plugin_description_file(controller_interface legged_controllers_plugins.xml)

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
ament_export_dependencies(rclcpp legged_common legged_interface legged_wbc legged_estimation controller_interface pluginlib ocs2_legged_robot_ros ocs2_self_collision_visualization angles std_msgs pinocchio)

ament_package()