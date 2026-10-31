cmake_minimum_required(VERSION 3.8)
project(acados_legged_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Standard C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# Find only ament_cmake to keep the structure of package ROS2
find_package(ament_cmake REQUIRED)

# Dependencies needed by the controller plugin
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# OCS2 / legged_* export Boost::system in their link interface, so Boost must be found before them
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(legged_controllers REQUIRED)
# needed by the headers included by legged_controllers/LeggedController.h
find_package(ocs2_legged_robot_ros REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(legged_interface REQUIRED)
find_package(legged_wbc REQUIRED)
find_package(legged_estimation REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(angles REQUIRED)
find_package(pinocchio REQUIRED)
# CONFIG: parsing of config.yaml and default path from the share directory
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)


# ACADOS configuration
set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
if(NOT ACADOS_DIR)
  message(FATAL_ERROR "ACADOS_SOURCE_DIR not setted!!")
endif()

# Gather all .c generated files
file(GLOB_RECURSE ACADOS_C_SOURCES
  "c_generated_code/*.c"
)
# Exclude the files main_*.c of test to avoid conflicts with main C++
list(FILTER ACADOS_C_SOURCES EXCLUDE REGEX ".*main_.*\\.c$")

# STATIC ACADOS: OCS2 (hpipm_catkin, blasfeo_catkin) and acados both ship a library with SONAME libhpipm.so / libblasfeo.so,
# but different versions. In the same process (Gazebo) only one is loaded -> libacados.so called OCS2's hpipm -> segfault.
# Fix: link a static build of acados (+ its hpipm/blasfeo/qpOASES_e) inside libmpc_solver.so and hide its symbols.
# The shared build in ${ACADOS_DIR} is left untouched (used by the python acados_template).
set(ACADOS_STATIC_DIR "${ACADOS_DIR}/install_static")
if(NOT EXISTS "${ACADOS_STATIC_DIR}/lib/libacados.a")
  message(FATAL_ERROR "Static acados not found in ${ACADOS_STATIC_DIR}: build it first (see the instructions)")
endif()

include_directories(
  include
  ${ACADOS_STATIC_DIR}/include
  ${ACADOS_STATIC_DIR}/include/acados
  ${ACADOS_STATIC_DIR}/include/blasfeo/include
  ${ACADOS_STATIC_DIR}/include/hpipm/include
  c_generated_code
)



# Creation of the wrapper lib
add_library(mpc_solver SHARED
  src/MpcSolver.cpp
  src/LipmConfig.cpp  # CONFIG: loader of config.yaml (used by the controller and test_solver)
  ${ACADOS_C_SOURCES}
)

# Link to acados
# STATIC ACADOS: static archives (order matters: acados -> hpipm -> blasfeo), no runtime dependency on libhpipm.so
target_link_libraries(mpc_solver
  ${ACADOS_STATIC_DIR}/lib/libacados.a
  ${ACADOS_STATIC_DIR}/lib/libhpipm.a
  ${ACADOS_STATIC_DIR}/lib/libblasfeo.a
  ${ACADOS_STATIC_DIR}/lib/libqpOASES_e.a
  m
  yaml-cpp::yaml-cpp  # CONFIG
)
# STATIC ACADOS: symbols coming from the static archives become local to libmpc_solver.so,
# so they are not mixed with the hpipm/blasfeo of OCS2 loaded in the same process
target_link_options(mpc_solver PRIVATE "LINKER:--exclude-libs,ALL")


# the controller was never compiled. Controller plugin library
add_library(${PROJECT_NAME} SHARED
  src/AcadosLeggedController.cpp
)

target_link_libraries(${PROJECT_NAME}
  mpc_solver
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  legged_interface
  legged_wbc
  legged_estimation
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_self_collision_visualization
  ocs2_msgs
  ocs2_centroidal_model
  std_msgs
  std_srvs
  geometry_msgs
  angles
  ament_index_cpp  # CONFIG
)

# legged_controllers exports also an executable target (legged_target_trajectories_publisher) that
# ament_target_dependencies tries to link -> take only its headers (handles + SafetyChecker are header-only)
target_include_directories(${PROJECT_NAME} PRIVATE ${legged_controllers_INCLUDE_DIRS})

# Same flags used by OCS2 / legged_controllers
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Register the plugin for ros2_control
pluginlib_export_plugin_description_file(controller_interface acados_legged_controller_plugins.xml)


# Export and installation
# EXECUTABLE TESTS
add_executable(test_solver src/test_solver.cpp)

# Link the executable to wrapper library
target_link_libraries(test_solver mpc_solver)
# CONFIG: default config.yaml for test_solver (the source one, so it can be run without arguments)
target_compile_definitions(test_solver PRIVATE LIPM_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml")


# Intall the wrapper lib
# Install also the controller plugin
install(TARGETS mpc_solver ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the executable in the folder for ROS2
install(TARGETS test_solver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# CONFIG: install config.yaml in share/acados_legged_controller/config (default path used by the controller)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# SIM: install the launch files (sim_launch.xml)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Publishes the TF odom -> base from /odom (robot in RViz)
install(PROGRAMS scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()