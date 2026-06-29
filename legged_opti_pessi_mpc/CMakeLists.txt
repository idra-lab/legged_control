cmake_minimum_required(VERSION 3.8)
project(legged_opti_pessi_mpc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Custom Message Generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootstepCommand.msg"
  "msg/MpcOutput.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

# Find OCS2 Modules
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_legged_robot REQUIRED)

find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Fetch modern typesupport target to bind generated messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


# ==========================================
# 2. MPC Core Library
# ==========================================
add_library(legged_opti_pessi_mpc_core SHARED
  src/core/OptiPessiAugmentedDynamics.cpp
  src/core/OptiPessiCouplingConstraint.cpp
  src/core/OptiPessiAugmentedCost.cpp
  src/core/OptiPessiObstacleConstraint.cpp
)
target_include_directories(legged_opti_pessi_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Fixed: Removed keyword visibility modifier to enforce plain signature compatibility
target_link_libraries(legged_opti_pessi_mpc_core ${cpp_typesupport_target})

# Fixed: Removed 'PUBLIC' keyword parameter
ament_target_dependencies(legged_opti_pessi_mpc_core
  ocs2_core
  ocs2_mpc
)


# ==========================================
# 3. MPC Node Executable
# ==========================================
add_executable(legged_opti_pessi_mpc_node src/nodes/legged_opti_pessi_mpc_node.cpp)
target_include_directories(legged_opti_pessi_mpc_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Fixed: Removed keyword visibility modifier
target_link_libraries(legged_opti_pessi_mpc_node legged_opti_pessi_mpc_core ${cpp_typesupport_target})

# Fixed: Removed 'PRIVATE' keyword parameter
ament_target_dependencies(legged_opti_pessi_mpc_node
  rclcpp
  ocs2_ros_interfaces
  ocs2_sqp
  nav_msgs
  ocs2_msgs
)


# ==========================================
# 4. Fetch External Dependencies (qpOASES)
# ==========================================
include(FetchContent)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable" FORCE)

FetchContent_Declare(qpOASES
  GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f
)
FetchContent_MakeAvailable(qpOASES)


# ==========================================
# 5. WBC Controller Node
# ==========================================
# add_executable(wbc_controller_node
#   src/nodes/wbc_controller_node.cpp
#   src/core/OptiPessiWbc.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/../legged_wbc/src/WbcBase.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/../legged_wbc/src/HoQp.cpp
# )

# target_include_directories(wbc_controller_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include/opti_pessi_control/nodes
#   ${CMAKE_CURRENT_SOURCE_DIR}/../legged_wbc/include
#   ${qpOASES_SOURCE_DIR}/include
# )
# # Fixed: Removed keyword visibility modifier
# target_link_libraries(wbc_controller_node qpOASES ${cpp_typesupport_target})

# # Fixed: Removed 'PRIVATE' keyword parameter
# ament_target_dependencies(wbc_controller_node
#   rclcpp
#   ocs2_msgs
#   unitree_go
#   nav_msgs
#   std_msgs
#   pinocchio
#   ocs2_centroidal_model
#   ocs2_pinocchio_interface
#   ocs2_legged_robot
# )


# ==========================================
# 6. State Estimator Node
# ==========================================
# add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
# target_include_directories(state_estimator_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# # Fixed: Removed keyword visibility modifier
# target_link_libraries(state_estimator_node ${cpp_typesupport_target})

# # Fixed: Removed 'PRIVATE' keyword parameter
# ament_target_dependencies(state_estimator_node
#   rclcpp
#   nav_msgs
#   unitree_go
# )


# ==========================================
# 7. Installation Configurations
# ==========================================
install(TARGETS
  legged_opti_pessi_mpc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  legged_opti_pessi_mpc_node
  # wbc_controller_node
  # state_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/visualize_mpc.py
  # scripts/state_estimator_mock.py
  scripts/mpc_interactor_gui.py
  scripts/mpc_to_lowcmd.py
  scripts/isaac_go2_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(legged_opti_pessi_mpc_core)

ament_package()