cmake_minimum_required(VERSION 3.12)
project(opti_pessi_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/OptiPessiModelParameters.cpp
  src/LipKinematics.cpp
  src/OptiPessiInterface.cpp
  src/dynamics/OptiPessiDynamicsAD.cpp
  src/cost/OptiPessiCost.cpp
  src/constraint/StageInequalityConstraint.cpp
  src/constraint/TerminalCopConstraint.cpp
  src/constraint/FirstInputConsensusConstraint.cpp
  src/initialization/OptiPessiInitializer.cpp
  src/simulation/ObstaclePlant.cpp
  src/simulation/ClosedLoopSimulation.cpp
  src/simulation/NpyIo.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ocs2_core
  ocs2_oc
  ocs2_ipm
  ocs2_mpc
  ocs2_robotic_tools
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  Boost::system
  Boost::filesystem
  Boost::log_setup
  Boost::log
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Standalone closed-loop simulation node.
set(MPC_EXEC ${PROJECT_NAME}_mpc)
add_executable(${MPC_EXEC} src/OptiPessiMpcNode.cpp)
target_link_libraries(${MPC_EXEC} PRIVATE ${PROJECT_NAME})
ament_target_dependencies(${MPC_EXEC} PUBLIC ocs2_core ocs2_oc ocs2_ipm ocs2_mpc ocs2_robotic_tools)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS ${MPC_EXEC}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS python/plot_mpc.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES python/plot_utils.py python/info_parser.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lip_step test/testLipStep.cpp)
  target_link_libraries(test_lip_step ${PROJECT_NAME})

  # Tests read the .info files straight out of the source tree, so they stay valid before install.
  ament_add_gtest(test_config_loading test/testConfigLoading.cpp)
  target_link_libraries(test_config_loading ${PROJECT_NAME})
  target_compile_definitions(test_config_loading PRIVATE OPTI_PESSI_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  # Generates CppAD libraries and runs a closed loop; needs a generous timeout.
  ament_add_gtest(test_s4_closed_loop test/testS4ClosedLoop.cpp TIMEOUT 600)
  target_link_libraries(test_s4_closed_loop ${PROJECT_NAME})
  target_compile_definitions(test_s4_closed_loop PRIVATE OPTI_PESSI_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_core ocs2_oc ocs2_ipm ocs2_mpc ocs2_robotic_tools Eigen3 Boost)
ament_package()
