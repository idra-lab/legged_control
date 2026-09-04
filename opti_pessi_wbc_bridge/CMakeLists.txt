cmake_minimum_required(VERSION 3.10)
project(opti_pessi_wbc_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
# ocs2_legged_robot transitively exports targets (via ocs2_sqp/ocs2_ipm/hpipm_catkin) that link
# against Boost::system; that imported target only exists once Boost is found here too (see
# legged_wbc/CMakeLists.txt for the same requirement).
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)

set(AMENT_DEPENDENCIES
        ocs2_core
        ocs2_centroidal_model
        ocs2_pinocchio_interface
        ocs2_legged_robot
        opti_pessi_interface
        pinocchio
)
foreach(Dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# NOTE: remaining sources are added by later tasks; the full set will be:
#   src/LegKinematics.cpp   (Task 3)
#   src/PhaseClock.cpp
#   src/PlanBuffer.cpp
#   src/StateFolder.cpp
#   src/ReferenceSynthesizer.cpp
add_library(${PROJECT_NAME} SHARED
        src/LegKinematics.cpp
        src/PhaseClock.cpp
        src/PlanBuffer.cpp
        src/StateFolder.cpp
        src/ReferenceSynthesizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PUBLIC
        -Wno-ignored-attributes
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)
# NOTE: "pinocchio" is deliberately excluded from the ament_target_dependencies() call below and
# linked separately via its modern imported target instead -- see the matching, more detailed
# comment at the test target below for why (dual conflicting pinocchio installs in this
# workspace; ament_target_dependencies() falls back to the legacy pinocchio_LIBRARIES variable,
# which resolves to the wrong (ROS jazzy) copy and segfaults at runtime when linked alongside
# openrobots' pinocchio::pinocchio).
set(LIB_AMENT_DEPENDENCIES ${AMENT_DEPENDENCIES})
list(REMOVE_ITEM LIB_AMENT_DEPENDENCIES pinocchio)
ament_target_dependencies(${PROJECT_NAME} ${LIB_AMENT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_URDF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/aliengo.urdf")
  ament_add_gtest(test_${PROJECT_NAME}
          test/testLegIndexing.cpp
          test/testModelOrdering.cpp
          test/testLegKinematics.cpp
          test/testPhaseClock.cpp
          test/testPlanBuffer.cpp
          test/testStateFolder.cpp
          test/testReferenceSynthesizer.cpp
  )
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
          OPTI_PESSI_BRIDGE_TEST_URDF="${TEST_URDF_PATH}")
  target_include_directories(test_${PROJECT_NAME} PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  # NOTE: "pinocchio" is deliberately excluded from the ament_target_dependencies() call below and
  # linked separately via its modern imported target instead. This workspace has two conflicting
  # pinocchio installs (ROS jazzy's own package and /opt/openrobots, see the colcon build command in
  # the task brief). find_package(pinocchio) above correctly resolves to openrobots because of that
  # -Dpinocchio_DIR override, but openrobots' pinocchioConfig.cmake only exposes the resolved
  # location through the legacy pinocchio_LIBRARIES variable via an unqualified find_library() call
  # -- which, independent of pinocchio_DIR, prefers whatever CMAKE_PREFIX_PATH turns up first, i.e.
  # ROS jazzy's copy. ament_target_dependencies() falls back to that legacy variable for any package
  # without a *_TARGETS variable (pinocchio has none), so passing "pinocchio" through it silently
  # links both incompatible pinocchio_default/pinocchio_parsers ABIs into the same binary, which
  # segfaults at runtime the moment Pinocchio code actually executes (e.g. createPinocchioInterface).
  # The pinocchio::pinocchio imported target, by contrast, carries its dependent libraries'
  # IMPORTED_LOCATION baked in directly from its own Config directory, so it always resolves to
  # openrobots consistently. The ${PROJECT_NAME} library target above (Task 3) receives the same
  # treatment via LIB_AMENT_DEPENDENCIES.
  set(TEST_AMENT_DEPENDENCIES ${AMENT_DEPENDENCIES})
  list(REMOVE_ITEM TEST_AMENT_DEPENDENCIES pinocchio)
  ament_target_dependencies(test_${PROJECT_NAME} ${TEST_AMENT_DEPENDENCIES})
  target_link_libraries(test_${PROJECT_NAME} pinocchio::pinocchio)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${AMENT_DEPENDENCIES})
ament_package()
