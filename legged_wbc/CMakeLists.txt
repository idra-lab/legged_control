cmake_minimum_required(VERSION 3.10)
project(legged_wbc)

set(CMAKE_EXPORT_CoMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

set(AMENT_DEPENDENCIES
        tf2
        tf2_geometry_msgs
        ocs2_legged_robot
        ocs2_sqp
        ocs2_ipm
        pinocchio
        qpoases_cmake
)

foreach(Dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")


###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Legged robot interface library
add_library(${PROJECT_NAME} SHARED
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

ament_export_include_directories(include)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(ocs2_legged_robot)
ament_export_dependencies(ocs2_sqp)
ament_export_dependencies(ocs2_ipm)
ament_export_dependencies(pinocchio)
ament_export_dependencies(qpoases_cmake)

ament_export_libraries(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
