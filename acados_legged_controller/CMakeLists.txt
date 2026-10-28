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

include_directories(
  include
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/acados
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  c_generated_code
)

link_directories(
  ${ACADOS_DIR}/lib
  # ${CMAKE_CURRENT_COURCE_DIR}/c_generated_code
)



# Creation of the wrapper lib
add_library(mpc_solver SHARED
  src/MpcSolver.cpp
  ${ACADOS_C_SOURCES}
)

# Link to acados
target_link_libraries(mpc_solver
  acados
  blasfeo
  hpipm
  # ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/libacados_ocp_solver_ocp_lipm_opti_pessi_0e58481e.so
)


# Export and installation
# EXECUTABLE TESTS
add_executable(test_solver src/test_solver.cpp)

# Link the executable to wrapper library
target_link_libraries(test_solver mpc_solver)


# Intall the wrapper lib
install(TARGETS mpc_solver
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


ament_package()