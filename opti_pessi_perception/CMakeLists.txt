cmake_minimum_required(VERSION 3.16)
project(opti_pessi_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(legged_controllers REQUIRED)  # only for its Obstacle/ObstacleArray messages
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)

# YOLOs-CPP is header-only. Its own CMakeLists only builds demos, so the checkout carries COLCON_IGNORE.
set(YOLOS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../YOLOs-CPP" CACHE PATH "YOLOs-CPP checkout")
set(ONNXRUNTIME_DIR "${YOLOS_CPP_DIR}/onnxruntime-linux-x64-gpu-1.20.1" CACHE PATH "ONNX Runtime release (include/, lib/)")
get_filename_component(YOLOS_CPP_DIR "${YOLOS_CPP_DIR}" ABSOLUTE)
get_filename_component(ONNXRUNTIME_DIR "${ONNXRUNTIME_DIR}" ABSOLUTE)
if(NOT EXISTS "${YOLOS_CPP_DIR}/include/yolos/tasks/detection.hpp")
  message(FATAL_ERROR "YOLOs-CPP not found in ${YOLOS_CPP_DIR}: pass -DYOLOS_CPP_DIR=<checkout>")
endif()
if(NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h" OR NOT EXISTS "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
  message(FATAL_ERROR "ONNX Runtime not found in ${ONNXRUNTIME_DIR}: pass -DONNXRUNTIME_DIR=<release folder>")
endif()
# The release's lib/cmake/onnxruntime config points at lib64/ and include/onnxruntime/, which the tarball does not have.
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_DIR}/include"
)
message(STATUS "YOLOs-CPP: ${YOLOS_CPP_DIR}")
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")

###########
## Build ##
###########

add_executable(yolo26_rgbd_detect
  src/yolo26_rgbd_detect.cpp
  src/Yolo26RGBD.cpp
)

target_include_directories(yolo26_rgbd_detect PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
# SYSTEM: keep the YOLOs-CPP headers' warnings out of this package's build.
target_include_directories(yolo26_rgbd_detect SYSTEM PRIVATE "${YOLOS_CPP_DIR}/include")

ament_target_dependencies(yolo26_rgbd_detect
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(yolo26_rgbd_detect
  legged_controllers::legged_controllers__rosidl_generator_cpp
  legged_controllers::legged_controllers__rosidl_typesupport_cpp
  ${OpenCV_LIBS}
  onnxruntime
)

target_compile_options(yolo26_rgbd_detect PRIVATE -Wall -Wextra)

# libonnxruntime.so is not on the loader path: find it (and its CUDA provider next to it) through the RPATH.
set_target_properties(yolo26_rgbd_detect PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
)

#############
## Install ##
#############

install(TARGETS yolo26_rgbd_detect
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# The node looks up a relative `model`/`labels` here first. Only what it can load: the .pt training weights stay in the source.
install(DIRECTORY weights/
  DESTINATION share/${PROJECT_NAME}/weights
  FILES_MATCHING PATTERN "*.onnx" PATTERN "*.names" PATTERN "*.txt"
)

ament_package()
