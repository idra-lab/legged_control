cmake_minimum_required(VERSION 3.12)
project(legged_unitree_description)

# Find ament_cmake instead of catkin
find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

# Mark resource files for installation into the ROS 2 share directory
install(
        DIRECTORY meshes urdf launch
        DESTINATION share/${PROJECT_NAME}
)

# Must be invoked at the very end of the CMake lists file
ament_package()