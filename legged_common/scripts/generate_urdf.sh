#!/usr/bin/env bash
# URDF file for the controllers (urdfFile parameter) from the robot xacro:
#   generate_urdf.sh <xacro> <robot_type> [camera] [lidar] [nuc] [slam_box] [output]
# camera, lidar: true by default; nuc, slam_box: false. output: /tmp/legged_control/<robot_type>.urdf by default.
# Prints the path of the file written, without a newline, for $(command ...) in a launch file. The arguments from
# --ros-args on, which a launch <node> appends, are ignored.
set -e
args=()
for a in "$@"; do
  [ "$a" = "--ros-args" ] && break
  args+=("$a")
done
robot_type=${args[1]}
out=${args[6]:-/tmp/legged_control/${robot_type}.urdf}
mkdir -p "$(dirname "$out")"
# written aside and renamed: a controller reading the file never sees half of it
ros2 run xacro xacro "${args[0]}" robot_type:="$robot_type" camera:="${args[2]:-true}" lidar:="${args[3]:-true}" \
  nuc:="${args[4]:-false}" slam_box:="${args[5]:-false}" > "$out.tmp"
mv "$out.tmp" "$out"
printf '%s' "$out"
