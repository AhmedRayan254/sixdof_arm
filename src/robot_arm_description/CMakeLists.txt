cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

# This package contains no compiled code, only description and config files.
# ament_cmake is still required to make it a valid ROS 2 package.
find_package(ament_cmake REQUIRED)

# Copy these directories into install/robot_arm_description/share/
# Without this, ros2 launch and xacro cannot find our files, even though
# you can plainly see them sitting in src/.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
