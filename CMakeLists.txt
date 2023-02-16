cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

# INCLUDE_DIRS include/${PROJECT_NAME}/
#  LIBRARIES traineeship
#  CATKIN_DEPENDS franka_ros moveit panda_moveit_config roscpp std_msgs
#  DEPENDS
#    roscpp
#    gazebo_ros
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)