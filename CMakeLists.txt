cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  gazebo_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  tf2_geometry_msgs
  geometric_shapes
  tf2_ros
  tf2_eigen
  actionlib_msgs
  visualization_msgs
  control_msgs
  pluginlib
  tf2
  sensor_msgs
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs visualization_msgs sensor_msgs # Or other packages containing msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# INCLUDE_DIRS include/${PROJECT_NAME}/
#  LIBRARIES ws_moveit
#  CATKIN_DEPENDS franka_ros moveit panda_moveit_config roscpp std_msgs
#  DEPENDS
#    roscpp
#    gazebo_ros


# catkin_package ensures that the libraries and executables created in the package are linked together
catkin_package(
  LIBRARIES
  INCLUDE_DIRS 
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    gazebo_ros
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs
    visualization_msgs
    actionlib_msgs
    control_msgs
    sensor_msgs
  DEPENDS
    EIGEN3
)


include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

# Allows for python scripts to be installed to the bin directory 
catkin_install_python(PROGRAMS scripts/pick_and_place.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Cpp nodes for pick and place
add_executable(pick_and_place scripts/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

