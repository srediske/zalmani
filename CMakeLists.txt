cmake_minimum_required(VERSION 3.0.2)
project(ur_robotiq_connector)

find_package(catkin REQUIRED
COMPONENTS roslaunch rospy
actionlib actionlib_msgs control_msgs gazebo_msgs geometry_msgs std_msgs std_srvs trajectory_msgs)

catkin_package(
CATKIN_DEPENDS actionlib actionlib_msgs control_msgs gazebo_msgs geometry_msgs roslaunch std_msgs std_srvs trajectory_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
