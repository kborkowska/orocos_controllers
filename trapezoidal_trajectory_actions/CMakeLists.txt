cmake_minimum_required(VERSION 2.8.3)
project(trapezoidal_trajectory_actions)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib trajectory_msgs  control_msgs)

add_action_files(DIRECTORY action FILES TrapezoidTrajectoryMVMA.action)

generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs control_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib genmsg trajectory_msgs control_msgs
)
