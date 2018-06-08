cmake_minimum_required(VERSION 2.8.3)
project(trapezoidal_trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation trajectory_msgs control_msgs 
std_msgs)

add_action_files(
  DIRECTORY action 
  FILES 
  TrapezoidTrajectory.action
)

add_message_files(
  DIRECTORY msg
  FILES
  TrapezoidGeneratorResult.msg
  TrapezoidGeneratorGoal.msg
)

generate_messages(DEPENDENCIES control_msgs std_msgs trajectory_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime control_msgs std_msgs trajectory_msgs 
)
