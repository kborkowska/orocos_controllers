cmake_minimum_required(VERSION 2.8.3)

project(internal_space_spline_trajectory_action)

find_package(catkin REQUIRED COMPONENTS rtt_ros trajectory_msgs control_msgs)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

orocos_component(${PROJECT_NAME} src/InternalSpaceSplineTrajectoryAction.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
orocos_generate_package()
