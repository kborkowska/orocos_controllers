cmake_minimum_required(VERSION 2.8.3)

project(internal_space_spline_trajectory_generator_limits)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rtt_ros trajectory_msgs std_msgs
trapezoidal_trajectory_msgs)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


orocos_component(${PROJECT_NAME} src/InternalSpaceSplineTrajectoryGeneratorLimits.cpp src/velocityprofile_spline.cpp)

add_dependencies(${PROJECT_NAME} ${trapezoidal_trajectory_msgs_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
orocos_generate_package()


