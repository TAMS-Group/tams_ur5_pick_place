cmake_minimum_required(VERSION 2.8.3)
project(PickAndPlace)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(PickAndPlaceTest src/PickAndPlaceTest.cpp)
target_link_libraries(PickAndPlaceTest ${catkin_LIBRARIES})
