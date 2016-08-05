cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_pick_place)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
  tf
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_and_place_test src/pick_and_place_test.cpp)
target_link_libraries(pick_and_place_test ${catkin_LIBRARIES})

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES})
