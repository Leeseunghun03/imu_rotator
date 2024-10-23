cmake_minimum_required(VERSION 3.0.2)
project(imu_rotator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_geometry_msgs
  Eigen3
  slam_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/imu_rotator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)