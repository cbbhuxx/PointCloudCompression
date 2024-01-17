cmake_minimum_required(VERSION 3.0.2)
project(point_sub)


find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # pcl_ros
  # sensor_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    # sensor_msgs
    std_msgs
  # DEPENDS Open3D
)


## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(subscriber
  src/subsrciber.cpp
)

target_link_libraries(subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES} )


