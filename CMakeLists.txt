cmake_minimum_required(VERSION 2.8.3)
project(pcd2csv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pcd2csv
 CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

link_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

