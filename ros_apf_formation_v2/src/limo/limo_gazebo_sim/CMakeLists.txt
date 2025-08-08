cmake_minimum_required(VERSION 2.8.3)
project(limo_gazebo_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf
  gazebo_dev
  gazebo_msgs
  message_filters
)
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES  gazebo_ros_ackerman
  # CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(odom_pub src/odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})
add_dependencies(odom_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_pub_test src/odom_pub_test.cpp)
target_link_libraries(odom_pub_test ${catkin_LIBRARIES})
add_dependencies(odom_pub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(gazebo_ros_ackerman src/gazebo_ros_ackerman_drive.cpp)
# target_link_libraries(gazebo_ros_ackerman ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# install(TARGETS gazebo_ros_ackerman
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

