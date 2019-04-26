cmake_minimum_required(VERSION 2.8.3)
project(traxxas)


find_package(catkin REQUIRED COMPONENTS
  joy
  sensor_msgs
  geometry_msgs
  serial
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")


generate_dynamic_reconfigure_options( config/motor.cfg )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traxxas 
  CATKIN_DEPENDS  geometry_msgs nav_msgs joy sensor_msgs roscpp rospy std_msgs tf dynamic_reconfigure
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_service_files(
  FILES
  unitStep.srv
)

 add_executable(WheelOdometry src/wheel_odometry.cpp)
 add_executable(esc_driver src/esc_driver.cpp)
 add_executable(teleop_node src/joy_to_traxxas_node.cpp)


 target_link_libraries(WheelOdometry ${catkin_LIBRARIES})
 target_link_libraries(esc_driver ${catkin_LIBRARIES})
 target_link_libraries(teleop_node ${catkin_LIBRARIES})


