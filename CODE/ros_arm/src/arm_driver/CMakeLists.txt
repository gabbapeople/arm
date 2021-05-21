cmake_minimum_required(VERSION 3.0.2)
project(arm_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(CATKIN_DEPENDS
  serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_driver src/arm_driver.cpp)
target_link_libraries(arm_driver ${catkin_LIBRARIES})
