cmake_minimum_required(VERSION 3.0.2)
project(decision_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(decision src/main.cpp)
target_link_libraries(decision ${catkin_LIBRARIES})


