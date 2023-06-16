cmake_minimum_required(VERSION 3.0.2)
project(tcp_package)

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

add_executable(tcp_server src/tcp_server.cpp)
target_link_libraries(tcp_server ${catkin_LIBRARIES})

add_executable(tcp_client src/tcp_client.cpp)
target_link_libraries(tcp_client ${catkin_LIBRARIES})

