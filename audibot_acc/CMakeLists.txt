cmake_minimum_required(VERSION 3.0.2)
project(audibot_acc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(audibot_acc
  src/audibot_acc.cpp
)
target_link_libraries(audibot_acc
  ${catkin_LIBRARIES}
)
