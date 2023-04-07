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

add_executable(final_project
  src/gps_example.cpp
)
target_link_libraries(final_project
  ${catkin_LIBRARIES}
)