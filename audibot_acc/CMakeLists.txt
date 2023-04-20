cmake_minimum_required(VERSION 3.0.2)
project(audibot_acc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
)

find_package(OpenCV REQUIRED)

  catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(audibot_acc
  src/audibot_acc.cpp
)
target_link_libraries(audibot_acc
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
