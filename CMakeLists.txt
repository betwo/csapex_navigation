cmake_minimum_required(VERSION 2.8.3)
project(csapex_navigation)

find_package(catkin REQUIRED COMPONENTS
  csapex_ros
  csapex_transform
  csapex
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csapex_navigation
#  CATKIN_DEPENDS csaoex_transform csapex roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_navigation
  src/goto.cpp
  src/approach_pose.cpp
)
target_link_libraries(csapex_navigation
  ${catkin_LIBRARIES}
  ${APEX_LIBS}
)

