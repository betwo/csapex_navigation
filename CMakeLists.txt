cmake_minimum_required(VERSION 2.8.3)
project(csapex_navigation)

find_package(catkin REQUIRED COMPONENTS
  csapex_ros
  csapex_transform
  csapex
  roscpp
  path_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES csapex_navigation
#  CATKIN_DEPENDS csaoex_transform csapex roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_navigation
  src/stop_movement.cpp
  src/follow_path.cpp
  src/plan_path.cpp
  src/plan_path_with_map.cpp
  src/goto.cpp
  src/approach_pose.cpp
  src/approach_pose_trailer.cpp
  src/backup.cpp
)
target_link_libraries(csapex_navigation
  ${catkin_LIBRARIES}
  ${APEX_LIBS}
)
add_dependencies(csapex_navigation ${catkin_EXPORTED_TARGETS})


add_library(csapex_navigation_core
  src/register_navigation.cpp
)
target_link_libraries(csapex_navigation_core
  ${catkin_LIBRARIES}
  ${APEX_LIBS}
)
add_dependencies(csapex_navigation_core ${catkin_EXPORTED_TARGETS})
