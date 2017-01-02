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
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS csaoex_transform csapex roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/set_lookat_point.cpp
  src/select_closest_pose.cpp
  src/move_robot.cpp
  src/stop_movement.cpp
  src/follow_path.cpp
  src/plan_path.cpp
  src/plan_path_with_map.cpp
  src/goto.cpp
  src/approach_pose.cpp
  src/approach_pose_trailer.cpp
  src/backup.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${APEX_LIBS}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_library(${PROJECT_NAME}_core
  src/register_navigation.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${APEX_LIBS}
)
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
