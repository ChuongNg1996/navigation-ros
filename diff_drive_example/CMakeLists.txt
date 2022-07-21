cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_example)

find_package(catkin REQUIRED roscpp std_msgs sensor_msgs nav_msgs geometry_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs
 DEPENDS Eigen3
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
)


add_executable(ekf_sensor_fusion_sim src/ekf_sensor_fusion_sim.cpp)
add_dependencies(ekf_sensor_fusion_sim  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_sensor_fusion_sim  ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(set_goals_1 src/set_goals_1.cpp)
add_dependencies(set_goals_1  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_goals_1 ${catkin_LIBRARIES})
