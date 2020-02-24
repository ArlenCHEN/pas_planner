cmake_minimum_required(VERSION 2.8.3)
project(rt_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(rt_planner_node src/rt_planner_node.cpp)

target_link_libraries(rt_planner_node
  ${catkin_LIBRARIES}
)
