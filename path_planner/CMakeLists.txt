cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  trajectory_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})



include_directories(include/${PROJECT_NAME}
                    ${catkin_INCLUDE_DIRS})

add_library(trajectory_planner_lib src/IK_mod.cpp src/trajectory_generation.cpp src/robot.cpp src/Quaternion.cpp)




target_link_libraries(trajectory_planner_lib ${catkin_LIBRARIES}
${Boost_LIBRARIES}
${GSTREAMER_LIBRARIES})
# BUILD

catkin_package(
               DEPENDS Eigen
               INCLUDE_DIRS include
               LIBRARIES trajectory_planner_lib)
               
add_executable(robot_node src/robot_class_test.cpp)
target_link_libraries(robot_node trajectory_planner_lib )