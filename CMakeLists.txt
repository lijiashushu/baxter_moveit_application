cmake_minimum_required(VERSION 2.8.3)
project(dual_rrt_star)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


#set(CMAKE_MODULE_PATH "/opt/ros/kinetic/share/")
	
find_package(moveit_core REQUIRED)
message("moveit_core_INCLUDE_DIRS: " ${moveit_core_INCLUDE_DIRS})
set(CMAKE_MODULE_PATH "/home/lijiashushu/git/flann-1.8.4-src/cmake")
find_package(Flann REQUIRED)
find_package(catkin REQUIRED COMPONENTS

    roscpp
    rospy
    std_msgs
    sensor_msgs
    kdl_parser
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    orocos_kdl
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_moveit_application
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(one_plan_node src/one_plan.cpp src/dual_cbirrt_star.cpp  include/dual_rrt_star/dual_cbirrt_star.h)
target_link_libraries(one_plan_node ${catkin_LIBRARIES})

add_executable(multi_plan_node src/multi_plan.cpp src/dual_cbirrt_star.cpp  include/dual_rrt_star/dual_cbirrt_star.h)
target_link_libraries(multi_plan_node ${catkin_LIBRARIES})


add_executable(minimum_jerk_node src/minimum_jerk_test.cpp )
target_link_libraries(minimum_jerk_node ${catkin_LIBRARIES}
        ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase blas ma27 gfortran f2c )
