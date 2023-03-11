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
find_package(orocos_kdl REQUIRED)
find_package(NLopt REQUIRED)
find_package(CUDA REQUIRED)

message("nlopt dir " ${NLopt_dir})
message("nlopt dir " ${NLopt_dir})
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
set(NLopt_INCLUDE_DIRS "/usr/local/include")
set(NLopt_LIBRARIES "/usr/local/lib/libnlopt.so")
message("nlopt path " ${NLopt_INCLUDE_DIRS})


add_executable(one_plan_node
        src/node/one_plan.cpp
        src/path_planner/task_space.cpp
        src/path_planner/joint_space.cpp
        src/trajectory_planner/minimum_jerk_planner/minimum_jerk_planner.cpp
        src/trajectory_planner/time_dual_constraint_planner/time_dual_constraint_planner.cpp
        src/trajectory_planner/time_dual_constraint_planner/trapezoid_profile.cpp
        )

target_link_libraries(one_plan_node ${catkin_LIBRARIES} ${NLopt_LIBRARIES}
        ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase blas ma27 gfortran f2c )

#path planner compare test
add_executable(compare_node
        src/node/path_planner_comparison.cpp
        src/path_planner/task_space.cpp
        )
target_link_libraries(compare_node ${catkin_LIBRARIES})

add_executable(joint_space_compare_node
        src/node/joint_space_path_planner.cpp
        src/path_planner/joint_space.cpp
        )
target_link_libraries(joint_space_compare_node ${catkin_LIBRARIES})

# trajectory planner multi experiments
add_executable(traj_multi_experiment_node
        src/node/trajectory_generator.cpp
        src/path_planner/task_space.cpp
        src/trajectory_planner/time_dual_constraint_planner/time_dual_constraint_planner.cpp
        src/trajectory_planner/time_dual_constraint_planner/trapezoid_profile.cpp
        )

target_link_libraries(traj_multi_experiment_node ${catkin_LIBRARIES} ${NLopt_LIBRARIES}
        ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase blas ma27 gfortran f2c )

#add_executable(multi_plan_node src/multi_plan.cpp src/dual_cbirrt_star.cpp  include/dual_rrt_star/dual_cbirrt_star.h)
#target_link_libraries(multi_plan_node ${catkin_LIBRARIES})


add_executable(time_dual_constraint_test_node
        src/node/time_dual_constraint_planner_test.cpp
        src/trajectory_planner/time_dual_constraint_planner/time_dual_constraint_planner.cpp
        src/trajectory_planner/time_dual_constraint_planner/trapezoid_profile.cpp)

target_link_libraries(time_dual_constraint_test_node ${catkin_LIBRARIES} ${NLopt_LIBRARIES}
        ooqpgensparse
        ooqpsparse
        ooqpgondzio
        ooqpbase blas ma27 gfortran f2c )


add_executable(moveit_planner
        src/node/using_moveit_planner.cpp)
target_link_libraries(moveit_planner ${catkin_LIBRARIES})

add_executable(main src/node/main.cpp)
#add_executable(trapezoid_profile_node src/trapezoid_profile.cpp)