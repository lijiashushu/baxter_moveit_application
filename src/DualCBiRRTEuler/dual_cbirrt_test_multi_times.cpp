#include <ros/ros.h>
#include <baxter_moveit_application/DualCBiRRTEuler/dual_cbirrt.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <algorithm>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>

#include <fstream>

#include <stdlib.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "cbirrt_test");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

    std::vector<double> test_start_value1 = {-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> slave_test_start_value1 = {-0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> test_start_value2 = {0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722};
    std::vector<double> slave_test_start_value2 = {-0.517204,-0.49348,1.79496,1.66956,0.302716,0.602833,-2.87906};
    std::vector<double> test_start_value3 = {0.75793,-0.888533,-2.01998,2.09217,-0.226114,-0.224376,2.39712};
    std::vector<double> slave_test_start_value3 = {-0.633959,-0.671778,2.03325,2.11988,0.716528,-0.334106,-3.05835};
    std::vector<double> test_start_value4 = {0.119411,-0.743597,-2.05219,2.14562,-0.840575,-1.39347,3.05791};
    std::vector<double> slave_test_start_value4 = {-0.119205,-0.74178,2.05281,2.14571,0.840017,-1.39411,-3.058};

    std::vector<std::vector<double>> four_start_master;
    four_start_master.push_back(test_start_value1);
    four_start_master.push_back(test_start_value2);
    four_start_master.push_back(test_start_value3);
    four_start_master.push_back(test_start_value4);
    std::vector<std::vector<double>> four_start_slave;
    four_start_slave.push_back(slave_test_start_value1);
    four_start_slave.push_back(slave_test_start_value2);
    four_start_slave.push_back(slave_test_start_value3);
    four_start_slave.push_back(slave_test_start_value4);


    std::vector<double> test_goal_value1 = {0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> slave_test_goal_value1 = {-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> test_goal_value2 = {0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696,3.05881};
    std::vector<double> slave_test_goal_value2 = {-0.640635,0.334855,1.43792,2.03636,-0.432353,0.425846,-3.05814};
    std::vector<double> test_goal_value3 = {0.378964,-0.235001,-1.09593,2.54013,0.629455,-0.946823,2.56304};
    std::vector<double> slave_test_goal_value3 = {-0.495711,0.18897,1.20076,2.51116,-0.470866,-0.757521,-3.05796};
    std::vector<double> test_goal_value4 = {0.188932,1.04619,-1.20316,2.47039,0.862578,-1.41834,-2.53144};
    std::vector<double> slave_test_goal_value4 = {-0.108729,1.0458,1.23816,2.45137,-0.881861,-1.45554,2.53844};

    std::vector<std::vector<double>> four_goal_master;
    four_goal_master.push_back(test_goal_value1);
    four_goal_master.push_back(test_goal_value2);
    four_goal_master.push_back(test_goal_value3);
    four_goal_master.push_back(test_goal_value4);
    std::vector<std::vector<double>> four_goal_slave;
    four_goal_slave.push_back(slave_test_goal_value1);
    four_goal_slave.push_back(slave_test_goal_value2);
    four_goal_slave.push_back(slave_test_goal_value3);
    four_goal_slave.push_back(slave_test_goal_value4);

    robot_model::RobotModelConstPtr robot_model_ptr = planning_scene_for_operate->getRobotModel();
    robot_state::RobotState robot_state = planning_scene_for_operate->getCurrentStateNonConst();
    const robot_state::JointModelGroup* planning_group = robot_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = robot_state.getJointModelGroup("right_arm"); //
    robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
    robot_state::RobotState goal_state = planning_scene_for_operate->getCurrentStateNonConst();


    std::ofstream out_file1[4];
    std::ofstream out_file2[4];


    for(size_t i=0; i<4; i++){
        out_file1[i].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/performance/no_try_fix_pose" + std::to_string(i) +".csv", std::ios::out | std::ios::trunc);
        out_file1[i]
                <<"planning_time"<<","
                <<"_seed"<<","
                <<"sample_counts"<<","
                <<"extend_try"<<","
                <<"constraint_project_success"<<","
                <<"constraint_project_success_rate"<<","
                <<"ik_success"<<","
                <<"ik_success_rate"<<","
                <<"extend_success"<<","
                <<"extend_success_rate"<<","
                <<"average_success_constraint_project_compute_times"<<","
                <<"average_ik_compute_times"<<","
                <<"average_success_ik_compute_times"<<","
                <<"total_sample_time"<<","
                <<"total_extend_time"<<","
                <<"total_project_time"<<","
                <<"total_ik_time"<<","
                <<"average_sample_time"<<","
                <<"average_extend_time"<<","
                <<"average_extend_project_time"<<","
                <<"average_extend_ik_time"<<","
                <<"average_extend_one_project_time"<<","
                <<"average_extend_one_ik_time"<<","
                <<std::endl;

        out_file2[i].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/performance/yes_try_fix_pose" + std::to_string(i) +".csv", std::ios::out | std::ios::trunc);
        out_file2[i]
                <<"planning_time"<<","
                <<"_seed"<<","
                <<"sample_counts"<<","
                <<"extend_try"<<","
                <<"constraint_project_success"<<","
                <<"constraint_project_success_rate"<<","
                <<"ik_success"<<","
                <<"ik_success_rate"<<","
                <<"extend_success"<<","
                <<"extend_success_rate"<<","
                <<"average_success_constraint_project_compute_times"<<","
                <<"average_ik_compute_times"<<","
                <<"average_success_ik_compute_times"<<","
                <<"total_sample_time"<<","
                <<"total_extend_time"<<","
                <<"total_project_time"<<","
                <<"total_ik_time"<<","
                <<"average_sample_time"<<","
                <<"average_extend_time"<<","
                <<"average_extend_project_time"<<","
                <<"average_extend_ik_time"<<","
                <<"average_extend_one_project_time"<<","
                <<"average_extend_one_ik_time"<<","
                <<std::endl;

    }

    int seed_nums = 1;
    std::ofstream out_file3[seed_nums][4];
    for(size_t i=0; i<seed_nums;i++){
        for(size_t j=0; j<4; j++){
            out_file3[i][j].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/extend_success_rate/pose"+std::to_string(i) + std::to_string(j) +".txt", std::ios::out | std::ios::trunc);
        }
    }

    std::ofstream out_file4[seed_nums][4];
    for(size_t i=0; i<seed_nums;i++){
        for(size_t j=0; j<4; j++){
            out_file4[i][j].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/manipulability/pose"+std::to_string(i) + std::to_string(j) +".txt", std::ios::out | std::ios::trunc);
        }
    }
    std::ofstream out_file5[seed_nums][4];
    for(size_t i=0; i<seed_nums;i++){
        for(size_t j=0; j<4; j++){
            out_file5[i][j].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/rate_manipulability/pose"+std::to_string(i) + std::to_string(j) +".txt", std::ios::out | std::ios::trunc);
        }
    }

    std::ofstream out_file6[seed_nums][4];
    for(size_t i=0; i<seed_nums;i++){
        for(size_t j=0; j<4; j++){
            out_file6[i][j].open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/reached_index/pose"+std::to_string(i) + std::to_string(j) +".txt", std::ios::out | std::ios::trunc);
        }
    }

    std::srand((unsigned)time(NULL));
    Eigen::Matrix<double ,15,1> rand_seeds;
    rand_seeds<<
              1607533959,
    711080370,
    1054425021,
    145545320,
    2081222915,
    757109597,
    1426350675,
    1779244099,
    2045518616,
    1343972192,
    120320971,
    1048582536,
    2000454500,
    581759900,
    1433231943;


    ros::Time time_1;
    ros::Time time_2;
    for(size_t ii = 0; ii<seed_nums; ii++){
//        int seed = std::rand();
        int seed = rand_seeds[ii];
//        for(size_t jj=0; jj<4; jj++){
//            DualCBiRRT my_planner1(1.0, seed, 0.00, jj, planning_scene_for_operate, planning_group, slave_group);
//
//            start_state.setJointGroupPositions(planning_group, four_start_master[jj]);
//            start_state.setJointGroupPositions(slave_group, four_start_slave[jj]);
//
//            goal_state.setJointGroupPositions(planning_group, four_goal_master[jj]);
//            goal_state.setJointGroupPositions(slave_group, four_goal_slave[jj]);
//            time_1 = ros::Time::now();
//            if(my_planner1.plan_task_space_dir(goal_state, start_state)){
//                ROS_INFO("my_planner1 pose %d success at time %d", (int)jj, (int)ii);
//            }
//            else{
//                ROS_INFO("my_planner1 pose %d fail at time %d", (int)jj, (int)ii);
//            }
//            time_2 = ros::Time::now();
//            out_file1[jj]<<(time_2 - time_1).toSec()<<",";
//            my_planner1.output_perdex_multi(out_file1[jj]);
//            my_planner1.output_new_manipulability(out_file4[ii][jj]);
////            my_planner1.output_extend_success_rate(out_file3[ii][jj]);
////            my_planner1.output_simple_manipulability(out_file4[ii][jj]);
////            my_planner1.output_rate_simple_manipulability(out_file5[ii][jj]);
//        }
        for(size_t jj=0; jj<4; jj++){
            DualCBiRRT my_planner2(1.0, seed, 0.00, jj, planning_scene_for_operate, planning_group, slave_group);
            start_state.setJointGroupPositions(planning_group, four_start_master[jj]);
            start_state.setJointGroupPositions(slave_group, four_start_slave[jj]);

            goal_state.setJointGroupPositions(planning_group, four_goal_master[jj]);
            goal_state.setJointGroupPositions(slave_group, four_goal_slave[jj]);
            time_1 = ros::Time::now();
            if(my_planner2.plan_task_space_dir_try_adjust(goal_state, start_state)){
                ROS_INFO("my_planner2 pose %d success at time %d", (int)jj, (int)ii);
            }
            else{
                ROS_INFO("my_planner2 pose %d fail at time %d", (int)jj, (int)ii);
            }
            time_2 = ros::Time::now();
            out_file2[jj]<<(time_2 - time_1).toSec()<<",";
            my_planner2.output_perdex_multi(out_file2[jj]);
            my_planner2.output_extend_success_rate(out_file3[ii][jj]);
            my_planner2.output_new_manipulability(out_file4[ii][jj]);
            my_planner2.output_rate_complex_manipulability(out_file5[ii][jj]);
            my_planner2.output_reached_index(out_file6[ii][jj]);
        }

    }
    for(size_t i=0; i<4; i++){
        out_file1[i].close();
        out_file2[i].close();

    }
    for(size_t i=0; i<seed_nums;i++){
        for(size_t j=0; j<4; j++){
            out_file3[i][j].close();
            out_file4[i][j].close();
            out_file5[i][j].close();
            out_file6[i][j].close();
        }
    }
    return 0;
}