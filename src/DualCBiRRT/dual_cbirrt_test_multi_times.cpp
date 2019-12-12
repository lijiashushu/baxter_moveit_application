#include <ros/ros.h>
#include <baxter_moveit_application/DualCBiRRT/dual_cbirrt.h>

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

    robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
    //Z:180 Y:90 X:-90   2.94792  1.56999 -1.76536


//    std::vector<double> test_start_value = {0.178307,-1.36637,-0.718743,2.32057,-1.28874,1.62442,2.4651};//有障碍时手臂平方位置
//    std::vector<double> test_start_value = {0.17109754, -0.87923624, -0.08423487,  1.712199,   -0.81049842,  2.09320188,  2.58848987}; //无障碍时手臂平方位置
//    std::vector<double> slave_test_start_value = {0.0633710, 0.118378, 1.5027523, 2.2347026,-0.579105, 0.054547, -1.11615}; //无障碍时手臂平方位置
//    std::vector<double> test_start_value = {0.178307,-1.36637,-0.718743,2.32057,-1.28874,1.62442,2.4651}; //有障碍物测试
//    std::vector<double> slave_test_start_value = {-0.644661 , 0.255123 ,  1.83284 ,  2.19888,  -0.36092  , 0.90258  , -1.1066}; //有障碍物测试
//    std::vector<double> test_start_value = {-0.202391,-1.01283,-0.709538,1.16068,-1.21936,1.51294,1.59967}; //narrow障碍物的起始左臂位置
//    std::vector<double> slave_test_start_value = {-0.0273947,-0.113638,2.14442,0.981496,-0.31,1.45411,-1.02899};//narrow障碍物的起始右臂位置
//    std::vector<double> test_start_value = {-0.375463,-1.09228,-0.440484,1.20106,1.76769,-1.57028,0.0672226}; //没有奇异、narrow障碍物的起始左臂位置
//    std::vector<double> slave_test_start_value = {-0.115289,-0.393004,1.72106,1.01171,-2.93258,-1.39411,0.332235};//没有奇异、narrow障碍物的起始右臂位置

    std::vector<double> test_start_value = {-0.0137274,-0.648781,-1.1192,0.880775,2.37693,-1.56809,-0.11713}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体
    std::vector<double> slave_test_start_value = {-0.0361471,-0.345621,1.6738,0.869939,-2.96241,-1.47801,0.298402};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体


    std::vector<double> both_start_value;
    const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //
    const robot_state::JointModelGroup* both_group = start_state.getJointModelGroup("both_arms"); //

    start_state.setJointGroupPositions(planning_group, test_start_value);
    start_state.setJointGroupPositions(slave_group, slave_test_start_value);
    start_state.copyJointGroupPositions(both_group, both_start_value);
    Eigen::Matrix<double ,14, 1 >both_start_value_matrix;
    for(size_t i=0; i<14; i++){
        both_start_value_matrix[i] = both_start_value[i];
    }
    std::cout<<"both_start_value_matrix\n"<<both_start_value_matrix.transpose()<<std::endl;


    planning_scene_for_operate->setCurrentState(start_state);
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(1).sleep();


    robot_state::RobotState goal_state = planning_scene_for_operate->getCurrentStateNonConst();


//    std::vector<double> test_goal_value = {-0.53121395, -1.14663671 , 0.21698349  ,2.33939883 ,-1.17448029  ,1.81105335,  2.82284528};//无障碍时手臂平方的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.64966, 0.0056597, 1.453030, 2.2167859, 0.0142739, 0.7887366, -1.69753346};//无障碍时手臂平方的目标右臂位置

//    std::vector<double> test_goal_value = {0.0511426,-0.422846,-0.602817,1.92707,-0.888771,1.20479,2.70597}; //平板类似桌子的障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.614005,  0.611334 ,  1.40829,   1.80571, -0.631447,   1.11582,  -1.56488}; //平板类似桌子的障碍物的目标右臂位置
//    std::vector<double> test_goal_value = {-0.0813673,-0.68199,-0.637715,1.9482,-1.13503,1.24992,2.59584};//narrow障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.485412,0.487359,1.66579,1.6767,-0.522427,1.24843,-1.42944};//narrow障碍物的目标右臂位置
//    std::vector<double> test_goal_value = {-0.233357,-0.754374,-0.490762,1.95377,1.90675,-1.34839,1.06295};//没有奇异、narrow障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.446697,-0.0863082,1.24614,1.77273,-2.93228,-1.08041,-0.381265};//没有奇异、narrow障碍物的目标右臂位置

    std::vector<double> test_goal_value = {0.104215,-0.367542,-0.866707,1.42165,2.45671,-1.38136,0.587156};//没有奇异、narrow障碍物的目标左臂位置，大桌子，增加抓取物体
    std::vector<double> slave_test_goal_value = {-0.260604,0.0155082,1.32256,1.4155,-3.02382,-1.21494,-0.264714};//没有奇异、narrow障碍物的目标右臂位置,大桌子，增加抓取物体

    goal_state.setJointGroupPositions(planning_group, test_goal_value);
    goal_state.setJointGroupPositions(slave_group, slave_test_goal_value);

    auto left_test = goal_state.getGlobalLinkTransform("left_gripper");
    auto left_rotation = left_test.rotation();
    auto left_euler = left_rotation.eulerAngles(2, 1, 0);
    auto left_pos = left_test.translation();
    std::cout<<"left_euler\n"<<left_euler.transpose()<<std::endl;
    std::cout<<"left_pos\n"<<left_pos.transpose()<<std::endl;

    auto right_test = goal_state.getGlobalLinkTransform("right_gripper");
    auto right_rotation = right_test.rotation();
    auto right_euler = right_rotation.eulerAngles(2, 1, 0);
    auto right_pos = right_test.translation();
    std::cout<<"right_euler\n"<<right_euler.transpose()<<std::endl;
    std::cout<<"right_pos\n"<<right_pos.transpose()<<std::endl;

    Eigen::Vector3d slave_goal_euler(left_euler[0], left_euler[1] + 3.1415926, left_euler[2] - 3.1415926);
    std::cout<<"slave_goal_euler\n"<<slave_goal_euler.transpose()<<std::endl;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    Eigen::Vector3d slave_goal_euler2 = slave_goal_rot_matrix.eulerAngles(2,1,0);
    std::cout<<"slave_goal_euler2\n"<<slave_goal_euler2.transpose()<<std::endl;

    std::ofstream out_file1;
    out_file1.open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/task_no_try_fix.csv", std::ios::out | std::ios::trunc);

    out_file1
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

    std::ofstream out_file2;
    out_file2.open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/task_yes_try_fix.csv", std::ios::out | std::ios::trunc);

    out_file2
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

    std::ofstream out_file3;
    out_file3.open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/meiyou_collide.csv", std::ios::out | std::ios::trunc);

    out_file3
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

    std::srand((unsigned)time(NULL));


    Eigen::Matrix<double ,10,1> rand_seeds;
    rand_seeds<<11437442140,
    500988018,
    465441008,
    1827005505,
    596817637,
    1530352337,
    820433818,
    315509430,
    622230346,
    1710628714;

    ros::Time time_1;
    ros::Time time_2;
    for(int ii = 0; ii<100; ii++){

        int seed = std::rand();

        DualCBiRRT my_planner1(1.0, seed, 0.05);
        time_1 = ros::Time::now();
        if(my_planner1.plan_task_space_dir_new(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group)){
            ROS_INFO("my_planner1 success at time %d", ii);
        }
        else{
            ROS_INFO("my_planner1 fail at time %d", ii);
        }
        time_2 = ros::Time::now();
        out_file1<<(time_2 - time_1).toSec()<<",";
        my_planner1.output_perdex_multi(out_file1);


        DualCBiRRT my_planner2(1.0, seed, 0.05);
        time_1 = ros::Time::now();
        if(my_planner2.plan_task_space_dir_try_adjust(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group)){
            ROS_INFO("my_planner2 success at time %d", ii);
        }
        else{
            ROS_INFO("my_planner2 fail at time %d", ii);
        }
        time_2 = ros::Time::now();
        out_file2<<(time_2 - time_1).toSec()<<",";
        my_planner2.output_perdex_multi(out_file2);

//        DualCBiRRT my_planner3(1.0, seed, 0.00);
//        if(my_planner3.plan(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group)){
//            ROS_INFO("my_planner3 success at time %d", ii);
//        }
//        else{
//            ROS_INFO("my_planner3 fail at time %d", ii);
//        }
//        my_planner3.output_perdex_multi(out_file3);
    }
    out_file1.close();
    out_file2.close();
    out_file3.close();


    return 0;
}