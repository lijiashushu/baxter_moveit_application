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

int main(int argc, char** argv){
    ros::init(argc, argv, "ik_test");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    DualCBiRRT my_planner(1.0);

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

    robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
    //Z:180 Y:90 X:-90   2.94792  1.56999 -1.76536




//    std::vector<double> test_start_value = {0.178307,-1.36637,-0.718743,2.32057,-1.28874,1.62442,2.4651};//有障碍时手臂平方位置
//    std::vector<double> test_start_value = {0.17109754, -0.87923624, -0.08423487,  1.712199,   -0.81049842,  2.09320188,  2.58848987}; //无障碍时手臂平方位置

    //求解测试数据，master 为Python程序规划出来的一个中间值，slave_test_start_value 为起始值所对应的slave
    std::vector<double> slave_test_start_value = {0.0633710, 0.118378, 1.5027523, 2.2347026,-0.579105, 0.054547, -1.11615}; //无障碍时手臂平方位置
//    std::vector<double> test_start_value = {-0.1962478, -1.019100, 0.073317, 2.0402, -1.00087, 1.94562, 2.7110    }; //测试的中间位置
//    std::vector<double> test_start_value = {-0.623942,-0.650041,-0.208132,1.99769,-1.4868,1.4407,2.90019}; //测试的中间位置
    std::vector<double> test_start_value = {-1.03453,-1.13675,0.54026,2.42528,-1.40089,1.85371,2.85445};
    std::vector<double> slave_computed_vector;


    Eigen::Matrix<double, 7, 1> slave_start_matrix;
    Eigen::Matrix<double, 7, 1> master_matrix;
    Eigen::Matrix<double, 7, 1> slave_computed_matrix;

    for(size_t i=0; i<7; i++){
        master_matrix[i] = test_start_value[i];
        slave_start_matrix[i] = slave_test_start_value[i];
    }


    const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //


    //***********************获取slave group的关节界限，用于求解IK***********************
    const robot_model::RobotModelConstPtr & baxter_robot_model_ptr = planning_scene_for_operate->getRobotModel();
    const std::vector<std::string>& slave_joint_names = slave_group->getVariableNames();
    std::pair<std::vector<double>, std::vector<double>> slave_joint_pos_bounds;
    std::vector<double> slave_joint_max_bounds;
    std::vector<double> slave_joint_min_bounds;
    for(size_t i=0; i<slave_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = baxter_robot_model_ptr->getVariableBounds(slave_joint_names[i]);
        slave_joint_max_bounds.push_back(tmp_bounds.max_position_);
        slave_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    slave_joint_pos_bounds.first = slave_joint_max_bounds;
    slave_joint_pos_bounds.second = slave_joint_min_bounds;
    //********************************************************************************


    if(my_planner.solve_IK_problem(slave_start_matrix, master_matrix, slave_computed_matrix, planning_group, slave_group, planning_scene_for_operate, slave_joint_pos_bounds)){
        std::cout<<"IK success !!!"<<std::endl;
        for(size_t i=0; i<7; i++){
            slave_computed_vector.push_back(slave_computed_matrix[i]);
        }
        start_state.setJointGroupPositions(planning_group, test_start_value);
        start_state.setJointGroupPositions(slave_group, slave_computed_vector);
    }


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


    return 0;
}