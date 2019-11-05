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
    ros::init(argc, argv, "cbirrt_test");
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
    std::vector<double> test_start_value = {0.17109754, -0.87923624, -0.08423487,  1.712199,   -0.81049842,  2.09320188,  2.58848987}; //无障碍时手臂平方位置
    std::vector<double> slave_test_start_value = {0.0633710, 0.118378, 1.5027523, 2.2347026,-0.579105, 0.054547, -1.11615}; //无障碍时手臂平方位置

    const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //

    start_state.setJointGroupPositions(planning_group, test_start_value);
    start_state.setJointGroupPositions(slave_group, slave_test_start_value);

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

//    std::vector<double> test_goal_value = {0.0511426,-0.422846,-0.602817,1.92707,-0.888771,1.20479,2.70597};//有障碍时手臂平方位置
    std::vector<double> test_goal_value = {-0.53121395, -1.14663671 , 0.21698349  ,2.33939883 ,-1.17448029  ,1.81105335,  2.82284528};//无障碍时手臂平方位置
    std::vector<double> slave_test_goal_value = {-0.64966, 0.0056597, 1.453030, 2.2167859, 0.0142739, 0.7887366, -1.69753346};//无障碍时手臂平方位置

    goal_state.setJointGroupPositions(planning_group, test_goal_value);
    goal_state.setJointGroupPositions(slave_group, slave_test_goal_value);

    if(my_planner.plan(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group)){
        std::cout<<"???"<<std::endl;
    }

    std::vector<robot_state::RobotState> result = my_planner.planning_result;
    ROS_INFO("waypoints num is %d", int(result.size()));


    //*********************整体都是在添加轨迹显示的消息内容太*****************************
    std::vector<geometry_msgs::Pose> result_pose;
    moveit_msgs::RobotTrajectory result_msg;
    trajectory_msgs::JointTrajectory path_point_msg;
    trajectory_msgs::JointTrajectoryPoint path_point_position_msg; //只需要添加关节位置点
    for(size_t i=0; i<result.size(); i++){
        std::vector<double> tmp1;
        std::vector<double> tmp2;
        result[i].copyJointGroupPositions(planning_group, tmp1);
        result[i].copyJointGroupPositions(planning_group, tmp2);
        tmp1.insert(tmp1.end(), tmp2.begin(), tmp2.end());
        path_point_position_msg.positions = tmp1;
        path_point_msg.points.push_back(path_point_position_msg);

        //就这一部分是添加姿态
        if(i%5==0) {
            geometry_msgs::Pose tmp_pose_msg;
            const Eigen::Affine3d end_pose = result[i].getGlobalLinkTransform("left_gripper");
            Eigen::Quaterniond end_quaternion(end_pose.rotation());
            tmp_pose_msg.position.x = end_pose(0, 3);
            tmp_pose_msg.position.y = end_pose(1, 3);
            tmp_pose_msg.position.z = end_pose(2, 3);
            tmp_pose_msg.orientation.x = end_quaternion.x();
            tmp_pose_msg.orientation.y = end_quaternion.y();
            tmp_pose_msg.orientation.z = end_quaternion.z();
            tmp_pose_msg.orientation.w = end_quaternion.w();
            result_pose.push_back(tmp_pose_msg);
        }
    }
    //********************************************************************

    //*********************显示规划的轨迹*****************************
    const std::vector<std::string>& master_joint_names = planning_group->getVariableNames();
    for(size_t i=0; i<master_joint_names.size(); i++){
        path_point_msg.joint_names.push_back(master_joint_names[i]);
    }
    const std::vector<std::string>& slave_joint_names = slave_group->getVariableNames();
    for(size_t i=0; i<slave_joint_names.size(); i++){
        path_point_msg.joint_names.push_back(slave_joint_names[i]);
    }

    path_point_msg.header.stamp = ros::Time::now();
    result_msg.joint_trajectory = path_point_msg;

    visualization_msgs::Marker delete_all_markers;
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(result_msg, planning_group);
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Simple RRT", rvt::WHITE, rvt::XLARGE);
    for (std::size_t i = 0; i < result_pose.size(); ++i)
        visual_tools.publishAxisLabeled(result_pose[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();



    // 创建一个DisplayTrajectory msg
    moveit_msgs::DisplayTrajectory display_traj_msg;
    moveit_msgs::RobotState start_state_msg;
    sensor_msgs::JointState start_angles_msg;

    //添加这个消息的第三个参数，开始的状态
    for(size_t i=0; i<master_joint_names.size(); i++){
        start_angles_msg.name.push_back(master_joint_names[i]);
        start_angles_msg.position.push_back(test_start_value[i]);
    }
    for(size_t i=0; i<slave_joint_names.size(); i++){
        start_angles_msg.name.push_back(slave_joint_names[i]);
        start_angles_msg.position.push_back(slave_test_start_value[i]);
    }
    start_state_msg.joint_state = start_angles_msg;
    display_traj_msg.trajectory_start = start_state_msg;

    //添加这个消息的第二个参数，开始的状态，可能显示多条轨迹所以是向量
    display_traj_msg.trajectory.push_back(result_msg);

    ros::Publisher display_traj_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    while (display_traj_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    display_traj_publisher.publish(display_traj_msg);
    ros::Duration(1).sleep();


    return 0;
}