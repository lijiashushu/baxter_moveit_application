#include <ros/ros.h>

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

#include <moveit/move_group_interface/move_group_interface.h>

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
//    std::srand((unsigned)time(NULL));

//    moveit::planning_interface::MoveGroupInterface move_group("left_arm");
////    0.523779 -0.0419838   0.183778
////    Eigen::Vector3d goal_pos (0.717521, -0.0286976,   0.669831);
//    Eigen::Vector3d goal_pos (0.580377, -0.0316619,   0.284903);
////    Eigen::Vector3d rpy(2.53466, 1.57962, 0.962707);
////    Eigen::Vector3d rpy(2.94767, 1.56975, 0.962707);
////    Eigen::Vector3d rpy(2.97894, 1.54465, 1.40875);
//    Eigen::Vector3d rpy(0, 0, 1.57);

    moveit::planning_interface::MoveGroupInterface move_group("right_arm");
//    0.523779 -0.0419838   0.183778
//    Eigen::Vector3d goal_pos (0.717521, -0.0286976 - 0.06,   0.669831);
    Eigen::Vector3d goal_pos (0.580377, -0.0916619,   0.284903);
////    Eigen::Vector3d rpy(2.53466, 1.57962, 0.962707);
////    Eigen::Vector3d rpy(2.94767, 1.56975, 0.962707);
////    Eigen::Vector3d rpy(2.97894, 1.54465, 1.40875);
    Eigen::Vector3d rpy(0, 0, -1.57);




//    Eigen::Vector3d rpy(0, 0, 0);

    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion;
    quaternion = yaw_angle*pitch_angle*roll_angle;

    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = goal_pos[0];
    goal_pose.position.y = goal_pos[1];
    goal_pose.position.z = goal_pos[2];
    goal_pose.orientation.w = quaternion.w();
    goal_pose.orientation.x = quaternion.x();
    goal_pose.orientation.y = quaternion.y();
    goal_pose.orientation.z = quaternion.z();

    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<double> last_state;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        std::cout<<"Success!"<<std::endl;
    }



    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

    robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
    //Z:180 Y:90 X:-90   2.94792  1.56999 -1.76536

    const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //
    const robot_state::JointModelGroup* both_group = start_state.getJointModelGroup("both_arms"); //

    start_state.setJointGroupPositions(slave_group, my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions);

//    start_state.setJointGroupPositions(planning_group, my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions);


    for(size_t i=0; i<my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions.size();i++){
        std::cout<<my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions[i]<<",";
    }
    std::cout<<std::endl;



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