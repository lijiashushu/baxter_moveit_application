#include <ros/ros.h>

#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>

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

    double _left_goal_yaw[4] = {0.0, 0.52, 1.05, 1.57};
    double _left_right_distance_z[4] = {0.2 * std::cos(0.0), 0.2 * std::cos(0.52), 0.2 * std::cos(1.05), 0.2 * std::cos(1.57)};
    double _left_right_distance_x[4] = {-0.2 * std::sin(0.0), -0.2 * std::sin(0.52), -0.2 * std::sin(1.05), -0.2 * std::sin(1.57)};
    double _left_right_euler_distance[4] = {0, 2*0.52, 2*1.05, 2*1.57};
    int constraint_index = 1;

//    KDL::Vector left_goal_pos(0.650064, 0.1,  0.4500058); //start pose
    KDL::Vector left_goal_pos(0.930064, 0.1,  0.2500058);
    KDL::Rotation left_goal_rot = KDL::Rotation::EulerZYX(_left_goal_yaw[constraint_index], 0, 1.57);
    KDL::Vector distance(_left_right_distance_x[constraint_index], 0, _left_right_distance_z[constraint_index]);
    KDL::Vector right_goal_pos = left_goal_rot * distance + left_goal_pos;
    KDL::Rotation right_goal_rot = KDL::Rotation::EulerZYX(_left_goal_yaw[constraint_index] - _left_right_euler_distance[constraint_index], 0, -1.57);

    moveit::planning_interface::MoveGroupInterface left_move_group("left_arm");
    moveit::planning_interface::MoveGroupInterface right_move_group("right_arm");
    
    geometry_msgs::Pose left_goal_pose;
    left_goal_pose.position.x = left_goal_pos[0];
    left_goal_pose.position.y = left_goal_pos[1];
    left_goal_pose.position.z = left_goal_pos[2];
    left_goal_rot.GetQuaternion(left_goal_pose.orientation.x, left_goal_pose.orientation.y, left_goal_pose.orientation.z, left_goal_pose.orientation.w);

    geometry_msgs::Pose right_goal_pose;
    right_goal_pose.position.x = right_goal_pos[0];
    right_goal_pose.position.y = right_goal_pos[1];
    right_goal_pose.position.z = right_goal_pos[2];
    right_goal_rot.GetQuaternion(right_goal_pose.orientation.x, right_goal_pose.orientation.y, right_goal_pose.orientation.z, right_goal_pose.orientation.w);

    left_move_group.setPoseTarget(left_goal_pose);
    right_move_group.setPoseTarget(right_goal_pose);

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            "robot_description");
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>(
            "planning_scene", 1);
    ros::WallDuration sleep_t(0.5);

    monitor_ptr->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();
    robot_state::RobotState result_state = planning_scene_for_operate->getCurrentStateNonConst();
    if(left_goal_pos(1) > 0) {
        //先 plan 左臂
        //*******************************************left plan**********************************************
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        bool left_success = (left_move_group.plan(left_plan) ==
                             moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (left_success) {
            std::cout << "Left Success!" << std::endl;
        } else {
            std::cout << "Left Fail!" << std::endl;
            exit(1);
        }

        result_state.setJointGroupPositions("left_arm",
                                                 left_plan.trajectory_.joint_trajectory.points[
                                                         left_plan.trajectory_.joint_trajectory.points.size() -
                                                         1].positions);

        std::cout << "Left Joint angles" << std::endl;
        for (size_t i = 0; i < left_plan.trajectory_.joint_trajectory.points[
                left_plan.trajectory_.joint_trajectory.points.size() -
                1].positions.size(); i++) {
            std::cout << left_plan.trajectory_.joint_trajectory.points[
                    left_plan.trajectory_.joint_trajectory.points.size() -
                    1].positions[i] << ",";
        }
        std::cout << std::endl;

        planning_scene_for_operate->setCurrentState(result_state);
        moveit_msgs::PlanningScene left_planning_scene_msg;
        planning_scene_for_operate->getPlanningSceneMsg(left_planning_scene_msg);
        left_planning_scene_msg.is_diff = true;

        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
            sleep_t.sleep();
        }
        planning_scene_diff_publisher.publish(left_planning_scene_msg);
        ros::Duration(1).sleep();


        //*******************************************right plan**********************************************
        moveit::planning_interface::MoveGroupInterface::Plan right_plan;
        bool right_success = (right_move_group.plan(right_plan) ==
                              moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (right_success) {
            std::cout << "Right Success!" << std::endl;
        } else {
            std::cout << "Right Fail!" << std::endl;
            exit(1);
        }
        right_move_group.move();


        result_state.setJointGroupPositions("right_arm",
                                                  right_plan.trajectory_.joint_trajectory.points[
                                                          right_plan.trajectory_.joint_trajectory.points.size() -
                                                          1].positions);

        std::cout << "Right Joint angles" << std::endl;
        for (size_t i = 0; i < right_plan.trajectory_.joint_trajectory.points[
                right_plan.trajectory_.joint_trajectory.points.size() -
                1].positions.size(); i++) {
            std::cout << right_plan.trajectory_.joint_trajectory.points[
                    right_plan.trajectory_.joint_trajectory.points.size() -
                    1].positions[i] << ",";
        }
        std::cout << std::endl;

        planning_scene_for_operate->setCurrentState(result_state);
        moveit_msgs::PlanningScene right_planning_scene_msg;
        planning_scene_for_operate->getPlanningSceneMsg(right_planning_scene_msg);
        right_planning_scene_msg.is_diff = true;

        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
            sleep_t.sleep();
        }
        planning_scene_diff_publisher.publish(right_planning_scene_msg);
        ros::Duration(1).sleep();
    }
    else{
        //先 plan 右臂
        //*******************************************right plan**********************************************
        moveit::planning_interface::MoveGroupInterface::Plan right_plan;
        bool right_success = (right_move_group.plan(right_plan) ==
                              moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (right_success) {
            std::cout << "Right Success!" << std::endl;
        } else {
            std::cout << "Right Fail!" << std::endl;
            exit(1);
        }


        result_state.setJointGroupPositions("right_arm",
                                            right_plan.trajectory_.joint_trajectory.points[
                                                    right_plan.trajectory_.joint_trajectory.points.size() -
                                                    1].positions);

        std::cout << "Right Joint angles" << std::endl;
        for (size_t i = 0; i < right_plan.trajectory_.joint_trajectory.points[
                right_plan.trajectory_.joint_trajectory.points.size() -
                1].positions.size(); i++) {
            std::cout << right_plan.trajectory_.joint_trajectory.points[
                    right_plan.trajectory_.joint_trajectory.points.size() -
                    1].positions[i] << ",";
        }
        std::cout << std::endl;

        planning_scene_for_operate->setCurrentState(result_state);
        moveit_msgs::PlanningScene right_planning_scene_msg;
        planning_scene_for_operate->getPlanningSceneMsg(right_planning_scene_msg);
        right_planning_scene_msg.is_diff = true;

        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
            sleep_t.sleep();
        }
        planning_scene_diff_publisher.publish(right_planning_scene_msg);
        ros::Duration(1).sleep();

        //*******************************************left plan**********************************************
        moveit::planning_interface::MoveGroupInterface::Plan left_plan;
        bool left_success = (left_move_group.plan(left_plan) ==
                             moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (left_success) {
            std::cout << "Left Success!" << std::endl;
        } else {
            std::cout << "Left Fail!" << std::endl;
            exit(1);
        }

        result_state.setJointGroupPositions("left_arm",
                                            left_plan.trajectory_.joint_trajectory.points[
                                                    left_plan.trajectory_.joint_trajectory.points.size() -
                                                    1].positions);

        std::cout << "Left Joint angles" << std::endl;
        for (size_t i = 0; i < left_plan.trajectory_.joint_trajectory.points[
                left_plan.trajectory_.joint_trajectory.points.size() -
                1].positions.size(); i++) {
            std::cout << left_plan.trajectory_.joint_trajectory.points[
                    left_plan.trajectory_.joint_trajectory.points.size() -
                    1].positions[i] << ",";
        }
        std::cout << std::endl;

        planning_scene_for_operate->setCurrentState(result_state);
        moveit_msgs::PlanningScene left_planning_scene_msg;
        planning_scene_for_operate->getPlanningSceneMsg(left_planning_scene_msg);
        left_planning_scene_msg.is_diff = true;

        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
            sleep_t.sleep();
        }
        planning_scene_diff_publisher.publish(left_planning_scene_msg);
        ros::Duration(1).sleep();
    }
    return 0;
}