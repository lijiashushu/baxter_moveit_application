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

    moveit::planning_interface::MoveGroupInterface left_move_group("left_arm");
    moveit::planning_interface::MoveGroupInterface right_move_group("right_arm");
    Eigen::Vector3d goal_pos;
    Eigen::Vector3d rpy;
    int if_left_arm = 0;
    if(if_left_arm){

//        goal_pos <<0.680433, -0.0916978+0.2,   0.284868;
        goal_pos <<0.750009, 0.0999061,  0.639999;

        rpy<<0, 0, 1.57;
    }
    else{

//        goal_pos <<0.680433, -0.0916978,   0.284868;
        goal_pos <<0.750064, -0.100005,  0.640058;
        rpy<<0, 0, -1.57;
    }

    if(if_left_arm){
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

        left_move_group.setPoseTarget(goal_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        std::vector<double> last_state;

        planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        monitor_ptr->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
        planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
        planning_scene_for_operate->decoupleParent();

//        moveit_msgs::AttachedCollisionObject attached_object;
//        moveit_msgs::CollisionObject collision_object;
//        if(planning_scene_for_operate->getCollisionObjectMsg(collision_object, "object")){std::cout<<"has object!!"<<std::endl;}
//        else{std::cout<<"no object!!"<<std::endl;}
//        attached_object.link_name="l_gripper_l_finger_tip";
//        attached_object.object = collision_object;
//        attached_object.touch_links = std::vector<std::string>{"l_gripper_l_finger_tip", "l_gripper_l_finger", "l_gripper_r_finger_tip", "l_gripper_r_finger"
//        };
//        moveit_msgs::PlanningScene planning_scene_msg;
//        planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
//        planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
//        planning_scene_msg.is_diff = true;
//        ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
//        ros::WallDuration sleep_t(0.5);
//        while (planning_scene_diff_publisher.getNumSubscribers() < 1)
//        {
//            sleep_t.sleep();
//        }
//        planning_scene_diff_publisher.publish(planning_scene_msg);
//        ros::Duration(1).sleep();
//
//        if(planning_scene_for_operate->getAttachedCollisionObjectMsg(attached_object, "object")){std::cout<<"has object!!"<<std::endl;}
//        else{std::cout<<"no object!!"<<std::endl;}
//
        bool success = (left_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success){
            std::cout<<"Success!"<<std::endl;
        }



        robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
        //Z:180 Y:90 X:-90   2.94792  1.56999 -1.76536

        const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //

        start_state.setJointGroupPositions(planning_group, my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions);

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
    }
    else{
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

        right_move_group.setPoseTarget(goal_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        std::vector<double> last_state;

        bool success = (right_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
        const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //

        start_state.setJointGroupPositions(slave_group, my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions);

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
    }

    return 0;
}