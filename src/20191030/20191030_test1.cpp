#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv){
//    ros::init(argc, argv, "test220191026");
//    ros::NodeHandle n;
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//
//    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
//    monitor_ptr->requestPlanningSceneState("get_planning_scene");
//    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
//
//    robot_state::RobotState & robot_state_write = ps->getCurrentStateNonConst();
//
//    const Eigen::Affine3d end_pose = robot_state_write.getGlobalLinkTransform("left_gripper");
//    auto end_rot_matrix = end_pose.rotation();
//    Eigen::Vector3d end_rot_eulerAngle=end_rot_matrix.eulerAngles(2,1,0);
//
//    ROS_INFO_STREAM("eulerAngle: " << end_rot_eulerAngle);

    Eigen::Matrix<double ,2,3> test;
    test<<1,2,3,4,5,6;
    std::cout<<test.row(1).head(2)<<std::endl;
    return 0;
}
