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
#include <eigen3/Eigen/Core>


int main(int argc, char** argv){
    Eigen::Vector3d euler1(3.04237, 1.56985, 1.47079);
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(euler1[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(euler1[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(euler1[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d euler1_rot_matrix;
    euler1_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;

    Eigen::Vector3d euler2(0.354233,  1.58171, -1.21038);
    Eigen::AngleAxisd goal_roll_angle2(Eigen::AngleAxisd(euler2[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle2(Eigen::AngleAxisd(euler2[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle2(Eigen::AngleAxisd(euler2[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d euler2_rot_matrix;
    euler2_rot_matrix = goal_yaw_angle2*goal_pitch_angle2*goal_roll_angle2;

    Eigen::Matrix3d err_matrix = euler2_rot_matrix * euler1_rot_matrix.inverse();
    Eigen::AngleAxisd err_angle_axis (err_matrix);

    std::cout<<err_angle_axis.angle()<<std::endl;
    std::cout<<err_angle_axis.axis()<<std::endl;
    return 0;
}