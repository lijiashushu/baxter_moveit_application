//
// Created by lijiashushu on 19-10-29.
//

#ifndef SRC_MY_RRT_H
#define SRC_MY_RRT_H

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

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <eigen3/Eigen/Core>

class CRRT{
public:
    CRRT(double probability);
    ~CRRT();

    void sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group);
    size_t near(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix);
    void steer(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, double step_size, Eigen::Matrix<double, 7, 1> & new_node_matrix);
    bool collision_check(Eigen::Matrix<double, 7, 1> & new_node_matrix, robot_state::RobotState & new_state, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr);
    bool plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group);
    void constraint_extend(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr);

    std::vector<robot_state::RobotState> planning_result;
private:

    //当对类的成员对象初始化时，执行的是对象类相应的构造函数。为了方便还是不把这些作为该类的属性，将其作为plan函数的参数
//    planning_scene::PlanningScenePtr _planning_scene_ptr;
//    robot_state::RobotState _planning_state;
//    const robot_state::JointModelGroup* _planning_group;
//
    std::vector<std::pair<robot_state::RobotState, size_t>> _rrt_tree_state;
    std::vector<Eigen::Matrix<double, 7, 1>> _rrt_tree_matrix;
    std::default_random_engine _random_engine;
    std::bernoulli_distribution _random_distribution;
    double _step_size;
    double _constrain_delta;
    double _pitch_min;
    double _pitch_max;
    double _yaw_min;
    double _yaw_max;

//
//    std::vector<std::vector<double>> _result;
//
//
//    robot_state::RobotState _random_state;
//    std::vector<double> _random_state_value;
//    Eigen::Matrix<double, 7, 1> _random_state_value_matrix;
//
//    size_t nearest_node_index;
//    Eigen::Matrix<double, 7, 1> _nearest_node_matrix;
};



#endif //SRC_MY_RRT_H
