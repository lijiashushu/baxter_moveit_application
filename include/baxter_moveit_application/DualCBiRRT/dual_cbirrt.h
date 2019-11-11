//
// Created by lijiashushu on 19-11-04.
//

#ifndef SRC_DUALCBIRRT_H
#define SRC_DUALCBIRRT_H

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

#include <limits.h>

struct PerformanceIndexOneExtend{
    int extend_num;
    int constraint_project_times;
    int project_success;
    int extend_success;
    int ik_project_times;
    int ik_success;
    int no_collide;

    PerformanceIndexOneExtend(){
        int tmp_max = std::numeric_limits<int>::max();
        extend_num = 0;
        constraint_project_times = 0;
        project_success = 0;
        extend_success = 0;
        ik_project_times = 0;
        ik_success = 0;
        no_collide = 0;
    }

};
struct PerformanceIndexOneSample{
    int sample_num;
    std::vector<PerformanceIndexOneExtend> tree_a;
    std::vector<PerformanceIndexOneExtend> tree_b;
};






class DualCBiRRT{
public:
    DualCBiRRT(double probability);
    ~DualCBiRRT();

    void sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group, random_numbers::RandomNumberGenerator &rng);

    size_t near_a_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix);
    size_t near_b_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix);


    bool plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group);

    void constraint_extend_a_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample);
    void constraint_extend_b_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample);

    bool solve_IK_problem(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneExtend & perdex_one_extend);

    void output_perdex();

    std::vector<robot_state::RobotState> planning_result;
private:

    //当对类的成员对象初始化时，执行的是对象类相应的构造函数。为了方便还是不把这些作为该类的属性，将其作为plan函数的参数
//    planning_scene::PlanningScenePtr _planning_scene_ptr;
//    robot_state::RobotState _planning_state;
//    const robot_state::JointModelGroup* _planning_group;
//
    std::vector<std::pair<robot_state::RobotState, size_t>> _a_rrt_tree_state;
    std::vector<std::pair<robot_state::RobotState, size_t>> _b_rrt_tree_state;
    std::vector< Eigen::Matrix<double, 14, 1> > _a_rrt_tree_matrix;
    std::vector< Eigen::Matrix<double, 14, 1> > _b_rrt_tree_matrix;
    std::default_random_engine _random_engine;
    std::bernoulli_distribution _random_distribution;
    double _step_size;
    double _constrain_delta;
    double _pitch_min;
    double _pitch_max;
    double _yaw_min;
    double _yaw_max;

    int _draw_count;
    
    std::vector<PerformanceIndexOneSample> _performance_record;



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
