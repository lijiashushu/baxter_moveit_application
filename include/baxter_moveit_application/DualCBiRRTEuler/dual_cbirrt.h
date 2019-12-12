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
#include <fstream>

struct PerformanceIndexOneExtend{
    int extend_num;
    int constraint_project_times;
    int project_success;
    int extend_success;
    int ik_project_times;
    int ik_success;
    int no_collide;
    double extend_total_spend_time = 0;
    double project_total_spend_time = 0;
    double ik_total_spend_time = 0;

    PerformanceIndexOneExtend(){
        int tmp_max = std::numeric_limits<int>::max();
        extend_num = 0;
        constraint_project_times = 0;
        project_success = 0;
        extend_success = 0;
        ik_project_times = 0;
        ik_success = 0;
        no_collide = 0;
        extend_total_spend_time = 0;
        project_total_spend_time = 0;
        ik_total_spend_time = 0;
    }

};

struct PerformanceIndexOneSample{
    int sample_num;
    double spend_time;
    std::vector<PerformanceIndexOneExtend> tree_a;
    std::vector<PerformanceIndexOneExtend> tree_b;
};

class DualCBiRRT{
public:
    DualCBiRRT(double probability, int seed, double alpha, size_t test_pose_num, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group);
    ~DualCBiRRT();

    void sample_task_state(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Vector4d & random_state_vector);

    size_t near_tree_task_space(Eigen::Vector4d & random_task_state, Eigen::Vector4d & nearest_node_task_state, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, bool if_tree_a);

    bool plan_task_space_dir(robot_state::RobotState & goal_state, robot_state::RobotState & start_state);
    bool plan_task_space_dir_try_adjust(robot_state::RobotState & goal_state, robot_state::RobotState & start_state);


    void constraint_extend_task_space_dir(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix,  PerformanceIndexOneSample & perdex_one_sample, bool if_tree_a, bool if_sample, Eigen::Vector4d& reached_task_pos, size_t & reached_index);
    void constraint_extend_task_space_dir_try_adjust(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix,   PerformanceIndexOneSample & perdex_one_sample, bool if_tree_a, bool if_sample, Eigen::Vector4d& reached_task_pos, size_t & reached_index);

    bool solve_IK_problem(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, PerformanceIndexOneExtend & perdex_one_extend);

    void output_perdex();
    void output_perdex_multi(std::ofstream & outFILE);
    void output_new_manipulability(std::ofstream & outFILE);

    double manipulability_compute(Eigen::Matrix<double, 7, 1> master_matrix, Eigen::Matrix<double, 7, 1> slave_matrix, Eigen::MatrixXd & master_ob_jac, Eigen::Vector3d master_ob_dir, double master_ob_dis, Eigen::MatrixXd & slave_ob_jac, Eigen::Vector3d slave_ob_dir, double slave_ob_dis, bool if_tree_a);
    double manipulability_compute_one_sample(bool if_a_tree, size_t reached_index, size_t nearest_index);
    double simple_dir_manipulability_compute(Eigen::Matrix<double, 7, 1> master_matrix, Eigen::Matrix<double, 7, 1> slave_matrix, bool if_tree_a);
    void init_all_dir_manipulability_compute(robot_state::RobotState & robot_state, Eigen::MatrixXd & master_ob_jac, Eigen::Vector3d master_ob_dir, double master_ob_dis, Eigen::MatrixXd & slave_ob_jac, Eigen::Vector3d slave_ob_dir, double slave_ob_dis, double & out_master_dir_manipulability, double & out_slave_dir_manipulability);
    void init_all_dir_manipulability_compute_all_obstacle(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability);
    void init_all_dir_manipulability_compute_all_obstacle_one_dir(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability, Eigen::Vector4d one_dir);
    void init_all_dir_manipulability_compute_all_obstacle_one_dir_test(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability, Eigen::Vector4d one_dir, visualization_msgs::MarkerArray & robot_collision_points, visualization_msgs::MarkerArray & object_colllision_points);


    void basic_manipulability_compute_once(std::vector<double> master_matrix, std::vector<double> slave_matrix, double & l_basic_manipu, double & r_basic_manipu);
    void dir_manipulability_compute_once(std::vector<double> start_master_matrix, std::vector<double> start_slave_matrix, std::vector<double> goal_master_matrix, std::vector<double> goal_slave_matrix, double & s_l_basic_manipu, double & s_r_basic_manipu, double & g_l_basic_manipu, double & g_r_basic_manipu);



    const std::vector<std::pair<robot_state::RobotState, size_t>> & get_tree_state_vector(bool if_a_tree);

    void output_extend_success_rate(std::ofstream & outFILE);
    void output_simple_manipulability(std::ofstream & outFILE);
    void output_rate_simple_manipulability(std::ofstream & outFILE);
    void output_complex_manipulability(std::ofstream & outFILE);
    void output_rate_complex_manipulability(std::ofstream & outFILE);
    void output_reached_index(std::ofstream & outFILE);

        std::vector<robot_state::RobotState> planning_result;
    std::vector<size_t > planning_result_index;
    std::vector<Eigen::Vector4d> planning_result_task_state_vector;

    std::string _baxter_torso_link_names[8] = {"collision_head_link_1",
                                               "collision_head_link_2",
                                               "display",
                                               "head",
                                               "pedestal",
                                               "screen",
                                               "sonar_ring",
                                               "torso"};
    std::string _master_link_all_collision_names[8] = {"l_gripper_l_finger",
                                                        "left_gripper_base",
                                                        "left_hand",
                                                        "left_lower_elbow",
                                                        "left_lower_forearm",
                                                        "left_upper_elbow",
                                                        "left_upper_forearm",
                                                        "left_wrist"};
    std::string _slave_link_all_collision_names[8] = {"r_gripper_r_finger",
                                                       "right_gripper_base",
                                                       "right_hand",
                                                       "right_lower_elbow",
                                                       "right_lower_forearm",
                                                       "right_upper_elbow",
                                                       "right_upper_forearm",
                                                       "right_wrist"};

    std::vector<double> _new_manipulability_mini_vec_explore;
    std::vector<double> _new_manipulability_mini_vec_exploit;
    std::set<const moveit::core::LinkModel *> _master_link_model_set;
    std::set<const moveit::core::LinkModel *> _slave_link_model_set;
    collision_detection::CollisionRobotConstPtr collision_robot;
    collision_detection::CollisionWorldConstPtr collision_world;
private:

    //moveit planning scene, baxter 相关
    planning_scene::PlanningScenePtr& planning_scene_ptr;
    const robot_state::JointModelGroup* planning_group;
    const robot_state::JointModelGroup* slave_group;
    std::string planning_group_name = "left_arm";
    std::string slave_group_name = "right_arm";

    std::pair<std::vector<double>, std::vector<double>> master_joint_pos_bounds;
    std::pair<std::vector<double>, std::vector<double>> slave_joint_pos_bounds;
    robot_model::RobotModelConstPtr robot_model_ptr;
    random_numbers::RandomNumberGenerator rng;

    std::map<std::string, Eigen::Vector3d> collision_object_origin;


    std::vector<std::pair<robot_state::RobotState, size_t>> _a_rrt_tree_state;
    std::vector<std::pair<robot_state::RobotState, size_t>> _b_rrt_tree_state;
    std::vector< Eigen::Matrix<double, 14, 1> > _a_rrt_tree_matrix;
    std::vector< Eigen::Matrix<double, 14, 1> > _b_rrt_tree_matrix;
    std::vector< Eigen::Vector4d > _a_rrt_tree_task_space_state;
    std::vector< Eigen::Vector4d > _b_rrt_tree_task_space_state;


    Eigen::Affine3d _start_master_pose;
    Eigen::Affine3d _start_slave_pose;
    Eigen::Affine3d _goal_master_pose;
    Eigen::Affine3d _goal_slave_pose;

    std::default_random_engine _random_engine;
    std::bernoulli_distribution _random_distribution;

    double _step_size;
    double _task_step_size = 0.02;
    double _constrain_delta = 0.01;
    double _pos_constrain_delta = 0.02;
    double _pitch_min;
    double _pitch_max;
    double _yaw_min[4] = {0-1.05, 0.52-1.05, 1.05-1.05, 1.57-1.05};
    double _yaw_max[4] = {0+1.05, 0.52+1.05, 1.05+1.05, 1.57+1.05};
    double _roll_min = 1.57 - 1.05;
    double _roll_max = 1.57 + 1.05;
    double _left_right_distance_z[4] = {0.2 * std::cos(0.0), 0.2 * std::cos(0.52), 0.2 * std::cos(1.05), 0.2 * std::cos(1.57)};
    double _left_right_distance_x[4] = {-0.2 * std::sin(0.0), -0.2 * std::sin(0.52), -0.2 * std::sin(1.05), -0.2 * std::sin(1.57)};
    double _left_right_euler_distance[4] = {0, 2*0.52, 2*1.05, 2*1.57};

    int _draw_count;

    int _max_planning_times = 1000;

    int _seed;
    double _alpha;
    double _error_coefficient = 0.5;
    std::vector<PerformanceIndexOneSample> _performance_record;


    size_t _test_pose_num;

    std::vector<double> _extend_success_rate;
    double _extend_times = 0;
    double _extend_success_times = 0;
    std::vector<double> _simple_dir_manipulability_vec;
    double _simple_dir_manipulability_sum = 0;
    std::vector<double> _rate_simple_manipulability_vec;
    std::vector<double> _complex_dir_manipulability_vec;
    double _complex_dir_manipulability_sum = 0;
    std::vector<double> _rate_complex_manipulability_vec;

    std::vector<double> _new_manipulability_master_vec_explore;
    std::vector<double> _new_manipulability_master_vec_exploit;
    std::vector<double> _new_manipulability_slave_vec_explore;
    std::vector<double> _new_manipulability_slave_vec_exploit;


    std::vector<size_t> _a_tree_reached_index_vec;
    std::vector<size_t> _b_tree_reached_index_vec;
    std::vector<size_t> _a_tree_nearest_index_vec;
    std::vector<size_t> _b_tree_nearest_index_vec;
    double _master_extend_manipulability;
    double _slave_extend_manipulability;
    std::vector<double> _a_tree_master_extend_manipulability;
    std::vector<double> _a_tree_slave_extend_manipulability;
    std::vector<double> _b_tree_master_extend_manipulability;
    std::vector<double> _b_tree_slave_extend_manipulability;
    std::vector<double> _a_tree_minimum_extend_manipulability;
    std::vector<double> _b_tree_minimum_extend_manipulability;

    std::vector<Eigen::MatrixXd>  a_tree_considered_master_ob_jac;
    std::vector<Eigen::Vector3d> a_tree_considered_master_ob_dir;
    std::vector<double> a_tree_considered_master_ob_dis;
    std::vector<Eigen::MatrixXd>  a_tree_considered_slave_ob_jac;
    std::vector<Eigen::Vector3d> a_tree_considered_slave_ob_dir;
    std::vector<double> a_tree_considered_slave_ob_dis;
    std::vector<Eigen::MatrixXd>  b_tree_considered_master_ob_jac;
    std::vector<Eigen::Vector3d> b_tree_considered_master_ob_dir;
    std::vector<double> b_tree_considered_master_ob_dis;
    std::vector<Eigen::MatrixXd>  b_tree_considered_slave_ob_jac;
    std::vector<Eigen::Vector3d> b_tree_considered_slave_ob_dir;
    std::vector<double> b_tree_considered_slave_ob_dis;


    std::vector<collision_detection::DistanceMap> _a_tree_master_dis_map;
    std::vector<collision_detection::DistanceMap> _a_tree_slave_dis_map;
    std::vector<collision_detection::DistanceMap> _b_tree_master_dis_map;
    std::vector<collision_detection::DistanceMap> _b_tree_slave_dis_map;

    std::vector<bool> _a_tree_has_been_extended;
    std::vector<bool> _b_tree_has_been_extended;

    std::vector<double> two_tree_minimum_dis;
    double current_two_tree_minimum_dis;
    double new_two_tree_minimum_dis;

    double master_one_dir_manipulaibility;
    double slave_one_dir_manipulaibility;
    std::vector<double> master_one_dir_manipulaibility_vec;
    std::vector<double> slave_one_dir_manipulaibility_vec;


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
