//
// Created by lijiashushu on 19-11-04.
//

#ifndef TASK_SPACE_PALNNER_H
#define TASK_SPACE_PALNNER_H

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
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <eigen3/Eigen/Core>
#include <kdl/frames.hpp>
#include <limits.h>
#include <fstream>
#include <math.h>

#include <algorithm>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <random_numbers/random_numbers.h>
#include <string>
#include <chrono>

#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <flann/flann.hpp>

#include "path_planner.h"

/*针对某一规划问题构建的规划器
 *创建对象时，应该什么都不输入，在内部构建环境状态信息。
*/
class TaskSpacePlanner : public PathPlanner{

    //调用 moveit 的 API 需要的变量
    planning_scene_monitor::PlanningSceneMonitorPtr _monitor_ptr;
    planning_scene::PlanningScenePtr _planning_scene_ptr;
    const robot_state::JointModelGroup* _both_group;
    const robot_state::JointModelGroup* _planning_group;
    const robot_state::JointModelGroup* _slave_group;

    //moveit 碰撞检测需要的变量
    collision_detection::CollisionRobotConstPtr _collision_robot;
    collision_detection::CollisionWorldConstPtr _collision_world;
    robot_model::RobotModelConstPtr _robot_model_ptr;
    std::map<std::string, Eigen::Vector3d> _collision_object_origin;
    std::pair<std::vector<double>, std::vector<double>> _master_joint_pos_bounds;
    std::pair<std::vector<double>, std::vector<double>> _slave_joint_pos_bounds;
    std::set<const moveit::core::LinkModel *> _master_link_model_set;
    std::set<const moveit::core::LinkModel *> _slave_link_model_set;
    const std::string _baxter_torso_link_names[8] = {"collision_head_link_1", "collision_head_link_2", "display","head", "pedestal", "screen", "sonar_ring", "torso"};
    std::string _master_link_all_collision_names[8] = {"l_gripper_l_finger", "left_gripper_base", "left_hand", "left_lower_elbow", "left_lower_forearm",
                                                       "left_upper_elbow", "left_upper_forearm",  "left_wrist"};
    std::string _slave_link_all_collision_names[8] = {"r_gripper_r_finger",  "right_gripper_base", "right_hand", "right_lower_elbow",
                                                      "right_lower_forearm", "right_upper_elbow",  "right_upper_forearm",  "right_wrist"};

    //visual tool
    ros::NodeHandle _nh;
    moveit_visual_tools::MoveItVisualTools _visual_tools;

    //产生随机数
    std::default_random_engine _random_engine;
    std::bernoulli_distribution _random_distribution;//产生一个伯努利分布的随机变量
    std::uniform_real_distribution<double> _random_uniform;

    //双臂闭环约束
    double _pitch_min;
    double _pitch_max;
    double _yaw_min[4] = {0-1.05, 0.52-1.05, 1.05-1.05, 1.57-1.05};
    double _yaw_max[4] = {0+1.05, 0.52+1.05, 1.05+1.05, 1.57+1.05};
    double _roll_min = 1.57 - 1.05;
    double _roll_max = 1.57 + 1.05;
    double _left_right_distance_z[4] = {0.2 * std::cos(0.0), 0.2 * std::cos(0.52), 0.2 * std::cos(1.05), 0.2 * std::cos(1.57)};
    double _left_right_distance_x[4] = {-0.2 * std::sin(0.0), -0.2 * std::sin(0.52), -0.2 * std::sin(1.05), -0.2 * std::sin(1.57)};
    double _left_right_euler_distance[4] = {0, 2*0.52, 2*1.05, 2*1.57};

    //算法运行过程中需要使用的状态变量
    Eigen::Matrix<double, 7, 1> _random_state_value_matrix;
    Eigen::Vector4d _random_state_vector;
    size_t _nearest_index;

    //算法中的控制参数
    std::vector<double> _startAngles;
    std::vector<double> _goalAngles;
    double _collision_check_step = 0.2;
    double _task_step_size; //扩展时一点点扩展，但是增长了0.1m之后才算作一个新的节点，相当于这个step是collision check的step
    int _extend_step_num = 5;
    int _max_planning_times = 5000; //最大采样次数
    double _planning_time;
    double _max_planning_time;
    double _error_coefficient = 0.5; //求解IK时的系数
    size_t _test_pose_num = 0; //使用哪个姿态约束
    int _seed;
    bool _if_manipuGrad;
    bool _if_rrt_star;
    bool _if_informed;
    std::vector<geometry_msgs::Point> _informed_sample_points;
    std::vector<std_msgs::ColorRGBA> _informed_sample_points_color;

    //informed sample parameter
    Eigen::Matrix4d _info_C_M;
    Eigen::Matrix4d _info_C_M_inverse;
    Eigen::Vector4d _info_centre;
    double _info_cMin;
    bool _foundPathFlag;

    //KDtree的内存指针,程序结束时释放空间
    std::vector<double*> _waitRelesePtr;


    /**
       *  @brief  Sample a four dimension state in task space
       *  @param  goal_state  the goal_state with a small probobality to be selected directly
       */
    void sample(robot_state::RobotState & goal_state, random_numbers::RandomNumberGenerator& rng);

    /**
       *  @brief  Sample a four dimension state in task space in the ellipsoid region
       *  @param  info_cBest  current best path cost
   */
    void sampleEllipsoid(double info_cBest, random_numbers::RandomNumberGenerator& rng);

    /**
       *  @brief  Find the nearest node in the tree
       *  @param  if_tree_a  indication the tree a or tree b
       */
    void nearest(bool if_tree_a, std::vector<flann::Index<flann::L2<double>>>& flann_index);

    /**
       *  @brief  Extend the tree from the nearest node in the tree.
       *  @param  if_tree_a  indicating the tree a or tree b
       *  @param  if_sample  indicating if this extend tries to extend to the sampled state or the reached state in other tree
       *  @return if success of fail to find the IK solution
       */
    int extend(bool if_tree_a, bool if_sample, double& totalExtendTime,  long& totalExtendCounts,
               std::vector<flann::Index<flann::L2<double>>>& flann_index);

    /**
       *  @brief  Check if the state saved in the tree with index "try_index" can extend tothe goal state( collision and ik)
       *  @param  if_tree_a  indicating the tree a or tree b
       *  @param  goal_state_vec  extending four dimension goal state
       *  @param  try_index  test state index in the tree
       *  @return if success of fail
   */
    bool extend_check(bool if_tree_a, const Eigen::Vector4d& goal_state_vec, int& try_index, double &remain_dis,
                      std::vector<flann::Index<flann::L2<double>>>& flann_index);

    bool extend_collision_check(bool if_tree_a, bool if_master, Eigen::Matrix<double, 7, 1> goal_matrix, int try_index);

    /**
       *  @brief  Find the IK solution for the goal pose
       *  @param  goal_vector  the four dimension goal state that the tree tries to extend to
       *  @param  goal_rot  the goal rotation
       *  @param  qs_state  the state that will be changed to the solved joint angles if success
       *  @param  qs_task_state_vector  input the current four dimension state and will be changed to the correspoing four dimension state of the
       *                                solved joint angles if success
       *  @param  qs_matrix  input the current joint angles and will be changed to the solved joint angles if success
       *  @return if success of fail to find the IK solution
       */
    bool solveIK(const Eigen::Vector4d & goal_vector,
                  const KDL::Rotation & goal_rot,
                  robot_state::RobotState &qs_state,
                  Eigen::Vector4d &qs_task_state_vector,
                  Eigen::Matrix<double, 7, 1> &qs_matrix,
                  bool is_master);

    /**
       *  @brief  Get the four dimension state from a RobotState
       *  @param  state  the operate state
       *  @param  is_master  if the master arm or the slave arm
       *  @return the four dimension state
       */
    Eigen::Vector4d get_task_state_vector(robot_state::RobotState &state, bool is_master);

    /**
       *  @brief  Get the joint angles matrix from a RobotState
       *  @param  state  the operate state
       *  @param  is_master  if the master arm or the slave arm
       *  @return the joint angles matrix
       */
    Eigen::Matrix<double, 14, 1> get_joint_angles_matrix(const robot_state::RobotState &state);
    
    /**
       *  @brief  Compute the basic manipulability
       *  @param  state  the operate state
       *  @param  is_master  if the master arm or the slave arm
       *  @return the manipulability
       */
    double manipulability(robot_state::RobotState & robot_state, bool is_master);

    /**
       *  @brief  Compute the manipulability gradient by finite difference method
       *  @param  state  the operate state
       *  @param  is_master  if the master arm or the slave arm
       *  @return the manipulability gradient
       */
    Eigen::Matrix<double, 7, 1> manipuGradient(robot_state::RobotState & robot_state, bool is_master);

    /**
       *  @brief  add a state to the rrt tree
       *  @param  masterAngles  master joint angles eigen matrix
       *  @param  slaveAngles  slave joint angles eigen matrix
       *  @param  masterState  master four dimension state vector
       *  @param  slaveState  slave four dimension state vector
       *  @param  robot_state  the moveit robot state
       *  @param  parrentIndex  the parent index
       *  @param  if_tree_a  if the a tree or b tree
       *  @param  cost  the cost from the tree root to this state
       *  @return the index of this state in the tree
      */
    int addState(const Eigen::Matrix<double, 7, 1> &masterAngles,
                const Eigen::Matrix<double, 7, 1> &slaveAngles,
                const Eigen::Vector4d &masterState,
                const Eigen::Vector4d &slaveState,
                const robot_state::RobotState &robot_state,
                int parrentIndex, bool if_tree_a, double cost);


public:
    //规划结果
    std::vector<robot_state::RobotState> _planning_result;
//    std::vector<size_t> _planning_result_index;
//    std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> _planning_result_task_state_vector;
    //std::vector<Eigen::Matrix<double, 14, 1>> _planning_result_joint_angles; 放在了 base class


    //储存生成的 RRT树，在_a_rrt_tree_robotstate容器中保存节点的父亲节点
    std::vector<std::pair<robot_state::RobotState, size_t>> _tree_robotstate[2];
    std::vector< std::pair<Eigen::Matrix<double, 14, 1>,double> > _tree_joints_angle_matrix[2];
    std::vector< std::pair<Eigen::Vector4d, Eigen::Vector4d>> _tree_task_space_vector[2];

    /**
       *  @brief  class constructor
       *  @param  nh  node handler of this node
       *  @param  seed  random seed for moveit generating random states
       *  @param  base_frame  frame for visual tools
       */
    TaskSpacePlanner(const ros::NodeHandle &nh, std::string base_frame = "base");
    ~TaskSpacePlanner();

    /**
       *  @brief  plan the path from the start state to the goal state
       *  @return find a feasible path or not
       */
    bool plan();

    /**
       *  @brief  plan the path from the start state to the goal state
       *  @param  outputFile  record the planning time and trajectory cost
       *  @return find a feasible path or not
       */
    bool plan(std::ofstream &outputFile);

    /**
       *  @brief  show the planning result by using visual tools
       */
    void showPath();

    /**
       *  @brief  the interface to get the tree data
       *  @param  if_a_tree  if tree a or tree b
       *  @return the const reference of the tree data
       */
    const std::vector<std::pair<robot_state::RobotState, size_t>> & get_tree_state_vector(bool if_a_tree);

    /**
       *  @brief  set parameters of the planner
       *  @param  biProbility  the probablity of samping randomly against choosing goal state directly
       *  @param  maxSampleTimes  max sample times
       *  @param  taskStep  extend step lengthe in task space
       *  @param  constrainIndex  the dual arm constrain index
       *  @param  startAngles  the precomputed start angles of both arms
       *  @param  goalAngles  the precomputed goal angles of both arms
       *  @param  displayStartState  if display the start state
       *  @param  if_manipuGrand  if activate manipulability inprovement in IK solver
       *  @param  if_rrt_star  if activate rrt* method
       */
    void initPara(int seed,
                  double biProbility,
                  int maxSampleTimes,
                  double maxPlanningTime,
                  double taskStep,
                  size_t constrainIndex,
                  const std::vector<double>& startAngles,
                  const std::vector<double>& goalAngles,
                  bool displayStartState,
                  bool if_manipuGrand,
                  bool if_rrt_star,
                  bool if_informed);

};


PathPlanner* creatPathPlanner(TaskSpacePlanner* tsp);

#endif
