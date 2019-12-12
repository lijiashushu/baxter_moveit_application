
#include <baxter_moveit_application/DualCBiRRTEuler/dual_cbirrt.h>
#include <limits.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <math.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_common.h>
#include <random_numbers/random_numbers.h>
#include <kdl/frames.hpp>

#define PI 3.1415926
#define SEED 2
#define ALPHA 0.05

DualCBiRRT::DualCBiRRT(double probability, int seed, double alpha, size_t test_pose_num, planning_scene::PlanningScenePtr& planning_scene_ptr, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group)
:_random_distribution(probability),_seed(seed), rng(seed), _alpha(alpha), _test_pose_num(test_pose_num), planning_scene_ptr(planning_scene_ptr), planning_group(planning_group), slave_group(slave_group){

    collision_robot = planning_scene_ptr->getCollisionRobot();
    collision_world = planning_scene_ptr->getCollisionWorld();
    robot_model_ptr = planning_scene_ptr->getRobotModel();
    
    const std::vector<std::string>& slave_joint_names = slave_group->getVariableNames();
    std::vector<double> slave_joint_max_bounds;
    std::vector<double> slave_joint_min_bounds;
    for(size_t i=0; i<slave_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = robot_model_ptr->getVariableBounds(slave_joint_names[i]);
        slave_joint_max_bounds.push_back(tmp_bounds.max_position_);
        slave_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    slave_joint_pos_bounds.first = slave_joint_max_bounds;
    slave_joint_pos_bounds.second = slave_joint_min_bounds;

    const std::vector<std::string>& master_joint_names = planning_group->getVariableNames();
    std::vector<double> master_joint_max_bounds;
    std::vector<double> master_joint_min_bounds;
    for(size_t i=0; i<master_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = robot_model_ptr->getVariableBounds(master_joint_names[i]);
        master_joint_max_bounds.push_back(tmp_bounds.max_position_);
        master_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    master_joint_pos_bounds.first = master_joint_max_bounds;
    master_joint_pos_bounds.second = master_joint_min_bounds;

    for(size_t i=0; i<8; i++) {
        _master_link_model_set.insert(robot_model_ptr->getLinkModel(_master_link_all_collision_names[i]));
        _slave_link_model_set.insert(robot_model_ptr->getLinkModel(_slave_link_all_collision_names[i]));
    }
    
    std::vector<moveit_msgs::CollisionObject> collision_ob_info_msgs;
    planning_scene_ptr->getCollisionObjectMsgs(collision_ob_info_msgs);
    std::string ob_name;
    Eigen::Vector3d ob_origin;
    for(size_t i=0; i<collision_ob_info_msgs.size(); i++){
        ob_name = collision_ob_info_msgs[i].id;
        ob_origin(0) = collision_ob_info_msgs[i].primitive_poses[0].position.x;
        ob_origin(1) = collision_ob_info_msgs[i].primitive_poses[0].position.y;
        ob_origin(2) = collision_ob_info_msgs[i].primitive_poses[0].position.z;
        
        collision_object_origin.insert(std::make_pair(ob_name, ob_origin));
    }
    _random_engine.seed(time(0));
}

DualCBiRRT::~DualCBiRRT(){}

void DualCBiRRT::sample_task_state(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Vector4d & random_state_vector) {
    if(_random_distribution(_random_engine)){
        //伯努利分布
        random_state.setToRandomPositions(planning_group, rng);
    }
    else{
        random_state = goal_state;
    }
    random_state.update();
//    random_state.setToRandomPositions(planning_group, rng);
    std::vector<double> random_state_value;
    random_state.copyJointGroupPositions(planning_group, random_state_value);
    for(size_t i=0; i<random_state_value.size(); i++) {
        random_state_value_matrix[i] = random_state_value[i];
    }
    random_state_vector.head(3) = random_state.getGlobalLinkTransform("left_gripper").translation();
    Eigen::Matrix3d tmp_rot_matrix = random_state.getGlobalLinkTransform("left_gripper").rotation();
    KDL::Rotation tmp;
    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            tmp(i,j) = tmp_rot_matrix(i,j);
        }
    }
    double foo1, foo2;
    tmp.GetEulerZYX(random_state_vector(3), foo1, foo2);
}

size_t DualCBiRRT::near_tree_task_space(Eigen::Vector4d & random_task_state, Eigen::Vector4d & nearest_node_task_state, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, bool if_tree_a)
{
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    if(if_tree_a){
        for(size_t i=0; i<_a_rrt_tree_task_space_state.size(); i++){
            Eigen::Vector4d dis_matrix = random_task_state - _a_rrt_tree_task_space_state[i];
            double distance = dis_matrix.norm();
            if(distance < minum_dis){
                minum_dis = distance;
                minum_index = i;
            }
        }
        nearest_node_task_state = _a_rrt_tree_task_space_state[minum_index];
        nearest_node_matrix = _a_rrt_tree_matrix[minum_index].head(7);
    }
    else{
        for(size_t i=0; i<_b_rrt_tree_task_space_state.size(); i++){
            Eigen::Vector4d dis_matrix = random_task_state - _b_rrt_tree_task_space_state[i];
            double distance = dis_matrix.norm();
            if(distance < minum_dis){
                minum_dis = distance;
                minum_index = i;
            }
        }
        nearest_node_task_state = _b_rrt_tree_task_space_state[minum_index];
        nearest_node_matrix = _b_rrt_tree_matrix[minum_index].head(7);
    }
    return minum_index;
}

void DualCBiRRT::constraint_extend_task_space_dir(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, PerformanceIndexOneSample & perdex_one_sample, bool if_tree_a, bool if_sample, Eigen::Vector4d& reached_task_pos, size_t & reached_index){
    size_t qs_old_index = nearst_node_index;

    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;

    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    std::vector<double> random_vector(7);

    for(size_t i=0; i < qs_vector.size(); i++){
        qs_vector[i] =  qs_matrix[i];
        qs_old_vector[i] =  qs_old_matrix[i];
        random_vector[i] = random_state_value_matrix[i];
    }

    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    random_state.setJointGroupPositions(planning_group, random_vector);
    random_state.update();
    qs_state.setJointGroupPositions(planning_group, qs_vector);
    qs_state.update();
    qs_old_state.setJointGroupPositions(planning_group, qs_old_vector);
    qs_old_state.update();

    Eigen::Vector4d random_task_state_vector;
    Eigen::Vector4d qs_task_state_vector;
    Eigen::Vector4d qs_old_task_state_vector;
    Eigen::Vector4d task_direction_vector;
    Eigen::Vector4d extend_goal_task_state_vector;
    Eigen::Vector3d extend_goal_task_state_euler;
    Eigen::Matrix3d extend_goal_rot_matrix;


    Eigen::Vector3d random_task_state_euler;
    Eigen::Vector3d qs_task_state_euler;
    Eigen::Vector3d qs_old_task_state_euler;
    Eigen::Matrix3d random_task_state_rot_matrix;
    Eigen::Matrix3d qs_task_state_rot_matrix;
    Eigen::Matrix3d qs_old_task_state_rot_matrix;
    KDL::Rotation random_task_state_kdl;
    KDL::Rotation qs_task_state_kdl;
    KDL::Rotation qs_old_task_state_kdl;

    random_task_state_vector.head(3) = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector.head(3)  = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector.head(3)  = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    random_task_state_rot_matrix = random_state.getGlobalLinkTransform("left_gripper").rotation();
    qs_task_state_rot_matrix = qs_state.getGlobalLinkTransform("left_gripper").rotation();
    qs_old_task_state_rot_matrix = qs_old_state.getGlobalLinkTransform("left_gripper").rotation();

    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            random_task_state_kdl(i, j) = random_task_state_rot_matrix(i,j);
            qs_task_state_kdl(i,j) = qs_task_state_rot_matrix(i,j);
            qs_old_task_state_kdl(i,j) = qs_old_task_state_rot_matrix(i,j);
        }
    }
    random_task_state_kdl.GetEulerZYX(random_task_state_euler(0), random_task_state_euler(1), random_task_state_euler(2));
    qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
    qs_old_task_state_kdl.GetEulerZYX(qs_old_task_state_euler(0), qs_old_task_state_euler(1), qs_old_task_state_euler(2));
    random_task_state_vector(3) = random_task_state_euler(0);
    qs_task_state_vector(3) = qs_task_state_euler(0);
    qs_old_task_state_vector(3) = qs_old_task_state_euler(0);

    if(random_task_state_vector(3) > _yaw_max[_test_pose_num]){
        random_task_state_vector(3) = _yaw_max[_test_pose_num];
    }
    else if(random_task_state_vector(3) < _yaw_min[_test_pose_num]){
        random_task_state_vector(3) = _yaw_min[_test_pose_num];
    }
    else{;}

    if(if_tree_a){
        ROS_WARN("a_tree");
    }
    else{
        ROS_WARN("b_tree");
    }

    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;

    Eigen::Matrix<double, 6, 1> task_delta_vector;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;

    //计算 slave 相关变量
    Eigen::Matrix<double, 7, 1> current_slave_angles_matrix;
    if(if_tree_a){
        current_slave_angles_matrix = _a_rrt_tree_matrix[nearst_node_index].tail(7);
    }
    else{
        current_slave_angles_matrix = _b_rrt_tree_matrix[nearst_node_index].tail(7);
    }

    Eigen::Matrix<double, 7, 1> computed_slave_angles_matrix;
    std::vector<double> slave_angles_vector(7);
    Eigen::Matrix<double, 14, 1> matrix_tree_element;

    //*******************************************向约束空间投影用到的变量*****************************************
    bool project_success = true;

    Eigen::Vector3d pos_error;
    Eigen::Vector3d euler_error;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;



    int performance_index_extend_num = 0;
    int extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if(if_sample){
            extend_num++;
            if(extend_num > 10){
                reached_state_matrix = qs_matrix;
                reached_task_pos = qs_task_state_vector;
                reached_index = qs_old_index;
                break;
            }
        }
        if ((random_task_state_vector - qs_task_state_vector).norm() < 0.005){
            ROS_WARN("!!!!!!!!!");

            if(if_tree_a){
                ROS_WARN("a_tree");
            }
            else{
                ROS_WARN("b_tree");
            }
            reached_state_matrix = qs_matrix;
            reached_task_pos = qs_task_state_vector;
            reached_index = qs_old_index;
            break;
        }
        else if((random_task_state_vector - qs_task_state_vector).norm() > (random_task_state_vector-qs_old_task_state_vector).norm()){

            reached_state_matrix = qs_old_matrix;
            reached_task_pos = qs_old_task_state_vector;
            if(if_tree_a){
                reached_index = _a_rrt_tree_state[qs_old_index].second;
                _a_rrt_tree_matrix.pop_back();
                _a_rrt_tree_state.pop_back();
                _a_rrt_tree_task_space_state.pop_back();
            }
            else{
                reached_index = _b_rrt_tree_state[qs_old_index].second;
                _b_rrt_tree_matrix.pop_back();
                _b_rrt_tree_state.pop_back();
                _b_rrt_tree_task_space_state.pop_back();
            }
            break;
        }
        else{
            _extend_times++;
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
                extend_goal_task_state_euler(0) = extend_goal_task_state_vector(3);
                extend_goal_task_state_euler(1) = 0.0;
                extend_goal_task_state_euler(2) = 1.57;
                Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[2], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[0], Eigen::Vector3d::UnitZ()));
                extend_goal_rot_matrix = yaw_angle*pitch_angle*roll_angle;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
                extend_goal_task_state_euler(0) = extend_goal_task_state_vector(3);
                extend_goal_task_state_euler(1) = 0.0;
                extend_goal_task_state_euler(2) = 1.57;
                Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[2], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[0], Eigen::Vector3d::UnitZ()));
                extend_goal_rot_matrix = yaw_angle*pitch_angle*roll_angle;
            }

            //*********************************performance_index 所用到的变量的定义********************************
            performance_index_extend_num++;
            PerformanceIndexOneExtend perdex_one_extend;
            perdex_one_extend.extend_num = performance_index_extend_num;
            ros::Time extend_start_time = ros::Time::now();
            ros::Time extend_end_time;
            ros::Time project_start_time;
            ros::Time project_end_time;
            ros::Time ik_start_time;
            ros::Time ik_end_time;
            //**************************************************************************************************
            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                pos_error = extend_goal_task_state_vector.head(3) - qs_task_state_vector.head(3);
                euler_error = extend_goal_task_state_euler - qs_task_state_euler;

                if((extend_goal_task_state_vector-qs_task_state_vector).norm() < 0.005 && euler_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{
                    error_rot_matrix = extend_goal_rot_matrix * (qs_task_state_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector.tail(3) = error_axis * error_angle;
                    task_delta_vector.head(3) = pos_error;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_pinv = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_pinv  * task_delta_vector;
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    for(size_t i=0; i<7; i++){
                        qs_vector[i] = qs_matrix[i];
                    }
                    qs_state.setJointGroupPositions(planning_group, qs_vector);
                    qs_state.update();

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                    else{
                        const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                        qs_task_state_vector.head(3) = end_pose.translation();
                        qs_task_state_rot_matrix = end_pose.rotation();
                        for(size_t i=0; i<3; i++){
                            for(size_t j=0; j<3; j++){
                                qs_task_state_kdl(i,j) = qs_task_state_rot_matrix(i,j);
                            }
                        }
                        qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
                        qs_task_state_vector(3) = qs_task_state_euler(0);
                    }
                }
            }
            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;

                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult res;
                req.group_name = "left_arm";
                collision_world->checkRobotCollision(req, res, *collision_robot, qs_state);

                if(!res.collision){
//                if(planning_scene_ptr->isStateColliding("left_arm"))
                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, perdex_one_extend);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();

//                        collision_detection::DistanceRequest collision_req_slave;
//                        collision_req_slave.enable_nearest_points = true;
//                        collision_req_slave.enable_signed_distance = true;
//                        collision_req_slave.active_components_only = &slave_link_model_set;
//                        collision_req_slave.group_name = "right_arm";
//                        collision_req_slave.type = collision_detection::DistanceRequestType::LIMITED;
//                        collision_detection::DistanceResult collision_res_slave;
//                        collision_detection::DistanceResult new_collision_res_slave;
//                        world_FCL.distanceRobot(collision_req_slave, collision_res_slave, *robot, qs_state);

                        collision_detection::CollisionRequest req;
                        req.group_name = "right_arm";
                        collision_detection::CollisionResult res;
                        collision_world->checkRobotCollision(req, res, *collision_robot, qs_state);

                        if(!res.collision){
                            perdex_one_extend.no_collide=1;

                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
                            _simple_dir_manipulability_sum+= simple_dir_manipulability_compute(qs_matrix, computed_slave_angles_matrix, if_tree_a);
                            _extend_success_times++;
                            perdex_one_extend.extend_success=1;
                            matrix_tree_element.head(7) = qs_matrix;
                            matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                            if(if_tree_a){
                                _a_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算
                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _a_rrt_tree_state.push_back(tmp);
                                qs_old_index = _a_rrt_tree_state.size() - 1;
                                _a_rrt_tree_task_space_state.push_back(qs_task_state_vector);
                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算
                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                _b_rrt_tree_task_space_state.push_back(qs_task_state_vector);
                            }
                        }
                        else{//slave collide fail
                            reached_state_matrix = qs_old_matrix;
                            reached_task_pos = qs_old_task_state_vector;
                            reached_index = qs_old_index;
                            perdex_one_extend.no_collide=0;
                            extend_end_time = ros::Time::now();
                            perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                            if(if_tree_a){
                                perdex_one_sample.tree_a.push_back(perdex_one_extend);
                            }
                            else{
                                perdex_one_sample.tree_b.push_back(perdex_one_extend);
                            }
                            _extend_success_rate.push_back(_extend_success_times / _extend_times);
                            _simple_dir_manipulability_vec.push_back(_simple_dir_manipulability_sum);
                            _rate_simple_manipulability_vec.push_back(_simple_dir_manipulability_sum * (_extend_success_times / _extend_times));
                            break;
                        }
                    }
                    else{//IK fail
                        std::cout<<"ik fail"<<std::endl;
                        reached_state_matrix = qs_old_matrix;
                        reached_task_pos = qs_old_task_state_vector;
                        reached_index = qs_old_index;
                        perdex_one_extend.ik_success = 0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }
                        _extend_success_rate.push_back(_extend_success_times / _extend_times);
                        _simple_dir_manipulability_vec.push_back(_simple_dir_manipulability_sum);
                        _rate_simple_manipulability_vec.push_back(_simple_dir_manipulability_sum * (_extend_success_times / _extend_times));
                        break;
                    }
                }
                else{//master collide fail
                    std::cout<<"master collide fail"<<std::endl;
                    reached_state_matrix = qs_old_matrix;
                    reached_task_pos = qs_old_task_state_vector;
                    reached_index = qs_old_index;
                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }
                    _extend_success_rate.push_back(_extend_success_times / _extend_times);
                    _simple_dir_manipulability_vec.push_back(_simple_dir_manipulability_sum);
                    _rate_simple_manipulability_vec.push_back(_simple_dir_manipulability_sum * (_extend_success_times / _extend_times));
                    break;
                }
            }
            else{//project fail
                std::cout<<"project fail"<<std::endl;
                reached_state_matrix = qs_old_matrix;
                reached_task_pos = qs_old_task_state_vector;
                reached_index = qs_old_index;
                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }
                _extend_success_rate.push_back(_extend_success_times / _extend_times);
                _simple_dir_manipulability_vec.push_back(_simple_dir_manipulability_sum);
                _rate_simple_manipulability_vec.push_back(_simple_dir_manipulability_sum * (_extend_success_times / _extend_times));
                break;
            }
            extend_end_time = ros::Time::now();
            perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
            if(if_tree_a){
                perdex_one_sample.tree_a.push_back(perdex_one_extend);
            }
            else{
                perdex_one_sample.tree_b.push_back(perdex_one_extend);
            }
        }
        _extend_success_rate.push_back(_extend_success_times / _extend_times);
        _simple_dir_manipulability_vec.push_back(_simple_dir_manipulability_sum);
        _rate_simple_manipulability_vec.push_back(_simple_dir_manipulability_sum * (_extend_success_times / _extend_times));
    }
    std::cout<<"outout!!"<<std::endl;
}

void DualCBiRRT::constraint_extend_task_space_dir_try_adjust(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, PerformanceIndexOneSample & perdex_one_sample, bool if_tree_a, bool if_sample, Eigen::Vector4d& reached_task_pos, size_t & reached_index){
    bool if_master_must_collide;
    bool if_slave_must_collide;

    size_t qs_old_index = nearst_node_index;

    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;

    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    std::vector<double> random_vector(7);

    for(size_t i=0; i < qs_vector.size(); i++){
        qs_vector[i] =  qs_matrix[i];
        qs_old_vector[i] =  qs_old_matrix[i];
        random_vector[i] = random_state_value_matrix[i];
    }

    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    random_state.setJointGroupPositions(planning_group, random_vector);
    random_state.update();
    qs_state.setJointGroupPositions(planning_group, qs_vector);
    qs_state.update();
    qs_old_state.setJointGroupPositions(planning_group, qs_old_vector);
    qs_old_state.update();

    Eigen::Vector4d random_task_state_vector;
    Eigen::Vector4d qs_task_state_vector;
    Eigen::Vector4d qs_old_task_state_vector;
    Eigen::Vector4d task_direction_vector;
    Eigen::Vector4d extend_goal_task_state_vector;
    Eigen::Vector3d extend_goal_task_state_euler;
    Eigen::Matrix3d extend_goal_rot_matrix;


    Eigen::Vector3d random_task_state_euler;
    Eigen::Vector3d qs_task_state_euler;
    Eigen::Vector3d qs_old_task_state_euler;
    Eigen::Matrix3d random_task_state_rot_matrix;
    Eigen::Matrix3d qs_task_state_rot_matrix;
    Eigen::Matrix3d qs_old_task_state_rot_matrix;
    KDL::Rotation random_task_state_kdl;
    KDL::Rotation qs_task_state_kdl;
    KDL::Rotation qs_old_task_state_kdl;

    random_task_state_vector.head(3) = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector.head(3)  = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector.head(3)  = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    random_task_state_rot_matrix = random_state.getGlobalLinkTransform("left_gripper").rotation();
    qs_task_state_rot_matrix = qs_state.getGlobalLinkTransform("left_gripper").rotation();
    qs_old_task_state_rot_matrix = qs_old_state.getGlobalLinkTransform("left_gripper").rotation();

    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            random_task_state_kdl(i, j) = random_task_state_rot_matrix(i,j);
            qs_task_state_kdl(i,j) = qs_task_state_rot_matrix(i,j);
            qs_old_task_state_kdl(i,j) = qs_old_task_state_rot_matrix(i,j);
        }
    }
    random_task_state_kdl.GetEulerZYX(random_task_state_euler(0), random_task_state_euler(1), random_task_state_euler(2));
    qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
    qs_old_task_state_kdl.GetEulerZYX(qs_old_task_state_euler(0), qs_old_task_state_euler(1), qs_old_task_state_euler(2));
    random_task_state_vector(3) = random_task_state_euler(0);
    qs_task_state_vector(3) = qs_task_state_euler(0);
    qs_old_task_state_vector(3) = qs_old_task_state_euler(0);

    if(random_task_state_vector(3) > _yaw_max[_test_pose_num]){
        random_task_state_vector(3) = _yaw_max[_test_pose_num];
    }
    else if(random_task_state_vector(3) < _yaw_min[_test_pose_num]){
        random_task_state_vector(3) = _yaw_min[_test_pose_num];
    }
    else{;}

    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;

    Eigen::Matrix<double, 6, 1> task_delta_vector;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;

    //计算 slave 相关变量
    Eigen::Matrix<double, 7, 1> current_slave_angles_matrix;
    if(if_tree_a){
        current_slave_angles_matrix = _a_rrt_tree_matrix[nearst_node_index].tail(7);
    }
    else{
        current_slave_angles_matrix = _b_rrt_tree_matrix[nearst_node_index].tail(7);
    }

    Eigen::Matrix<double, 7, 1> computed_slave_angles_matrix;
    std::vector<double> slave_angles_vector(7);
    Eigen::Matrix<double, 14, 1> matrix_tree_element;

    //*******************************************向约束空间投影用到的变量*****************************************
    bool project_success = true;

    Eigen::Vector3d pos_error;
    Eigen::Vector3d euler_error;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;



    int performance_index_extend_num = 0;
    int extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if(if_sample){
            extend_num++;
            if(extend_num > 10){
                reached_state_matrix = qs_matrix;
                reached_task_pos = qs_task_state_vector;
                reached_index = qs_old_index;
                break;
            }
        }
        if ((random_task_state_vector - qs_task_state_vector).norm() < 0.005){
            reached_state_matrix = qs_matrix;
            reached_task_pos = qs_task_state_vector;
            reached_index = qs_old_index;
            break;
        }
        else if((random_task_state_vector - qs_task_state_vector).norm() > (random_task_state_vector-qs_old_task_state_vector).norm()){
            reached_state_matrix = qs_old_matrix;
            reached_task_pos = qs_old_task_state_vector;
            if(if_tree_a){
                reached_index = _a_rrt_tree_state[qs_old_index].second;
                _a_rrt_tree_matrix.pop_back();
                _a_rrt_tree_state.pop_back();
                _a_rrt_tree_task_space_state.pop_back();

//                a_tree_considered_master_ob_jac.pop_back();
//                a_tree_considered_master_ob_dir.pop_back();
//                a_tree_considered_master_ob_dis.pop_back();
//                a_tree_considered_slave_ob_jac.pop_back();
//                a_tree_considered_slave_ob_dir.pop_back();
//                a_tree_considered_slave_ob_dis.pop_back();
                _a_tree_master_dis_map.pop_back();
                _a_tree_slave_dis_map.pop_back();
                _a_tree_has_been_extended.pop_back();
            }
            else{
                reached_index = _b_rrt_tree_state[qs_old_index].second;
                _b_rrt_tree_matrix.pop_back();
                _b_rrt_tree_state.pop_back();
                _b_rrt_tree_task_space_state.pop_back();

//                b_tree_considered_master_ob_jac.pop_back();
//                b_tree_considered_master_ob_dir.pop_back();
//                b_tree_considered_master_ob_dis.pop_back();
//                b_tree_considered_slave_ob_jac.pop_back();
//                b_tree_considered_slave_ob_dir.pop_back();
//                b_tree_considered_slave_ob_dis.pop_back();
                _b_tree_master_dis_map.pop_back();
                _b_tree_slave_dis_map.pop_back();
                _b_tree_has_been_extended.pop_back();
            }

            break;
        }
        else{
            _extend_times++;
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
                extend_goal_task_state_euler(0) = extend_goal_task_state_vector(3);
                extend_goal_task_state_euler(1) = 0.0;
                extend_goal_task_state_euler(2) = 1.57;
                Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[2], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[0], Eigen::Vector3d::UnitZ()));
                extend_goal_rot_matrix = yaw_angle*pitch_angle*roll_angle;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
                extend_goal_task_state_euler(0) = extend_goal_task_state_vector(3);
                extend_goal_task_state_euler(1) = 0.0;
                extend_goal_task_state_euler(2) = 1.57;
                Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[2], Eigen::Vector3d::UnitX()));
                Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[1], Eigen::Vector3d::UnitY()));
                Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(extend_goal_task_state_euler[0], Eigen::Vector3d::UnitZ()));
                extend_goal_rot_matrix = yaw_angle*pitch_angle*roll_angle;
            }
//            std::cout<<"extend_goal_task_state_vector\n"<<extend_goal_task_state_vector.transpose()<<std::endl;
//            std::cout<<"random_task_state_vector\n"<<random_task_state_vector.transpose()<<std::endl;
//            std::cout<<"qs_task_state_vector\n"<<qs_task_state_vector.transpose()<<std::endl;
//            std::cout<<"(random_task_state_vector - qs_task_state_vector).norm()  "<<(random_task_state_vector - qs_task_state_vector).norm()<<std::endl;
//            std::cout<<"(extend_goal_task_state_vector-qs_task_state_vector).norm()  "<<(extend_goal_task_state_vector-qs_task_state_vector).norm()<<std::endl;

            //*********************************performance_index 所用到的变量的定义********************************
            performance_index_extend_num++;
            PerformanceIndexOneExtend perdex_one_extend;
            perdex_one_extend.extend_num = performance_index_extend_num;
            ros::Time extend_start_time = ros::Time::now();
            ros::Time extend_end_time;
            ros::Time project_start_time;
            ros::Time project_end_time;
            ros::Time ik_start_time;
            ros::Time ik_end_time;
            //**************************************************************************************************
            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                pos_error = extend_goal_task_state_vector.head(3) - qs_task_state_vector.head(3);
                euler_error = extend_goal_task_state_euler - qs_task_state_euler;

                if((extend_goal_task_state_vector-qs_task_state_vector).norm() < 0.005 && euler_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{
                    error_rot_matrix = extend_goal_rot_matrix * (qs_task_state_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector.tail(3) = error_axis * error_angle;
                    task_delta_vector.head(3) = pos_error;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_pinv = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_pinv  * task_delta_vector;
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    for(size_t i=0; i<7; i++){
                        qs_vector[i] = qs_matrix[i];
                    }
                    qs_state.setJointGroupPositions(planning_group, qs_vector);
                    qs_state.update();

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                    else{
                        const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                        qs_task_state_vector.head(3) = end_pose.translation();
                        qs_task_state_rot_matrix = end_pose.rotation();
                        for(size_t i=0; i<3; i++){
                            for(size_t j=0; j<3; j++){
                                qs_task_state_kdl(i,j) = qs_task_state_rot_matrix(i,j);
                            }
                        }
                        qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
                        qs_task_state_vector(3) = qs_task_state_euler(0);
                    }
                }
            }

            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;

                //begin********************先计算左臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************
                collision_detection::DistanceMap master_dis_map;

                collision_detection::DistanceRequest collision_req_master_world;
                collision_req_master_world.enable_nearest_points = true;
                collision_req_master_world.enable_signed_distance = true;
                collision_req_master_world.active_components_only = &_master_link_model_set;
                collision_req_master_world.group_name = "left_arm";
                collision_req_master_world.type = collision_detection::DistanceRequestType::LIMITED;

                collision_detection::DistanceResult collision_res_master_world;
                collision_detection::DistanceResult new_collision_res_master_world;


                collision_world->distanceRobot(collision_req_master_world, collision_res_master_world, *collision_robot, qs_state);
                collision_detection::DistanceResultsData master_min_dis_world = collision_res_master_world.minimum_distance;
                double master_collide_dis_value = master_min_dis_world.distance;
                std::cout<<"master_min_dis_world  "<<master_min_dis_world.distance<<std::endl;
                Eigen::Vector3d master_collide_pos = master_min_dis_world.nearest_points[1];
                
                std::string master_collide_link_name  = master_min_dis_world.link_names[1];
                const robot_state::LinkModel* master_collision_link = qs_state.getLinkModel(master_collide_link_name);
                Eigen::MatrixXd master_collision_point_jac;
                qs_state.getJacobian(planning_group, master_collision_link, master_collide_pos, master_collision_point_jac);

                if(collision_res_master_world.minimum_distance.distance < 0.02){
                    Eigen::Vector3d master_robot_collision_point;
                    Eigen::Vector3d master_object_collision_point;
                    master_robot_collision_point = qs_state.getGlobalLinkTransform(master_min_dis_world.link_names[1]) * master_collide_pos;
                    master_object_collision_point = collision_object_origin[master_min_dis_world.link_names[0]] + master_min_dis_world.nearest_points[0];
                    Eigen::Vector3d master_collide_normal = master_robot_collision_point - master_object_collision_point;
                    master_collide_normal.normalize();
                    

                    double self_motion_at_collision_point_velocity_magnititude;
                    Eigen::Vector3d self_motion_at_collision_point_velocity;
                    Eigen::MatrixXd self_motion_vec;

                    Eigen::MatrixXd master_jac = qs_state.getJacobian(planning_group);
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(master_jac, Eigen::ComputeFullV);
                    Eigen::MatrixXd V = svd_holder.matrixV();
//                    std::cout<<"V\n  "<<V<<std::endl;
                    self_motion_vec = V.rightCols(1);

                    //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0
                    Eigen::Matrix<double ,6, 1> tmp;
                    tmp = (master_collision_point_jac * self_motion_vec);
                    self_motion_at_collision_point_velocity = tmp.head(3);
                    self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(master_collide_normal);
                    std::cout<<"self_motion_at_collision_point_velocity_magnititude  "<<self_motion_at_collision_point_velocity_magnititude<<std::endl;

                    std::cout<<"ratio  "<<-1.2 * master_collide_dis_value / self_motion_at_collision_point_velocity_magnititude<<std::endl;
                    qs_matrix +=  (-1.2 * master_collide_dis_value / self_motion_at_collision_point_velocity_magnititude) * self_motion_vec;

                    for(size_t i=0; i<7; i++){
                        qs_vector[i] = qs_matrix[i];
                    }
                    qs_state.setJointGroupPositions(planning_group, qs_vector);
                    qs_state.update();
                    if(qs_state.satisfiesBounds(0.05)){
                        std::cout<<"new state satisfiesBounds"<<std::endl;
                        std::cout<<qs_state.getGlobalLinkTransform("left_gripper").translation().transpose()<<std::endl;
                        collision_world->distanceRobot(collision_req_master_world, new_collision_res_master_world, *collision_robot, qs_state);
                        std::cout<<"new_master_min_dis_world  "<<new_collision_res_master_world.minimum_distance.distance<<std::endl;
                        if(!(new_collision_res_master_world.minimum_distance.distance < 0.02)){
                            master_min_dis_world = new_collision_res_master_world.minimum_distance;
                            master_collide_dis_value = master_min_dis_world.distance;
                            master_collide_pos = master_min_dis_world.nearest_points[1];
                            master_collide_normal = master_min_dis_world.normal;
                            master_collide_link_name  = master_min_dis_world.link_names[1];
                            const robot_state::LinkModel* new_master_collision_link = qs_state.getLinkModel(master_collide_link_name);
                            qs_state.getJacobian(planning_group, new_master_collision_link, master_collide_pos, master_collision_point_jac);


                            std::cout<<"master fix success!"<<std::endl;
                            ROS_WARN("master fix success!");
                            master_dis_map = new_collision_res_master_world.distances;

                            if_master_must_collide = false;
                            qs_task_state_vector.head(3) = qs_state.getGlobalLinkTransform("left_gripper").translation();
                            qs_task_state_rot_matrix = qs_state.getGlobalLinkTransform("left_gripper").rotation();
                            for(size_t i=0;i<3;i++){
                                for(size_t j=0;j<3;j++){
                                    qs_task_state_kdl(i,j) =  qs_task_state_rot_matrix(i,j);
                                }
                            }
                            qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
                            qs_task_state_vector(3) = qs_task_state_euler(0);
                        }
                        else{
                            std::cout<<"master fix fail!"<<std::endl;
                            if_master_must_collide = true;
                        }
                    }
                    else{
                        std::cout<<"new state doesn't satisfiesBounds"<<std::endl;
                        if_master_must_collide = true;
                    }
                }
                else{
                    master_dis_map = collision_res_master_world.distances;
                    if_master_must_collide = false;
                }
                //end********************先计算左臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************

                if(!if_master_must_collide){
                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, perdex_one_extend);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();

                        //begin********************计算右臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************
                        collision_detection::DistanceMap slave_dis_map;

                        collision_detection::DistanceRequest collision_req_slave_world;
                        collision_req_slave_world.enable_nearest_points = true;
                        collision_req_slave_world.enable_signed_distance = true;
                        collision_req_slave_world.active_components_only = &_slave_link_model_set;
                        collision_req_slave_world.group_name = "right_arm";
                        collision_req_slave_world.type = collision_detection::DistanceRequestType::LIMITED;

                        collision_detection::DistanceResult collision_res_slave_world;
                        collision_detection::DistanceResult new_collision_res_slave_world;

                        collision_world->distanceRobot(collision_req_slave_world, collision_res_slave_world, *collision_robot, qs_state);
                        collision_detection::DistanceResultsData slave_min_dis_world = collision_res_slave_world.minimum_distance;
                        double slave_collide_dis_value = slave_min_dis_world.distance;
                        std::cout<<"slave_min_dis_world  "<<slave_min_dis_world.distance<<std::endl;
                        Eigen::Vector3d slave_collide_pos = slave_min_dis_world.nearest_points[1];
                        std::string slave_collide_link_name  = slave_min_dis_world.link_names[1];
                        const robot_state::LinkModel* slave_collision_link = qs_state.getLinkModel(slave_collide_link_name);
                        Eigen::MatrixXd slave_collision_point_jac;
                        qs_state.getJacobian(slave_group, slave_collision_link, slave_collide_pos, slave_collision_point_jac);

                        if(collision_res_slave_world.minimum_distance.distance < 0.02){
                            Eigen::Vector3d slave_robot_collision_point;
                            Eigen::Vector3d slave_object_collision_point;
                            slave_robot_collision_point = qs_state.getGlobalLinkTransform(slave_min_dis_world.link_names[1]) * slave_collide_pos;
                            slave_object_collision_point = collision_object_origin[slave_min_dis_world.link_names[0]] + slave_min_dis_world.nearest_points[0];
                            Eigen::Vector3d slave_collide_normal = slave_robot_collision_point - slave_object_collision_point;
                            slave_collide_normal.normalize();

                            double self_motion_at_collision_point_velocity_magnititude;
                            Eigen::Vector3d self_motion_at_collision_point_velocity;
                            Eigen::MatrixXd self_motion_vec;


                            Eigen::MatrixXd slave_jac = qs_state.getJacobian(slave_group);
                            Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_jac, Eigen::ComputeFullV);
                            Eigen::MatrixXd V = svd_holder.matrixV();
//                    std::cout<<"V\n  "<<V<<std::endl;
                            self_motion_vec = V.rightCols(1);

                            //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0
                            Eigen::Matrix<double ,6, 1> tmp;
                            tmp = (slave_collision_point_jac * self_motion_vec);
                            self_motion_at_collision_point_velocity = tmp.head(3);
                            self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(slave_collide_normal);
                            std::cout<<"self_motion_at_collision_point_velocity_magnititude  "<<self_motion_at_collision_point_velocity_magnititude<<std::endl;



                            std::cout<<"ratio  "<<-1.2 * slave_collide_dis_value / self_motion_at_collision_point_velocity_magnititude<<std::endl;
                            qs_matrix +=  (-1.2 * slave_collide_dis_value / self_motion_at_collision_point_velocity_magnititude) * self_motion_vec;

                            for(size_t i=0; i<7; i++){
                                qs_vector[i] = qs_matrix[i];
                            }
                            qs_state.setJointGroupPositions(slave_group, qs_vector);
                            qs_state.update();
                            if(qs_state.satisfiesBounds(0.05)){
                                std::cout<<"new state satisfiesBounds"<<std::endl;
                                std::cout<<qs_state.getGlobalLinkTransform("right_gripper").translation().transpose()<<std::endl;
                                collision_world->distanceRobot(collision_req_slave_world, new_collision_res_slave_world, *collision_robot, qs_state);
                                std::cout<<"new_slave_min_dis_world  "<<new_collision_res_slave_world.minimum_distance.distance<<std::endl;
                                if(!(new_collision_res_slave_world.minimum_distance.distance < 0.02)){
                                    slave_min_dis_world = new_collision_res_slave_world.minimum_distance;
                                    slave_collide_dis_value = slave_min_dis_world.distance;
                                    slave_collide_pos = slave_min_dis_world.nearest_points[1];
                                    slave_collide_normal = slave_min_dis_world.normal;
                                    slave_collide_link_name = slave_min_dis_world.link_names[1];
                                    const robot_state::LinkModel* new_slave_collision_link = qs_state.getLinkModel(slave_collide_link_name);
                                    qs_state.getJacobian(slave_group, new_slave_collision_link, slave_collide_pos, slave_collision_point_jac);

                                    slave_dis_map = new_collision_res_slave_world.distances;

                                    std::cout<<"slave fix success!"<<std::endl;
                                    ROS_WARN("slave fix success!");
                                    if_slave_must_collide = false;
                                    qs_task_state_vector.head(3) = qs_state.getGlobalLinkTransform("right_gripper").translation();
                                    qs_task_state_rot_matrix = qs_state.getGlobalLinkTransform("right_gripper").rotation();
                                    for(size_t i=0;i<3;i++){
                                        for(size_t j=0;j<3;j++){
                                            qs_task_state_kdl(i,j) =  qs_task_state_rot_matrix(i,j);
                                        }
                                    }
                                    qs_task_state_kdl.GetEulerZYX(qs_task_state_euler(0), qs_task_state_euler(1), qs_task_state_euler(2));
                                    qs_task_state_vector(3) = qs_task_state_euler(0);
                                }
                                else{
                                    std::cout<<"slave fix fail!"<<std::endl;
                                    if_slave_must_collide = true;
                                }
                            }
                            else{
                                std::cout<<"new state doesn't satisfiesBounds"<<std::endl;
                                if_slave_must_collide = true;
                            }
                        }
                        else{
                            slave_dis_map = collision_res_slave_world.distances;
                            if_slave_must_collide = false;
                        }

                        //end********************计算右臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************

                        if(!if_slave_must_collide){
                            perdex_one_extend.no_collide=1;
                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
//                            _complex_dir_manipulability_sum += manipulability_compute(qs_matrix, computed_slave_angles_matrix, master_collision_point_jac, master_collide_normal, master_collide_dis_value, slave_collision_point_jac, slave_collide_normal, slave_collide_dis_value, if_tree_a);
                            _extend_success_times++;
                            perdex_one_extend.extend_success=1;
                            matrix_tree_element.head(7) = qs_matrix;
                            matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                            if(if_tree_a){
                                _a_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算
                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _a_rrt_tree_state.push_back(tmp);
                                qs_old_index = _a_rrt_tree_state.size() - 1;
                                _a_rrt_tree_task_space_state.push_back(qs_task_state_vector);

//                                a_tree_considered_master_ob_jac.push_back(master_collision_point_jac);
//                                a_tree_considered_master_ob_dir.push_back(master_collide_normal);
//                                a_tree_considered_master_ob_dis.push_back(master_collide_dis_value);
//                                a_tree_considered_slave_ob_jac.push_back(slave_collision_point_jac);
//                                a_tree_considered_slave_ob_dir.push_back(slave_collide_normal);
//                                a_tree_considered_slave_ob_dis.push_back(slave_collide_dis_value);

                                _a_tree_master_dis_map.push_back(master_dis_map);
                                _a_tree_slave_dis_map.push_back(slave_dis_map);
                                _a_tree_has_been_extended.push_back(false);
                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算
                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                _b_rrt_tree_task_space_state.push_back(qs_task_state_vector);

//                                b_tree_considered_master_ob_jac.push_back(master_collision_point_jac);
//                                b_tree_considered_master_ob_dir.push_back(master_collide_normal);
//                                b_tree_considered_master_ob_dis.push_back(master_collide_dis_value);
//                                b_tree_considered_slave_ob_jac.push_back(slave_collision_point_jac);
//                                b_tree_considered_slave_ob_dir.push_back(slave_collide_normal);
//                                b_tree_considered_slave_ob_dis.push_back(slave_collide_dis_value);

                                _b_tree_master_dis_map.push_back(master_dis_map);
                                _b_tree_slave_dis_map.push_back(slave_dis_map);
                                _b_tree_has_been_extended.push_back(false);
                            }
                        }
                        else{//slave collide fail
                            reached_state_matrix = qs_old_matrix;
                            reached_task_pos = qs_old_task_state_vector;
                            reached_index = qs_old_index;
                            perdex_one_extend.no_collide=0;
                            extend_end_time = ros::Time::now();
                            perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                            if(if_tree_a){
                                perdex_one_sample.tree_a.push_back(perdex_one_extend);
                            }
                            else{
                                perdex_one_sample.tree_b.push_back(perdex_one_extend);
                            }
                            _extend_success_rate.push_back(_extend_success_times / _extend_times);
                            _complex_dir_manipulability_vec.push_back(_complex_dir_manipulability_sum);
                            _rate_complex_manipulability_vec.push_back(_complex_dir_manipulability_sum * (_extend_success_times / _extend_times));
                            break;
                        }
                    }
                    else{//IK fail
                        std::cout<<"ik fail"<<std::endl;
                        reached_state_matrix = qs_old_matrix;
                        reached_task_pos = qs_old_task_state_vector;
                        reached_index = qs_old_index;
                        perdex_one_extend.ik_success = 0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }
                        _extend_success_rate.push_back(_extend_success_times / _extend_times);
                        _complex_dir_manipulability_vec.push_back(_complex_dir_manipulability_sum);
                        _rate_complex_manipulability_vec.push_back(_complex_dir_manipulability_sum * (_extend_success_times / _extend_times));
                        break;
                    }
                }
                else{//master collide fail
                    std::cout<<"master collide fail"<<std::endl;
                    reached_state_matrix = qs_old_matrix;
                    reached_task_pos = qs_old_task_state_vector;
                    reached_index = qs_old_index;
                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }
                    _extend_success_rate.push_back(_extend_success_times / _extend_times);
                    _complex_dir_manipulability_vec.push_back(_complex_dir_manipulability_sum);
                    _rate_complex_manipulability_vec.push_back(_complex_dir_manipulability_sum * (_extend_success_times / _extend_times));
                    break;
                }
            }
            else{//project fail
                std::cout<<"project fail"<<std::endl;
                reached_state_matrix = qs_old_matrix;
                reached_task_pos = qs_old_task_state_vector;
                reached_index = qs_old_index;
                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }
                _extend_success_rate.push_back(_extend_success_times / _extend_times);
                _complex_dir_manipulability_vec.push_back(_complex_dir_manipulability_sum);
                _rate_complex_manipulability_vec.push_back(_complex_dir_manipulability_sum * (_extend_success_times / _extend_times));
                break;
            }
            extend_end_time = ros::Time::now();
            perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
            if(if_tree_a){
                perdex_one_sample.tree_a.push_back(perdex_one_extend);
            }
            else{
                perdex_one_sample.tree_b.push_back(perdex_one_extend);
            }
        }
        _extend_success_rate.push_back(_extend_success_times / _extend_times);
        _complex_dir_manipulability_vec.push_back(_complex_dir_manipulability_sum);
        _rate_complex_manipulability_vec.push_back(_complex_dir_manipulability_sum * (_extend_success_times / _extend_times));
    }
    std::cout<<"outout!!"<<std::endl;
}

bool DualCBiRRT::plan_task_space_dir(robot_state::RobotState & goal_state, robot_state::RobotState & start_state){

    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    Eigen::Vector4d start_task_vector;
    Eigen::Matrix3d start_task_rot_matrix;
    KDL::Rotation start_task_kdl;
    Eigen::Vector4d goal_task_vector;
    Eigen::Matrix3d goal_task_rot_matrix;
    KDL::Rotation goal_task_kdl;

    _start_master_pose = start_state.getGlobalLinkTransform("left_gripper");
    _start_slave_pose = start_state.getGlobalLinkTransform("right_gripper");
    _goal_master_pose = goal_state.getGlobalLinkTransform("left_gripper");
    _goal_slave_pose = goal_state.getGlobalLinkTransform("right_gripper");

    start_task_vector.head(3) = _start_master_pose.translation();
    start_task_rot_matrix = _start_master_pose.rotation();
    goal_task_vector.head(3) = _goal_master_pose.translation();
    goal_task_rot_matrix = _goal_master_pose.rotation();
    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            start_task_kdl(i,j) = start_task_rot_matrix(i,j);
            goal_task_kdl(i,j) = goal_task_rot_matrix(i,j);
        }
    }
    double foo1,foo2;
    start_task_kdl.GetEulerZYX(start_task_vector(3), foo1, foo2);
    goal_task_kdl.GetEulerZYX(goal_task_vector(3), foo1, foo2);

    _a_rrt_tree_task_space_state.push_back(start_task_vector);
    _b_rrt_tree_task_space_state.push_back(goal_task_vector);

    std::cout<<"_a_rrt_tree_task_space_state[0]\n"<<_a_rrt_tree_task_space_state[0].transpose()<<std::endl;
    std::cout<<"_b_rrt_tree_task_space_state[0]\n"<<_b_rrt_tree_task_space_state[0].transpose()<<std::endl;

    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState a_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState b_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();


    Eigen::Matrix<double, 7, 1> random_state_value_matrix;
    Eigen::Vector4d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector4d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector4d a_tree_reached_vector;
    Eigen::Vector4d b_tree_reached_vector;
    size_t a_tree_reached_index;
    size_t b_tree_reached_index;
    Eigen::Matrix<double, 7, 1> goal_value_matrix;
    Eigen::Matrix<double, 7, 1> start_value_matrix;
    Eigen::Matrix<double, 7, 1> slave_goal_value_matrix;
    Eigen::Matrix<double, 7, 1> slave_start_value_matrix;

    size_t nearest_node_index;

    std::vector<double> goal_state_value; //用来判断是不是采样到了goal_state
    goal_state.copyJointGroupPositions(planning_group, goal_state_value);
    std::vector<double> start_state_value;
    start_state.copyJointGroupPositions(planning_group, start_state_value);
    ROS_INFO("goal_state_value.size()=%d", int(goal_state_value.size()));
    for(size_t i=0; i<goal_state_value.size(); i++){
        goal_value_matrix[i] = goal_state_value[i];
        start_value_matrix[i] = start_state_value[i];
    }

    std::vector<double> slave_goal_state_value;
    goal_state.copyJointGroupPositions(slave_group, slave_goal_state_value);
    std::vector<double> slave_start_state_value;
    start_state.copyJointGroupPositions(slave_group, slave_start_state_value);
    for(size_t i=0; i<slave_goal_state_value.size(); i++){
        slave_goal_value_matrix[i] = slave_goal_state_value[i];
        slave_start_value_matrix[i] = slave_start_state_value[i];
    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

    _simple_dir_manipulability_sum += simple_dir_manipulability_compute(start_value_matrix, slave_start_value_matrix, true);
    _simple_dir_manipulability_sum += simple_dir_manipulability_compute(goal_value_matrix, slave_goal_value_matrix, false);


    std::cout<<"init_a_matrix_tree\n  "<<_a_rrt_tree_matrix[0].transpose() <<std::endl;
    std::cout<<"init_b_matrix_tree\n  "<<_b_rrt_tree_matrix[0].transpose() <<std::endl;


    bool extend_order = true; //两棵树轮流向采样的方向扩展
    ros::Time sample_start_time;
    ros::Time sample_end_time;

    for(int count=0; count < _max_planning_times; count++){
        sample_start_time = ros::Time::now();

        PerformanceIndexOneSample perdex_one_sample;
        perdex_one_sample.sample_num = count;
        std::cout<<"count: "<<count<<std::endl;
        //先扩展a树


        if(extend_order){
            sample_task_state(goal_state, random_state, random_state_value_matrix, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix,  perdex_one_sample, extend_order, true, a_tree_reached_vector, a_tree_reached_index);
            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix,  perdex_one_sample, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);

            std::cout<<"a_tree_reached_vector: "<<a_tree_reached_vector.transpose()<<std::endl;
            std::cout<<"b_tree_reached_vector: "<<b_tree_reached_vector.transpose()<<std::endl;

            if((a_tree_reached_vector - b_tree_reached_vector).norm() < 0.005)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = a_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_a_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_a_rrt_tree_task_space_state[back_track]);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());
                std::reverse(planning_result_index.begin(), planning_result_index.end());
                std::reverse(planning_result_task_state_vector.begin(), planning_result_task_state_vector.end());

                //添加后半部分路径点
                back_track = b_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_b_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_b_rrt_tree_task_space_state[back_track]);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                sample_end_time = ros::Time::now();
                perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
                _performance_record.push_back(perdex_one_sample);
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, perdex_one_sample, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, perdex_one_sample, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);

            std::cout<<"a_tree_reached_vector: "<<a_tree_reached_vector.transpose()<<std::endl;
            std::cout<<"b_tree_reached_vector: "<<b_tree_reached_vector.transpose()<<std::endl;
            if((a_tree_reached_vector - b_tree_reached_vector).norm() < 0.005)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = a_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_a_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_a_rrt_tree_task_space_state[back_track]);

                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());
                std::reverse(planning_result_index.begin(), planning_result_index.end());
                std::reverse(planning_result_task_state_vector.begin(), planning_result_task_state_vector.end());

                //添加后半部分路径点
                back_track = b_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_b_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_b_rrt_tree_task_space_state[back_track]);

                    back_track = _b_rrt_tree_state[back_track].second;
                }
                sample_end_time = ros::Time::now();
                perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
                _performance_record.push_back(perdex_one_sample);
                return true;
            }
            else{
                extend_order = true;
            }

        }
        sample_end_time = ros::Time::now();
        perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
        _performance_record.push_back(perdex_one_sample);
    }
    return false;
}

bool DualCBiRRT::plan_task_space_dir_try_adjust(robot_state::RobotState & goal_state, robot_state::RobotState & start_state){

    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);


    Eigen::Vector4d start_task_vector;
    Eigen::Matrix3d start_task_rot_matrix;
    KDL::Rotation start_task_kdl;
    Eigen::Vector4d goal_task_vector;
    Eigen::Matrix3d goal_task_rot_matrix;
    KDL::Rotation goal_task_kdl;

    _start_master_pose = start_state.getGlobalLinkTransform("left_gripper");
    _start_slave_pose = start_state.getGlobalLinkTransform("right_gripper");
    _goal_master_pose = goal_state.getGlobalLinkTransform("left_gripper");
    _goal_slave_pose = goal_state.getGlobalLinkTransform("right_gripper");

    start_task_vector.head(3) = _start_master_pose.translation();
    start_task_rot_matrix = _start_master_pose.rotation();
    goal_task_vector.head(3) = _goal_master_pose.translation();
    goal_task_rot_matrix = _goal_master_pose.rotation();
    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            start_task_kdl(i,j) = start_task_rot_matrix(i,j);
            goal_task_kdl(i,j) = goal_task_rot_matrix(i,j);
        }
    }
    double foo1,foo2;
    start_task_kdl.GetEulerZYX(start_task_vector(3), foo1, foo2);
    goal_task_kdl.GetEulerZYX(goal_task_vector(3), foo1, foo2);

    _a_rrt_tree_task_space_state.push_back(start_task_vector);
    _b_rrt_tree_task_space_state.push_back(goal_task_vector);

    current_two_tree_minimum_dis = (start_task_vector - goal_task_vector).norm();
    two_tree_minimum_dis.push_back(current_two_tree_minimum_dis);


    std::cout<<"_a_rrt_tree_task_space_state[0]\n"<<_a_rrt_tree_task_space_state[0].transpose()<<std::endl;
    std::cout<<"_b_rrt_tree_task_space_state[0]\n"<<_b_rrt_tree_task_space_state[0].transpose()<<std::endl;
    //获取slave group的关节界限，用于求解IK
    const robot_model::RobotModelConstPtr & baxter_robot_model_ptr = planning_scene_ptr->getRobotModel();
    const std::vector<std::string>& slave_joint_names = slave_group->getVariableNames();
    std::pair<std::vector<double>, std::vector<double>> slave_joint_pos_bounds;
    std::vector<double> slave_joint_max_bounds;
    std::vector<double> slave_joint_min_bounds;
    for(size_t i=0; i<slave_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = baxter_robot_model_ptr->getVariableBounds(slave_joint_names[i]);
        slave_joint_max_bounds.push_back(tmp_bounds.max_position_);
        slave_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    slave_joint_pos_bounds.first = slave_joint_max_bounds;
    slave_joint_pos_bounds.second = slave_joint_min_bounds;


    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState a_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState b_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();


    Eigen::Matrix<double, 7, 1> random_state_value_matrix;
    Eigen::Vector4d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector4d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector4d a_tree_reached_vector;
    Eigen::Vector4d b_tree_reached_vector;
    size_t a_tree_reached_index;
    size_t b_tree_reached_index;
    Eigen::Matrix<double, 7, 1> goal_value_matrix;
    Eigen::Matrix<double, 7, 1> start_value_matrix;
    Eigen::Matrix<double, 7, 1> slave_goal_value_matrix;
    Eigen::Matrix<double, 7, 1> slave_start_value_matrix;

    size_t nearest_node_index;

    std::vector<double> goal_state_value; //用来判断是不是采样到了goal_state
    goal_state.copyJointGroupPositions(planning_group, goal_state_value);
    std::vector<double> start_state_value;
    start_state.copyJointGroupPositions(planning_group, start_state_value);
    ROS_INFO("goal_state_value.size()=%d", int(goal_state_value.size()));
    for(size_t i=0; i<goal_state_value.size(); i++){
        goal_value_matrix[i] = goal_state_value[i];
        start_value_matrix[i] = start_state_value[i];
    }

    std::vector<double> slave_goal_state_value;
    goal_state.copyJointGroupPositions(slave_group, slave_goal_state_value);
    std::vector<double> slave_start_state_value;
    start_state.copyJointGroupPositions(slave_group, slave_start_state_value);
    for(size_t i=0; i<slave_goal_state_value.size(); i++){
        slave_goal_value_matrix[i] = slave_goal_state_value[i];
        slave_start_value_matrix[i] = slave_start_state_value[i];
    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

    std::cout<<"init_a_matrix_tree\n  "<<_a_rrt_tree_matrix[0].transpose() <<std::endl;
    std::cout<<"init_b_matrix_tree\n  "<<_b_rrt_tree_matrix[0].transpose() <<std::endl;

    _a_tree_has_been_extended.push_back(false);
    _b_tree_has_been_extended.push_back(false);
    //begin****************为了计算初始的manipulability，计算一下碰撞的距离******************************//
    start_state.update();
    goal_state.update();
    collision_detection::DistanceRequest collision_req_master_world;
    collision_req_master_world.enable_nearest_points = true;
    collision_req_master_world.enable_signed_distance = true;
    collision_req_master_world.active_components_only = &_master_link_model_set;
    collision_req_master_world.group_name = "left_arm";
    collision_req_master_world.type = collision_detection::DistanceRequestType::LIMITED;

    collision_detection::DistanceResult collision_res_master_world;
    collision_detection::DistanceResult new_collision_res_master_world;

    collision_world->distanceRobot(collision_req_master_world, collision_res_master_world, *collision_robot, start_state);

    collision_detection::DistanceMap start_master_dis_map = collision_res_master_world.distances;
//    ROS_WARN("master_dis_map");
//    for(auto it = master_dis_map.begin(); it != master_dis_map.end(); it++){
//        std::cout<<it->first.first<<" "<<it->first.second<<" "<<it->second.size()<<" "<<(it->second)[0].distance<<std::endl;
//    }


    collision_detection::DistanceResultsData master_min_dis_world = collision_res_master_world.minimum_distance;
    double master_collide_dis_value = master_min_dis_world.distance;
    Eigen::Vector3d master_collide_pos = master_min_dis_world.nearest_points[1];
    Eigen::Vector3d master_collide_normal = master_min_dis_world.normal;
    std::string master_collide_link_name  = master_min_dis_world.link_names[1];



    std::cout<<"master_min_dis_world  "<<master_min_dis_world.distance<<std::endl;
    std::cout<<"master_collide_pos  "<<master_collide_pos.transpose()<<std::endl;
    std::cout<<"master_collide_link_name0  "<<master_min_dis_world.link_names[0]<<std::endl;
    std::cout<<"master_collide_link_name1  "<<master_collide_link_name<<std::endl;
    std::cout<<"normal  "<<master_min_dis_world.normal.transpose()<<std::endl;

    const robot_state::LinkModel* master_collision_link = start_state.getLinkModel(master_collide_link_name);
    Eigen::MatrixXd master_collision_point_jac;
    start_state.getJacobian(planning_group, master_collision_link, master_collide_pos, master_collision_point_jac);
//    std::cout<<"collide jac\n"<<master_collision_point_jac<<std::endl;
//    std::cout<<"end jac\n"<<start_state.getJacobian(planning_group)<<std::endl;


    collision_detection::DistanceRequest collision_req_slave_world;
    collision_req_slave_world.enable_nearest_points = true;
    collision_req_slave_world.enable_signed_distance = true;
    collision_req_slave_world.active_components_only = &_slave_link_model_set;
    collision_req_slave_world.group_name = "right_arm";
    collision_req_slave_world.type = collision_detection::DistanceRequestType::LIMITED;

    collision_detection::DistanceResult collision_res_slave_world;
    collision_detection::DistanceResult new_collision_res_slave_world;

    collision_world->distanceRobot(collision_req_slave_world, collision_res_slave_world, *collision_robot, start_state);

    collision_detection::DistanceMap start_slave_dis_map = collision_res_slave_world.distances;

    
    collision_detection::DistanceResultsData slave_min_dis_world = collision_res_slave_world.minimum_distance;
    double slave_collide_dis_value = slave_min_dis_world.distance;
    Eigen::Vector3d slave_collide_pos = slave_min_dis_world.nearest_points[1];
    Eigen::Vector3d slave_collide_normal = slave_min_dis_world.normal;
    std::string slave_collide_link_name  = slave_min_dis_world.link_names[1];

    std::cout<<"slave_min_dis_world  "<<slave_min_dis_world.distance<<std::endl;
    std::cout<<"slave_collide_pos  "<<slave_collide_pos.transpose()<<std::endl;
    std::cout<<"slave_collide_link_name0  "<<slave_min_dis_world.link_names[0]<<std::endl;
    std::cout<<"slave_collide_link_name  "<<slave_collide_link_name<<std::endl;
    std::cout<<"normal  "<<slave_min_dis_world.normal.transpose()<<std::endl;

    const robot_state::LinkModel* slave_collision_link = start_state.getLinkModel(slave_collide_link_name);
    Eigen::MatrixXd slave_collision_point_jac;
    start_state.getJacobian(slave_group, slave_collision_link, slave_collide_pos, slave_collision_point_jac);

    _complex_dir_manipulability_sum += manipulability_compute(start_value_matrix, slave_start_value_matrix, master_collision_point_jac, master_collide_normal, master_collide_dis_value, slave_collision_point_jac, slave_collide_normal, slave_collide_dis_value, true);
    a_tree_considered_master_ob_jac.push_back(master_collision_point_jac);
    a_tree_considered_master_ob_dir.push_back(master_collide_normal);
    a_tree_considered_master_ob_dis.push_back(master_collide_dis_value);
    a_tree_considered_slave_ob_jac.push_back(slave_collision_point_jac);
    a_tree_considered_slave_ob_dir.push_back(slave_collide_normal);
    a_tree_considered_slave_ob_dis.push_back(slave_collide_dis_value);

    //begin****************为了计算初始的manipulability，计算一下碰撞的距离******************************//
    collision_world->distanceRobot(collision_req_master_world, new_collision_res_master_world, *collision_robot, goal_state);
    collision_detection::DistanceMap goal_master_dis_map = new_collision_res_master_world.distances;
    master_min_dis_world = new_collision_res_master_world.minimum_distance;
    master_collide_dis_value = master_min_dis_world.distance;
    master_collide_pos = master_min_dis_world.nearest_points[1];
    master_collide_normal = master_min_dis_world.normal;
    master_collide_link_name  = master_min_dis_world.link_names[1];
    std::cout<<"master_min_dis_world  "<<master_min_dis_world.distance<<std::endl;
    std::cout<<"master_collide_pos  "<<master_collide_pos.transpose()<<std::endl;
    std::cout<<"master_collide_link_name0  "<<master_min_dis_world.link_names[0]<<std::endl;
    std::cout<<"master_collide_link_name  "<<master_collide_link_name<<std::endl;
    std::cout<<"normal  "<<master_min_dis_world.normal.transpose()<<std::endl;
    const robot_state::LinkModel* new_master_collision_link = start_state.getLinkModel(master_collide_link_name);
    start_state.getJacobian(planning_group, new_master_collision_link, master_collide_pos, master_collision_point_jac);

    collision_world->distanceRobot(collision_req_slave_world, new_collision_res_slave_world, *collision_robot, goal_state);
    collision_detection::DistanceMap goal_slave_dis_map = new_collision_res_slave_world.distances;
    slave_min_dis_world = new_collision_res_slave_world.minimum_distance;
    slave_collide_dis_value = slave_min_dis_world.distance;
    slave_collide_pos = slave_min_dis_world.nearest_points[1];
    slave_collide_normal = slave_min_dis_world.normal;
    slave_collide_link_name  = slave_min_dis_world.link_names[1];
    std::cout<<"slave_min_dis_world  "<<slave_min_dis_world.distance<<std::endl;
    std::cout<<"slave_collide_pos  "<<slave_collide_pos.transpose()<<std::endl;
    std::cout<<"slave_collide_link_name0  "<<slave_min_dis_world.link_names[0]<<std::endl;
    std::cout<<"slave_collide_link_name  "<<slave_collide_link_name<<std::endl;
    std::cout<<"normal  "<<slave_min_dis_world.normal.transpose()<<std::endl;
    const robot_state::LinkModel* new_slave_collision_link = start_state.getLinkModel(slave_collide_link_name);
    start_state.getJacobian(slave_group, new_slave_collision_link, slave_collide_pos, slave_collision_point_jac);

    b_tree_considered_master_ob_jac.push_back(master_collision_point_jac);
    b_tree_considered_master_ob_dir.push_back(master_collide_normal);
    b_tree_considered_master_ob_dis.push_back(master_collide_dis_value);
    b_tree_considered_slave_ob_jac.push_back(slave_collision_point_jac);
    b_tree_considered_slave_ob_dir.push_back(slave_collide_normal);
    b_tree_considered_slave_ob_dis.push_back(slave_collide_dis_value);
    //end****************为了计算初始的manipulability，计算一下碰撞的距离******************************//



    double start_master_manipulability;
    double start_slave_manipulability;
    double goal_master_manipulability;
    double goal_slave_manipulability;
//    init_all_dir_manipulability_compute(start_state, master_collision_point_jac, master_collide_normal, master_collide_dis_value, slave_collision_point_jac, slave_collide_normal, slave_collide_dis_value, start_master_manipulability, start_slave_manipulability);
//    init_all_dir_manipulability_compute(goal_state, master_collision_point_jac, master_collide_normal, master_collide_dis_value, slave_collision_point_jac, slave_collide_normal, slave_collide_dis_value, goal_master_manipulability, goal_slave_manipulability);
    _a_tree_master_dis_map.push_back(start_master_dis_map);
    _a_tree_slave_dis_map.push_back(start_slave_dis_map);
    _b_tree_master_dis_map.push_back(goal_master_dis_map);
    _b_tree_slave_dis_map.push_back(goal_slave_dis_map);
    init_all_dir_manipulability_compute_all_obstacle(start_state, start_master_dis_map, start_slave_dis_map, start_master_manipulability, start_slave_manipulability);
    _new_manipulability_master_vec_explore.push_back(start_master_manipulability);
    _new_manipulability_slave_vec_explore.push_back(start_slave_manipulability);
    init_all_dir_manipulability_compute_all_obstacle(goal_state, goal_master_dis_map, goal_slave_dis_map, goal_master_manipulability, goal_slave_manipulability);
    _new_manipulability_master_vec_exploit.push_back(goal_master_manipulability);
    _new_manipulability_slave_vec_exploit.push_back(goal_slave_manipulability);
    std::cout<<"start_manipulability: "<<start_master_manipulability<<" "<< start_slave_manipulability<<std::endl;
    std::cout<<"goal_manipulability: "<<goal_master_manipulability<<" "<< goal_slave_manipulability<<std::endl;

    init_all_dir_manipulability_compute_all_obstacle_one_dir(start_state, start_master_dis_map, start_slave_dis_map, master_one_dir_manipulaibility, slave_one_dir_manipulaibility, (goal_task_vector - start_task_vector));
    master_one_dir_manipulaibility_vec.push_back(master_one_dir_manipulaibility);
    slave_one_dir_manipulaibility_vec.push_back(slave_one_dir_manipulaibility);



    bool extend_order = true; //两棵树轮流向采样的方向扩展
    ros::Time sample_start_time;
    ros::Time sample_end_time;


    double master_explore = 0;
    double slave_explore = 0;
    double master_exploit = 0;
    double slave_exploit = 0;
    double mini_explore = 0;
    double mini_exploit = 0;
    if(start_master_manipulability < start_slave_manipulability){
        mini_explore = start_master_manipulability;
    }
    else{
        mini_explore = start_slave_manipulability;
    }
    if(goal_master_manipulability < goal_slave_manipulability){
        mini_exploit = goal_master_manipulability;
    }
    else{
        mini_exploit = goal_slave_manipulability;
    }
    _new_manipulability_mini_vec_explore.push_back(mini_explore);
    _new_manipulability_mini_vec_exploit.push_back(mini_exploit);
    for(int count=0; count < _max_planning_times; count++){
        sample_start_time = ros::Time::now();

        PerformanceIndexOneSample perdex_one_sample;
        perdex_one_sample.sample_num = count;
        std::cout<<"count: "<<count<<std::endl;
        //先扩展a树


        if(extend_order){
            sample_task_state(goal_state, random_state, random_state_value_matrix, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, perdex_one_sample, extend_order, true, a_tree_reached_vector, a_tree_reached_index);

            double master_manipulability = 0, slave_manipulability = 0;
            if(! _a_tree_has_been_extended[a_tree_reached_index]) {
                init_all_dir_manipulability_compute_all_obstacle(_a_rrt_tree_state[a_tree_reached_index].first, _a_tree_master_dis_map[a_tree_reached_index], _a_tree_slave_dis_map[a_tree_reached_index], master_manipulability, slave_manipulability);
                _a_tree_has_been_extended[a_tree_reached_index] = true;
            }
            if (master_manipulability < slave_manipulability) {
                mini_explore += master_manipulability;
            } else {
                mini_explore += slave_manipulability;
            }
            _new_manipulability_master_vec_explore.push_back(master_explore = master_manipulability);
            _new_manipulability_slave_vec_explore.push_back(slave_explore = slave_manipulability);
            _new_manipulability_mini_vec_explore.push_back(mini_explore);
            _a_tree_reached_index_vec.push_back(a_tree_reached_index);
            _a_tree_nearest_index_vec.push_back(nearest_node_index);
            init_all_dir_manipulability_compute_all_obstacle_one_dir(_a_rrt_tree_state[nearest_node_index].first, _a_tree_master_dis_map[nearest_node_index], _a_tree_slave_dis_map[nearest_node_index], _master_extend_manipulability, _slave_extend_manipulability, random_task_state_vector - _a_rrt_tree_task_space_state[nearest_node_index]);
            if(_slave_extend_manipulability < _master_extend_manipulability){
                _a_tree_minimum_extend_manipulability.push_back(_slave_extend_manipulability);
            }
            else{
                _a_tree_minimum_extend_manipulability.push_back(_master_extend_manipulability);
            }

            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_try_adjust(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, perdex_one_sample, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);
            master_manipulability = 0;
            slave_manipulability = 0;
            if(! _b_tree_has_been_extended[b_tree_reached_index]) {
                init_all_dir_manipulability_compute_all_obstacle(_b_rrt_tree_state[b_tree_reached_index].first, _b_tree_master_dis_map[b_tree_reached_index], _b_tree_slave_dis_map[b_tree_reached_index], master_manipulability, slave_manipulability);
                _b_tree_has_been_extended[b_tree_reached_index] = true;
            }
            if (master_manipulability < slave_manipulability) {
                mini_exploit += master_manipulability;
            } else {
                mini_exploit += slave_manipulability;
            }
            _new_manipulability_master_vec_exploit.push_back(master_exploit = master_manipulability);
            _new_manipulability_slave_vec_exploit.push_back(slave_exploit = slave_manipulability);
            _new_manipulability_mini_vec_exploit.push_back(mini_exploit);
            _b_tree_reached_index_vec.push_back(b_tree_reached_index);
            _b_tree_nearest_index_vec.push_back(nearest_node_index);

            new_two_tree_minimum_dis = (a_tree_reached_vector - b_tree_reached_vector).norm();
            if(new_two_tree_minimum_dis < current_two_tree_minimum_dis){
                current_two_tree_minimum_dis = new_two_tree_minimum_dis;
            }
            two_tree_minimum_dis.push_back(current_two_tree_minimum_dis);
            init_all_dir_manipulability_compute_all_obstacle_one_dir(_b_rrt_tree_state[nearest_node_index].first, _b_tree_master_dis_map[nearest_node_index], _b_tree_slave_dis_map[nearest_node_index], _master_extend_manipulability, _slave_extend_manipulability, a_tree_reached_vector - _b_rrt_tree_task_space_state[nearest_node_index]);
            if(_slave_extend_manipulability < _master_extend_manipulability){
                _b_tree_minimum_extend_manipulability.push_back(_slave_extend_manipulability);
            }
            else{
                _b_tree_minimum_extend_manipulability.push_back(_master_extend_manipulability);
            }

            std::cout<<"a_tree_reached_vector: "<<a_tree_reached_vector.transpose()<<std::endl;
            std::cout<<"b_tree_reached_vector: "<<b_tree_reached_vector.transpose()<<std::endl;

            if((a_tree_reached_vector - b_tree_reached_vector).norm() < 0.005)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = a_tree_reached_index;
                std::cout<<"heh "<<_a_rrt_tree_task_space_state[back_track].transpose()<<std::endl;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_a_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_a_rrt_tree_task_space_state[back_track]);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());
                std::reverse(planning_result_index.begin(), planning_result_index.end());
                std::reverse(planning_result_task_state_vector.begin(), planning_result_task_state_vector.end());

                //添加后半部分路径点
                back_track = b_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_b_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_b_rrt_tree_task_space_state[back_track]);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                sample_end_time = ros::Time::now();
                perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
                _performance_record.push_back(perdex_one_sample);
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, perdex_one_sample, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            double master_manipulability = 0, slave_manipulability = 0;
            if(! _b_tree_has_been_extended[b_tree_reached_index]) {
                init_all_dir_manipulability_compute_all_obstacle(_b_rrt_tree_state[b_tree_reached_index].first, _b_tree_master_dis_map[b_tree_reached_index], _b_tree_slave_dis_map[b_tree_reached_index], master_manipulability, slave_manipulability);
                _b_tree_has_been_extended[b_tree_reached_index] = true;
            }
            if (master_manipulability < slave_manipulability) {
                mini_explore += master_manipulability;
            }
            else {
                mini_explore += slave_manipulability;
            }
            _new_manipulability_master_vec_explore.push_back(master_explore = master_manipulability);
            _new_manipulability_slave_vec_explore.push_back(slave_explore = slave_manipulability);
            _new_manipulability_mini_vec_explore.push_back(mini_explore);
            _b_tree_reached_index_vec.push_back(b_tree_reached_index);
            _b_tree_nearest_index_vec.push_back(nearest_node_index);

            init_all_dir_manipulability_compute_all_obstacle_one_dir(_b_rrt_tree_state[nearest_node_index].first, _b_tree_master_dis_map[nearest_node_index], _b_tree_slave_dis_map[nearest_node_index], _master_extend_manipulability, _slave_extend_manipulability, random_task_state_vector - _b_rrt_tree_task_space_state[nearest_node_index]);
            if(_slave_extend_manipulability < _master_extend_manipulability){
                _b_tree_minimum_extend_manipulability.push_back(_slave_extend_manipulability);
            }
            else{
                _b_tree_minimum_extend_manipulability.push_back(_master_extend_manipulability);
            }


            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_try_adjust(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, perdex_one_sample, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);
            master_manipulability = 0;
            slave_manipulability = 0;
            if(! _a_tree_has_been_extended[a_tree_reached_index]) {
                init_all_dir_manipulability_compute_all_obstacle(_a_rrt_tree_state[a_tree_reached_index].first, _a_tree_master_dis_map[a_tree_reached_index], _a_tree_slave_dis_map[a_tree_reached_index], master_manipulability, slave_manipulability);
                _a_tree_has_been_extended[a_tree_reached_index] = true;
            }
            if(master_manipulability < slave_manipulability){
                mini_exploit += master_manipulability;
            }
            else {
                mini_exploit += slave_manipulability;
            }
            _new_manipulability_master_vec_exploit.push_back(master_exploit = master_manipulability);
            _new_manipulability_slave_vec_exploit.push_back(slave_exploit = slave_manipulability);
            _new_manipulability_mini_vec_exploit.push_back(mini_exploit);
            _a_tree_reached_index_vec.push_back(a_tree_reached_index);
            _a_tree_nearest_index_vec.push_back(nearest_node_index);

            new_two_tree_minimum_dis = (a_tree_reached_vector - b_tree_reached_vector).norm();
            if(new_two_tree_minimum_dis < current_two_tree_minimum_dis){
                current_two_tree_minimum_dis = new_two_tree_minimum_dis;
            }
            two_tree_minimum_dis.push_back(current_two_tree_minimum_dis);

            init_all_dir_manipulability_compute_all_obstacle_one_dir(_a_rrt_tree_state[nearest_node_index].first, _a_tree_master_dis_map[nearest_node_index], _a_tree_slave_dis_map[nearest_node_index], _master_extend_manipulability, _slave_extend_manipulability, b_tree_reached_vector - _a_rrt_tree_task_space_state[nearest_node_index]);
            if(_slave_extend_manipulability < _master_extend_manipulability){
                _a_tree_minimum_extend_manipulability.push_back(_slave_extend_manipulability);
            }
            else{
                _a_tree_minimum_extend_manipulability.push_back(_master_extend_manipulability);
            }


            std::cout<<"a_tree_reached_vector: "<<a_tree_reached_vector.transpose()<<std::endl;
            std::cout<<"b_tree_reached_vector: "<<b_tree_reached_vector.transpose()<<std::endl;
            if((a_tree_reached_vector - b_tree_reached_vector).norm() < 0.005)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = a_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_a_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_a_rrt_tree_task_space_state[back_track]);

                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());
                std::reverse(planning_result_index.begin(), planning_result_index.end());
                std::reverse(planning_result_task_state_vector.begin(), planning_result_task_state_vector.end());

                //添加后半部分路径点
                back_track = b_tree_reached_index;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    planning_result_index.push_back(_b_rrt_tree_state[back_track].second);
                    planning_result_task_state_vector.push_back(_b_rrt_tree_task_space_state[back_track]);

                    back_track = _b_rrt_tree_state[back_track].second;
                }
                sample_end_time = ros::Time::now();
                perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
                _performance_record.push_back(perdex_one_sample);
                return true;
            }
            else{
                extend_order = true;
            }

        }
        sample_end_time = ros::Time::now();
        perdex_one_sample.spend_time = double((sample_end_time - sample_start_time).nsec)/1000000000;
        _performance_record.push_back(perdex_one_sample);
    }

    return false;
}

bool DualCBiRRT::solve_IK_problem(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, PerformanceIndexOneExtend & perdex_one_extend){
    //************************************获取函数参数，当前的master的各个关节值以及slave的各个关节值，存储在 RobotState 中*************************************
    robot_state::RobotState master_state = planning_scene_ptr->getCurrentStateNonConst();//用来保存这次计算所参考的master的状态，函数中不会更改
    robot_state::RobotState slave_state = planning_scene_ptr->getCurrentStateNonConst(); //用来保存计算到的当前的slave的状态，循环中多次更改
    std::vector<double> master_joint_value_vector;
    std::vector<double> slave_joint_value_vector;
    for(size_t i=0; i<7; i++){
        master_joint_value_vector.push_back(master_state_value_matrix[i]);
        slave_joint_value_vector.push_back(slave_state_value_matrix[i]);
    }

    master_state.setJointGroupPositions(planning_group, master_joint_value_vector);
    master_state.update();
    slave_state.setJointGroupPositions(slave_group,slave_joint_value_vector);
    slave_state.update();
    //*******************************************************************************************************************************************
    //*********************利用 RobotState 得到 master 的位置向量以及欧拉角向量*************************************
    const Eigen::Affine3d & master_end_pose = master_state.getGlobalLinkTransform("left_gripper");
    auto master_end_rot_matrix = master_end_pose.rotation();
    KDL::Rotation master_end_rot_kdl;
    for(size_t i=0;i<3;i++){
        for(size_t j=0;j<3;j++){
            master_end_rot_kdl(i,j) = master_end_rot_matrix(i,j);
        }
    }
    Eigen::Vector3d master_euler;
    master_end_rot_kdl.GetEulerZYX(master_euler(0), master_euler(1), master_euler(2));
    Eigen::Vector3d master_end_pos = master_end_pose.translation();
    //********************************************************************************************************

    //*********************利用 RobotState 得到 slave 的位置向量、欧拉角向量以及雅克比矩阵、雅克比伪逆矩阵*************************************
    const Eigen::Affine3d & slave_end_pose = slave_state.getGlobalLinkTransform("right_gripper");
    auto slave_end_rot_matrix = slave_end_pose.rotation();
    KDL::Rotation slave_end_rot_kdl;
    for(size_t i=0;i<3;i++){
        for(size_t j=0;j<3;j++){
            slave_end_rot_kdl(i,j) = slave_end_rot_matrix(i,j);
        }
    }
    Eigen::Vector3d slave_euler;
    slave_end_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1), slave_euler(2));
    Eigen::Vector3d slave_end_pos = slave_end_pose.translation();
    Eigen::MatrixXd slave_end_jacobian;
    Eigen::MatrixXd slave_end_jacobian_mp_inverse;
    slave_end_jacobian = slave_state.getJacobian(slave_group);
    //********************************************************************************************************

    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************
    //最开始的闭环约束方式
//    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] - 3.1415926);
    Eigen::Vector3d slave_goal_euler(master_euler[0] - _left_right_euler_distance[_test_pose_num], master_euler[1], master_euler[2] - 3.1415926);

    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(_left_right_distance_x[_test_pose_num], 0, _left_right_distance_z[_test_pose_num]);
//    Eigen::Vector3d distance(0, 0, 0.2);

    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;


    //*****************************************************************************************

    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
//    std::cout<<"master_end_pos "<<master_end_pos.transpose()<<std::endl;
//    std::cout<<"master_euler "<<master_euler.transpose()<<std::endl;
//    std::cout<<"slave_goal_pos "<<slave_goal_pos.transpose()<<std::endl;
//    std::cout<<"slave_end_pos "<<slave_end_pos.transpose()<<std::endl;
//    std::cout<<"slave_goal_euler "<<slave_goal_euler.transpose()<<std::endl;
//    std::cout<<"slave_euler "<<slave_euler.transpose()<<std::endl;

    Eigen::Vector3d pos_error = slave_goal_pos - slave_end_pos;
    Eigen::Matrix3d rot_error_matrix = slave_end_rot_matrix * slave_goal_rot_matrix.inverse();
    Eigen::AngleAxisd rot_error_axis_angle(rot_error_matrix);
    double rot_error_angle = rot_error_axis_angle.angle();
    Eigen::Vector3d rot_error_3vector = rot_error_angle * rot_error_axis_angle.axis();
    Eigen::Vector3d euler_error = slave_goal_euler - slave_euler;//画图测试一下欧拉角误差和轴角误差

    Eigen::Vector3d last_pos_error = pos_error;
    Eigen::Vector3d last_euler_error = euler_error;

    Eigen::Matrix<double, 6, 1>  stack_error;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    //*****************************************************************************************

    //********************************性能优化函数**************************
    Eigen::Matrix<double, 7, 1>  delta_H;
    Eigen::Matrix<double, 7, 1>  max_min_square;
    double compute_tmp;
    for(size_t i=0; i<7 ;i++){
        compute_tmp = slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i];
        max_min_square[i] = compute_tmp * compute_tmp;
    }
    //********************************************************************
    std::vector<Eigen::Matrix<double, 1, 3>> pos_error_draw;
    std::vector<Eigen::Matrix<double, 1, 3>> euler_error_draw;
    std::vector<Eigen::Matrix<double, 1, 7>> joint_angles_draw;
    std::vector<double> rot_error_angle_draw;


    int count = 0;
    while (true){
        count++;
        std::cout<<"computing IK "<<count<<std::endl;
        if(pos_error.norm() <0.005 && euler_error.norm() < 0.005){
            result_state_value_matrix = slave_state_value_matrix;
            perdex_one_extend.ik_project_times = count;
            return true;
        }
        else{
            //计算消除末端在任务空间的误差所需要的速度
            rot_error_3vector = rot_error_axis_angle.angle() * rot_error_axis_angle.axis();
            stack_error.head(3) = pos_error;
            stack_error.tail(3) = rot_error_3vector;

            stack_error = (_error_coefficient * stack_error) / 0.01;

            slave_end_jacobian = slave_state.getJacobian(slave_group);
            slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose() + 0.01 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
            //计算关节增量
            joint_delta_vector = slave_end_jacobian_mp_inverse * stack_error;
            joint_delta_vector *= 0.01;

//            std::cout<<"\n\n"<<std::endl;
//            std::cout<<slave_state_value_matrix.transpose()<<std::endl;
//            std::cout<<joint_delta_vector.transpose()<<std::endl;
//            std::cout<<joint_delta_vector.norm()<<std::endl;
//            std::cout<<"pos error  "<<pos_error.transpose()<<std::endl;
//            std::cout<<"pos error norm "<<pos_error.norm()<<std::endl;
//            std::cout<<"euler_error  "<<euler_error.transpose()<<std::endl;
//            std::cout<<"euler_error norm "<<euler_error.norm()<<std::endl;

            if(joint_delta_vector.norm() < 0.001){
                perdex_one_extend.ik_project_times = count;
                std::cout<<"computing IK6 fail"<<std::endl;
                return false;
            }
            else{

                slave_state_value_matrix += joint_delta_vector;
                //更新 slave RobotState, state 更新肯定没有问题
                for(size_t i=0; i<7; i++){
                    slave_joint_value_vector[i] = slave_state_value_matrix[i];
                }
                slave_state.setJointGroupPositions(slave_group, slave_joint_value_vector);
                slave_state.update();
                if(slave_state.satisfiesBounds(slave_group, 0.05)){
                    //更新末端误差
                    const Eigen::Affine3d & slave_end_pose_tmp = slave_state.getGlobalLinkTransform("right_gripper");
                    slave_end_rot_matrix = slave_end_pose_tmp.rotation();
                    slave_end_pos = slave_end_pose_tmp.translation();
                    for(size_t i=0;i<3;i++){
                        for(size_t j=0;j<3;j++){
                            slave_end_rot_kdl(i,j) = slave_end_rot_matrix(i,j);
                        }
                    }
                    slave_end_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1),slave_euler(2));
                    euler_error = slave_goal_euler - slave_euler;
                    pos_error = slave_goal_pos - slave_end_pos;
                    rot_error_matrix = slave_goal_rot_matrix * (slave_end_rot_matrix.inverse());
                    rot_error_axis_angle = rot_error_matrix;
                }
                else{
                    perdex_one_extend.ik_project_times = count;
                    std::cout<<"computing IK5 fail"<<std::endl;
                    return false;
                }
            }
        }
    }
}

double DualCBiRRT::manipulability_compute(Eigen::Matrix<double, 7, 1> master_matrix, Eigen::Matrix<double, 7, 1> slave_matrix, Eigen::MatrixXd & master_ob_jac, Eigen::Vector3d master_ob_dir, double master_ob_dis, Eigen::MatrixXd & slave_ob_jac, Eigen::Vector3d slave_ob_dir, double slave_ob_dis, bool if_tree_a){
    Eigen::MatrixXd master_jacobian;
    std::vector<double> master_joint_angles(7);
    for(size_t i=0; i<7; i++){
        master_joint_angles[i] = master_matrix[i];
    }
    robot_state::RobotState master_state = planning_scene_ptr->getCurrentStateNonConst();
    master_state.setJointGroupPositions(planning_group, master_joint_angles);
    master_jacobian = master_state.getJacobian(planning_group);

    Eigen::MatrixXd slave_jacobian;
    std::vector<double> slave_joint_angles(7);
    for(size_t i=0; i<7; i++){
        slave_joint_angles[i] = slave_matrix[i];
    }
    robot_state::RobotState slave_state = planning_scene_ptr->getCurrentStateNonConst();
    slave_state.setJointGroupPositions(slave_group, slave_joint_angles);
    slave_jacobian = slave_state.getJacobian(slave_group);

    //joint_limit_penality
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //ob penality
    double master_alpha = 50;
    double master_beta = 2;
    double master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta) * (master_beta / master_ob_dis + master_alpha);
    double master_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;
    master_extended_v.head(3) = master_ob_dir * master_ob_dis;
    master_extended_v.tail(3) = Eigen::Vector3d::Zero();
    master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;
    Eigen::Matrix<double ,6,7> master_o_positive;
    Eigen::Matrix<double ,6,7> master_o_negative;

    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];
    //ob penality
    double slave_alpha = 50;
    double slave_beta = 2;
    double slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta * (slave_beta / slave_ob_dis + slave_alpha));
    double slave_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;
    slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
    slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
    slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;
    Eigen::Matrix<double ,6,7> slave_o_positive;
    Eigen::Matrix<double ,6,7> slave_o_negative;

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_matrix[i] - master_joint_pos_bounds.second[i], 2) * (2*master_matrix[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                /(4 * std::pow(master_joint_pos_bounds.first[i] - master_matrix[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_matrix[i], 2));
        if((master_matrix[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] > master_matrix[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }
        master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;

        slave_delta_h[i] = (std::pow(slave_matrix[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_matrix[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                            /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_matrix[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_matrix[i], 2));
        if((slave_matrix[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] > slave_matrix[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
        slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;

    }

    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<7; j++){
            if(master_ob_dir[i] > 0){
                master_o_negative(i,j) = 1;
                master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
            }
            else{
                master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                master_o_positive(i,j) = 1;
            }
            if(slave_ob_dir[i] > 0){
                slave_o_negative(i,j) = 1;
                slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
            }
            else{
                slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                slave_o_positive(i,j) = 1;
            }
        }
    }
    for(size_t i=0; i<3; i++) {
        for (size_t j = 0; j < 7; j++) {
            master_o_negative(i+3,j) = 1;
            master_o_positive(i+3,j) = 1;
            slave_o_negative(i+3,j) = 1;
            slave_o_positive(i+3,j) = 1;
        }
    }

    Eigen::Matrix<double, 6, 7> master_L;
    Eigen::Matrix<double, 6, 7> master_O;
    Eigen::Matrix<double, 6, 7> slave_L;
    Eigen::Matrix<double, 6, 7> slave_O;



    //begin********************计算扩展的方向**********************************
    Eigen::Affine3d master_pose;
    master_pose = master_state.getGlobalLinkTransform("left_gripper");
    Eigen::Affine3d master_dir_pose;
    Eigen::Vector3d master_dir_pos;
    Eigen::AngleAxisd master_dir_axis_angle;
    Eigen::Vector3d master_dir_axis;
    Eigen::Matrix<double,6,1> master_dir;
    if(if_tree_a){
        master_dir_pose = _goal_master_pose * master_pose.inverse();
    }
    else{
        master_dir_pose = _start_master_pose * master_pose.inverse();
    }


    master_dir_pos = master_dir_pose.translation();
    master_dir_axis_angle = master_dir_pose.rotation();
    master_dir_axis = master_dir_axis_angle.axis();
    master_dir.head(3) = master_dir_pos;
    master_dir.tail(3) = master_dir_axis;


    Eigen::Affine3d slave_pose;
    slave_pose = slave_state.getGlobalLinkTransform("right_gripper");
    Eigen::Affine3d slave_dir_pose;
    Eigen::Vector3d slave_dir_pos;
    Eigen::AngleAxisd slave_dir_axis_angle;
    Eigen::Vector3d slave_dir_axis;
    Eigen::Matrix<double,6,1> slave_dir;
    if(if_tree_a){
        slave_dir_pose = _goal_slave_pose * slave_pose.inverse();
    }
    else{
        slave_dir_pose = _start_slave_pose * slave_pose.inverse();
    }
    slave_dir_pos = slave_dir_pose.translation();
    slave_dir_axis_angle = slave_dir_pose.rotation();
    slave_dir_axis = slave_dir_axis_angle.axis();
    slave_dir.head(3) = slave_dir_pos;
    slave_dir.tail(3) = slave_dir_axis;


    Eigen::Vector3d P_GL(0, 0, 0.2);
    Eigen::Vector4d P_GL2(0, 0, 0.2, 1);
    std::cout<<"slave_pos  "<<slave_pose.translation().transpose()<<std::endl;
    std::cout<<"slave_pos_computed  "<<(master_pose * P_GL).transpose()<<std::endl;
    std::cout<<"slave_new_pos  "<<_goal_slave_pose.translation().transpose()<<std::endl;
    std::cout<<"slave_pos_change\n  "<<(_goal_slave_pose.translation() - slave_pose.translation()).transpose() <<std::endl;
    std::cout<<"master_pose_change\n  "<<(_goal_master_pose.translation() - master_pose.translation()).transpose() <<std::endl;
    std::cout<<"slave_new_pos2\n  "<<((_goal_master_pose.matrix() - master_pose.matrix())*P_GL2).transpose() <<std::endl;
    //end********************计算扩展的方向**********************************


    for(size_t i=0; i<6; i++){
        for(size_t j=0; j<7; j++){
            if(master_dir[i] < 0){
                master_O(i,j) = master_o_negative(i,j);
                if(master_jacobian(i,j) > 0)
                {
                    master_L(i,j) = master_p_negative[j];
                }
                else{
                    master_L(i,j) = master_p_postive[j];
                }
            }
            else{
                master_O(i,j) = master_o_positive(i,j);
                if(master_jacobian(i,j) > 0)
                {
                    master_L(i,j) = master_p_postive[j];
                }
                else{
                    master_L(i,j) = master_p_negative[j];
                }
            }

            if(slave_dir[i] < 0){
                slave_O(i,j) = slave_o_negative(i,j);
                if(slave_jacobian(i,j) > 0)
                {
                    slave_L(i,j) = slave_p_negative[j];
                }
                else{
                    slave_L(i,j) = slave_p_postive[j];
                }
            }
            else{
                slave_O(i,j) = slave_o_positive(i,j);
                if(slave_jacobian(i,j) > 0)
                {
                    slave_L(i,j) = slave_p_postive[j];
                }
                else{
                    slave_L(i,j) = slave_p_negative[j];
                }
            }
        }
    }

    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    master_manipulability_jac = (master_L.array() * master_O.array() * master_jacobian.array()).matrix();
    master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> master_solver(master_manipulability_jjt);
    Eigen::Matrix<double,6,6> master_eigenvalue;
    Eigen::Matrix<double,6,6> master_eigenvector;
    master_eigenvalue = master_solver.pseudoEigenvalueMatrix();
    master_eigenvector = master_solver.pseudoEigenvectors();


    double master_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        master_dir_manipulability += std::pow(master_eigenvector.col(i).dot(master_dir) * master_eigenvalue(i,i), 2);
    }


    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;
    slave_manipulability_jac = (slave_L.array() * slave_O.array() * slave_jacobian.array()).matrix();
    slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> slave_solver(slave_manipulability_jjt);
    Eigen::Matrix<double,6,6> slave_eigenvalue;
    Eigen::Matrix<double,6,6> slave_eigenvector;
    slave_eigenvalue = slave_solver.pseudoEigenvalueMatrix();
    slave_eigenvector = slave_solver.pseudoEigenvectors();


    double slave_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        slave_dir_manipulability += std::pow(slave_eigenvector.col(i).dot(slave_dir) * slave_eigenvalue(i,i), 2);
    }

    std::cout<<master_dir_manipulability<<" "<<slave_dir_manipulability<<std::endl;

    if(master_dir_manipulability < slave_dir_manipulability)
        return master_dir_manipulability;
    else
        return slave_dir_manipulability;
}

void DualCBiRRT::init_all_dir_manipulability_compute(robot_state::RobotState & robot_state, Eigen::MatrixXd & master_ob_jac, Eigen::Vector3d master_ob_dir, double master_ob_dis, Eigen::MatrixXd & slave_ob_jac, Eigen::Vector3d slave_ob_dir, double slave_ob_dis, double & out_master_dir_manipulability, double & out_slave_dir_manipulability){
    Eigen::Matrix<double, 6 ,1>  master_all_dir[8];
    Eigen::Matrix<double, 6 ,1>  slave_all_dir[8];

    std::vector<double> master_joint_angles;
    std::vector<double> slave_joint_angles;
    robot_state.copyJointGroupPositions("left_arm", master_joint_angles);
    robot_state.copyJointGroupPositions("right_arm", slave_joint_angles);

    Eigen::Affine3d master_pose;
    Eigen::Affine3d slave_pose;
    Eigen::Matrix3d master_rot;
    Eigen::Matrix3d slave_rot;
    Eigen::Vector3d master_pos;
    Eigen::Vector3d slave_pos;
    KDL::Rotation master_rot_kdl;
    KDL::Rotation slave_rot_kdl;
    Eigen::Vector3d master_euler;
    Eigen::Vector3d slave_euler;

    master_pose = robot_state.getGlobalLinkTransform("left_gripper");
    slave_pose = robot_state.getGlobalLinkTransform("right_gripper");
    master_rot = master_pose.rotation();
    slave_rot = slave_pose.rotation();
    master_pos = master_pose.translation();
    slave_pos = slave_pose.translation();


    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            master_rot_kdl(i,j) = master_rot(i, j);
            slave_rot_kdl(i,j) = slave_rot(i, j);
        }
    }

    master_rot_kdl.GetEulerZYX(master_euler(0), master_euler(1), master_euler(2));
    slave_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1), slave_euler(2));

    Eigen::Vector3d euler_vel(1, 0, 0);

    Eigen::Vector3d master_w_dir;
    Eigen::Vector3d slave_w_dir;

    Eigen::Matrix3d master_euler_to_w;
    Eigen::Matrix3d slave_euler_to_w;

    master_euler_to_w(0, 0) = cos(master_euler(1))*cos(master_euler(2));
    master_euler_to_w(0, 1) = -sin(master_euler(2));
    master_euler_to_w(0, 2) = 0;
    master_euler_to_w(1, 0) = cos(master_euler(1))*sin(master_euler(2));
    master_euler_to_w(1, 1) = cos(master_euler(2));
    master_euler_to_w(1, 2) = 0;
    master_euler_to_w(2, 0) = -sin(master_euler(1));
    master_euler_to_w(2, 1) = 0;
    master_euler_to_w(2, 2) = 1;
    slave_euler_to_w(0, 0) = cos(slave_euler(1))*cos(slave_euler(2));
    slave_euler_to_w(0, 1) = -sin(slave_euler(2));
    slave_euler_to_w(0, 2) = 0;
    slave_euler_to_w(1, 0) = cos(slave_euler(1))*sin(slave_euler(2));
    slave_euler_to_w(1, 1) = cos(slave_euler(2));
    slave_euler_to_w(1, 2) = 0;
    slave_euler_to_w(2, 0) = -sin(slave_euler(1));
    slave_euler_to_w(2, 1) = 0;
    slave_euler_to_w(2, 2) = 1;

    master_w_dir = master_euler_to_w * euler_vel;
    master_w_dir.normalize();
    slave_w_dir = slave_euler_to_w * euler_vel;
    slave_w_dir.normalize();

    master_all_dir[0] << 1, 0, 0, 0, 0, 0;
    master_all_dir[1] << -1, 0, 0, 0, 0, 0;
    master_all_dir[2] << 0, 1, 0, 0, 0, 0;
    master_all_dir[3] << 0, -1, 0, 0, 0, 0;
    master_all_dir[4] << 0, 0, 1, 0, 0, 0;
    master_all_dir[5] << 0, 0, -1, 0, 0, 0;
    master_all_dir[6] << 0, 0, 0, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[7] << 0, 0, 0, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);

    slave_all_dir[0] << 1, 0, 0, 0, 0, 0;
    slave_all_dir[1] << -1, 0, 0, 0, 0, 0;
    slave_all_dir[2] << 0, 1, 0, 0, 0, 0;
    slave_all_dir[3] << 0, -1, 0, 0, 0, 0;
    slave_all_dir[4] << 0, 0, 1, 0, 0, 0;
    slave_all_dir[5] << 0, 0, -1, 0, 0, 0;
    slave_all_dir[6] << 0, 0, 0, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[7] << 0, 0, 0, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);

    Eigen::MatrixXd master_jacobian;
    master_jacobian = robot_state.getJacobian(planning_group);

    Eigen::MatrixXd slave_jacobian;
    slave_jacobian = robot_state.getJacobian(slave_group);

    //joint_limit_penality
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //ob penality
    double master_alpha = 50;
    double master_beta = 2;
    double master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta) * (master_beta / master_ob_dis + master_alpha);
    double master_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;
    master_extended_v.head(3) = master_ob_dir * master_ob_dis;
    master_extended_v.tail(3) = Eigen::Vector3d::Zero();
    master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;
    Eigen::Matrix<double ,6,7> master_o_positive;
    Eigen::Matrix<double ,6,7> master_o_negative;

    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];
    //ob penality
    double slave_alpha = 50;
    double slave_beta = 2;
    double slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta) * (slave_beta / slave_ob_dis + slave_alpha);
    double slave_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;
    slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
    slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
    slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;
    Eigen::Matrix<double ,6,7> slave_o_positive;
    Eigen::Matrix<double ,6,7> slave_o_negative;

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_joint_angles[i] - master_joint_pos_bounds.second[i], 2) * (2*master_joint_angles[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                            /(4 * std::pow(master_joint_pos_bounds.first[i] - master_joint_angles[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_joint_angles[i], 2));
        if((master_joint_angles[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] > master_joint_angles[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }
        master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;

        std::cout<<slave_joint_angles[i]<<std::endl;
        std::cout<<slave_joint_pos_bounds.first[i]<<std::endl;
        std::cout<<slave_joint_pos_bounds.second[i]<<std::endl;

        slave_delta_h[i] = (std::pow(slave_joint_angles[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_joint_angles[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                           /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_joint_angles[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_joint_angles[i], 2));
        if((slave_joint_angles[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] > slave_joint_angles[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
        slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;

    }

    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<7; j++){
            // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
            if(master_ob_dir[i] < 0){
                master_o_negative(i,j) = 1;
                master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
            }
            else{
                master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                master_o_positive(i,j) = 1;
            }
            if(slave_ob_dir[i] < 0){
                slave_o_negative(i,j) = 1;
                slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
            }
            else{
                slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                slave_o_positive(i,j) = 1;
            }
        }
    }

    double master_dir_manipulability = 0;
    double slave_dir_manipulability = 0;

    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;

    Eigen::Matrix<double, 6, 7> master_L[8];
    Eigen::Matrix<double, 6, 7> master_O[8];
    Eigen::Matrix<double, 6, 7> slave_L[8];
    Eigen::Matrix<double, 6, 7> slave_O[8];


    for(size_t dir_num = 0; dir_num < 8; dir_num++){
        for(size_t i=0; i<6; i++){
            for(size_t j=0; j<7; j++){
                //先确定关节限制的权重矩阵
                if(master_all_dir[dir_num][i] < 0){
                    if(master_jacobian(i,j) > 0)
                    {
                        master_L[dir_num](i,j) = master_p_negative[j];
                    }
                    else{
                        master_L[dir_num](i,j) = master_p_postive[j];
                    }
                }
                else{
                    if(master_jacobian(i,j) > 0)
                    {
                        master_L[dir_num](i,j) = master_p_postive[j];
                    }
                    else{
                        master_L[dir_num](i,j) = master_p_negative[j];
                    }
                }

                if(slave_all_dir[dir_num][i] < 0){
                    if(slave_jacobian(i,j) > 0)
                    {
                        slave_L[dir_num](i,j) = slave_p_negative[j];
                    }
                    else{
                        slave_L[dir_num](i,j) = slave_p_postive[j];
                    }
                }
                else{
                    if(slave_jacobian(i,j) > 0)
                    {
                        slave_L[dir_num](i,j) = slave_p_postive[j];
                    }
                    else{
                        slave_L[dir_num](i,j) = slave_p_negative[j];
                    }
                }
                //再确定障碍物限制的权重矩阵
                if(i < 3){
                    if(master_ob_jac.row(i).dot(master_jacobian.row(i)) > 0)
                    {
                        if(master_all_dir[dir_num][i] < 0){
                            master_O[dir_num](i, j) = master_o_negative(i, j);
                        }
                        else{
                            master_O[dir_num](i, j) = master_o_positive(i, j);
                        }
                    }
                    else{
                        if(master_all_dir[dir_num][i] < 0){
                            master_O[dir_num](i, j) = master_o_positive(i, j);
                        }
                        else{
                            master_O[dir_num](i, j) = master_o_negative(i, j);
                        }
                    }
                    if(slave_ob_jac.row(i).dot(slave_jacobian.row(i)) > 0)
                    {
                        if(slave_all_dir[dir_num][i] < 0){
                            slave_O[dir_num](i, j) = slave_o_negative(i, j);
                        }
                        else{
                            slave_O[dir_num](i, j) = slave_o_positive(i, j);
                        }
                    }
                    else{
                        if(slave_all_dir[dir_num][i] < 0){
                            slave_O[dir_num](i, j) = slave_o_positive(i, j);
                        }
                        else{
                            slave_O[dir_num](i, j) = slave_o_negative(i, j);
                        }
                    }
                }
                else{
                    master_O[dir_num](i, j) = 1;
                    slave_O[dir_num](i, j) = 1;
                }
            }
        }


//        std::cout<<"dir_num: "<<dir_num<<std::endl;
//        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//        std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
//        std::cout<<"master_On"<<master_O[dir_num]<<std::endl;
//        std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;

        //既考虑关节限制也考虑碰撞
//        master_manipulability_jac = (master_L[dir_num].array() * master_O[dir_num].array() * master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_L[dir_num].array() * slave_O[dir_num].array() * slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();
        //只考虑碰撞
//        master_manipulability_jac = (master_O[dir_num].array() * master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_O[dir_num].array() * slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();
          for(size_t i =0 ;i<6; i++){
              for(size_t j=0; j<7; j++){
                  master_manipulability_jac(i, j) =  master_O[dir_num](i,j) * master_jacobian(i,j);
                  slave_manipulability_jac(i, j) =  slave_O[dir_num](i,j) * slave_jacobian(i,j);
              }
          }
          master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
          slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        //啥都不考虑
//        master_manipulability_jac = (master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        master_dir_manipulability += std::sqrt(master_manipulability_jjt.determinant());
        slave_dir_manipulability += std::sqrt(slave_manipulability_jjt.determinant());

        if(std::isnan(master_dir_manipulability) or std::isnan(slave_dir_manipulability)){
            Eigen::Matrix<double,7,1> slave_delta_P_theta_matrix;
            Eigen::Matrix<double,7,1> slave_delta_h_matrix;
            for(size_t i=0; i<7; i++){
                slave_delta_P_theta_matrix(i) = slave_delta_P_theta[i];
                slave_delta_h_matrix(i) = slave_delta_h[i];
            }
            std::cout<<"slave_ob_theta_dot\n"<<slave_ob_theta_dot<<std::endl;
            std::cout<<"slave_delta_P_theta\n"<<slave_delta_P_theta_matrix.transpose()<<std::endl;
            std::cout<<"slave_o_positive\n"<<slave_o_positive<<std::endl;
            std::cout<<"slave_o_negative\n"<<slave_o_negative<<std::endl;
            std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
            std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
            std::cout<<"master_jacobian\n"<<master_jacobian<<std::endl;
            std::cout<<"slave_delta_h\n"<<slave_delta_h_matrix.transpose()<<std::endl;
            std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
            std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;
            std::cout<<"slave_jacobian\n"<<slave_jacobian<<std::endl;
            std::cout<<""<<std::endl;
        }

    }

    out_master_dir_manipulability = master_dir_manipulability;
    out_slave_dir_manipulability = slave_dir_manipulability;
}

void DualCBiRRT::init_all_dir_manipulability_compute_all_obstacle(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability){
    Eigen::Matrix<double, 6 ,1>  master_all_dir[16];
    Eigen::Matrix<double, 6 ,1>  slave_all_dir[16];

    std::vector<double> master_joint_angles;
    std::vector<double> slave_joint_angles;
    robot_state.copyJointGroupPositions("left_arm", master_joint_angles);
    robot_state.copyJointGroupPositions("right_arm", slave_joint_angles);

    Eigen::Affine3d master_pose;
    Eigen::Affine3d slave_pose;
    Eigen::Matrix3d master_rot;
    Eigen::Matrix3d slave_rot;
    Eigen::Vector3d master_pos;
    Eigen::Vector3d slave_pos;
    KDL::Rotation master_rot_kdl;
    KDL::Rotation slave_rot_kdl;
    Eigen::Vector3d master_euler;
    Eigen::Vector3d slave_euler;

    master_pose = robot_state.getGlobalLinkTransform("left_gripper");
    slave_pose = robot_state.getGlobalLinkTransform("right_gripper");
    master_rot = master_pose.rotation();
    slave_rot = slave_pose.rotation();
    master_pos = master_pose.translation();
    slave_pos = slave_pose.translation();


    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            master_rot_kdl(i,j) = master_rot(i, j);
            slave_rot_kdl(i,j) = slave_rot(i, j);
        }
    }

    master_rot_kdl.GetEulerZYX(master_euler(0), master_euler(1), master_euler(2));
    slave_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1), slave_euler(2));

    Eigen::Vector3d euler_vel(1, 0, 0);

    Eigen::Vector3d master_w_dir;
    Eigen::Vector3d slave_w_dir;

    Eigen::Matrix3d master_euler_to_w;
    Eigen::Matrix3d slave_euler_to_w;

    master_euler_to_w(0, 0) = cos(master_euler(1))*cos(master_euler(2));
    master_euler_to_w(0, 1) = -sin(master_euler(2));
    master_euler_to_w(0, 2) = 0;
    master_euler_to_w(1, 0) = cos(master_euler(1))*sin(master_euler(2));
    master_euler_to_w(1, 1) = cos(master_euler(2));
    master_euler_to_w(1, 2) = 0;
    master_euler_to_w(2, 0) = -sin(master_euler(1));
    master_euler_to_w(2, 1) = 0;
    master_euler_to_w(2, 2) = 1;
    slave_euler_to_w(0, 0) = cos(slave_euler(1))*cos(slave_euler(2));
    slave_euler_to_w(0, 1) = -sin(slave_euler(2));
    slave_euler_to_w(0, 2) = 0;
    slave_euler_to_w(1, 0) = cos(slave_euler(1))*sin(slave_euler(2));
    slave_euler_to_w(1, 1) = cos(slave_euler(2));
    slave_euler_to_w(1, 2) = 0;
    slave_euler_to_w(2, 0) = -sin(slave_euler(1));
    slave_euler_to_w(2, 1) = 0;
    slave_euler_to_w(2, 2) = 1;

    master_w_dir = master_euler_to_w * euler_vel;
    master_w_dir.normalize();
    slave_w_dir = slave_euler_to_w * euler_vel;
    slave_w_dir.normalize();

    master_all_dir[0] << 1, 1, 1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[1] << 1, 1, 1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[2] << 1, 1, -1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[3] << 1, 1, -1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[4] << 1, -1, 1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[5] << 1, -1, 1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[6] << 1, -1, -1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[7] << 1, -1, -1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[8] << -1, 1, 1,master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[9] << -1, 1, 1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[10] << -1, 1, -1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[11] << -1, 1, -1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[12] << -1, -1, 1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[13] << -1, -1, 1, -master_w_dir(0), -master_w_dir(1), -master_w_dir(2);
    master_all_dir[14] << -1, -1, -1, master_w_dir(0), master_w_dir(1), master_w_dir(2);
    master_all_dir[15] << -1, -1, -1,-master_w_dir(0), -master_w_dir(1), -master_w_dir(2);

    slave_all_dir[0] << 1, 1, 1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[1] << 1, 1, 1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[2] << 1, 1, -1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[3] << 1, 1, -1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[4] << 1, -1, 1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[5] << 1, -1, 1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[6] << 1, -1, -1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[7] << 1, -1, -1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[8] << -1, 1, 1,slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[9] << -1, 1, 1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[10] << -1, 1, -1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[11] << -1, 1, -1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[12] << -1, -1, 1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[13] << -1, -1, 1, -slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    slave_all_dir[14] << -1, -1, -1, slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    slave_all_dir[15] << -1, -1, -1,-slave_w_dir(0), -slave_w_dir(1), -slave_w_dir(2);
    
    Eigen::MatrixXd master_jacobian;
    master_jacobian = robot_state.getJacobian(planning_group);

    Eigen::MatrixXd slave_jacobian;
    slave_jacobian = robot_state.getJacobian(slave_group);

    //begin*******************************joint_limit_penality********************************
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_joint_angles[i] - master_joint_pos_bounds.second[i], 2) * (2*master_joint_angles[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                            /(4 * std::pow(master_joint_pos_bounds.first[i] - master_joint_angles[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_joint_angles[i], 2));
        if((master_joint_angles[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] > master_joint_angles[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }

        slave_delta_h[i] = (std::pow(slave_joint_angles[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_joint_angles[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                           /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_joint_angles[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_joint_angles[i], 2));
        if((slave_joint_angles[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] > slave_joint_angles[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
    }
    
    Eigen::Matrix<double, 6, 7> master_L[16];
    Eigen::Matrix<double, 6, 7> slave_L[16];
    for(size_t dir_num = 0; dir_num < 16; dir_num++) {
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 7; j++) {
                //先确定关节限制的权重矩阵
                if (master_all_dir[dir_num][i] < 0) {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    }
                } else {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    }
                }

                if (slave_all_dir[dir_num][i] < 0) {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    }
                } else {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    }
                }
            }
        }
    }
    //end*******************************joint_limit_penality********************************

    
    //begin**********************************ob penality************************************
    double master_alpha = 50;
    double master_beta = 2;
    double master_dis_thre = 0.1;
    double master_ob_dis = 0;
    double master_delta_P_d;
    Eigen::Vector3d master_ob_dir;
    double master_delta_P_theta[7];
    std::string master_link_name;
    Eigen::Vector3d master_nearest_point;

    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;

    std::vector<Eigen::MatrixXd> master_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_negative_vec;
    collision_detection::DistanceResultsData master_tmp_dis_res_data;
    for(auto it = master_dis_map.begin(); it!=master_dis_map.end();it++){
        if(it->second[0].distance > master_dis_thre){
            
            master_tmp_dis_res_data = it->second[0];
            master_ob_dis = master_tmp_dis_res_data.distance;
            master_ob_dir = master_tmp_dis_res_data.normal;
            master_link_name = master_tmp_dis_res_data.link_names[1];
            master_nearest_point = master_tmp_dis_res_data.nearest_points[1];
            const robot_state::LinkModel* master_collision_link = robot_state.getLinkModel(master_link_name);
            Eigen::MatrixXd master_ob_jac;
            robot_state.getJacobian(planning_group, master_collision_link, master_nearest_point, master_ob_jac);
            
            master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta) * (master_beta / master_ob_dis + master_alpha);
            master_extended_v.head(3) = master_ob_dir * master_ob_dis;
            master_extended_v.tail(3) = Eigen::Vector3d::Zero();
            master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;

            Eigen::Matrix<double ,3,7> master_o_positive;
            Eigen::Matrix<double ,3,7> master_o_negative;
            
            for(size_t i=0; i<7; i++){
                master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(master_ob_dir[i] < 0){
                        master_o_negative(i,j) = 1;
                        master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                    }
                    else{
                        master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                        master_o_positive(i,j) = 1;
                    }
                }
            }
            master_o_positive_vec.push_back(master_o_positive);
            master_o_negative_vec.push_back(master_o_negative);
            master_ob_jac_vec.push_back(master_ob_jac);
        }
    }


    double slave_alpha = 50;
    double slave_beta = 2;
    double slave_dis_thre = 0.1;
    double slave_ob_dis = 0;
    double slave_delta_P_d;
    Eigen::Vector3d slave_ob_dir;
    double slave_delta_P_theta[7];
    std::string slave_link_name;
    Eigen::Vector3d slave_nearest_point;

    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;

    std::vector<Eigen::MatrixXd> slave_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_negative_vec;
    collision_detection::DistanceResultsData slave_tmp_dis_res_data;
    for(auto it = slave_dis_map.begin(); it!=slave_dis_map.end();it++){
        if(it->second[0].distance > slave_dis_thre){

            slave_tmp_dis_res_data = it->second[0];
            slave_ob_dis = slave_tmp_dis_res_data.distance;
            slave_ob_dir = slave_tmp_dis_res_data.normal;
            slave_link_name = slave_tmp_dis_res_data.link_names[1];
            slave_nearest_point = slave_tmp_dis_res_data.nearest_points[1];
            const robot_state::LinkModel* slave_collision_link = robot_state.getLinkModel(slave_link_name);
            Eigen::MatrixXd slave_ob_jac;
            robot_state.getJacobian(slave_group, slave_collision_link, slave_nearest_point, slave_ob_jac);

            slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta) * (slave_beta / slave_ob_dis + slave_alpha);
            slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
            slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
            slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;

            Eigen::Matrix<double ,3,7> slave_o_positive;
            Eigen::Matrix<double ,3,7> slave_o_negative;

            for(size_t i=0; i<7; i++){
                slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(slave_ob_dir[i] < 0){
                        slave_o_negative(i,j) = 1;
                        slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                    }
                    else{
                        slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                        slave_o_positive(i,j) = 1;
                    }
                }
            }
            slave_o_positive_vec.push_back(slave_o_positive);
            slave_o_negative_vec.push_back(slave_o_negative);
            slave_ob_jac_vec.push_back(slave_ob_jac);
        }
    }

    Eigen::Matrix<double, 6, 7> master_O[16];
    Eigen::Matrix<double, 6, 7> slave_O[16];
    for(size_t dir_num = 0; dir_num < 16; dir_num++) {
        master_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < master_ob_jac_vec.size(); link_num++){
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                        if (master_ob_jac_vec[link_num].row(i).dot(master_jacobian.row(i)) > 0) {
                            if (master_all_dir[dir_num][i] < 0) {
                                master_O[dir_num](i, j) *= master_o_negative_vec[link_num](i, j);
                            } 
                            else {
                                master_O[dir_num](i, j) *= master_o_positive_vec[link_num](i, j);
                            }
                        } 
                        else {
                            if (master_all_dir[dir_num][i] < 0) {
                                master_O[dir_num](i, j) *= master_o_positive_vec[link_num](i, j);
                            } 
                            else {
                                master_O[dir_num](i, j) *= master_o_negative_vec[link_num](i, j);
                            }
                        }
                    } 
                    else {
                        master_O[dir_num](i, j) *= 1;
                    }
                }
            }   
        }
        slave_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < slave_ob_jac_vec.size(); link_num++){
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                        if (slave_ob_jac_vec[link_num].row(i).dot(slave_jacobian.row(i)) > 0) {
                            if (slave_all_dir[dir_num][i] < 0) {
                                slave_O[dir_num](i, j) *= slave_o_negative_vec[link_num](i, j);
                            }
                            else {
                                slave_O[dir_num](i, j) *= slave_o_positive_vec[link_num](i, j);
                            }
                        }
                        else {
                            if (slave_all_dir[dir_num][i] < 0) {
                                slave_O[dir_num](i, j) *= slave_o_positive_vec[link_num](i, j);
                            }
                            else {
                                slave_O[dir_num](i, j) *= slave_o_negative_vec[link_num](i, j);
                            }
                        }
                    }
                    else {
                        slave_O[dir_num](i, j) *= 1;
                    }
                }
            }
        }
    }
    //end**********************************ob penality************************************


    //begin**********************************final manipulability ************************************
    double master_dir_manipulability = 0;
    double slave_dir_manipulability = 0;

    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;
    
    for(size_t dir_num = 0; dir_num < 16; dir_num++){
//        std::cout<<"dir_num: "<<dir_num<<std::endl;
//        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//        std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
//        std::cout<<"master_On"<<master_O[dir_num]<<std::endl;
//        std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;

        //既考虑关节限制也考虑碰撞
        master_manipulability_jac = (master_L[dir_num].array() * master_O[dir_num].array() * master_jacobian.array()).matrix();
        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
        slave_manipulability_jac = (slave_L[dir_num].array() * slave_O[dir_num].array() * slave_jacobian.array()).matrix();
        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();
        //只考虑碰撞
//        master_manipulability_jac = (master_O[dir_num].array() * master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_O[dir_num].array() * slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        //啥都不考虑
//        master_manipulability_jac = (master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        master_dir_manipulability += std::sqrt(master_manipulability_jjt.determinant());
        slave_dir_manipulability += std::sqrt(slave_manipulability_jjt.determinant());
    }
    //end**********************************final manipulability ************************************

//        if(std::isnan(master_dir_manipulability) or std::isnan(slave_dir_manipulability)){
//            Eigen::Matrix<double,7,1> slave_delta_P_theta_matrix;
//            Eigen::Matrix<double,7,1> slave_delta_h_matrix;
//            for(size_t i=0; i<7; i++){
//                slave_delta_P_theta_matrix(i) = slave_delta_P_theta[i];
//                slave_delta_h_matrix(i) = slave_delta_h[i];
//            }
//            std::cout<<"slave_ob_theta_dot\n"<<slave_ob_theta_dot<<std::endl;
//            std::cout<<"slave_delta_P_theta\n"<<slave_delta_P_theta_matrix.transpose()<<std::endl;
//            std::cout<<"slave_o_positive\n"<<slave_o_positive<<std::endl;
//            std::cout<<"slave_o_negative\n"<<slave_o_negative<<std::endl;
//            std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//            std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
//            std::cout<<"master_jacobian\n"<<master_jacobian<<std::endl;
//            std::cout<<"slave_delta_h\n"<<slave_delta_h_matrix.transpose()<<std::endl;
//            std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
//            std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;
//            std::cout<<"slave_jacobian\n"<<slave_jacobian<<std::endl;
//            std::cout<<""<<std::endl;
//        }

    

    out_master_dir_manipulability = master_dir_manipulability;
    out_slave_dir_manipulability = slave_dir_manipulability;
    if(master_dir_manipulability < 0 or slave_dir_manipulability < 0){
        std::cout<<"????"<<std::endl;

    }
}

void DualCBiRRT::init_all_dir_manipulability_compute_all_obstacle_one_dir(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability, Eigen::Vector4d one_dir){
    Eigen::Matrix<double, 6 ,1>  master_all_dir[1];
    Eigen::Matrix<double, 6 ,1>  slave_all_dir[1];

    std::vector<double> master_joint_angles;
    std::vector<double> slave_joint_angles;
    robot_state.copyJointGroupPositions("left_arm", master_joint_angles);
    robot_state.copyJointGroupPositions("right_arm", slave_joint_angles);

    Eigen::Affine3d master_pose;
    Eigen::Affine3d slave_pose;
    Eigen::Matrix3d master_rot;
    Eigen::Matrix3d slave_rot;
    Eigen::Vector3d master_pos;
    Eigen::Vector3d slave_pos;
    KDL::Rotation master_rot_kdl;
    KDL::Rotation slave_rot_kdl;
    Eigen::Vector3d master_euler;
    Eigen::Vector3d slave_euler;

    master_pose = robot_state.getGlobalLinkTransform("left_gripper");
    slave_pose = robot_state.getGlobalLinkTransform("right_gripper");
    master_rot = master_pose.rotation();
    slave_rot = slave_pose.rotation();
    master_pos = master_pose.translation();
    slave_pos = slave_pose.translation();


    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            master_rot_kdl(i,j) = master_rot(i, j);
            slave_rot_kdl(i,j) = slave_rot(i, j);
        }
    }

    master_rot_kdl.GetEulerZYX(master_euler(0), master_euler(1), master_euler(2));
    slave_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1), slave_euler(2));

    Eigen::Vector3d euler_vel(one_dir(3), 0, 0);

    Eigen::Vector3d master_w_dir;
    Eigen::Vector3d slave_w_dir;

    Eigen::Matrix3d master_euler_to_w;
    Eigen::Matrix3d slave_euler_to_w;

    master_euler_to_w(0, 0) = cos(master_euler(1))*cos(master_euler(2));
    master_euler_to_w(0, 1) = -sin(master_euler(2));
    master_euler_to_w(0, 2) = 0;
    master_euler_to_w(1, 0) = cos(master_euler(1))*sin(master_euler(2));
    master_euler_to_w(1, 1) = cos(master_euler(2));
    master_euler_to_w(1, 2) = 0;
    master_euler_to_w(2, 0) = -sin(master_euler(1));
    master_euler_to_w(2, 1) = 0;
    master_euler_to_w(2, 2) = 1;
    slave_euler_to_w(0, 0) = cos(slave_euler(1))*cos(slave_euler(2));
    slave_euler_to_w(0, 1) = -sin(slave_euler(2));
    slave_euler_to_w(0, 2) = 0;
    slave_euler_to_w(1, 0) = cos(slave_euler(1))*sin(slave_euler(2));
    slave_euler_to_w(1, 1) = cos(slave_euler(2));
    slave_euler_to_w(1, 2) = 0;
    slave_euler_to_w(2, 0) = -sin(slave_euler(1));
    slave_euler_to_w(2, 1) = 0;
    slave_euler_to_w(2, 2) = 1;

//    std::cout<<"master_euler_to_w\n"<<master_euler_to_w<<std::endl;
//    std::cout<<"slave_euler_to_w\n"<<slave_euler_to_w<<std::endl;
    master_w_dir = master_euler_to_w * euler_vel;
    slave_w_dir = slave_euler_to_w * euler_vel;

//    std::cout<<"master_w_dir\n"<<master_w_dir<<std::endl;
//    std::cout<<"slave_w_dir\n"<<slave_w_dir<<std::endl;

    master_all_dir[0] << one_dir(0), one_dir(1), one_dir(2), master_w_dir(0), master_w_dir(1), master_w_dir(2);
    slave_all_dir[0] << one_dir(0), one_dir(1), one_dir(2), slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    master_all_dir[0].normalize();
    slave_all_dir[0].normalize();
    

//    std::cout<<"master_all_dir\n"<<master_all_dir[0].transpose()<<std::endl;
//    std::cout<<"slave_all_dir\n"<<slave_all_dir[0].transpose()<<std::endl;

    Eigen::MatrixXd master_jacobian;
    master_jacobian = robot_state.getJacobian(planning_group);

    Eigen::MatrixXd slave_jacobian;
    slave_jacobian = robot_state.getJacobian(slave_group);

    //begin*******************************joint_limit_penality********************************
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_joint_angles[i] - master_joint_pos_bounds.second[i], 2) * (2*master_joint_angles[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                            /(4 * std::pow(master_joint_pos_bounds.first[i] - master_joint_angles[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_joint_angles[i], 2));
        if((master_joint_angles[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] - master_joint_angles[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }

        slave_delta_h[i] = (std::pow(slave_joint_angles[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_joint_angles[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                           /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_joint_angles[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_joint_angles[i], 2));
        if((slave_joint_angles[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] - slave_joint_angles[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
    }

    Eigen::Matrix<double, 6, 7> master_L[1];
    Eigen::Matrix<double, 6, 7> slave_L[1];
    for(size_t dir_num = 0; dir_num < 1; dir_num++) {
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 7; j++) {
                //先确定关节限制的权重矩阵
                if (master_all_dir[dir_num][i] < 0) {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    }
                } else {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    }
                }

                if (slave_all_dir[dir_num][i] < 0) {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    }
                } else {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    }
                }
            }
        }
    }
    //end*******************************joint_limit_penality********************************


    //begin**********************************ob penality************************************
    double master_alpha = 50;
    double master_beta = 1.5;
    double master_dis_thre = 0.1;
    double master_ob_dis = 0;
    double master_delta_P_d;
    Eigen::Vector3d master_ob_dir;
    double master_delta_P_theta[7];
    std::string master_link_name;
    Eigen::Vector3d master_nearest_point;

    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;

    Eigen::Vector3d master_robot_collision_point;
    Eigen::Vector3d master_object_collision_point;
    
    std::vector<Eigen::MatrixXd> master_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_negative_vec;
    collision_detection::DistanceResultsData master_tmp_dis_res_data;
    for(auto it = master_dis_map.begin(); it!=master_dis_map.end();it++){
        if(it->second[0].distance < master_dis_thre){
            std::cout<<it->second[0].link_names[0]<<" "<< it->second[0].link_names[1]<<" "<<it->second[0].distance<<std::endl;
            master_tmp_dis_res_data = it->second[0];
            master_ob_dis = master_tmp_dis_res_data.distance;
//            master_ob_dir = master_tmp_dis_res_data.normal;
            master_link_name = master_tmp_dis_res_data.link_names[1];
            master_nearest_point = master_tmp_dis_res_data.nearest_points[1];
            const robot_state::LinkModel* master_collision_link = robot_state.getLinkModel(master_link_name);
            Eigen::MatrixXd master_ob_jac;
            robot_state.getJacobian(planning_group, master_collision_link, master_nearest_point, master_ob_jac);

            master_robot_collision_point = robot_state.getGlobalLinkTransform(master_link_name) * master_tmp_dis_res_data.nearest_points[1];
            moveit_msgs::CollisionObject collision_object_msg;
            planning_scene_ptr->getCollisionObjectMsg(collision_object_msg, master_tmp_dis_res_data.link_names[0]);
            Eigen::Vector3d object_original(collision_object_msg.primitive_poses[0].position.x, collision_object_msg.primitive_poses[0].position.y, collision_object_msg.primitive_poses[0].position.z);
            master_object_collision_point = master_tmp_dis_res_data.nearest_points[0] + object_original;
            master_ob_dir = master_robot_collision_point - master_object_collision_point;
            master_ob_dir.normalize();
            
            master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta) * (master_beta / master_ob_dis + master_alpha);
            master_extended_v.head(3) = master_ob_dir * master_ob_dis;
            master_extended_v.tail(3) = Eigen::Vector3d::Zero();
            master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;

            Eigen::Matrix<double ,3,7> master_o_positive;
            Eigen::Matrix<double ,3,7> master_o_negative;

            for(size_t i=0; i<7; i++){
                master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(master_ob_dir[i] < 0){
                        master_o_negative(i,j) = 1;
                        master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                    }
                    else{
                        master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                        master_o_positive(i,j) = 1;
                    }
                }
            }
            master_o_positive_vec.push_back(master_o_positive);
            master_o_negative_vec.push_back(master_o_negative);
            master_ob_jac_vec.push_back(master_ob_jac);
//            std::cout<<"one dir  "<<one_dir.transpose()<<std::endl;
            std::cout<<"master_ob_dir  "<<master_ob_dir.transpose()<<std::endl;
//            std::cout<<master_link_name<<std::endl;
//            std::cout<<"distance  "<<master_ob_dis<<std::endl;
            std::cout<<"master_o_positive\n"<<master_o_positive<<std::endl;
            std::cout<<"master_o_negative\n"<<master_o_negative<<std::endl;
            std::cout<<"\n\n"<<std::endl;
        }
    }
    if(master_o_positive_vec.empty()){
        master_o_positive_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        master_o_negative_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        master_ob_jac_vec.push_back(master_jacobian);
    }


    double slave_alpha = 50;
    double slave_beta = 1.5;
    double slave_dis_thre = 0.1;
    double slave_ob_dis = 0;
    double slave_delta_P_d;
    Eigen::Vector3d slave_ob_dir;
    double slave_delta_P_theta[7];
    std::string slave_link_name;
    Eigen::Vector3d slave_nearest_point;

    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;
    
    Eigen::Vector3d slave_robot_collision_point;
    Eigen::Vector3d slave_object_collision_point;
    
    std::vector<Eigen::MatrixXd> slave_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_negative_vec;
    collision_detection::DistanceResultsData slave_tmp_dis_res_data;
    for(auto it = slave_dis_map.begin(); it!=slave_dis_map.end();it++){
        if(it->second[0].distance < slave_dis_thre){
            std::cout<<it->second[0].link_names[0]<<" "<< it->second[0].link_names[1]<<" "<<it->second[0].distance<<std::endl;
            slave_tmp_dis_res_data = it->second[0];
            slave_ob_dis = slave_tmp_dis_res_data.distance;
//            slave_ob_dir = slave_tmp_dis_res_data.normal;
            slave_link_name = slave_tmp_dis_res_data.link_names[1];
            slave_nearest_point = slave_tmp_dis_res_data.nearest_points[1];
            const robot_state::LinkModel* slave_collision_link = robot_state.getLinkModel(slave_link_name);
            Eigen::MatrixXd slave_ob_jac;
            robot_state.getJacobian(slave_group, slave_collision_link, slave_nearest_point, slave_ob_jac);


            slave_robot_collision_point = robot_state.getGlobalLinkTransform(slave_link_name) * slave_tmp_dis_res_data.nearest_points[1];
            moveit_msgs::CollisionObject collision_object_msg;
            planning_scene_ptr->getCollisionObjectMsg(collision_object_msg, slave_tmp_dis_res_data.link_names[0]);
            Eigen::Vector3d object_original(collision_object_msg.primitive_poses[0].position.x, collision_object_msg.primitive_poses[0].position.y, collision_object_msg.primitive_poses[0].position.z);
            slave_object_collision_point = slave_tmp_dis_res_data.nearest_points[0] + object_original;
            slave_ob_dir = slave_robot_collision_point - slave_object_collision_point;
            slave_ob_dir.normalize();
            
            
            slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta) * (slave_beta / slave_ob_dis + slave_alpha);
            slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
            slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
            slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;

            Eigen::Matrix<double ,3,7> slave_o_positive;
            Eigen::Matrix<double ,3,7> slave_o_negative;

            for(size_t i=0; i<7; i++){
                slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(slave_ob_dir[i] < 0){
                        slave_o_negative(i,j) = 1;
                        slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                    }
                    else{
                        slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                        slave_o_positive(i,j) = 1;
                    }
                }
            }
            slave_o_positive_vec.push_back(slave_o_positive);
            slave_o_negative_vec.push_back(slave_o_negative);
            slave_ob_jac_vec.push_back(slave_ob_jac);
//            std::cout<<"slave_tmp_dis_res_data.nearest_points[0]\n"<<slave_tmp_dis_res_data.nearest_points[0].transpose()<<std::endl;
//            std::cout<<"slave_tmp_dis_res_data.nearest_points[1]\n"<<slave_tmp_dis_res_data.nearest_points[1].transpose()<<std::endl;
            Eigen::Vector3d tmp;
            tmp = slave_tmp_dis_res_data.nearest_points[1] -slave_tmp_dis_res_data.nearest_points[0];
            tmp.normalize();
//            std::cout<<"right_end_pos\n"<<robot_state.getGlobalLinkTransform("right_gripper").translation().transpose()<<std::endl;
//            std::cout<<"1-0\n"<<tmp.transpose()<<std::endl;
//
            std::cout<<"slave_ob_dir\n"<<slave_ob_dir.transpose()<<std::endl;
            std::cout<<"slave_o_positive\n"<<slave_o_positive<<std::endl;
            std::cout<<"slave_o_negative\n"<<slave_o_negative<<std::endl;
            std::cout<<"\n\n"<<std::endl;
        }
    }
    //防止没有障碍物的情况
    if(slave_o_positive_vec.empty()){
        slave_o_positive_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        slave_o_negative_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        slave_ob_jac_vec.push_back(slave_jacobian);
    }

    Eigen::Matrix<double, 6, 7> master_O[1];
    Eigen::Matrix<double, 6, 7> slave_O[1];
    Eigen::Matrix<double, 6, 1> master_end_cause_collision_point;
    Eigen::Matrix<double, 6, 1> slave_end_cause_collision_point;
    
    for(size_t dir_num = 0; dir_num < 1; dir_num++) {
        master_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < master_ob_jac_vec.size(); link_num++){
            master_end_cause_collision_point = master_ob_jac_vec[link_num] * (master_jacobian.transpose() * master_all_dir[dir_num]);
            
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                        if (master_end_cause_collision_point[i] < 0) {
                            if(master_O[dir_num](i, j) > master_o_negative_vec[link_num](i, j)){
                                master_O[dir_num](i, j) = master_o_negative_vec[link_num](i, j);
                            }
                        }
                        else {
                            if(master_O[dir_num](i, j) > master_o_positive_vec[link_num](i, j)){
                                master_O[dir_num](i, j) = master_o_positive_vec[link_num](i, j);
                            }
                        }
                    }
                    else {
                        master_O[dir_num](i, j) *= 1;
                    }
                }
            }
        }
        slave_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < slave_ob_jac_vec.size(); link_num++){
            slave_end_cause_collision_point = slave_ob_jac_vec[link_num] * (slave_jacobian.transpose() * slave_all_dir[dir_num]);

            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                        if (slave_end_cause_collision_point[i] < 0) {
                            if(slave_O[dir_num](i, j) > slave_o_negative_vec[link_num](i, j)){
                                slave_O[dir_num](i, j) = slave_o_negative_vec[link_num](i, j);
                            }
                        }
                        else {
                            if(slave_O[dir_num](i, j) > slave_o_positive_vec[link_num](i, j)){
                                slave_O[dir_num](i, j) = slave_o_positive_vec[link_num](i, j);
                            }
                        }
                    }
                    else {
                        slave_O[dir_num](i, j) *= 1;
                    }
                }
            }
        }
    }
    //end**********************************ob penality************************************


    //begin**********************************final manipulability ************************************
    double master_dir_manipulability = 0;
    double slave_dir_manipulability = 0;

    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;

    for(size_t dir_num = 0; dir_num < 1; dir_num++){
//        std::cout<<"dir_num: "<<dir_num<<std::endl;
//        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//        std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
        std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
        std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;
//        std::cout<<"master_jacobian\n"<<master_jacobian<<std::endl;
//        std::cout<<"slave_jacobian\n"<<slave_jacobian<<std::endl;
//        std::cout<<"master_manipulability_jac\n"<<master_manipulability_jac<<std::endl;
//        std::cout<<"slave_manipulability_jac\n"<<slave_manipulability_jac<<std::endl;

        //既考虑关节限制也考虑碰撞
        master_manipulability_jac = (master_L[dir_num].array() * master_O[dir_num].array() * master_jacobian.array()).matrix();
        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
        slave_manipulability_jac = (slave_L[dir_num].array() * slave_O[dir_num].array() * slave_jacobian.array()).matrix();
        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

//        std::cout<<"master normal jacobian\n"<<master_jacobian<<std::endl;
//        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//        std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
//        std::cout<<"master peanal jacobian\n"<<master_manipulability_jac<<std::endl;
//        std::cout<<"jjt\n"<<master_manipulability_jjt<<std::endl;

        //只考虑碰撞
//        master_manipulability_jac = (master_O[dir_num].array() * master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_O[dir_num].array() * slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        //啥都不考虑
//        master_manipulability_jac = (master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();


//        Eigen::EigenSolver<Eigen::Matrix<double,6,6>> master_jjt_solver(master_manipulability_jjt);
//        Eigen::Matrix<double,6,6> master_jjt_eigen_vale_matrix;
//        Eigen::Matrix<double,6,6> master_jjt_eigen_vector_matrix;
//        master_jjt_eigen_vale_matrix = master_jjt_solver.pseudoEigenvalueMatrix();
//        master_jjt_eigen_vector_matrix = master_jjt_solver.pseudoEigenvectors();

        Eigen::JacobiSVD<Eigen::MatrixXd> master_svd_holder(master_manipulability_jac, Eigen::ComputeFullU);
        Eigen::MatrixXd master_singular;
        Eigen::MatrixXd master_U;
        master_singular = master_svd_holder.singularValues();
        master_U = master_svd_holder.matrixU();
        for(size_t i=0; i<6; i++){
            master_dir_manipulability += std::abs(master_U.col(i).dot(master_all_dir[dir_num]) * master_singular(i));
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> slave_svd_holder(slave_manipulability_jac, Eigen::ComputeFullU);
        Eigen::MatrixXd slave_singular;
        Eigen::MatrixXd slave_U;
        slave_singular = slave_svd_holder.singularValues();
        slave_U = slave_svd_holder.matrixU();
        for(size_t i=0; i<6; i++){
            slave_dir_manipulability += std::abs(slave_U.col(i).dot(slave_all_dir[dir_num]) * slave_singular(i));
        }

//        std::cout<<"master_singular\n"<<master_singular<<std::endl;
//        std::cout<<"slave_singular\n"<<slave_singular<<std::endl;
//        Eigen::EigenSolver<Eigen::Matrix<double,6,6>> slave_jjt_solver(slave_manipulability_jjt);
//        Eigen::Matrix<double,6,6> slave_jjt_eigen_vale_matrix;
//        Eigen::Matrix<double,6,6> slave_jjt_eigen_vector_matrix;
//        slave_jjt_eigen_vale_matrix = slave_jjt_solver.pseudoEigenvalueMatrix();
//        slave_jjt_eigen_vector_matrix = slave_jjt_solver.pseudoEigenvectors();
//        for(size_t i=0; i<6; i++){
//            slave_dir_manipulability += std::abs(slave_jjt_eigen_vector_matrix.col(i).dot(slave_all_dir[dir_num]) * slave_jjt_eigen_vale_matrix(i,i));
//        }
//        master_dir_manipulability += std::sqrt(master_manipulability_jjt.determinant());
//        slave_dir_manipulability += std::sqrt(slave_manipulability_jjt.determinant());
    }
    //end**********************************final manipulability ************************************
    out_master_dir_manipulability = master_dir_manipulability;
    out_slave_dir_manipulability = slave_dir_manipulability;
    if(master_dir_manipulability < 0 or slave_dir_manipulability < 0){
        std::cout<<"????"<<std::endl;
    }
}

void DualCBiRRT::init_all_dir_manipulability_compute_all_obstacle_one_dir_test(robot_state::RobotState & robot_state, collision_detection::DistanceMap & master_dis_map, collision_detection::DistanceMap & slave_dis_map,  double & out_master_dir_manipulability, double & out_slave_dir_manipulability, Eigen::Vector4d one_dir, visualization_msgs::MarkerArray & robot_collision_points, visualization_msgs::MarkerArray & object_colllision_points){
    Eigen::Matrix<double, 6 ,1>  master_all_dir[1];
    Eigen::Matrix<double, 6 ,1>  slave_all_dir[1];

    std::vector<double> master_joint_angles;
    std::vector<double> slave_joint_angles;
    robot_state.copyJointGroupPositions("left_arm", master_joint_angles);
    robot_state.copyJointGroupPositions("right_arm", slave_joint_angles);

    Eigen::Affine3d master_pose;
    Eigen::Affine3d slave_pose;
    Eigen::Matrix3d master_rot;
    Eigen::Matrix3d slave_rot;
    Eigen::Vector3d master_pos;
    Eigen::Vector3d slave_pos;
    KDL::Rotation master_rot_kdl;
    KDL::Rotation slave_rot_kdl;
    Eigen::Vector3d master_euler;
    Eigen::Vector3d slave_euler;

    master_pose = robot_state.getGlobalLinkTransform("left_gripper");
    slave_pose = robot_state.getGlobalLinkTransform("right_gripper");
    master_rot = master_pose.rotation();
    slave_rot = slave_pose.rotation();
    master_pos = master_pose.translation();
    slave_pos = slave_pose.translation();


    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            master_rot_kdl(i,j) = master_rot(i, j);
            slave_rot_kdl(i,j) = slave_rot(i, j);
        }
    }

    master_rot_kdl.GetEulerZYX(master_euler(0), master_euler(1), master_euler(2));
    slave_rot_kdl.GetEulerZYX(slave_euler(0), slave_euler(1), slave_euler(2));

    Eigen::Vector3d euler_vel(one_dir(3), 0, 0);

    Eigen::Vector3d master_w_dir;
    Eigen::Vector3d slave_w_dir;

    Eigen::Matrix3d master_euler_to_w;
    Eigen::Matrix3d slave_euler_to_w;

    master_euler_to_w(0, 0) = cos(master_euler(1))*cos(master_euler(2));
    master_euler_to_w(0, 1) = -sin(master_euler(2));
    master_euler_to_w(0, 2) = 0;
    master_euler_to_w(1, 0) = cos(master_euler(1))*sin(master_euler(2));
    master_euler_to_w(1, 1) = cos(master_euler(2));
    master_euler_to_w(1, 2) = 0;
    master_euler_to_w(2, 0) = -sin(master_euler(1));
    master_euler_to_w(2, 1) = 0;
    master_euler_to_w(2, 2) = 1;
    slave_euler_to_w(0, 0) = cos(slave_euler(1))*cos(slave_euler(2));
    slave_euler_to_w(0, 1) = -sin(slave_euler(2));
    slave_euler_to_w(0, 2) = 0;
    slave_euler_to_w(1, 0) = cos(slave_euler(1))*sin(slave_euler(2));
    slave_euler_to_w(1, 1) = cos(slave_euler(2));
    slave_euler_to_w(1, 2) = 0;
    slave_euler_to_w(2, 0) = -sin(slave_euler(1));
    slave_euler_to_w(2, 1) = 0;
    slave_euler_to_w(2, 2) = 1;

    std::cout<<"master_euler_to_w\n"<<master_euler_to_w<<std::endl;
    std::cout<<"slave_euler_to_w\n"<<slave_euler_to_w<<std::endl;
    master_w_dir = master_euler_to_w * euler_vel;
    slave_w_dir = slave_euler_to_w * euler_vel;

    std::cout<<"master_w_dir\n"<<master_w_dir<<std::endl;
    std::cout<<"slave_w_dir\n"<<slave_w_dir<<std::endl;

    master_all_dir[0] << one_dir(0), one_dir(1), one_dir(2), master_w_dir(0), master_w_dir(1), master_w_dir(2);
    slave_all_dir[0] << one_dir(0), one_dir(1), one_dir(2), slave_w_dir(0), slave_w_dir(1), slave_w_dir(2);
    master_all_dir[0].normalize();
    slave_all_dir[0].normalize();

    std::cout<<"master_all_dir\n"<<master_all_dir[0].transpose()<<std::endl;
    std::cout<<"slave_all_dir\n"<<slave_all_dir[0].transpose()<<std::endl;

    Eigen::MatrixXd master_jacobian;
    master_jacobian = robot_state.getJacobian(planning_group);

    Eigen::MatrixXd slave_jacobian;
    slave_jacobian = robot_state.getJacobian(slave_group);

    //begin*******************************joint_limit_penality********************************
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_joint_angles[i] - master_joint_pos_bounds.second[i], 2) * (2*master_joint_angles[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                            /(4 * std::pow(master_joint_pos_bounds.first[i] - master_joint_angles[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_joint_angles[i], 2));
        if((master_joint_angles[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] - master_joint_angles[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }

        slave_delta_h[i] = (std::pow(slave_joint_angles[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_joint_angles[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                           /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_joint_angles[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_joint_angles[i], 2));
        if((slave_joint_angles[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] - slave_joint_angles[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
    }

    Eigen::Matrix<double, 6, 7> master_L[1];
    Eigen::Matrix<double, 6, 7> slave_L[1];
    for(size_t dir_num = 0; dir_num < 1; dir_num++) {
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 7; j++) {
                //先确定关节限制的权重矩阵
                if (master_all_dir[dir_num][i] < 0) {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    }
                } else {
                    if (master_jacobian(i, j) > 0) {
                        master_L[dir_num](i, j) = master_p_postive[j];
                    } else {
                        master_L[dir_num](i, j) = master_p_negative[j];
                    }
                }

                if (slave_all_dir[dir_num][i] < 0) {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    }
                } else {
                    if (slave_jacobian(i, j) > 0) {
                        slave_L[dir_num](i, j) = slave_p_postive[j];
                    } else {
                        slave_L[dir_num](i, j) = slave_p_negative[j];
                    }
                }
            }
        }
    }
    //end*******************************joint_limit_penality********************************


    //begin**********************************ob penality************************************
    int collision_pair_count = 0;

    double master_alpha = 50;
    double master_beta = 1.5;
    double master_dis_thre = 0.1;
    double master_ob_dis = 0;
    double master_delta_P_d;
    Eigen::Vector3d master_ob_dir;
    double master_delta_P_theta[7];
    std::string master_link_name;
    Eigen::Vector3d master_nearest_point;

    Eigen::Vector3d master_robot_collision_point;
    Eigen::Vector3d master_object_collision_point;


    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;

    std::vector<Eigen::MatrixXd> master_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> master_o_negative_vec;
    collision_detection::DistanceResultsData master_tmp_dis_res_data;
    for(auto it = master_dis_map.begin(); it!=master_dis_map.end();it++){
        std::cout<<it->second[0].link_names[0]<<" "<< it->second[0].link_names[1]<<" "<<it->second[0].distance<<std::endl;

        if(it->second[0].distance < master_dis_thre){
            master_tmp_dis_res_data = it->second[0];
            master_ob_dis = master_tmp_dis_res_data.distance;
            master_ob_dir = master_tmp_dis_res_data.normal;
            master_link_name = master_tmp_dis_res_data.link_names[1];
            master_nearest_point = master_tmp_dis_res_data.nearest_points[1];
            std::cout<<"Computed dis   "<<(master_tmp_dis_res_data.nearest_points[1] - master_tmp_dis_res_data.nearest_points[0]).norm()<<std::endl;

            const robot_state::LinkModel* master_collision_link = robot_state.getLinkModel(master_link_name);
            Eigen::MatrixXd master_ob_jac;
            robot_state.getJacobian(planning_group, master_collision_link, master_nearest_point, master_ob_jac);

//            master_robot_collision_point = robot_state.getGlobalLinkTransform(master_tmp_dis_res_data.link_names[1]) * master_nearest_point;
//            Eigen::Affine3d aatmp;
//            aatmp = master_collision_link->getCollisionOriginTransforms()[0] * robot_state.getGlobalLinkTransform(master_tmp_dis_res_data.link_names[1]);
//            master_robot_collision_point = aatmp * master_nearest_point;
            master_robot_collision_point = robot_state.getGlobalLinkTransform(master_tmp_dis_res_data.link_names[1])  * master_nearest_point;
            moveit_msgs::CollisionObject collision_object_msg;
            planning_scene_ptr->getCollisionObjectMsg(collision_object_msg, master_tmp_dis_res_data.link_names[0]);
            Eigen::Vector3d tmp(collision_object_msg.primitive_poses[0].position.x, collision_object_msg.primitive_poses[0].position.y, collision_object_msg.primitive_poses[0].position.z);
            master_object_collision_point = master_tmp_dis_res_data.nearest_points[0] + tmp;
            master_ob_dir = master_robot_collision_point - master_object_collision_point;
            master_ob_dir.normalize();

            visualization_msgs::Marker robot_collision_point;
            robot_collision_point.header.frame_id = "base";
            robot_collision_point.header.stamp = ros::Time::now();
            robot_collision_point.type = visualization_msgs::Marker::SPHERE;
            robot_collision_point.action = visualization_msgs::Marker::ADD;
            robot_collision_point.id = collision_pair_count++;
            robot_collision_point.ns = "collision_point";
            robot_collision_point.pose.orientation.x = 0.0;
            robot_collision_point.pose.orientation.y = 0.0;
            robot_collision_point.pose.orientation.z = 0.0;
            robot_collision_point.pose.orientation.w = 1.0;
            robot_collision_point.pose.position.x = master_robot_collision_point(0);
            robot_collision_point.pose.position.y = master_robot_collision_point(1);
            robot_collision_point.pose.position.z = master_robot_collision_point(2);
            robot_collision_point.color.a = 1;
            robot_collision_point.color.b = 1.0;
            robot_collision_point.scale.x = 0.03;
            robot_collision_point.scale.y = 0.03;
            robot_collision_point.scale.z = 0.03;
//            geometry_msgs::Point robot_pt;
//            robot_pt.x = master_robot_collision_point(0);
//            robot_pt.y = master_robot_collision_point(1);
//            robot_pt.z = master_robot_collision_point(2);
//            robot_collision_point.points.push_back(robot_pt);
            robot_collision_points.markers.push_back(robot_collision_point);

            visualization_msgs::Marker object_collision_point;
            object_collision_point.header.frame_id = "base";
            object_collision_point.header.stamp = ros::Time::now();
            object_collision_point.type = visualization_msgs::Marker::SPHERE;
            object_collision_point.action = visualization_msgs::Marker::ADD;
            object_collision_point.id = collision_pair_count++;
            object_collision_point.ns = "collision_point";
            object_collision_point.pose.orientation.x = 0.0;
            object_collision_point.pose.orientation.y = 0.0;
            object_collision_point.pose.orientation.z = 0.0;
            object_collision_point.pose.orientation.w = 1.0;
            object_collision_point.pose.position.x = master_object_collision_point(0);
            object_collision_point.pose.position.y = master_object_collision_point(1);
            object_collision_point.pose.position.z = master_object_collision_point(2);
            object_collision_point.color.a = 1.0;
            object_collision_point.color.g = 1.0;
            object_collision_point.scale.x = 0.03;
            object_collision_point.scale.y = 0.03;
            object_collision_point.scale.z = 0.03;
//            geometry_msgs::Point object_pt;
//            object_pt.x = master_object_collision_point(0);
//            object_pt.y = master_object_collision_point(1);
//            object_pt.z = master_object_collision_point(2);
//            object_collision_point.points.push_back(object_pt);
            object_colllision_points.markers.push_back(object_collision_point);
            
            master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta) * (master_beta / master_ob_dis + master_alpha);
            master_extended_v.head(3) = master_ob_dir * master_ob_dis;
            master_extended_v.tail(3) = Eigen::Vector3d::Zero();
            master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;

            Eigen::Matrix<double ,3,7> master_o_positive;
            Eigen::Matrix<double ,3,7> master_o_negative;

            for(size_t i=0; i<7; i++){
                master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(master_ob_dir[i] < 0){
                        master_o_negative(i,j) = 1;
                        master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                    }
                    else{
                        master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                        master_o_positive(i,j) = 1;
                    }
                }
            }
            master_o_positive_vec.push_back(master_o_positive);
            master_o_negative_vec.push_back(master_o_negative);
            master_ob_jac_vec.push_back(master_ob_jac);
//            std::cout<<"one dir  "<<one_dir.transpose()<<std::endl;
//            std::cout<<"master_ob_dir  "<<master_ob_dir.transpose()<<std::endl;
//            std::cout<<master_link_name<<std::endl;
//            std::cout<<"distance  "<<master_ob_dis<<std::endl;
            std::cout<<"master_o_positive\n"<<master_o_positive<<std::endl;
            std::cout<<"master_o_negative\n"<<master_o_negative<<std::endl;
//            std::cout<<"\n\n"<<std::endl;
        }
    }
    if(master_o_positive_vec.empty()){
        master_o_positive_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        master_o_negative_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        master_ob_jac_vec.push_back(master_jacobian);
    }


    double slave_alpha = 50;
    double slave_beta = 1.5;
    double slave_dis_thre = 0.1;
    double slave_ob_dis = 0;
    double slave_delta_P_d;
    Eigen::Vector3d slave_ob_dir;
    double slave_delta_P_theta[7];
    std::string slave_link_name;
    std::string slave_object_name;
    Eigen::Vector3d slave_nearest_point;

    Eigen::Vector3d slave_robot_collision_point;
    Eigen::Vector3d slave_object_collision_point;



    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;

    std::vector<Eigen::MatrixXd> slave_ob_jac_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_positive_vec;
    std::vector<Eigen::Matrix<double ,3,7>> slave_o_negative_vec;
    collision_detection::DistanceResultsData slave_tmp_dis_res_data;
    for(auto it = slave_dis_map.begin(); it!=slave_dis_map.end();it++){
        std::cout<<it->second[0].link_names[0]<<" "<< it->second[0].link_names[1]<<" "<<it->second[0].distance<<std::endl;
        if(it->second[0].distance < slave_dis_thre){
            std::cout<<"object_point: "<<it->second[0].nearest_points[0].transpose()<<std::endl;

            slave_tmp_dis_res_data = it->second[0];
            slave_ob_dis = slave_tmp_dis_res_data.distance;
//            slave_ob_dir = slave_tmp_dis_res_data.normal;
            slave_link_name = slave_tmp_dis_res_data.link_names[1];
            slave_nearest_point = slave_tmp_dis_res_data.nearest_points[1];
            std::cout<<"Computed dis   "<<(slave_tmp_dis_res_data.nearest_points[1] - slave_tmp_dis_res_data.nearest_points[0]).norm()<<std::endl;
            const robot_state::LinkModel* slave_collision_link = robot_state.getLinkModel(slave_link_name);
            Eigen::MatrixXd slave_ob_jac;
            robot_state.getJacobian(slave_group, slave_collision_link, slave_nearest_point, slave_ob_jac);

            slave_robot_collision_point = robot_state.getGlobalLinkTransform(slave_tmp_dis_res_data.link_names[1]) * slave_nearest_point;
//            Eigen::Affine3d aatmp;
//            aatmp = slave_collision_link->getCollisionOriginTransforms()[0] * robot_state.getGlobalLinkTransform(slave_tmp_dis_res_data.link_names[1]);
//            slave_robot_collision_point = aatmp * slave_nearest_point;
            moveit_msgs::CollisionObject collision_object_msg;
            planning_scene_ptr->getCollisionObjectMsg(collision_object_msg, slave_tmp_dis_res_data.link_names[0]);
            Eigen::Vector3d tmp(collision_object_msg.primitive_poses[0].position.x, collision_object_msg.primitive_poses[0].position.y, collision_object_msg.primitive_poses[0].position.z);
            slave_object_collision_point = slave_tmp_dis_res_data.nearest_points[0] + tmp;
            slave_ob_dir = slave_robot_collision_point - slave_object_collision_point;
            slave_ob_dir.normalize();


            visualization_msgs::Marker robot_collision_point;
            robot_collision_point.header.frame_id = "base";
            robot_collision_point.header.stamp = ros::Time::now();
            robot_collision_point.type = visualization_msgs::Marker::SPHERE;
            robot_collision_point.action = visualization_msgs::Marker::ADD;
            robot_collision_point.id = collision_pair_count++;
            robot_collision_point.ns = "collision_point";
            robot_collision_point.pose.orientation.x = 0.0;
            robot_collision_point.pose.orientation.y = 0.0;
            robot_collision_point.pose.orientation.z = 0.0;
            robot_collision_point.pose.orientation.w = 1.0;
            robot_collision_point.pose.position.x = slave_robot_collision_point(0);
            robot_collision_point.pose.position.y = slave_robot_collision_point(1);
            robot_collision_point.pose.position.z = slave_robot_collision_point(2);
            robot_collision_point.color.a = 1.0;
            robot_collision_point.color.b = 1.0;
            robot_collision_point.scale.x = 0.03;
            robot_collision_point.scale.y = 0.03;
            robot_collision_point.scale.z = 0.03;
//            geometry_msgs::Point robot_pt;
//            robot_pt.x = slave_robot_collision_point(0);
//            robot_pt.y = slave_robot_collision_point(1);
//            robot_pt.z = slave_robot_collision_point(2);
//            robot_collision_point.points.push_back(robot_pt);
            robot_collision_points.markers.push_back(robot_collision_point);

            visualization_msgs::Marker object_collision_point;
            object_collision_point.header.frame_id = "base";
            object_collision_point.header.stamp = ros::Time::now();
            object_collision_point.type = visualization_msgs::Marker::SPHERE;
            object_collision_point.action = visualization_msgs::Marker::ADD;
            object_collision_point.id = collision_pair_count++;
            object_collision_point.ns = "collision_point";
            object_collision_point.pose.orientation.x = 0.0;
            object_collision_point.pose.orientation.y = 0.0;
            object_collision_point.pose.orientation.z = 0.0;
            object_collision_point.pose.orientation.w = 1.0;
            object_collision_point.pose.position.x = slave_object_collision_point(0);
            object_collision_point.pose.position.y = slave_object_collision_point(1);
            object_collision_point.pose.position.z = slave_object_collision_point(2);
            object_collision_point.color.a = 1.0;
            object_collision_point.color.g = 1.0;
            object_collision_point.scale.x = 0.03;
            object_collision_point.scale.y = 0.03;
            object_collision_point.scale.z = 0.03;
//            geometry_msgs::Point object_pt;
//            object_pt.x = slave_object_collision_point(0);
//            object_pt.y = slave_object_collision_point(1);
//            object_pt.z = slave_object_collision_point(2);
//            object_collision_point.points.push_back(object_pt);
            object_colllision_points.markers.push_back(object_collision_point);
            

            slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta) * (slave_beta / slave_ob_dis + slave_alpha);
            slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
            slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
            slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;

            Eigen::Matrix<double ,3,7> slave_o_positive;
            Eigen::Matrix<double ,3,7> slave_o_negative;

            for(size_t i=0; i<7; i++){
                slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;
            }
            for(size_t i=0; i<3; i++){
                for(size_t j=0; j<7; j++){
                    // dir 的方向是障碍物指向机械臂， 和机械臂运动的是相反的
                    if(slave_ob_dir[i] < 0){
                        slave_o_negative(i,j) = 1;
                        slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                    }
                    else{
                        slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                        slave_o_positive(i,j) = 1;
                    }
                }
            }
            slave_o_positive_vec.push_back(slave_o_positive);
            slave_o_negative_vec.push_back(slave_o_negative);
            slave_ob_jac_vec.push_back(slave_ob_jac);
            std::cout<<"slave_tmp_dis_res_data.nearest_points[0]\n"<<slave_tmp_dis_res_data.nearest_points[0].transpose()<<std::endl;
            std::cout<<"slave_tmp_dis_res_data.nearest_points[1]\n"<<slave_tmp_dis_res_data.nearest_points[1].transpose()<<std::endl;
            tmp = slave_tmp_dis_res_data.nearest_points[1] -slave_tmp_dis_res_data.nearest_points[0];
            tmp.normalize();
            std::cout<<"right_end_pos\n"<<robot_state.getGlobalLinkTransform("right_gripper").translation().transpose()<<std::endl;
            std::cout<<"slave_ob_dir\n"<<slave_ob_dir.transpose()<<std::endl;
            std::cout<<"slave_o_positive\n"<<slave_o_positive<<std::endl;
            std::cout<<"slave_o_negative\n"<<slave_o_negative<<std::endl;
        }
    }
    //防止没有障碍物的情况
    if(slave_o_positive_vec.empty()){
        slave_o_positive_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        slave_o_negative_vec.push_back(Eigen::Matrix<double ,3,7>::Ones());
        slave_ob_jac_vec.push_back(slave_jacobian);
    }

    Eigen::Matrix<double, 6, 7> master_O[1];
    Eigen::Matrix<double, 6, 7> slave_O[1];
    for(size_t dir_num = 0; dir_num < 1; dir_num++) {
        Eigen::Matrix<double, 6, 1> master_end_cause_collision_point;


        master_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < master_ob_jac_vec.size(); link_num++){
            master_end_cause_collision_point = master_ob_jac_vec[link_num] * (master_jacobian.transpose() * master_all_dir[dir_num]);

            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                        if (master_end_cause_collision_point[i] < 0) {
                            master_O[dir_num](i, j) *= master_o_negative_vec[link_num](i, j);
                        }
                        else {
                            master_O[dir_num](i, j) *= master_o_positive_vec[link_num](i, j);
                        }
                    }
                    else {
                        master_O[dir_num](i, j) *= 1;
                    }
                }
            }
        }

        Eigen::Matrix<double, 6, 1> slave_end_cause_collision_point;

        slave_O[dir_num] = Eigen::Matrix<double, 6, 7>::Ones();
        for(size_t link_num = 0; link_num < slave_ob_jac_vec.size(); link_num++){
            slave_end_cause_collision_point = slave_ob_jac_vec[link_num] * (slave_jacobian.transpose() * slave_all_dir[dir_num]);
            for (size_t i = 0; i < 6; i++) {
                for (size_t j = 0; j < 7; j++) {
                    //确定障碍物限制的权重矩阵
                    if (i < 3) {
                            if (slave_end_cause_collision_point[i] < 0) {
                                slave_O[dir_num](i, j) *= slave_o_negative_vec[link_num](i, j);
                            }
                            else {
                                slave_O[dir_num](i, j) *= slave_o_positive_vec[link_num](i, j);
                            }
                    }
                    else {
                        slave_O[dir_num](i, j) *= 1;
                    }
                }
            }
        }
    }
    //end**********************************ob penality************************************


    //begin**********************************final manipulability ************************************
    double master_dir_manipulability = 0;
    double slave_dir_manipulability = 0;

    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;

    for(size_t dir_num = 0; dir_num < 1; dir_num++){
        std::cout<<"dir_num: "<<dir_num<<std::endl;
        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
        std::cout<<"slave_L\n"<<slave_L[dir_num]<<std::endl;
        std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
        std::cout<<"slave_O\n"<<slave_O[dir_num]<<std::endl;
        std::cout<<"master_jacobian\n"<<master_jacobian<<std::endl;
        std::cout<<"slave_jacobian\n"<<slave_jacobian<<std::endl;
        std::cout<<"master_manipulability_jac\n"<<master_manipulability_jac<<std::endl;
        std::cout<<"slave_manipulability_jac\n"<<slave_manipulability_jac<<std::endl;

        //既考虑关节限制也考虑碰撞
        master_manipulability_jac = (master_L[dir_num].array() * master_O[dir_num].array() * master_jacobian.array()).matrix();
        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
        slave_manipulability_jac = (slave_L[dir_num].array() * slave_O[dir_num].array() * slave_jacobian.array()).matrix();
        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

//        std::cout<<"master normal jacobian\n"<<master_jacobian<<std::endl;
//        std::cout<<"master_L\n"<<master_L[dir_num]<<std::endl;
//        std::cout<<"master_O\n"<<master_O[dir_num]<<std::endl;
//        std::cout<<"master peanal jacobian\n"<<master_manipulability_jac<<std::endl;
//        std::cout<<"jjt\n"<<master_manipulability_jjt<<std::endl;

        //只考虑碰撞
//        master_manipulability_jac = (master_O[dir_num].array() * master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_O[dir_num].array() * slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();

        //啥都不考虑
//        master_manipulability_jac = (master_jacobian.array()).matrix();
//        master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
//        slave_manipulability_jac = (slave_jacobian.array()).matrix();
//        slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();


//        Eigen::EigenSolver<Eigen::Matrix<double,6,6>> master_jjt_solver(master_manipulability_jjt);
//        Eigen::Matrix<double,6,6> master_jjt_eigen_vale_matrix;
//        Eigen::Matrix<double,6,6> master_jjt_eigen_vector_matrix;
//        master_jjt_eigen_vale_matrix = master_jjt_solver.pseudoEigenvalueMatrix();
//        master_jjt_eigen_vector_matrix = master_jjt_solver.pseudoEigenvectors();


        Eigen::JacobiSVD<Eigen::MatrixXd> master_svd_holder(master_manipulability_jac, Eigen::ComputeFullU);
        Eigen::MatrixXd master_singular;
        Eigen::MatrixXd master_U;
        master_singular = master_svd_holder.singularValues();
        master_U = master_svd_holder.matrixU();
        for(size_t i=0; i<6; i++){
            master_dir_manipulability += std::abs(master_U.col(i).dot(master_all_dir[dir_num]) * master_singular(i));
            std::cout<<"master"<<std::endl;
            std::cout<<i<<" "<<std::abs(master_U.col(i).dot(master_all_dir[dir_num]) * master_singular(i))<<std::endl;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> slave_svd_holder(slave_manipulability_jac, Eigen::ComputeFullU);
        Eigen::MatrixXd slave_singular;
        Eigen::MatrixXd slave_U;
        slave_singular = slave_svd_holder.singularValues();
        slave_U = slave_svd_holder.matrixU();
        for(size_t i=0; i<6; i++){
            slave_dir_manipulability += std::abs(slave_U.col(i).dot(slave_all_dir[dir_num]) * slave_singular(i));
            std::cout<<"slave"<<std::endl;
            std::cout<<i<<" "<<std::abs(slave_U.col(i).dot(slave_all_dir[dir_num]) * slave_singular(i))<<std::endl;
        }

        std::cout<<"master_singular\n"<<master_singular<<std::endl;
        std::cout<<"slave_singular\n"<<slave_singular<<std::endl;
//        Eigen::EigenSolver<Eigen::Matrix<double,6,6>> slave_jjt_solver(slave_manipulability_jjt);
//        Eigen::Matrix<double,6,6> slave_jjt_eigen_vale_matrix;
//        Eigen::Matrix<double,6,6> slave_jjt_eigen_vector_matrix;
//        slave_jjt_eigen_vale_matrix = slave_jjt_solver.pseudoEigenvalueMatrix();
//        slave_jjt_eigen_vector_matrix = slave_jjt_solver.pseudoEigenvectors();
//        for(size_t i=0; i<6; i++){
//            slave_dir_manipulability += std::abs(slave_jjt_eigen_vector_matrix.col(i).dot(slave_all_dir[dir_num]) * slave_jjt_eigen_vale_matrix(i,i));
//        }
//        master_dir_manipulability += std::sqrt(master_manipulability_jjt.determinant());
//        slave_dir_manipulability += std::sqrt(slave_manipulability_jjt.determinant());
    }
    //end**********************************final manipulability ************************************
    out_master_dir_manipulability = master_dir_manipulability;
    out_slave_dir_manipulability = slave_dir_manipulability;
    if(master_dir_manipulability < 0 or slave_dir_manipulability < 0){
        std::cout<<"????"<<std::endl;
    }
}

double DualCBiRRT::manipulability_compute_one_sample(bool if_a_tree, size_t reached_index, size_t nearest_index){
    Eigen::MatrixXd master_jacobian;
    Eigen::MatrixXd master_ob_jac;
    Eigen::Vector3d master_ob_dir;
    double master_ob_dis;
    Eigen::Matrix<double, 7 ,1> master_matrix;
    Eigen::Affine3d master_pose;
    Eigen::Affine3d master_target_pose;

    Eigen::MatrixXd slave_jacobian;
    Eigen::MatrixXd slave_ob_jac;
    Eigen::Vector3d slave_ob_dir;
    double slave_ob_dis;
    Eigen::Matrix<double, 7 ,1> slave_matrix;
    Eigen::Affine3d slave_pose;
    Eigen::Affine3d slave_target_pose;


    if(if_a_tree){
        robot_state::RobotState considered_state = _b_rrt_tree_state[nearest_index].first;
        robot_state::RobotState target_state = _a_rrt_tree_state[reached_index].first;

        master_jacobian = considered_state.getJacobian(planning_group);
        slave_jacobian = considered_state.getJacobian(slave_group);

        master_ob_jac = b_tree_considered_master_ob_jac[nearest_index];
        master_ob_dir = b_tree_considered_master_ob_dir[nearest_index];
        master_ob_dis = b_tree_considered_master_ob_dis[nearest_index];
        slave_ob_jac = b_tree_considered_slave_ob_jac[nearest_index];
        slave_ob_dir = b_tree_considered_slave_ob_dir[nearest_index];
        slave_ob_dis = b_tree_considered_slave_ob_dis[nearest_index];

        master_matrix = _b_rrt_tree_matrix[nearest_index].head(7);
        slave_matrix = _b_rrt_tree_matrix[nearest_index].tail(7);

        master_pose = considered_state.getGlobalLinkTransform("left_gripper");
        master_target_pose = target_state.getGlobalLinkTransform("left_gripper");
        slave_pose = considered_state.getGlobalLinkTransform("right_gripper");
        slave_target_pose = target_state.getGlobalLinkTransform("right_gripper");
    }
    else{
        robot_state::RobotState considered_state = _a_rrt_tree_state[nearest_index].first;
        robot_state::RobotState target_state = _b_rrt_tree_state[reached_index].first;

        master_jacobian = considered_state.getJacobian(planning_group);
        slave_jacobian = considered_state.getJacobian(slave_group);

        master_ob_jac = a_tree_considered_master_ob_jac[nearest_index];
        master_ob_dir = a_tree_considered_master_ob_dir[nearest_index];
        master_ob_dis = a_tree_considered_master_ob_dis[nearest_index];
        slave_ob_jac = a_tree_considered_slave_ob_jac[nearest_index];
        slave_ob_dir = a_tree_considered_slave_ob_dir[nearest_index];
        slave_ob_dis = a_tree_considered_slave_ob_dis[nearest_index];

        master_matrix = _a_rrt_tree_matrix[nearest_index].head(7);
        slave_matrix = _a_rrt_tree_matrix[nearest_index].tail(7);

        master_pose = considered_state.getGlobalLinkTransform("left_gripper");
        master_target_pose = target_state.getGlobalLinkTransform("left_gripper");
        slave_pose = considered_state.getGlobalLinkTransform("right_gripper");
        slave_target_pose = target_state.getGlobalLinkTransform("right_gripper");
    }


    //joint_limit_penality
    double master_p_postive[7];
    double master_p_negative[7];
    double master_delta_h[7];
    //ob penality
    double master_alpha = 50;
    double master_beta = 0.5;
    double master_delta_P_d = -std::exp(-master_alpha * master_ob_dis) * std::pow(master_ob_dis, -master_beta * (master_beta / master_ob_dis + master_alpha));
    double master_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> master_extended_v;
    Eigen::Matrix<double ,7,1> master_ob_theta_dot;
    master_extended_v.head(3) = master_ob_dir * master_ob_dis;
    master_extended_v.tail(3) = Eigen::Vector3d::Zero();
    master_ob_theta_dot = master_ob_jac.transpose() * master_extended_v;
    Eigen::Matrix<double ,6,7> master_o_positive;
    Eigen::Matrix<double ,6,7> master_o_negative;

    //joint_limit_penality
    double slave_p_postive[7];
    double slave_p_negative[7];
    double slave_delta_h[7];
    //ob penality
    double slave_alpha = 50;
    double slave_beta = 0.5;
    double slave_delta_P_d = -std::exp(-slave_alpha * slave_ob_dis) * std::pow(slave_ob_dis, -slave_beta) * (slave_beta / slave_ob_dis + slave_alpha);
    double slave_delta_P_theta[7];
    Eigen::Matrix<double ,6,1> slave_extended_v;
    Eigen::Matrix<double ,7,1> slave_ob_theta_dot;
    slave_extended_v.head(3) = slave_ob_dir * slave_ob_dis;
    slave_extended_v.tail(3) = Eigen::Vector3d::Zero();
    slave_ob_theta_dot = slave_ob_jac.transpose() * slave_extended_v;
    Eigen::Matrix<double ,6,7> slave_o_positive;
    Eigen::Matrix<double ,6,7> slave_o_negative;

    for(size_t i=0; i<7; i++){
        master_delta_h[i] = (std::pow(master_matrix[i] - master_joint_pos_bounds.second[i], 2) * (2*master_matrix[i] - master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i]))
                            /(4 * std::pow(master_joint_pos_bounds.first[i] - master_matrix[i], 2) * std::pow(master_joint_pos_bounds.second[i] - master_matrix[i], 2));
        if((master_matrix[i] - master_joint_pos_bounds.second[i]) > (master_joint_pos_bounds.first[i] > master_matrix[i])){
            master_p_negative[i] = 1;
            master_p_postive[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
        }
        else{
            master_p_negative[i] = 1/std::sqrt(1 + std::abs(master_delta_h[i]));
            master_p_postive[i] = 1;
        }
        master_delta_P_theta[i] = master_delta_P_d * master_ob_theta_dot[i] / master_ob_dis;

        slave_delta_h[i] = (std::pow(slave_matrix[i] - slave_joint_pos_bounds.second[i], 2) * (2*slave_matrix[i] - slave_joint_pos_bounds.first[i] - slave_joint_pos_bounds.second[i]))
                           /(4 * std::pow(slave_joint_pos_bounds.first[i] - slave_matrix[i], 2) * std::pow(slave_joint_pos_bounds.second[i] - slave_matrix[i], 2));
        if((slave_matrix[i] - slave_joint_pos_bounds.second[i]) > (slave_joint_pos_bounds.first[i] > slave_matrix[i])){
            slave_p_negative[i] = 1;
            slave_p_postive[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
        }
        else{
            slave_p_negative[i] = 1/std::sqrt(1 + std::abs(slave_delta_h[i]));
            slave_p_postive[i] = 1;
        }
        slave_delta_P_theta[i] = slave_delta_P_d * slave_ob_theta_dot[i] / slave_ob_dis;

    }

    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<7; j++){
            if(master_ob_dir[i] > 0){
                master_o_negative(i,j) = 1;
                master_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
            }
            else{
                master_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(master_delta_P_theta[j]));
                master_o_positive(i,j) = 1;
            }
            if(slave_ob_dir[i] > 0){
                slave_o_negative(i,j) = 1;
                slave_o_positive(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
            }
            else{
                slave_o_negative(i,j) = 1 / std::sqrt(1 + std::abs(slave_delta_P_theta[j]));
                slave_o_positive(i,j) = 1;
            }
        }
    }
    for(size_t i=0; i<3; i++) {
        for (size_t j = 0; j < 7; j++) {
            master_o_negative(i+3,j) = 1;
            master_o_positive(i+3,j) = 1;
            slave_o_negative(i+3,j) = 1;
            slave_o_positive(i+3,j) = 1;
        }
    }

    Eigen::Matrix<double, 6, 7> master_L;
    Eigen::Matrix<double, 6, 7> master_O;
    Eigen::Matrix<double, 6, 7> slave_L;
    Eigen::Matrix<double, 6, 7> slave_O;



    //begin********************计算扩展的方向**********************************
    Eigen::Affine3d master_dir_pose;
    Eigen::Vector3d master_dir_pos;
    Eigen::AngleAxisd master_dir_axis_angle;
    Eigen::Vector3d master_dir_axis;
    Eigen::Matrix<double,6,1> master_dir;

    master_dir_pose = master_target_pose * master_pose.inverse();

    master_dir_pos = master_dir_pose.translation();
    master_dir_axis_angle = master_dir_pose.rotation();
    master_dir_axis = master_dir_axis_angle.axis();
    master_dir.head(3) = master_dir_pos;
    master_dir.tail(3) = master_dir_axis;

    Eigen::Affine3d slave_dir_pose;
    Eigen::Vector3d slave_dir_pos;
    Eigen::AngleAxisd slave_dir_axis_angle;
    Eigen::Vector3d slave_dir_axis;
    Eigen::Matrix<double,6,1> slave_dir;

    slave_dir_pose = slave_target_pose * slave_pose.inverse();

    slave_dir_pos = slave_dir_pose.translation();
    slave_dir_axis_angle = slave_dir_pose.rotation();
    slave_dir_axis = slave_dir_axis_angle.axis();
    slave_dir.head(3) = slave_dir_pos;
    slave_dir.tail(3) = slave_dir_axis;
    //end********************计算扩展的方向**********************************


    for(size_t i=0; i<6; i++){
        for(size_t j=0; j<7; j++){
            if(master_dir[i] < 0){
                master_O(i,j) = master_o_negative(i,j);
                if(master_jacobian(i,j) > 0)
                {
                    master_L(i,j) = master_p_negative[j];
                }
                else{
                    master_L(i,j) = master_p_postive[j];
                }
            }
            else{
                master_O(i,j) = master_o_positive(i,j);
                if(master_jacobian(i,j) > 0)
                {
                    master_L(i,j) = master_p_postive[j];
                }
                else{
                    master_L(i,j) = master_p_negative[j];
                }
            }

            if(slave_dir[i] < 0){
                slave_O(i,j) = slave_o_negative(i,j);
                if(slave_jacobian(i,j) > 0)
                {
                    slave_L(i,j) = slave_p_negative[j];
                }
                else{
                    slave_L(i,j) = slave_p_postive[j];
                }
            }
            else{
                slave_O(i,j) = slave_o_positive(i,j);
                if(slave_jacobian(i,j) > 0)
                {
                    slave_L(i,j) = slave_p_postive[j];
                }
                else{
                    slave_L(i,j) = slave_p_negative[j];
                }
            }
        }
    }



    Eigen::Matrix<double, 6, 7> master_manipulability_jac;
    Eigen::Matrix<double, 6, 6> master_manipulability_jjt;
    master_manipulability_jac = (master_L.array() * master_O.array() * master_jacobian.array()).matrix();
    master_manipulability_jjt = master_manipulability_jac * master_manipulability_jac.transpose();
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> master_solver(master_manipulability_jjt);
    Eigen::Matrix<double,6,6> master_eigenvalue;
    Eigen::Matrix<double,6,6> master_eigenvector;
    master_eigenvalue = master_solver.pseudoEigenvalueMatrix();
    master_eigenvector = master_solver.pseudoEigenvectors();


    double master_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        master_dir_manipulability += std::pow(master_eigenvector.col(i).dot(master_dir) * master_eigenvalue(i,i), 2);
    }

    Eigen::Matrix<double, 6, 7> slave_manipulability_jac;
    Eigen::Matrix<double, 6, 6> slave_manipulability_jjt;
    slave_manipulability_jac = (slave_L.array() * slave_O.array() * slave_jacobian.array()).matrix();
    slave_manipulability_jjt = slave_manipulability_jac * slave_manipulability_jac.transpose();
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> slave_solver(slave_manipulability_jjt);
    Eigen::Matrix<double,6,6> slave_eigenvalue;
    Eigen::Matrix<double,6,6> slave_eigenvector;
    slave_eigenvalue = slave_solver.pseudoEigenvalueMatrix();
    slave_eigenvector = slave_solver.pseudoEigenvectors();


    double slave_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        slave_dir_manipulability += std::pow(slave_eigenvector.col(i).dot(slave_dir) * slave_eigenvalue(i,i), 2);
    }


    if(std::isnan(master_dir_manipulability) or std::isnan(slave_dir_manipulability)){
        Eigen::Matrix<double,7,1> slave_delta_P_theta_matrix;
        for(size_t i=0; i<7; i++){
            slave_delta_P_theta_matrix(i) = slave_delta_P_theta[i];
        }
        std::cout<<"slave_ob_theta_dot\n"<<slave_ob_theta_dot<<std::endl;
        std::cout<<"slave_delta_P_theta\n"<<slave_delta_P_theta_matrix.transpose()<<std::endl;
        std::cout<<"slave_o_positive\n"<<slave_o_positive<<std::endl;
        std::cout<<"slave_o_negative\n"<<slave_o_negative<<std::endl;
        std::cout<<"master_L\n"<<master_L<<std::endl;
        std::cout<<"master_O\n"<<master_O<<std::endl;
        std::cout<<"master_jacobian\n"<<master_jacobian<<std::endl;
        std::cout<<"slave_L\n"<<slave_L<<std::endl;
        std::cout<<"slave_O\n"<<slave_O<<std::endl;
        std::cout<<"slave_jacobian\n"<<slave_jacobian<<std::endl;
        std::cout<<""<<std::endl;
    }


    if(master_dir_manipulability < slave_dir_manipulability)
        return master_dir_manipulability;
    else
        return slave_dir_manipulability;
}

double DualCBiRRT::simple_dir_manipulability_compute(Eigen::Matrix<double, 7, 1> master_matrix, Eigen::Matrix<double, 7, 1> slave_matrix, bool if_tree_a){
    Eigen::MatrixXd master_jacobian;
    Eigen::MatrixXd master_jjt;
    std::vector<double> master_joint_angles(7);
    for(size_t i=0; i<7; i++){
        master_joint_angles[i] = master_matrix[i];
    }
    robot_state::RobotState master_state = planning_scene_ptr->getCurrentStateNonConst();
    master_state.setJointGroupPositions(planning_group, master_joint_angles);
    master_jacobian = master_state.getJacobian(planning_group);
    master_jjt = master_jacobian * master_jacobian.transpose();
    Eigen::Affine3d master_pose;
    master_pose = master_state.getGlobalLinkTransform("left_gripper");
    Eigen::Affine3d master_dir_pose;
    Eigen::Vector3d master_dir_pos;
    Eigen::AngleAxisd master_dir_axis_angle;
    Eigen::Vector3d master_dir_axis;
    Eigen::Matrix<double,6,1> master_dir;
    if(if_tree_a){
        master_dir_pose = _goal_master_pose * master_pose.inverse();
    }
    else{
        master_dir_pose = _start_master_pose * master_pose.inverse();
    }
    master_dir_pos = master_dir_pose.translation();
    master_dir_axis_angle = master_dir_pose.rotation();
    master_dir_axis = master_dir_axis_angle.axis();
    master_dir.head(3) = master_dir_pos;
    master_dir.tail(3) = master_dir_axis;
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> master_solver(master_jjt);
    Eigen::Matrix<double,6,6> master_eigenvalue;
    Eigen::Matrix<double,6,6> master_eigenvector;
    master_eigenvalue = master_solver.pseudoEigenvalueMatrix();
    master_eigenvector = master_solver.pseudoEigenvectors();
    double master_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        master_dir_manipulability += std::pow(master_eigenvector.col(i).dot(master_dir) * master_eigenvalue(i,i), 2);
    }

    Eigen::MatrixXd slave_jacobian;
    Eigen::MatrixXd slave_jjt;
    std::vector<double> slave_joint_angles(7);
    for(size_t i=0; i<7; i++){
        slave_joint_angles[i] = slave_matrix[i];
    }
    robot_state::RobotState slave_state = planning_scene_ptr->getCurrentStateNonConst();
    slave_state.setJointGroupPositions(slave_group, slave_joint_angles);
    slave_jacobian = slave_state.getJacobian(slave_group);
    slave_jjt = slave_jacobian * slave_jacobian.transpose();
    Eigen::Affine3d slave_pose;
    slave_pose = slave_state.getGlobalLinkTransform("right_gripper");
    Eigen::Affine3d slave_dir_pose;
    Eigen::Vector3d slave_dir_pos;
    Eigen::AngleAxisd slave_dir_axis_angle;
    Eigen::Vector3d slave_dir_axis;
    Eigen::Matrix<double,6,1> slave_dir;
    if(if_tree_a){
        slave_dir_pose = _goal_slave_pose * slave_pose.inverse();
    }
    else{
        slave_dir_pose = _start_slave_pose * slave_pose.inverse();
    }
    slave_dir_pos = slave_dir_pose.translation();
    slave_dir_axis_angle = slave_dir_pose.rotation();
    slave_dir_axis = slave_dir_axis_angle.axis();
    slave_dir.head(3) = slave_dir_pos;
    slave_dir.tail(3) = slave_dir_axis;
    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> slave_solver(slave_jjt);
    Eigen::Matrix<double,6,6> slave_eigenvalue;
    Eigen::Matrix<double,6,6> slave_eigenvector;
    slave_eigenvalue = slave_solver.pseudoEigenvalueMatrix();
    slave_eigenvector = slave_solver.pseudoEigenvectors();
    double slave_dir_manipulability = 0;
    for(size_t i=0; i<6; i++){
        slave_dir_manipulability += std::pow(slave_eigenvector.col(i).dot(slave_dir) * slave_eigenvalue(i,i), 2);
    }

    if(master_dir_manipulability < slave_dir_manipulability)
        return master_dir_manipulability;
    else
        return slave_dir_manipulability;
}

void DualCBiRRT::basic_manipulability_compute_once(std::vector<double> master_matrix, std::vector<double> slave_matrix, double & l_basic_manipu, double & r_basic_manipu) {
    Eigen::MatrixXd master_jacobian;
    Eigen::MatrixXd master_jjt;
    std::vector<double> master_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        master_joint_angles[i] = master_matrix[i];
    }
    robot_state::RobotState master_state = planning_scene_ptr->getCurrentStateNonConst();
    master_state.setJointGroupPositions(planning_group, master_joint_angles);
    master_state.update();
    master_jacobian = master_state.getJacobian(planning_group);
    master_jjt = master_jacobian * master_jacobian.transpose();

    Eigen::MatrixXd slave_jacobian;
    Eigen::MatrixXd slave_jjt;
    std::vector<double> slave_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        slave_joint_angles[i] = slave_matrix[i];
    }
    robot_state::RobotState slave_state = planning_scene_ptr->getCurrentStateNonConst();
    slave_state.setJointGroupPositions(planning_group, slave_joint_angles);
    slave_state.update();
    slave_jacobian = slave_state.getJacobian(planning_group);
    slave_jjt = slave_jacobian * slave_jacobian.transpose();

    l_basic_manipu = std::sqrt(master_jjt.determinant());
    r_basic_manipu = std::sqrt(slave_jjt.determinant());
}

void DualCBiRRT::dir_manipulability_compute_once(std::vector<double> start_master_matrix, std::vector<double> start_slave_matrix, std::vector<double> goal_master_matrix, std::vector<double> goal_slave_matrix, double & s_l_basic_manipu, double & s_r_basic_manipu, double & g_l_basic_manipu, double & g_r_basic_manipu) {
    //******************计算雅克比信息***********************************
    Eigen::MatrixXd s_master_jacobian;
    Eigen::MatrixXd s_master_jjt;
    std::vector<double> s_master_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        s_master_joint_angles[i] = start_master_matrix[i];
    }
    robot_state::RobotState s_master_state = planning_scene_ptr->getCurrentStateNonConst();
    s_master_state.setJointGroupPositions(planning_group, s_master_joint_angles);
    s_master_jacobian = s_master_state.getJacobian(planning_group);
    s_master_jjt = s_master_jacobian * s_master_jacobian.transpose();
    Eigen::Affine3d s_master_pose;
    s_master_pose = s_master_state.getGlobalLinkTransform("left_gripper");

    Eigen::MatrixXd g_master_jacobian;
    Eigen::MatrixXd g_master_jjt;
    std::vector<double> g_master_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        g_master_joint_angles[i] = goal_master_matrix[i];
    }
    robot_state::RobotState g_master_state = planning_scene_ptr->getCurrentStateNonConst();
    g_master_state.setJointGroupPositions(planning_group, g_master_joint_angles);
    g_master_jacobian = g_master_state.getJacobian(planning_group);
    g_master_jjt = g_master_jacobian * g_master_jacobian.transpose();
    Eigen::Affine3d g_master_pose;
    g_master_pose = g_master_state.getGlobalLinkTransform("left_gripper");

    Eigen::MatrixXd s_slave_jacobian;
    Eigen::MatrixXd s_slave_jjt;
    std::vector<double> s_slave_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        s_slave_joint_angles[i] = start_slave_matrix[i];
    }
    robot_state::RobotState s_slave_state = planning_scene_ptr->getCurrentStateNonConst();
    s_slave_state.setJointGroupPositions(planning_group, s_slave_joint_angles);
    s_slave_jacobian = s_slave_state.getJacobian(planning_group);
    s_slave_jjt = s_slave_jacobian * s_slave_jacobian.transpose();
    Eigen::Affine3d s_slave_pose;
    s_slave_pose = s_slave_state.getGlobalLinkTransform("left_gripper");


    Eigen::MatrixXd g_slave_jacobian;
    Eigen::MatrixXd g_slave_jjt;
    std::vector<double> g_slave_joint_angles(7);
    for (size_t i = 0; i < 7; i++) {
        g_slave_joint_angles[i] = goal_slave_matrix[i];
    }
    robot_state::RobotState g_slave_state = planning_scene_ptr->getCurrentStateNonConst();
    g_slave_state.setJointGroupPositions(planning_group, g_slave_joint_angles);
    g_slave_jacobian = g_slave_state.getJacobian(planning_group);
    g_slave_jjt = g_slave_jacobian * g_slave_jacobian.transpose();
    Eigen::Affine3d g_slave_pose;
    g_slave_pose = g_slave_state.getGlobalLinkTransform("left_gripper");

    //******************计算方向***********************************
    Eigen::Affine3d s_g_master_pose;
    s_g_master_pose = g_master_pose * s_master_pose.inverse();
    Eigen::Vector3d s_g_master_pos;
    Eigen::Vector3d s_g_master_axis;
    Eigen::AngleAxisd s_g_master_axis_g_angle;
    Eigen::Matrix<double,6,1> s_g_master_dir;
    s_g_master_pos = s_g_master_pose.translation();
    s_g_master_axis_g_angle = s_g_master_pose.rotation();
    s_g_master_axis = s_g_master_axis_g_angle.axis();
    s_g_master_dir.head(3) = s_g_master_pos;
    s_g_master_dir.tail(3) = s_g_master_axis;

    Eigen::Affine3d g_s_master_pose;
    g_s_master_pose = s_master_pose * g_master_pose.inverse();
    Eigen::Vector3d g_s_master_pos;
    Eigen::Vector3d g_s_master_axis;
    Eigen::AngleAxisd g_s_master_axig_s_angle;
    Eigen::Matrix<double,6,1> g_s_master_dir;
    g_s_master_pos = g_s_master_pose.translation();
    g_s_master_axig_s_angle = g_s_master_pose.rotation();
    g_s_master_axis = g_s_master_axig_s_angle.axis();
    g_s_master_dir.head(3) = g_s_master_pos;
    g_s_master_dir.tail(3) = g_s_master_axis;

    Eigen::Affine3d s_g_slave_pose;
    s_g_slave_pose = g_slave_pose * s_slave_pose.inverse();
    Eigen::Vector3d s_g_slave_pos;
    Eigen::Vector3d s_g_slave_axis;
    Eigen::AngleAxisd s_g_slave_axis_g_angle;
    Eigen::Matrix<double,6,1> s_g_slave_dir;
    s_g_slave_pos = s_g_slave_pose.translation();
    s_g_slave_axis_g_angle = s_g_slave_pose.rotation();
    s_g_slave_axis = s_g_slave_axis_g_angle.axis();
    s_g_slave_dir.head(3) = s_g_slave_pos;
    s_g_slave_dir.tail(3) = s_g_slave_axis;

    Eigen::Affine3d g_s_slave_pose;
    g_s_slave_pose = s_slave_pose * g_slave_pose.inverse();
    Eigen::Vector3d g_s_slave_pos;
    Eigen::Vector3d g_s_slave_axis;
    Eigen::AngleAxisd g_s_slave_axig_s_angle;
    Eigen::Matrix<double,6,1> g_s_slave_dir;
    g_s_slave_pos = g_s_slave_pose.translation();
    g_s_slave_axig_s_angle = g_s_slave_pose.rotation();
    g_s_slave_axis = g_s_slave_axig_s_angle.axis();
    g_s_slave_dir.head(3) = g_s_slave_pos;
    g_s_slave_dir.tail(3) = g_s_slave_axis;

    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> s_master_solver(s_master_jjt);
    Eigen::Matrix<double,6,6> s_master_eigen_vale_matrix;
    Eigen::Matrix<double,6,6> s_master_eigen_vector_matrix;
    s_master_eigen_vale_matrix = s_master_solver.pseudoEigenvalueMatrix();
    s_master_eigen_vector_matrix = s_master_solver.pseudoEigenvectors();
    s_l_basic_manipu = 0;
    for(size_t i=0; i<6; i++){
        s_l_basic_manipu += s_master_eigen_vector_matrix.col(i).dot(s_g_master_dir) * s_master_eigen_vale_matrix(i,i);
    }

    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> g_master_solver(g_master_jjt);
    Eigen::Matrix<double,6,6> g_master_eigen_vale_matrix;
    Eigen::Matrix<double,6,6> g_master_eigen_vector_matrix;
    g_master_eigen_vale_matrix = g_master_solver.pseudoEigenvalueMatrix();
    g_master_eigen_vector_matrix = g_master_solver.pseudoEigenvectors();
    g_l_basic_manipu = 0;
    for(size_t i=0; i<6; i++){
        g_l_basic_manipu += g_master_eigen_vector_matrix.col(i).dot(g_s_master_dir) * g_master_eigen_vale_matrix(i,i);
    }

    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> s_slave_solver(s_slave_jjt);
    Eigen::Matrix<double,6,6> s_slave_eigen_vale_matrix;
    Eigen::Matrix<double,6,6> s_slave_eigen_vector_matrix;
    s_slave_eigen_vale_matrix = s_slave_solver.pseudoEigenvalueMatrix();
    s_slave_eigen_vector_matrix = s_slave_solver.pseudoEigenvectors();
    s_r_basic_manipu = 0;
    for(size_t i=0; i<6; i++){
        s_r_basic_manipu += s_slave_eigen_vector_matrix.col(i).dot(s_g_slave_dir) * s_slave_eigen_vale_matrix(i,i);
    }

    Eigen::EigenSolver<Eigen::Matrix<double,6,6>> g_slave_solver(g_slave_jjt);
    Eigen::Matrix<double,6,6> g_slave_eigen_vale_matrix;
    Eigen::Matrix<double,6,6> g_slave_eigen_vector_matrix;
    g_slave_eigen_vale_matrix = g_slave_solver.pseudoEigenvalueMatrix();
    g_slave_eigen_vector_matrix = g_slave_solver.pseudoEigenvectors();
    g_r_basic_manipu = 0;
    for(size_t i=0; i<6; i++){
        g_r_basic_manipu += g_slave_eigen_vector_matrix.col(i).dot(g_s_slave_dir) * g_slave_eigen_vale_matrix(i,i);
    }
    
}

void DualCBiRRT::output_perdex() {
    int sample_counts = _performance_record.size();
    int extend_try = 0;
    int constraint_project_success=0;
    int ik_success = 0;
    int extend_success = 0;
    int constraint_project_compute_times = 0;
    int ik_compute_times = 0;
    int constraint_success_project_compute_times = 0;
    int ik_success_compute_times = 0;
    double constraint_project_success_rate = 0;
    double ik_success_rate = 0;
    double extend_success_rate=0;
    double average_success_constraint_project_compute_times = 0;
    double average_ik_compute_times = 0;
    double average_success_ik_compute_times = 0;

    double total_sample_time = 0;
    double total_extend_time = 0;
    double total_project_time = 0;
    double total_ik_time = 0;
    double average_sample_time = 0;
    double average_extend_time = 0;
    double average_extend_project_time = 0;
    double average_extend_ik_time = 0;
    double average_extend_one_project_time = 0;
    double average_extend_one_ik_time = 0;


    for (size_t i=0; i<sample_counts; i++){
        total_sample_time += _performance_record[i].spend_time;

        for(size_t j=0; j<_performance_record[i].tree_a.size(); j++){
            extend_try++;
            total_extend_time += _performance_record[i].tree_a[j].extend_total_spend_time;
            total_project_time += _performance_record[i].tree_a[j].project_total_spend_time;
            total_ik_time += _performance_record[i].tree_a[j].ik_total_spend_time;
            constraint_project_compute_times += _performance_record[i].tree_a[j].constraint_project_times;
            ik_compute_times += _performance_record[i].tree_a[j].ik_project_times;
            if(_performance_record[i].tree_a[j].project_success)
            {
                constraint_project_success++;
                constraint_success_project_compute_times += _performance_record[i].tree_a[j].constraint_project_times;
                if(_performance_record[i].tree_a[j].ik_success){
                    ik_success++;
                    ik_success_compute_times += _performance_record[i].tree_a[j].ik_project_times;
                    if(_performance_record[i].tree_a[j].extend_success){
                        extend_success++;
                    }
                }
            }
        }
        for(size_t j=0; j<_performance_record[i].tree_b.size(); j++){
            extend_try++;
            total_extend_time += _performance_record[i].tree_b[j].extend_total_spend_time;
            total_project_time += _performance_record[i].tree_b[j].project_total_spend_time;
            total_ik_time += _performance_record[i].tree_b[j].ik_total_spend_time;
            constraint_project_compute_times += _performance_record[i].tree_b[j].constraint_project_times;
            ik_compute_times += _performance_record[i].tree_b[j].ik_project_times;
            if(_performance_record[i].tree_b[j].project_success)
            {
                constraint_project_success++;
                constraint_success_project_compute_times += _performance_record[i].tree_b[j].constraint_project_times;
                if(_performance_record[i].tree_b[j].ik_success){
                    ik_success++;
                    ik_success_compute_times += _performance_record[i].tree_b[j].ik_project_times;
                    if(_performance_record[i].tree_b[j].extend_success){
                        extend_success++;
                    }
                }
            }
        }
    }

    constraint_project_success_rate = double(constraint_project_success)/double(extend_try);
    ik_success_rate = double(ik_success)/double(extend_try);
    extend_success_rate = double(extend_success)/double(extend_try);
    average_success_constraint_project_compute_times = (constraint_success_project_compute_times)/double(constraint_project_success);
    average_ik_compute_times = (ik_compute_times)/double(extend_try);
    average_success_ik_compute_times = (ik_success_compute_times)/double(ik_success);

    average_sample_time = total_sample_time / sample_counts;
    average_extend_time = total_extend_time / extend_try;
    average_extend_project_time = total_project_time / extend_try;
    average_extend_ik_time = total_ik_time / extend_try;
    average_extend_one_project_time = total_project_time / constraint_project_compute_times;
    average_extend_one_ik_time = total_ik_time / ik_compute_times;

    std::cout<<"\n\n=========================================="<<std::endl;
    std::cout<<"_seed:  "<<_seed<<"\n";
    std::cout<<"sample_counts:  "<<sample_counts<<"\n";
    std::cout<<"extend_try:  "<<extend_try<<"\n";
    std::cout<<"constraint_project_success:  "<<constraint_project_success<<"\n";
    std::cout<<"constraint_project_success_rate:  "<<constraint_project_success_rate<<"\n";
    std::cout<<"ik_success:  "<<ik_success<<"\n";
    std::cout<<"ik_success_rate:  "<<ik_success_rate<<"\n";
    std::cout<<"extend_success:  "<<extend_success<<"\n";
    std::cout<<"extend_success_rate:  "<<extend_success_rate<<"\n";
    std::cout<<"average_success_constraint_project_compute_times:  "<<average_success_constraint_project_compute_times<<"\n";
    std::cout<<"average_ik_compute_times:  "<<average_ik_compute_times<<"\n";
    std::cout<<"average_success_ik_compute_times:  "<<average_success_ik_compute_times<<"\n";
    std::cout<<"total_sample_time:  "<<total_sample_time<<"\n";
    std::cout<<"total_extend_time:  "<<total_extend_time<<"\n";
    std::cout<<"total_project_time:  "<<total_project_time<<"\n";
    std::cout<<"total_ik_time:  "<<total_ik_time<<"\n";
    std::cout<<"average_sample_time:  "<<average_sample_time<<"\n";
    std::cout<<"average_extend_time:  "<<average_extend_time<<"\n";
    std::cout<<"average_extend_project_time:  "<<average_extend_project_time<<"\n";
    std::cout<<"average_extend_ik_time:  "<<average_extend_ik_time<<"\n";
    std::cout<<"average_extend_one_project_time:  "<<average_extend_one_project_time<<"\n";
    std::cout<<"average_extend_one_ik_time:  "<<average_extend_one_ik_time<<"\n";
    std::cout<<"==========================================\n\n"<<std::endl;

}

void DualCBiRRT::output_perdex_multi(std::ofstream & outFILE){
    int sample_counts = _performance_record.size();
    int extend_try = 0;
    int constraint_project_success=0;
    int ik_success = 0;
    int extend_success = 0;
    int constraint_project_compute_times = 0;
    int ik_compute_times = 0;
    int constraint_success_project_compute_times = 0;
    int ik_success_compute_times = 0;
    double constraint_project_success_rate = 0;
    double ik_success_rate = 0;
    double extend_success_rate=0;
    double average_success_constraint_project_compute_times = 0;
    double average_ik_compute_times = 0;
    double average_success_ik_compute_times = 0;

    double total_sample_time = 0;
    double total_extend_time = 0;
    double total_project_time = 0;
    double total_ik_time = 0;
    double average_sample_time = 0;
    double average_extend_time = 0;
    double average_extend_project_time = 0;
    double average_extend_ik_time = 0;
    double average_extend_one_project_time = 0;
    double average_extend_one_ik_time = 0;


    for (size_t i=0; i<sample_counts; i++){
        total_sample_time += _performance_record[i].spend_time;

        for(size_t j=0; j<_performance_record[i].tree_a.size(); j++){
            extend_try++;
            total_extend_time += _performance_record[i].tree_a[j].extend_total_spend_time;
            total_project_time += _performance_record[i].tree_a[j].project_total_spend_time;
            total_ik_time += _performance_record[i].tree_a[j].ik_total_spend_time;
            constraint_project_compute_times += _performance_record[i].tree_a[j].constraint_project_times;
            ik_compute_times += _performance_record[i].tree_a[j].ik_project_times;
            if(_performance_record[i].tree_a[j].project_success)
            {
                constraint_project_success++;
                constraint_success_project_compute_times += _performance_record[i].tree_a[j].constraint_project_times;
                if(_performance_record[i].tree_a[j].ik_success){
                    ik_success++;
                    ik_success_compute_times += _performance_record[i].tree_a[j].ik_project_times;
                    if(_performance_record[i].tree_a[j].extend_success){
                        extend_success++;
                    }
                }
            }
        }
        for(size_t j=0; j<_performance_record[i].tree_b.size(); j++){
            extend_try++;
            total_extend_time += _performance_record[i].tree_b[j].extend_total_spend_time;
            total_project_time += _performance_record[i].tree_b[j].project_total_spend_time;
            total_ik_time += _performance_record[i].tree_b[j].ik_total_spend_time;
            constraint_project_compute_times += _performance_record[i].tree_b[j].constraint_project_times;
            ik_compute_times += _performance_record[i].tree_b[j].ik_project_times;
            if(_performance_record[i].tree_b[j].project_success)
            {
                constraint_project_success++;
                constraint_success_project_compute_times += _performance_record[i].tree_b[j].constraint_project_times;
                if(_performance_record[i].tree_b[j].ik_success){
                    ik_success++;
                    ik_success_compute_times += _performance_record[i].tree_b[j].ik_project_times;
                    if(_performance_record[i].tree_b[j].extend_success){
                        extend_success++;
                    }
                }
            }
        }
    }

    constraint_project_success_rate = double(constraint_project_success)/double(extend_try);
    ik_success_rate = double(ik_success)/double(extend_try);
    extend_success_rate = double(extend_success)/double(extend_try);
    average_success_constraint_project_compute_times = (constraint_success_project_compute_times)/double(constraint_project_success);
    average_ik_compute_times = (ik_compute_times)/double(extend_try);
    average_success_ik_compute_times = (ik_success_compute_times)/double(ik_success);

    average_sample_time = total_sample_time / sample_counts;
    average_extend_time = total_extend_time / extend_try;
    average_extend_project_time = total_project_time / extend_try;
    average_extend_ik_time = total_ik_time / extend_try;
    average_extend_one_project_time = total_project_time / constraint_project_compute_times;
    average_extend_one_ik_time = total_ik_time / ik_compute_times;


    outFILE
    <<_seed<<","
    <<sample_counts<<","
    <<extend_try<<","
    <<constraint_project_success<<","
    <<constraint_project_success_rate<<","
    <<ik_success<<","
    <<ik_success_rate<<","
    <<extend_success<<","
    <<extend_success_rate<<","
    <<average_success_constraint_project_compute_times<<","
    <<average_ik_compute_times<<","
    <<average_success_ik_compute_times<<","
    <<total_sample_time<<","
    <<total_extend_time<<","
    <<total_project_time<<","
    <<total_ik_time<<","
    <<average_sample_time<<","
    <<average_extend_time<<","
    <<average_extend_project_time<<","
    <<average_extend_ik_time<<","
    <<average_extend_one_project_time<<","
    <<average_extend_one_ik_time<<","
    <<std::endl;
}

void DualCBiRRT::output_extend_success_rate(std::ofstream & outFILE){
    for(size_t i=0; i<_extend_success_rate.size(); i++){
        outFILE<<_extend_success_rate[i]<<std::endl;
    }
}

void DualCBiRRT::output_simple_manipulability(std::ofstream & outFILE){
    for(size_t i=0; i<_simple_dir_manipulability_vec.size(); i++){
        outFILE<<_simple_dir_manipulability_vec[i]<<std::endl;
    }
}

void DualCBiRRT::output_rate_simple_manipulability(std::ofstream & outFILE){
    for(size_t i=0; i<_rate_simple_manipulability_vec.size(); i++){
        outFILE<<_rate_simple_manipulability_vec[i]<<std::endl;
    }
}

void DualCBiRRT::output_complex_manipulability(std::ofstream & outFILE){
    for(size_t i=0; i<_complex_dir_manipulability_vec.size(); i++){
        outFILE<<_complex_dir_manipulability_vec[i]<<std::endl;
    }
}

void DualCBiRRT::output_rate_complex_manipulability(std::ofstream & outFILE){
    for(size_t i=0; i<_rate_complex_manipulability_vec.size(); i++){
        outFILE<<_rate_complex_manipulability_vec[i]<<std::endl;
    }
}

void DualCBiRRT::output_new_manipulability(std::ofstream & outFILE){
    for(size_t i=0; i<_new_manipulability_master_vec_explore.size(); i++){
        outFILE<<_new_manipulability_master_vec_explore[i]<<" "<<_new_manipulability_slave_vec_explore[i]<<" "<<
        _new_manipulability_master_vec_exploit[i]<<" "<<_new_manipulability_slave_vec_exploit[i]<<" "<<_new_manipulability_mini_vec_explore[i]<<" "<<_new_manipulability_mini_vec_exploit[i]
        <<" "<<two_tree_minimum_dis[i]<<" "<<master_one_dir_manipulaibility_vec[i]<<" "<<slave_one_dir_manipulaibility_vec[i]<<std::endl;
    }
}

void DualCBiRRT::output_reached_index(std::ofstream & outFILE){
    for(size_t i=0; i<_a_tree_reached_index_vec.size(); i++){
        outFILE<<_a_tree_reached_index_vec[i]<<" "<<_a_tree_nearest_index_vec[i]<<" "<<_a_tree_minimum_extend_manipulability[i]<<"    "<<_b_tree_reached_index_vec[i]<<" "<<_b_tree_nearest_index_vec[i]<<" "<<_b_tree_minimum_extend_manipulability[i]<<std::endl;
    }
}

const std::vector<std::pair<robot_state::RobotState, size_t>> & DualCBiRRT::get_tree_state_vector(bool if_a_tree){
    if(if_a_tree){
        return _a_rrt_tree_state;
    }
    else{
        return _b_rrt_tree_state;
    }

}