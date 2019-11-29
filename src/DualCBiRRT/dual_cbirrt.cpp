
#include <baxter_moveit_application/DualCBiRRT/dual_cbirrt.h>
#include <limits.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <Eigen/SVD>
#include <math.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_common.h>
#include <random_numbers/random_numbers.h>

#define PI 3.1415926
#define SEED 2
#define ALPHA 0.05

DualCBiRRT::DualCBiRRT(double probability, int seed, double alpha):_random_distribution(probability), _seed(seed), _alpha(alpha){
    _random_engine.seed(time(0));
    _step_size = 0.05;
    _task_step_size = 0.02;
    _constrain_delta = 0.01; //弧度值
    _pos_constrain_delta = 0.02;

    _pitch_min =  0 - 1.05;
    _pitch_max =  0 + 1.05 ;
//    _yaw_min = 2.94792  - 0.00025;
//    _yaw_max = 2.94792  + 0.00025;
    _yaw_min = 0 - 1.05;
    _yaw_max = 0 + 1.05;

    _roll_min = 1.57 - 1.05;
    _roll_max = 1.57 + 1.05;

    _draw_count = 0;

    _error_coefficient = 0.5;


    _max_planning_times = 5000;
    _left_right_distance = 0.2;

    _master_link_names.push_back("left_upper_shoulder");
    _master_link_names.push_back("left_lower_shoulder");
    _master_link_names.push_back("left_upper_elbow");
    _master_link_names.push_back("left_lower_elbow");
    _master_link_names.push_back("left_upper_forearm");
    _master_link_names.push_back("left_lower_forearm");
    _master_link_names.push_back("left_gripper_base");

    _slave_link_names.push_back("right_upper_shoulder");
    _slave_link_names.push_back("right_lower_shoulder");
    _slave_link_names.push_back("right_upper_elbow");
    _slave_link_names.push_back("right_lower_elbow");
    _slave_link_names.push_back("right_upper_forearm");
    _slave_link_names.push_back("right_lower_forearm");
    _slave_link_names.push_back("right_gripper_base");

}

DualCBiRRT::~DualCBiRRT(){}

void DualCBiRRT::sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group, random_numbers::RandomNumberGenerator &rng) {
    if(_random_distribution(_random_engine)){
        //伯努利分布
        random_state.setToRandomPositions(planning_group, rng);
    }
    else{
        random_state = goal_state;
    }
//    random_state.setToRandomPositions(planning_group, rng);
    std::vector<double> random_state_value;
    random_state.copyJointGroupPositions(planning_group, random_state_value);
    for(size_t i=0; i<random_state_value.size(); i++) {
        random_state_value_matrix[i] = random_state_value[i];
    }
}

void DualCBiRRT::sample_task_state(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group, random_numbers::RandomNumberGenerator &rng, Eigen::Vector3d & random_state_vector) {
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
    random_state_vector = random_state.getGlobalLinkTransform("left_gripper").translation();
}

size_t DualCBiRRT::near_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, bool if_tree_a) {
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    if(if_tree_a){
        for(size_t i=0; i<_a_rrt_tree_matrix.size(); i++){
            Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _a_rrt_tree_matrix[i].head(7);
            double distance = dis_matrix.norm();
            if(distance < minum_dis){
                minum_dis = distance;
                minum_index = i;
            }
        }
        nearest_node_matrix = _a_rrt_tree_matrix[minum_index].head(7);
    }
    else{
        for(size_t i=0; i<_b_rrt_tree_matrix.size(); i++){
            Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _b_rrt_tree_matrix[i].head(7);
            double distance = dis_matrix.norm();
            if(distance < minum_dis){
                minum_dis = distance;
                minum_index = i;
            }
        }
        nearest_node_matrix = _b_rrt_tree_matrix[minum_index].head(7);
    }

    return minum_index;
}

size_t DualCBiRRT::near_tree_task_space(Eigen::Vector3d & random_task_state, Eigen::Vector3d & nearest_node_task_state, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, bool if_tree_a)
{
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    if(if_tree_a){
        for(size_t i=0; i<_a_rrt_tree_task_space_state.size(); i++){
            Eigen::Vector3d dis_matrix = random_task_state - _a_rrt_tree_task_space_state[i];
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
            Eigen::Vector3d dis_matrix = random_task_state - _b_rrt_tree_task_space_state[i];
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

void DualCBiRRT::constraint_extend_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a){
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;
    collision_req.group_name = "both_arms";

    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;
    Eigen::MatrixXd end_jacobian_down_3;
    Eigen::MatrixXd end_jacobian_down_3_pinv;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 3, 1> task_delta_vector;
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

    int performance_index_extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if ((qs_matrix - random_state_value_matrix).norm() < 0.05){
            reached_state_matrix = qs_matrix;
            break;
        }
        else if((random_state_value_matrix-qs_matrix).norm() > (random_state_value_matrix-qs_old_matrix).norm()){
            reached_state_matrix = qs_old_matrix;
            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_state = qs_state;
            direction_matrix = random_state_value_matrix - qs_matrix;
            double norm = direction_matrix.norm();
            direction_matrix = direction_matrix / norm;
            if(_step_size <= norm){
                qs_matrix = qs_matrix + direction_matrix * _step_size;
            }
            else{
                qs_matrix = qs_matrix + direction_matrix * norm;
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

            //**********************************************向约束空间投影*****************************************
            double yaw_error = 0;
            double pitch_error = 0;
            double roll_error=0;
            double total_error = 0;
            bool project_success = true;
            Eigen::Vector3d end_rot_eulerAngle;
            Eigen::Vector3d required_eulerAngle;
            Eigen::Matrix3d required_rot_matrix;
            Eigen::Matrix3d error_rot_matrix;
            Eigen::AngleAxisd error_axis_angle;
            Eigen::AngleAxis<double >::Vector3 error_axis;
            double error_angle;

            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);



                roll_error = 1.57 - end_rot_eulerAngle[2];

//                if(end_rot_eulerAngle[1] < _pitch_min){
//                    pitch_error = _pitch_min - end_rot_eulerAngle[1];
//                }
//                else if(end_rot_eulerAngle[1] > _pitch_max){
//                    pitch_error = _pitch_max - end_rot_eulerAngle[1];
//                }
//                else{
//                    pitch_error = 0;
//                }

                pitch_error = 0 - end_rot_eulerAngle[1];

                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector = error_axis * error_angle;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

//                    std::cout<<"required_eulerAngle:  "<<required_eulerAngle.transpose()<<std::endl;
//                    std::cout<<"error_axis_angle_angle:  "<<error_axis_angle.angle()<<std::endl;
//                    std::cout<<"error_axis_angle_axis:  "<<error_axis_angle.axis().transpose()<<std::endl;

                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_down_3 = end_jacobian.bottomRows(3);
                    end_jacobian_down_3_pinv = end_jacobian_down_3.transpose() * ((end_jacobian_down_3 * end_jacobian_down_3.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_down_3_pinv  * task_delta_vector;
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。
//                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }
            }
            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;
                ik_start_time =  ros::Time::now();
                bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                ik_end_time = ros::Time::now();
                perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                if(ik_result){
                    perdex_one_extend.ik_success = 1;
                    for(size_t i=0; i<7; i++){
                        slave_angles_vector[i] = computed_slave_angles_matrix[i];
                    }
                    qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                    qs_state.update();
                    if(planning_scene_ptr->isStateValid(qs_state, "both_arms")){
                        perdex_one_extend.no_collide=1;
                        //将节点加入树中
                        std::cout<<"adding a state"<<std::endl;
                        perdex_one_extend.extend_success=1;
                        matrix_tree_element.head(7) = qs_matrix;
                        matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                        if(if_tree_a){
                            _a_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _a_rrt_tree_state.push_back(tmp);
                            qs_old_index = _a_rrt_tree_state.size() - 1;
                        }
                        else{
                            _b_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _b_rrt_tree_state.push_back(tmp);
                            qs_old_index = _b_rrt_tree_state.size() - 1;
                        }

                    }
                    else{
                        reached_state_matrix = qs_old_matrix;

                        perdex_one_extend.no_collide=0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }
                        break;
                    }
                }
                else{
                    reached_state_matrix = qs_old_matrix;

                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }                    break;
                }
            }
            else{
                reached_state_matrix = qs_old_matrix;

                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }                break;
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

    }
}

void DualCBiRRT::constraint_extend_tree_try_adjust(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a){
    std::set<const moveit::core::LinkModel *> master_link_model_set;
    std::set<const moveit::core::LinkModel *> slave_link_model_set;
    for(size_t i=0; i<_slave_link_names.size(); i++){
        master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        slave_link_model_set.insert(planning_group->getLinkModel(_slave_link_names[i]));
    }
    std::vector<double> master_link_distance_values;
    std::vector<Eigen::Vector3d> master_link_normals;
    std::vector<Eigen::Vector3d> master_link_minimum_dis_points;
    std::vector<Eigen::MatrixXd> master_link_minimum_dis_point_jac_matrices;





    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;
    collision_req.group_name = "both_arms";

    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;
    Eigen::MatrixXd end_jacobian_down_3;
    Eigen::MatrixXd end_jacobian_down_3_pinv;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 3, 1> task_delta_vector;
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

    int performance_index_extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if ((qs_matrix - random_state_value_matrix).norm() < 0.05){
            reached_state_matrix = qs_matrix;
            break;
        }
        else if((random_state_value_matrix-qs_matrix).norm() > (random_state_value_matrix-qs_old_matrix).norm()){
            reached_state_matrix = qs_old_matrix;
            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_state = qs_state;
            direction_matrix = random_state_value_matrix - qs_matrix;
            double norm = direction_matrix.norm();
            direction_matrix = direction_matrix / norm;
            if(_step_size <= norm){
                qs_matrix = qs_matrix + direction_matrix * _step_size;
            }
            else{
                qs_matrix = qs_matrix + direction_matrix * norm;
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

            //**********************************************向约束空间投影*****************************************
            double yaw_error = 0;
            double pitch_error = 0;
            double roll_error=0;
            double total_error = 0;
            bool project_success = true;
            Eigen::Vector3d end_rot_eulerAngle;
            Eigen::Vector3d required_eulerAngle;
            Eigen::Matrix3d required_rot_matrix;
            Eigen::Matrix3d error_rot_matrix;
            Eigen::AngleAxisd error_axis_angle;
            Eigen::AngleAxis<double >::Vector3 error_axis;
            double error_angle;

            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);



                roll_error = 1.57 - end_rot_eulerAngle[2];

//                if(end_rot_eulerAngle[1] < _pitch_min){
//                    pitch_error = _pitch_min - end_rot_eulerAngle[1];
//                }
//                else if(end_rot_eulerAngle[1] > _pitch_max){
//                    pitch_error = _pitch_max - end_rot_eulerAngle[1];
//                }
//                else{
//                    pitch_error = 0;
//                }

                pitch_error = 0 - end_rot_eulerAngle[1];

                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector = error_axis * error_angle;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

//                    std::cout<<"required_eulerAngle:  "<<required_eulerAngle.transpose()<<std::endl;
//                    std::cout<<"error_axis_angle_angle:  "<<error_axis_angle.angle()<<std::endl;
//                    std::cout<<"error_axis_angle_axis:  "<<error_axis_angle.axis().transpose()<<std::endl;

                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_down_3 = end_jacobian.bottomRows(3);
                    end_jacobian_down_3_pinv = end_jacobian_down_3.transpose() * ((end_jacobian_down_3 * end_jacobian_down_3.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_down_3_pinv  * task_delta_vector;
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。
//                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }
            }
            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;
                ik_start_time =  ros::Time::now();
                bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                ik_end_time = ros::Time::now();
                perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                if(ik_result){
                    perdex_one_extend.ik_success = 1;
                    for(size_t i=0; i<7; i++){
                        slave_angles_vector[i] = computed_slave_angles_matrix[i];
                    }
                    qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                    qs_state.update();

                    collision_detection::DistanceRequest collision_req_master;
                    collision_req_master.enable_nearest_points = true;
                    collision_req_master.enable_signed_distance = true;
                    collision_req_master.active_components_only = &master_link_model_set;
                    collision_req_master.group_name = "left_arm";
                    collision_req_master.type = collision_detection::DistanceRequestType::LIMITED
                            ;
                    collision_detection::DistanceResult collision_res_master;
                    world_FCL.distanceRobot(collision_req_master, collision_res_master, *robot, qs_state);
                    collision_detection::DistanceMap distances = collision_res_master.distances;
                    for(auto it=distances.begin(); it!=distances.end();it++){
                        std::cout<< it->first.first <<" and "<<it->first.second<<"   "<<it->second[0].link_names[0]<<" and "<<it->second[0].link_names[1]<<std::endl;
                        if(it->second[0].distance < 0.1){
                            master_link_distance_values.push_back(it->second[0].distance);
                            master_link_minimum_dis_points.push_back(it->second[0].nearest_points[1]);
                            master_link_normals.push_back(it->second[0].normal);
                        }
                    }




                    collision_detection::DistanceRequest collision_req_slave;
                    collision_req_slave.enable_nearest_points = true;
                    collision_req_slave.enable_signed_distance = true;
                    collision_req_slave.active_components_only = &slave_link_model_set;
                    collision_req_slave.group_name = "right_arm";
                    collision_req_slave.type = collision_detection::DistanceRequestType::SINGLE;
                    collision_detection::DistanceResult collision_res_slave;
                    world_FCL.distanceRobot(collision_req_slave, collision_res_slave, *robot, qs_state);
                    
                    if((!collision_res_master.collision) && (!collision_res_slave.collision)){
                        perdex_one_extend.no_collide=1;
                        //将节点加入树中
                        std::cout<<"adding a state"<<std::endl;
                        perdex_one_extend.extend_success=1;
                        matrix_tree_element.head(7) = qs_matrix;
                        matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                        if(if_tree_a){
                            _a_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _a_rrt_tree_state.push_back(tmp);
                            qs_old_index = _a_rrt_tree_state.size() - 1;
                        }
                        else{
                            _b_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _b_rrt_tree_state.push_back(tmp);
                            qs_old_index = _b_rrt_tree_state.size() - 1;
                        }

                    }
                    else{
                        collision_detection::DistanceResultsData master_min_dis = collision_res_master.minimum_distance;
                        collision_detection::DistanceResultsData slave_min_dis = collision_res_slave.minimum_distance;
                        std::cout<<"master_min_dis "<<master_min_dis.link_names[0]<<" "<<master_min_dis.link_names[1]<<" "<<master_min_dis.distance<<std::endl;
                        std::cout<<"slave_min_dis "<<slave_min_dis.link_names[0]<<" "<<slave_min_dis.link_names[1]<<" "<<slave_min_dis.distance<<std::endl;
                        std::cout<<qs_state.getGlobalLinkTransform("left_gripper").translation().transpose()<<std::endl;

                        double master_collide_dis_value = master_min_dis.distance;
                        if(master_collide_dis_value < 0){
                            Eigen::Vector3d collide_pos = master_min_dis.nearest_points[1];
                            Eigen::Vector3d collide_normal = master_min_dis.normal;
                            const std::string collide_link_name  = master_min_dis.link_names[1];
                            const robot_state::LinkModel* collision_link = qs_state.getLinkModel(collide_link_name);
                            Eigen::MatrixXd collision_point_jac;
                            qs_state.getJacobian(planning_group, collision_link, collide_pos, collision_point_jac);
                            Eigen::MatrixXd master_jac = qs_state.getJacobian(planning_group);
                            Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(master_jac, Eigen::ComputeFullV);
                            Eigen::MatrixXd V = svd_holder.matrixV();
                            std::cout<<"V\n  "<<V<<std::endl;
                            Eigen::MatrixXd self_motion_vec = V.rightCols(1);

                            //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0
                            
                            Eigen::Matrix<double ,6, 1> tmp;
                            Eigen::Vector3d self_motion_at_collision_point_velocity;
                            tmp = (collision_point_jac * self_motion_vec);
                            self_motion_at_collision_point_velocity = tmp.head(3);
                            double self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(collide_normal);
                            std::cout<<"self_motion_at_collision_point_velocity_magnititude  "<<self_motion_at_collision_point_velocity_magnititude<<std::endl;
                            if(self_motion_at_collision_point_velocity_magnititude > 0){
                                std::cout<<"ratio  "<<-1.1 * master_collide_dis_value / self_motion_at_collision_point_velocity_magnititude<<std::endl;
                                qs_matrix +=  self_motion_vec;
                            }
                            for(size_t i=0; i<7; i++){
                                qs_vector[i] = qs_matrix[i];
                            }
                            qs_state.setJointGroupPositions(planning_group, qs_vector);
                            qs_state.update();
                            if(qs_state.satisfiesBounds(0.05)){
                                std::cout<<"new state satisfiesBounds"<<std::endl;
                                std::cout<<qs_state.getGlobalLinkTransform("left_gripper").translation().transpose()<<std::endl;
                                world_FCL.distanceRobot(collision_req_master, collision_res_master, *robot, qs_state);
                                if(!collision_res_master.collision){
                                    std::cout<<"master fix success!"<<std::endl;
                                }
                                else{
                                    std::cout<<"master fix fail!"<<std::endl;
                                }
                            }
                            else{
                                std::cout<<"new state doesn't satisfiesBounds"<<std::endl;
                            }
                        }
                        double slave_collide_dis_value = slave_min_dis.distance;
                        if(slave_collide_dis_value < 0){
                            Eigen::Vector3d collide_pos = slave_min_dis.nearest_points[1];
                            Eigen::Vector3d collide_normal = slave_min_dis.normal;
                            const std::string collide_link_name  = slave_min_dis.link_names[1];
                            const robot_state::LinkModel* collision_link = qs_state.getLinkModel(collide_link_name);
                            Eigen::MatrixXd collision_point_jac;
                            qs_state.getJacobian(slave_group, collision_link, collide_pos, collision_point_jac);
                            Eigen::MatrixXd slave_jac = qs_state.getJacobian(slave_group);
                            Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_jac, Eigen::ComputeFullV);
                            Eigen::MatrixXd V = svd_holder.matrixV();
                            std::cout<<"V\n  "<<V<<std::endl;
                            Eigen::MatrixXd self_motion_vec = V.rightCols(1);

                            //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0

                            Eigen::Matrix<double ,6, 1> tmp;
                            Eigen::Vector3d self_motion_at_collision_point_velocity;
                            tmp = (collision_point_jac * self_motion_vec);
                            self_motion_at_collision_point_velocity = tmp.head(3);
                            double self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(collide_normal);
                            std::cout<<"self_motion_at_collision_point_velocity_magnititude  "<<self_motion_at_collision_point_velocity_magnititude<<std::endl;
                            if(self_motion_at_collision_point_velocity_magnititude > 0){
                                std::cout<<"ratio  "<<-1.1 * slave_collide_dis_value / self_motion_at_collision_point_velocity_magnititude<<std::endl;
                                qs_matrix +=  self_motion_vec;
                            }
                            for(size_t i=0; i<7; i++){
                                qs_vector[i] = qs_matrix[i];
                            }
                            qs_state.setJointGroupPositions(slave_group, qs_vector);
                            qs_state.update();
                            if(qs_state.satisfiesBounds(0.05)){
                                std::cout<<"new state satisfiesBounds"<<std::endl;
                                std::cout<<qs_state.getGlobalLinkTransform("left_gripper").translation().transpose()<<std::endl;
                                world_FCL.distanceRobot(collision_req_slave, collision_res_slave, *robot, qs_state);
                                if(!collision_res_slave.collision){
                                    std::cout<<"slave fix success!"<<std::endl;
                                }
                                else{
                                    std::cout<<"slave fix fail!"<<std::endl;
                                }
                            }
                            else{
                                std::cout<<"new state doesn't satisfiesBounds"<<std::endl;
                            }
                        }

                        reached_state_matrix = qs_old_matrix;
                        perdex_one_extend.no_collide=0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }
                        break;
                    }
                }
                else{
                    reached_state_matrix = qs_old_matrix;

                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }                    break;
                }
            }
            else{
                reached_state_matrix = qs_old_matrix;

                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }                break;
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
    }
}

void DualCBiRRT::constraint_extend_tree_dense_collide_new(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a){
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;
    collision_req.group_name = "both_arms";

    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;
    Eigen::MatrixXd end_jacobian_null_space;
    Eigen::MatrixXd end_jacobian_down_3;
    Eigen::MatrixXd end_jacobian_down_3_pinv;
    Eigen::MatrixXd end_jacobian_down_3_null_space;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 3, 1> task_delta_vector;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    Eigen::Matrix<double, 7, 1> null_space_joint_delta_vector;

    //计算 slave 相关变量
    Eigen::Matrix<double, 7, 1> current_slave_angles_matrix;
    if(if_tree_a){
        current_slave_angles_matrix = _a_rrt_tree_matrix[nearst_node_index].tail(7);
    }
    else{
        current_slave_angles_matrix = _b_rrt_tree_matrix[nearst_node_index].tail(7);
    }
    std::cout<<"current_slave_angles_matrix\n"<< current_slave_angles_matrix.transpose()<<std::endl;

    Eigen::Matrix<double, 7, 1> computed_slave_angles_matrix;
    std::vector<double> slave_angles_vector(7);

    Eigen::Matrix<double, 14, 1> matrix_tree_element;

    int performance_index_extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if ((qs_matrix - random_state_value_matrix).norm() < 0.05){
            reached_state_matrix = qs_matrix;
            break;
        }
        else if((random_state_value_matrix-qs_matrix).norm() > (random_state_value_matrix-qs_old_matrix).norm()){
            reached_state_matrix = qs_old_matrix;
            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_state = qs_state;
            direction_matrix = random_state_value_matrix - qs_matrix;
            double norm = direction_matrix.norm();
            direction_matrix = direction_matrix / norm;
            if(_step_size <= norm){
                qs_matrix = qs_matrix + direction_matrix * _step_size;
            }
            else{
                qs_matrix = qs_matrix + direction_matrix * norm;
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
            //*************************************************************************************************************************

            //**********************************************向约束空间投影***************************************************
            double yaw_error = 0;
            double pitch_error = 0;
            double roll_error=0;
            double total_error = 0;
            bool project_success = false;
            Eigen::Vector3d end_rot_eulerAngle;
            Eigen::Vector3d required_eulerAngle;
            Eigen::Matrix3d required_rot_matrix;
            Eigen::Matrix3d error_rot_matrix;
            Eigen::AngleAxisd error_axis_angle;
            Eigen::AngleAxis<double >::Vector3 error_axis;
            double error_angle;

            //************************************计算距离障碍物最近距离信息**************************************
            const std::set<const robot_model::LinkModel*> planning_link_model = planning_group->getUpdatedLinkModelsSet();
            collision_detection::DistanceResultsData min_distance;
            double minimum_dis_value;
            Eigen::Vector3d away_from_collision_normal;
            Eigen::Vector3d robot_minimum_dis_point;
            std::string link_name;
            Eigen::MatrixXd minimum_collision_dis_jac;
            Eigen::MatrixXd minimum_collision_dis_jac_pinv;
            Eigen::MatrixXd minimum_collision_dis_jac_up_3;
            Eigen::MatrixXd minimum_collision_dis_jac_up_3_pinv;
            Eigen::Matrix<double, 3, 1> avoid_obstacle_vel;
            //****************************************************************************************************


            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);


                roll_error = 1.57 - end_rot_eulerAngle[2];

                pitch_error = 0 - end_rot_eulerAngle[1];

                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }
                std::cout<<"rot error "<< roll_error<<" "<<pitch_error<<" "<<yaw_error<<std::endl;

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{

                    //碰撞信息计算
                    collision_detection::DistanceRequest dis_req;
                    collision_detection::DistanceResult dis_res;
                    dis_req.group_name = "left_arm";
                    dis_req.active_components_only = &planning_link_model;
                    dis_req.enable_nearest_points = true;
                    dis_req.type = collision_detection::DistanceRequestType::SINGLE;
                    world_FCL.distanceRobot(dis_req, dis_res, *robot, qs_state);
                    min_distance = dis_res.minimum_distance;
                    minimum_dis_value = min_distance.distance;
                    away_from_collision_normal = min_distance.normal;
                    robot_minimum_dis_point = min_distance.nearest_points[1];
                    link_name = min_distance.link_names[1];

                    if(!(link_name =="")){
                        const robot_state::LinkModel* collision_link = qs_state.getLinkModel(link_name);
                        if(qs_state.getJacobian(planning_group, collision_link, robot_minimum_dis_point, minimum_collision_dis_jac)){
                        }
                        else{
                            exit(1);
                        }
                        minimum_collision_dis_jac_up_3 = minimum_collision_dis_jac.topRows(3);
                        minimum_collision_dis_jac_up_3_pinv = minimum_collision_dis_jac_up_3.transpose()*((minimum_collision_dis_jac_up_3 * (minimum_collision_dis_jac_up_3.transpose()) +  0.01 * Eigen::Matrix<double,3,3>::Identity() ).inverse());
                        double alpha;
                        alpha = 2.0/(1+std::exp((2*minimum_dis_value/0.1-1)*4));
                        avoid_obstacle_vel = alpha * away_from_collision_normal;
//                        avoid_obstacle_vel /= 0.01;
                    }
                    else{
                        minimum_collision_dis_jac_up_3 = Eigen::Matrix<double,3,7>::Zero();
                        minimum_collision_dis_jac_up_3_pinv = Eigen::Matrix<double,7,3>::Zero();
                        avoid_obstacle_vel = Eigen::Matrix<double,3,1>::Zero();
                    }


                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();

                    task_delta_vector = error_axis * error_angle;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

//                    std::cout<<"required_eulerAngle:  "<<required_eulerAngle.transpose()<<std::endl;
//                    std::cout<<"error_axis_angle_angle:  "<<error_axis_angle.angle()<<std::endl;
//                    std::cout<<"error_axis_angle_axis:  "<<error_axis_angle.axis().transpose()<<std::endl;


                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_down_3 = end_jacobian.bottomRows(3);
                    end_jacobian_down_3_pinv =  end_jacobian_down_3.transpose() * ((end_jacobian_down_3 * end_jacobian_down_3.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_down_3_pinv  * task_delta_vector;
                    Eigen::Matrix<double, 3, 1> closest_point_vel;
                    closest_point_vel = minimum_collision_dis_jac_up_3 * joint_delta_vector;
                    if(closest_point_vel.dot(away_from_collision_normal) > 0){
                        ;
                    }
                    else{
                        end_jacobian_down_3_null_space = Eigen::Matrix<double, 7, 7>::Identity() - end_jacobian_down_3_pinv * end_jacobian_down_3;
                        null_space_joint_delta_vector = end_jacobian_down_3_null_space * (minimum_collision_dis_jac_up_3_pinv * avoid_obstacle_vel);
                        joint_delta_vector += null_space_joint_delta_vector;
                    }
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;
                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。
                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
//                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }
            }
            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;
                ik_start_time =  ros::Time::now();
                bool ik_result = solve_IK_problem_dense_collide_new(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                ik_end_time = ros::Time::now();
                perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                if(ik_result){
                    perdex_one_extend.ik_success = 1;
                    for(size_t i=0; i<7; i++){
                        slave_angles_vector[i] = computed_slave_angles_matrix[i];
                    }
                    qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                    qs_state.update();
                    if(planning_scene_ptr->isStateValid(qs_state, "both_arms")){
                        perdex_one_extend.no_collide=1;
                        //将节点加入树中
                        std::cout<<"adding a state"<<std::endl;
                        perdex_one_extend.extend_success=1;
                        matrix_tree_element.head(7) = qs_matrix;
                        matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                        if(if_tree_a){
                            _a_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _a_rrt_tree_state.push_back(tmp);
                            qs_old_index = _a_rrt_tree_state.size() - 1;
                        }
                        else{
                            _b_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _b_rrt_tree_state.push_back(tmp);
                            qs_old_index = _b_rrt_tree_state.size() - 1;
                        }

                    }
                    else{
                        reached_state_matrix = qs_old_matrix;
                        std::cout<<"state collide"<<std::endl;
                        perdex_one_extend.no_collide=0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }
                        break;
                    }
                }
                else{
                    reached_state_matrix = qs_old_matrix;

                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }
                    break;
                }
            }
            else{
                reached_state_matrix = qs_old_matrix;

                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }                break;
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

    }
}

void DualCBiRRT::constraint_extend_tree_adjust_extend_dir(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a, bool if_sample){
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collision_res;
    collision_req.group_name = "both_arms";

    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;
    Eigen::MatrixXd end_jacobian_down_3;
    Eigen::MatrixXd end_jacobian_down_3_pinv;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 3, 1> task_delta_vector;
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

    int performance_index_extend_num = 0;
    while (true){
        std::cout<<"extending"<<std::endl;
        if ((qs_matrix - random_state_value_matrix).norm() < 0.05){
            reached_state_matrix = qs_matrix;
            break;
        }
        else if((random_state_value_matrix-qs_matrix).norm() > (random_state_value_matrix-qs_old_matrix).norm()){
            reached_state_matrix = qs_old_matrix;
            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_state = qs_state;
            direction_matrix = random_state_value_matrix - qs_matrix;
            double norm = direction_matrix.norm();
            direction_matrix = direction_matrix / norm;
            if(_step_size <= norm){
                if(!if_sample){
                    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> dir_adjust_pair;
                    if(if_tree_a){
                        dir_adjust_pair = _a_rrt_tree_adjust_weight_and_joints_master[nearst_node_index];
                    }
                    else{
                        dir_adjust_pair = _b_rrt_tree_adjust_weight_and_joints_master[nearst_node_index];
                    }
                    for(size_t i=0; i<dir_adjust_pair.first.size(); i++){
                        direction_matrix += dir_adjust_pair.first[i] * dir_adjust_pair.second[i];
                    }
                    direction_matrix.normalize();
                }
                qs_matrix = qs_matrix + direction_matrix * _step_size;
            }
            else{
                qs_matrix = qs_matrix + direction_matrix * norm;
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

            //**********************************************向约束空间投影*****************************************
            double yaw_error = 0;
            double pitch_error = 0;
            double roll_error=0;
            double total_error = 0;
            bool project_success = true;
            Eigen::Vector3d end_rot_eulerAngle;
            Eigen::Vector3d required_eulerAngle;
            Eigen::Matrix3d required_rot_matrix;
            Eigen::Matrix3d error_rot_matrix;
            Eigen::AngleAxisd error_axis_angle;
            Eigen::AngleAxis<double >::Vector3 error_axis;
            double error_angle;

            int project_count = 0;
            project_start_time = ros::Time::now();
            while (true){
                project_count++;
                std::cout<<"projecting "<< project_count <<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);



                roll_error = 1.57 - end_rot_eulerAngle[2];

//                if(end_rot_eulerAngle[1] < _pitch_min){
//                    pitch_error = _pitch_min - end_rot_eulerAngle[1];
//                }
//                else if(end_rot_eulerAngle[1] > _pitch_max){
//                    pitch_error = _pitch_max - end_rot_eulerAngle[1];
//                }
//                else{
//                    pitch_error = 0;
//                }

                pitch_error = 0 - end_rot_eulerAngle[1];

                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector = error_axis * error_angle;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;

//                    std::cout<<"required_eulerAngle:  "<<required_eulerAngle.transpose()<<std::endl;
//                    std::cout<<"error_axis_angle_angle:  "<<error_axis_angle.angle()<<std::endl;
//                    std::cout<<"error_axis_angle_axis:  "<<error_axis_angle.axis().transpose()<<std::endl;

                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_down_3 = end_jacobian.bottomRows(3);
                    end_jacobian_down_3_pinv = end_jacobian_down_3.transpose() * ((end_jacobian_down_3 * end_jacobian_down_3.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_down_3_pinv  * task_delta_vector;
                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。
//                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }
            }
            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;


                std::vector<double> master_link_distance_values;
                std::vector<std::string> master_link_names;
                std::vector<Eigen::Vector3d> master_link_normals;
                std::vector<Eigen::Vector3d> master_link_minimum_dis_points;
                std::vector<Eigen::MatrixXd> master_link_minimum_dis_point_jac_matrices;
                
                collision_detection::DistanceRequest collision_req_master;
                collision_req_master.enable_nearest_points = true;
                collision_req_master.enable_signed_distance = true;
                collision_req_master.active_components_only = &_master_link_model_set;
                collision_req_master.group_name = "left_arm";
                collision_req_master.type = collision_detection::DistanceRequestType::LIMITED;
                        
                collision_detection::DistanceResult collision_res_master;
                world_FCL.distanceRobot(collision_req_master, collision_res_master, *robot, qs_state);
                
                if(!collision_res_master.collision){
                    std::vector<double> master_weight_vector;
                    std::vector<Eigen::Matrix<double,7,1>> master_normalized_joint_vector;
                    collision_detection::DistanceMap master_distances = collision_res_master.distances;
                    for(auto it=master_distances.begin(); it!=master_distances.end();it++){
//                        std::cout<< it->first.first <<" and "<<it->first.second<<"   "<<it->second[0].link_names[0]<<" and "<<it->second[0].link_names[1]<<std::endl;
                        if(it->second[0].distance < 0.1){
                            master_link_names.push_back(it->first.second);
                            master_link_distance_values.push_back(it->second[0].distance);
                            master_link_minimum_dis_points.push_back(it->second[0].nearest_points[1]);
                            master_link_normals.push_back(it->second[0].normal);
                        }
                    }
                    for(size_t i=0; i<master_link_distance_values.size(); i++){
                        double weight = 0.5/(1+std::exp((2*master_link_distance_values[i]/0.1-1)*4));
                        master_weight_vector.push_back(weight);
                        Eigen::MatrixXd jac;
                        Eigen::MatrixXd jac_up_3;
                        Eigen::MatrixXd jac_up_3_pinv;
                        qs_state.getJacobian(planning_group, planning_group->getLinkModel(master_link_names[i]), master_link_minimum_dis_points[i], jac);
                        jac_up_3 = jac.topRows(3);
                        jac_up_3_pinv = jac_up_3.transpose() * ((jac_up_3 * (jac_up_3.transpose())).inverse() + 0.001 * Eigen::Matrix3d::Identity());
                        Eigen::Matrix<double, 7, 1> joint_vector;
                        joint_vector = jac_up_3_pinv * master_link_normals[i];
                        joint_vector.normalize();
                        master_normalized_joint_vector.push_back(joint_vector);
                    }

                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();
                        
                        if(planning_scene_ptr->isStateValid(qs_state, "right_arm")){
                            perdex_one_extend.no_collide=1;
                            
                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
                            perdex_one_extend.extend_success=1;
                            matrix_tree_element.head(7) = qs_matrix;
                            matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                            if(if_tree_a){
                                _a_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _a_rrt_tree_state.push_back(tmp);
                                qs_old_index = _a_rrt_tree_state.size() - 1;


                                std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> master_make_pair;
                                master_make_pair.first = master_weight_vector;
                                master_make_pair.second = master_normalized_joint_vector;
                                _a_rrt_tree_adjust_weight_and_joints_master.push_back(master_make_pair);

                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;

                                std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> master_make_pair;
                                master_make_pair.first = master_weight_vector;
                                master_make_pair.second = master_normalized_joint_vector;
                                _b_rrt_tree_adjust_weight_and_joints_master.push_back(master_make_pair);
                            }

                        }
                        else{
                            reached_state_matrix = qs_old_matrix;

                            perdex_one_extend.no_collide=0;
                            extend_end_time = ros::Time::now();
                            perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                            if(if_tree_a){
                                perdex_one_sample.tree_a.push_back(perdex_one_extend);
                            }
                            else{
                                perdex_one_sample.tree_b.push_back(perdex_one_extend);
                            }
                            break;
                        }
                    }
                    else{
                        reached_state_matrix = qs_old_matrix;

                        perdex_one_extend.ik_success = 0;
                        extend_end_time = ros::Time::now();
                        perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                        if(if_tree_a){
                            perdex_one_sample.tree_a.push_back(perdex_one_extend);
                        }
                        else{
                            perdex_one_sample.tree_b.push_back(perdex_one_extend);
                        }                    break;
                    }
                    
                    
                } 
                else{
                    reached_state_matrix = qs_old_matrix;

                    perdex_one_extend.ik_success = 0;
                    extend_end_time = ros::Time::now();
                    perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                    if(if_tree_a){
                        perdex_one_sample.tree_a.push_back(perdex_one_extend);
                    }
                    else{
                        perdex_one_sample.tree_b.push_back(perdex_one_extend);
                    }                    break;
                }
                
                
                
            }
            else{
                reached_state_matrix = qs_old_matrix;

                perdex_one_extend.project_success = 0;
                extend_end_time = ros::Time::now();
                perdex_one_extend.extend_total_spend_time = double((extend_end_time - extend_start_time).nsec)/1000000000;
                if(if_tree_a){
                    perdex_one_sample.tree_a.push_back(perdex_one_extend);
                }
                else{
                    perdex_one_sample.tree_b.push_back(perdex_one_extend);
                }                break;
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

    }
}

void DualCBiRRT::constraint_extend_task_space_dir(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a, bool if_sample, Eigen::Vector3d& reached_task_pos, size_t & reached_index){

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

    Eigen::Vector3d random_task_state_vector;
    Eigen::Vector3d qs_task_state_vector;
    Eigen::Vector3d qs_old_task_state_vector;
    Eigen::Vector3d task_direction_vector;
    Eigen::Vector3d extend_goal_task_state_vector;

    random_task_state_vector = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    double test = qs_old_task_state_vector[0];
    std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
    std::cout<<"random_task_state_vector: "<<random_task_state_vector.transpose()<<std::endl;
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
    double yaw_error = 0;
    double pitch_error = 0;
    double roll_error=0;
    double total_error = 0;
    bool project_success = true;
    Eigen::Vector3d end_rot_eulerAngle;
    Eigen::Vector3d required_eulerAngle;
    Eigen::Matrix3d end_rot_matrix;
    Eigen::Matrix3d required_rot_matrix;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;
    Eigen::Vector3d pos_error;


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
            reached_index = _a_rrt_tree_state[qs_old_index].second;
            _a_rrt_tree_matrix.pop_back();
            _a_rrt_tree_state.pop_back();
            _a_rrt_tree_task_space_state.pop_back();

            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
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

                roll_error = 1.57 - end_rot_eulerAngle[2];
                pitch_error = 0 - end_rot_eulerAngle[1];
                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                pos_error = extend_goal_task_state_vector - qs_task_state_vector;

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta && pos_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
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
                    const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                    qs_task_state_vector = end_pose.translation();

                    end_rot_matrix = end_pose.rotation();
                    error_axis_angle = end_rot_matrix;
                    end_rot_eulerAngle = end_rot_matrix.eulerAngles(2, 1, 0);

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        std::cout<<"end_rot_eulerAngle  "<<end_rot_eulerAngle.transpose()<<std::endl;
                        std::cout<<"required_eulerAngle  "<<required_eulerAngle.transpose()<<std::endl;
                        std::cout<<"error_axis_angle  "<<error_axis_angle.angle()<<std::endl;
                        std::cout<<"pos_error "<<pos_error.transpose()<<std::endl;
                        project_success = false;
                        break;
                    }
                }
            }

            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;

                if(planning_scene_ptr->isStateValid(qs_state, "left_arm")){

                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();

                        if(planning_scene_ptr->isStateValid(qs_state, "right_arm")){
                            perdex_one_extend.no_collide=1;

                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
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
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;

                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
                                std::cout<<"extend_goal_vector : "<<extend_goal_task_state_vector.transpose()<<std::endl;
                                std::cout<<"random_vector : "<<random_task_state_vector.transpose()<<std::endl;
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
    }
    std::cout<<"outout!!"<<std::endl;
}

void DualCBiRRT::constraint_extend_task_space_dir_dense_collide(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group,std::pair<std::vector<double>, std::vector<double>>& master_joint_pos_bounds, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a, bool if_sample, Eigen::Vector3d& reached_task_pos, size_t & reached_index){

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

    Eigen::Vector3d random_task_state_vector;
    Eigen::Vector3d qs_task_state_vector;
    Eigen::Vector3d qs_old_task_state_vector;
    Eigen::Vector3d task_direction_vector;
    Eigen::Vector3d extend_goal_task_state_vector;

    random_task_state_vector = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    double test = qs_old_task_state_vector[0];
    std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
    std::cout<<"random_task_state_vector: "<<random_task_state_vector.transpose()<<std::endl;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;
    Eigen::MatrixXd end_jacobian_null_space;

    Eigen::Matrix<double, 6, 1> task_delta_vector;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    Eigen::Matrix<double, 7, 1> null_space_joint_delta_vector;

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
    double yaw_error = 0;
    double pitch_error = 0;
    double roll_error=0;
    double total_error = 0;
    bool project_success = true;
    Eigen::Vector3d end_rot_eulerAngle;
    Eigen::Vector3d required_eulerAngle;
    Eigen::Matrix3d end_rot_matrix;
    Eigen::Matrix3d required_rot_matrix;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;
    Eigen::Vector3d pos_error;
    //****************************************************************************************************

    //********************************性能优化函数**************************
    Eigen::Matrix<double, 7, 1>  delta_H;
    Eigen::Matrix<double, 7, 1>  max_min_square;
    double compute_tmp;
    for(size_t i=0; i<7 ;i++){
        compute_tmp = master_joint_pos_bounds.first[i] - master_joint_pos_bounds.second[i];
        max_min_square[i] = compute_tmp * compute_tmp;
    }
    //********************************************************************

    //************************************计算距离障碍物最近距离信息**************************************
    const std::set<const robot_model::LinkModel*> planning_link_model = planning_group->getUpdatedLinkModelsSet();
    collision_detection::DistanceResultsData min_distance;
    double minimum_dis_value;
    Eigen::Vector3d away_from_collision_normal;
    Eigen::Vector3d robot_minimum_dis_point;
    std::string link_name;
    Eigen::MatrixXd minimum_collision_dis_jac;
    Eigen::MatrixXd minimum_collision_dis_jac_pinv;
    Eigen::MatrixXd minimum_collision_dis_jac_up_3;
    Eigen::MatrixXd minimum_collision_dis_jac_up_3_pinv;
    Eigen::Matrix<double, 3, 1> avoid_obstacle_vel;
    //****************************************************************************************************
    int extend_num = 0;
    int performance_index_extend_num = 0;
    while (true){
        if(if_sample){
            extend_num++;
            if(extend_num > 10){
                reached_state_matrix = qs_matrix;
                reached_task_pos = qs_task_state_vector;
                reached_index = qs_old_index;
                break;
            }
        }
        std::cout<<"extending"<<std::endl;
        if ((random_task_state_vector - qs_task_state_vector).norm() < 0.005){
            reached_state_matrix = qs_matrix;
            reached_task_pos = qs_task_state_vector;
            reached_index = qs_old_index;
            break;
        }
        else if((random_task_state_vector - qs_task_state_vector).norm() > (random_task_state_vector-qs_old_task_state_vector).norm()){
            reached_state_matrix = qs_old_matrix;
            reached_task_pos = qs_old_task_state_vector;
            reached_index = _a_rrt_tree_state[qs_old_index].second;
            _a_rrt_tree_matrix.pop_back();
            _a_rrt_tree_state.pop_back();
            _a_rrt_tree_task_space_state.pop_back();

            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
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

                roll_error = 1.57 - end_rot_eulerAngle[2];
                pitch_error = 0 - end_rot_eulerAngle[1];
                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                pos_error = extend_goal_task_state_vector - qs_task_state_vector;

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta && pos_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{

                    //碰撞信息计算
                    collision_detection::DistanceRequest dis_req;
                    collision_detection::DistanceResult dis_res;
                    dis_req.group_name = "left_arm";
                    dis_req.active_components_only = &planning_link_model;
                    dis_req.enable_nearest_points = true;
                    dis_req.type = collision_detection::DistanceRequestType::SINGLE;
                    world_FCL.distanceRobot(dis_req, dis_res, *robot, qs_state);
                    min_distance = dis_res.minimum_distance;
                    minimum_dis_value = min_distance.distance;
                    away_from_collision_normal = min_distance.normal;
                    robot_minimum_dis_point = min_distance.nearest_points[1];
                    link_name = min_distance.link_names[1];

                    if(!(link_name =="")){
                        const robot_state::LinkModel* collision_link = qs_state.getLinkModel(link_name);
                        if(qs_state.getJacobian(planning_group, collision_link, robot_minimum_dis_point, minimum_collision_dis_jac)){
                        }
                        else{
                            exit(1);
                        }
                        minimum_collision_dis_jac_up_3 = minimum_collision_dis_jac.topRows(3);
                        minimum_collision_dis_jac_up_3_pinv = minimum_collision_dis_jac_up_3.transpose()*((minimum_collision_dis_jac_up_3 * (minimum_collision_dis_jac_up_3.transpose()) +  0.01 * Eigen::Matrix<double,3,3>::Identity() ).inverse());
                        double alpha;
                        alpha = 0.05/(1+std::exp((2*minimum_dis_value/0.03-1)*4));
                        avoid_obstacle_vel = alpha * away_from_collision_normal;
//                        avoid_obstacle_vel /= 0.01;
                    }
                    else{
                        minimum_collision_dis_jac_up_3 = Eigen::Matrix<double,3,7>::Zero();
                        minimum_collision_dis_jac_up_3_pinv = Eigen::Matrix<double,7,3>::Zero();
                        avoid_obstacle_vel = Eigen::Matrix<double,3,1>::Zero();
                    }


                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector.tail(3) = error_axis * error_angle;
                    task_delta_vector.head(3) = pos_error;
                    task_delta_vector = _error_coefficient * task_delta_vector / 0.01;


                    end_jacobian = qs_state.getJacobian(planning_group);
                    end_jacobian_pinv = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse());
                    joint_delta_vector =  end_jacobian_pinv  * task_delta_vector;
                    end_jacobian_null_space = Eigen::Matrix<double, 7, 7>::Identity() - end_jacobian_pinv * end_jacobian;


                    Eigen::Matrix<double, 3, 1> closest_point_vel;
                    closest_point_vel = minimum_collision_dis_jac_up_3 * joint_delta_vector;
                    if(closest_point_vel.dot(away_from_collision_normal) > 0){
                        ;
                    }
                    else{

                        null_space_joint_delta_vector = end_jacobian_null_space * (minimum_collision_dis_jac_up_3_pinv * avoid_obstacle_vel);
                        joint_delta_vector += null_space_joint_delta_vector;
                    }

                    for(size_t i=0; i<7; i++){
                        delta_H[i] =  -0.25 * max_min_square[i] * (2*qs_matrix[i]-master_joint_pos_bounds.first[i]-master_joint_pos_bounds.second[i]) / (  pow((master_joint_pos_bounds.first[i]-qs_matrix[i]),2) * pow((master_joint_pos_bounds.second[i]-qs_matrix[i]),2)  );
                    }

                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    for(size_t i=0; i<7; i++){
                        qs_vector[i] = qs_matrix[i];
                    }
                    qs_state.setJointGroupPositions(planning_group, qs_vector);
                    qs_state.update();
                    const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                    qs_task_state_vector = end_pose.translation();

                    end_rot_matrix = end_pose.rotation();
                    end_rot_eulerAngle = end_rot_matrix.eulerAngles(2, 1, 0);

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }
            }

            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;

                if(planning_scene_ptr->isStateValid(qs_state, "left_arm")){

                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem_dense_collide_new(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();

                        if(planning_scene_ptr->isStateValid(qs_state, "right_arm")){
                            perdex_one_extend.no_collide=1;

                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
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
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;

                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
                                std::cout<<"extend_goal_vector : "<<extend_goal_task_state_vector.transpose()<<std::endl;
                                std::cout<<"random_vector : "<<random_task_state_vector.transpose()<<std::endl;
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
    }
    std::cout<<"outout!!"<<std::endl;
}

void DualCBiRRT::constraint_extend_task_space_dir_new(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a, bool if_sample, Eigen::Vector3d& reached_task_pos, size_t & reached_index){
    std::set<const moveit::core::LinkModel *> master_link_model_set;
    std::set<const moveit::core::LinkModel *> slave_link_model_set;
    for(size_t i=0; i<_slave_link_names.size(); i++){
        master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }


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

    Eigen::Vector3d random_task_state_vector;
    Eigen::Vector3d qs_task_state_vector;
    Eigen::Vector3d qs_old_task_state_vector;
    Eigen::Vector3d task_direction_vector;
    Eigen::Vector3d extend_goal_task_state_vector;

    random_task_state_vector = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    double test = qs_old_task_state_vector[0];
    std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
    std::cout<<"random_task_state_vector: "<<random_task_state_vector.transpose()<<std::endl;
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
    double yaw_error = 0;
    double pitch_error = 0;
    double roll_error=0;
    double total_error = 0;
    bool project_success = true;
    Eigen::Vector3d end_rot_eulerAngle;
    Eigen::Vector3d required_eulerAngle;
    Eigen::Matrix3d end_rot_matrix;
    Eigen::Matrix3d required_rot_matrix;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;
    Eigen::Vector3d pos_error;


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
            reached_index = _a_rrt_tree_state[qs_old_index].second;
            _a_rrt_tree_matrix.pop_back();
            _a_rrt_tree_state.pop_back();
            _a_rrt_tree_task_space_state.pop_back();

            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
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

                roll_error = 1.57 - end_rot_eulerAngle[2];
                pitch_error = 0 - end_rot_eulerAngle[1];
                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                pos_error = extend_goal_task_state_vector - qs_task_state_vector;

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta && pos_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
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
                    const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                    qs_task_state_vector = end_pose.translation();

                    end_rot_matrix = end_pose.rotation();
                    error_axis_angle = end_rot_matrix;
                    end_rot_eulerAngle = end_rot_matrix.eulerAngles(2, 1, 0);

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        std::cout<<"end_rot_eulerAngle  "<<end_rot_eulerAngle.transpose()<<std::endl;
                        std::cout<<"required_eulerAngle  "<<required_eulerAngle.transpose()<<std::endl;
                        std::cout<<"error_axis_angle  "<<error_axis_angle.angle()<<std::endl;
                        std::cout<<"pos_error "<<pos_error.transpose()<<std::endl;
                        project_success = false;
                        break;
                    }
                }
            }

            project_end_time = ros::Time::now();
            perdex_one_extend.project_total_spend_time = double((project_end_time - project_start_time).nsec) / 1000000000;
            perdex_one_extend.constraint_project_times = project_count;
            //如果投影成功，qs_state肯定更新过
            if(project_success){
                perdex_one_extend.project_success = 1;


                collision_detection::DistanceRequest collision_req_master;
                collision_req_master.enable_nearest_points = true;
                collision_req_master.enable_signed_distance = true;
                collision_req_master.active_components_only = &master_link_model_set;
                collision_req_master.group_name = "left_arm";
                collision_req_master.type = collision_detection::DistanceRequestType::LIMITED;
                collision_detection::DistanceResult collision_res_master;
                collision_detection::DistanceResult new_collision_res_master;
                world_FCL.distanceRobot(collision_req_master, collision_res_master, *robot, qs_state);


                if(!collision_res_master.collision){
//                    if(!planning_scene_ptr->isStateValid(qs_state, "left_arm")){
//                        std::cout<<"isStateColliding  "<<planning_scene_ptr->isStateColliding(qs_state, "left_arm")<<std::endl;
//                        std::cout<<"minimum distance  "<<collision_res_master.minimum_distance.distance<<std::endl;
//                        std::cout<<"computed_master_angles_matrix\n  "<<qs_matrix.transpose()<<std::endl;
//                        Eigen::Matrix<double ,7,1>up_bounds;
//                        Eigen::Matrix<double ,7,1>low_bounds;
//                        for(size_t i=0;i<7;i++){
//                            up_bounds[i] = slave_joint_pos_bounds.first[i];
//                            low_bounds[i] = slave_joint_pos_bounds.second[i];
//                        }
//                        std::cout<<"slave_up_bounds\n"<<up_bounds.transpose()<<std::endl;
//                        std::cout<<"slave_low_bounds\n"<<low_bounds.transpose()<<std::endl;
//                        std::cout<<std::endl;
//                    }


                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
                    ik_end_time = ros::Time::now();
                    perdex_one_extend.ik_total_spend_time = double((ik_end_time - ik_start_time).nsec) / 1000000000;
                    if(ik_result){
                        perdex_one_extend.ik_success = 1;
                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);
                        qs_state.update();


                        collision_detection::DistanceRequest collision_req_slave;
                        collision_req_slave.enable_nearest_points = true;
                        collision_req_slave.enable_signed_distance = true;
                        collision_req_slave.active_components_only = &slave_link_model_set;
                        collision_req_slave.group_name = "right_arm";
                        collision_req_slave.type = collision_detection::DistanceRequestType::LIMITED;
                        collision_detection::DistanceResult collision_res_slave;
                        collision_detection::DistanceResult new_collision_res_slave;
                        world_FCL.distanceRobot(collision_req_slave, collision_res_slave, *robot, qs_state);

                        if(!collision_res_slave.collision){
//                            if(!planning_scene_ptr->isStateValid(qs_state, "right_arm")){
//                                std::cout<<"minimum distance  "<<collision_res_slave.minimum_distance.distance<<std::endl;
//                                std::cout<<"computed_slave_angles_matrix\n  "<<computed_slave_angles_matrix.transpose()<<std::endl;
//                                Eigen::Matrix<double ,7,1>up_bounds;
//                                Eigen::Matrix<double ,7,1>low_bounds;
//                                for(size_t i=0;i<7;i++){
//                                    up_bounds[i] = slave_joint_pos_bounds.first[i];
//                                    low_bounds[i] = slave_joint_pos_bounds.second[i];
//                                }
//                                std::cout<<"slave_up_bounds\n"<<up_bounds.transpose()<<std::endl;
//                                std::cout<<"slave_low_bounds\n"<<low_bounds.transpose()<<std::endl;
//                                std::cout<<std::endl;
//                            }
                            perdex_one_extend.no_collide=1;

                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
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
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;

                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
                                std::cout<<"extend_goal_vector : "<<extend_goal_task_state_vector.transpose()<<std::endl;
                                std::cout<<"random_vector : "<<random_task_state_vector.transpose()<<std::endl;
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
    }
    std::cout<<"outout!!"<<std::endl;
}

void DualCBiRRT::constraint_extend_task_space_dir_try_adjust(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds, PerformanceIndexOneSample & perdex_one_sample, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot, bool if_tree_a, bool if_sample, Eigen::Vector3d& reached_task_pos, size_t & reached_index){
    std::set<const moveit::core::LinkModel *> master_link_model_set;
    std::set<const moveit::core::LinkModel *> slave_link_model_set;
    for(size_t i=0; i<_slave_link_names.size(); i++){
        master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }
    std::vector<double> master_link_distance_values;
    std::vector<Eigen::Vector3d> master_link_normals;
    std::vector<Eigen::Vector3d> master_link_minimum_dis_points;
    std::vector<Eigen::MatrixXd> master_link_minimum_dis_point_jac_matrices;
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

    Eigen::Vector3d random_task_state_vector;
    Eigen::Vector3d qs_task_state_vector;
    Eigen::Vector3d qs_old_task_state_vector;
    Eigen::Vector3d task_direction_vector;
    Eigen::Vector3d extend_goal_task_state_vector;

    random_task_state_vector = random_state.getGlobalLinkTransform("left_gripper").translation();
    qs_task_state_vector = qs_state.getGlobalLinkTransform("left_gripper").translation();
    qs_old_task_state_vector = qs_old_state.getGlobalLinkTransform("left_gripper").translation();
    double test = qs_old_task_state_vector[0];
    std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
    std::cout<<"random_task_state_vector: "<<random_task_state_vector.transpose()<<std::endl;
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
    double yaw_error = 0;
    double pitch_error = 0;
    double roll_error=0;
    double total_error = 0;
    bool project_success = true;
    Eigen::Vector3d end_rot_eulerAngle;
    Eigen::Vector3d required_eulerAngle;
    Eigen::Matrix3d end_rot_matrix;
    Eigen::Matrix3d required_rot_matrix;
    Eigen::Matrix3d error_rot_matrix;
    Eigen::AngleAxisd error_axis_angle;
    Eigen::AngleAxis<double >::Vector3 error_axis;
    double error_angle;
    Eigen::Vector3d pos_error;


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
            reached_index = _a_rrt_tree_state[qs_old_index].second;
            _a_rrt_tree_matrix.pop_back();
            _a_rrt_tree_state.pop_back();
            _a_rrt_tree_task_space_state.pop_back();

            break;
        }
        else{
            qs_old_matrix = qs_matrix;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_state = qs_state;

            //计算出了一个任务空间中的目标
            task_direction_vector = random_task_state_vector - qs_task_state_vector;
            double norm = task_direction_vector.norm();
            task_direction_vector = task_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * _task_step_size;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + task_direction_vector * norm;
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

                roll_error = 1.57 - end_rot_eulerAngle[2];
                pitch_error = 0 - end_rot_eulerAngle[1];
                if(end_rot_eulerAngle[1] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[1];
                }
                else{
                    yaw_error = 0;
                }

                pos_error = extend_goal_task_state_vector - qs_task_state_vector;

                total_error = sqrt(roll_error*roll_error + pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta && pos_error.norm() < 0.005){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2] + roll_error;
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;


                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
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
                    const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                    qs_task_state_vector = end_pose.translation();

                    end_rot_matrix = end_pose.rotation();
                    end_rot_eulerAngle = end_rot_matrix.eulerAngles(2, 1, 0);

                    if (!qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
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
                collision_detection::DistanceRequest collision_req_master;
                collision_req_master.enable_nearest_points = true;
                collision_req_master.enable_signed_distance = true;
                collision_req_master.active_components_only = &master_link_model_set;
                collision_req_master.group_name = "left_arm";
                collision_req_master.type = collision_detection::DistanceRequestType::LIMITED;

                collision_detection::DistanceResult collision_res_master;
                collision_detection::DistanceResult new_collision_res_master;
                world_FCL.distanceRobot(collision_req_master, collision_res_master, *robot, qs_state);
                
                
                if(collision_res_master.collision){
                    collision_detection::DistanceResultsData master_min_dis = collision_res_master.minimum_distance;
                    double master_collide_dis_value = master_min_dis.distance;
                    std::cout<<"master_min_dis  "<<master_collide_dis_value<<std::endl;

                    Eigen::Vector3d collide_pos = master_min_dis.nearest_points[1];
                    Eigen::Vector3d collide_normal = master_min_dis.normal;
                    const std::string collide_link_name  = master_min_dis.link_names[1];
                    const robot_state::LinkModel* collision_link = qs_state.getLinkModel(collide_link_name);
                    Eigen::MatrixXd collision_point_jac;
                    qs_state.getJacobian(planning_group, collision_link, collide_pos, collision_point_jac);
                    Eigen::MatrixXd master_jac = qs_state.getJacobian(planning_group);
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(master_jac, Eigen::ComputeFullV);
                    Eigen::MatrixXd V = svd_holder.matrixV();
//                    std::cout<<"V\n  "<<V<<std::endl;
                    Eigen::MatrixXd self_motion_vec = V.rightCols(1);

                    //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0
                    Eigen::Matrix<double ,6, 1> tmp;
                    Eigen::Vector3d self_motion_at_collision_point_velocity;
                    tmp = (collision_point_jac * self_motion_vec);
                    self_motion_at_collision_point_velocity = tmp.head(3);
                    double self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(collide_normal);
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
                        world_FCL.distanceRobot(collision_req_master, new_collision_res_master, *robot, qs_state);
                        std::cout<<"new_master_min_dis  "<<new_collision_res_master.minimum_distance.distance<<std::endl;
                        if(!new_collision_res_master.collision){
                            std::cout<<"master fix success!"<<std::endl;
                            ROS_WARN("master fix success!");
                            if_master_must_collide = false;
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

                    if_master_must_collide = false;
                }
                //end********************先计算左臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************



                if(!if_master_must_collide){
                    ik_start_time =  ros::Time::now();
                    bool ik_result = solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds, perdex_one_extend, world_FCL, robot);
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
                        collision_detection::DistanceRequest collision_req_slave;
                        collision_req_slave.enable_nearest_points = true;
                        collision_req_slave.enable_signed_distance = true;
                        collision_req_slave.active_components_only = &slave_link_model_set;
                        collision_req_slave.group_name = "right_arm";
                        collision_req_slave.type = collision_detection::DistanceRequestType::LIMITED;

                        collision_detection::DistanceResult collision_res_slave;
                        collision_detection::DistanceResult new_collision_res_slave;
                        world_FCL.distanceRobot(collision_req_slave, collision_res_slave, *robot, qs_state);


                        if(collision_res_slave.collision){
                            collision_detection::DistanceResultsData slave_min_dis = collision_res_slave.minimum_distance;
                            double slave_collide_dis_value = slave_min_dis.distance;
                            std::cout<<"slave_min_dis  "<<slave_collide_dis_value<<std::endl;

                            Eigen::Vector3d collide_pos = slave_min_dis.nearest_points[1];
                            Eigen::Vector3d collide_normal = slave_min_dis.normal;
                            const std::string collide_link_name  = slave_min_dis.link_names[1];
                            const robot_state::LinkModel* collision_link = qs_state.getLinkModel(collide_link_name);
                            Eigen::MatrixXd collision_point_jac;
                            qs_state.getJacobian(slave_group, collision_link, collide_pos, collision_point_jac);
                            Eigen::MatrixXd slave_jac = qs_state.getJacobian(slave_group);
                            Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_jac, Eigen::ComputeFullV);
                            Eigen::MatrixXd V = svd_holder.matrixV();
//                    std::cout<<"V\n  "<<V<<std::endl;
                            Eigen::MatrixXd self_motion_vec = V.rightCols(1);

                            //雅克比矩阵乘V的最后一列确实计算出来的速度几乎等于0
                            Eigen::Matrix<double ,6, 1> tmp;
                            Eigen::Vector3d self_motion_at_collision_point_velocity;
                            tmp = (collision_point_jac * self_motion_vec);
                            self_motion_at_collision_point_velocity = tmp.head(3);
                            double self_motion_at_collision_point_velocity_magnititude = self_motion_at_collision_point_velocity.dot(collide_normal);
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
                                world_FCL.distanceRobot(collision_req_slave, new_collision_res_slave, *robot, qs_state);
                                std::cout<<"new_slave_min_dis  "<<new_collision_res_slave.minimum_distance.distance<<std::endl;
                                if(!new_collision_res_slave.collision){
                                    std::cout<<"slave fix success!"<<std::endl;
                                    ROS_WARN("slave fix success!");
                                    if_slave_must_collide = false;
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

                            if_slave_must_collide = false;
                        }
                        //end********************计算右臂的碰撞距离，如果碰撞尝试通过 self motion 修改，修改还不行才确定会碰撞************************
                        
                        
                        
                        if(!if_slave_must_collide){
                            perdex_one_extend.no_collide=1;

                            //将节点加入树中
                            std::cout<<"adding a state"<<std::endl;
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
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;

                            }
                            else{
                                _b_rrt_tree_matrix.push_back(matrix_tree_element);
                                current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                                std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                                _b_rrt_tree_state.push_back(tmp);
                                qs_old_index = _b_rrt_tree_state.size() - 1;
                                std::cout<<"qs_task_state_vector: "<<qs_task_state_vector.transpose()<<std::endl;
                                std::cout<<"extend_goal_vector : "<<extend_goal_task_state_vector.transpose()<<std::endl;
                                std::cout<<"random_vector : "<<random_task_state_vector.transpose()<<std::endl;
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
    }
    std::cout<<"outout!!"<<std::endl;
}

bool DualCBiRRT::plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){


    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);


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
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
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

    //计算 slave 相关变量
//    Eigen::Matrix<double, 7, 1> start_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> start_computed_slave_angles_matrix;
//    std::vector<double> start_slave_angles_vector;
//    Eigen::Matrix<double, 7, 1> goal_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> goal_computed_slave_angles_matrix;
//    std::vector<double> goal_slave_angles_vector;
//    start_state.copyJointGroupPositions(slave_group, start_slave_angles_vector);
//    goal_state.copyJointGroupPositions(slave_group, goal_slave_angles_vector);
//    for(size_t i=0; i<7; i++){
//        start_current_slave_angles_matrix[i] = start_slave_angles_vector[i];
//        goal_current_slave_angles_matrix[i] = goal_slave_angles_vector[i];
//    }
//    if(solve_IK_problem(start_current_slave_angles_matrix, start_value_matrix, start_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute start slave state success!!!");
//        std::cout<<start_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute start slave state failed!!!");
//        return false;
//    }
//    if(solve_IK_problem(goal_current_slave_angles_matrix, goal_value_matrix, goal_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute goal slave state success!!!");
//        std::cout<<goal_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute goal slave state failed!!!");
//        return false;
//    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

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
            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(a_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(b_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
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

bool DualCBiRRT::plan_try_adjust(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){


    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);


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
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
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

    //计算 slave 相关变量
//    Eigen::Matrix<double, 7, 1> start_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> start_computed_slave_angles_matrix;
//    std::vector<double> start_slave_angles_vector;
//    Eigen::Matrix<double, 7, 1> goal_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> goal_computed_slave_angles_matrix;
//    std::vector<double> goal_slave_angles_vector;
//    start_state.copyJointGroupPositions(slave_group, start_slave_angles_vector);
//    goal_state.copyJointGroupPositions(slave_group, goal_slave_angles_vector);
//    for(size_t i=0; i<7; i++){
//        start_current_slave_angles_matrix[i] = start_slave_angles_vector[i];
//        goal_current_slave_angles_matrix[i] = goal_slave_angles_vector[i];
//    }
//    if(solve_IK_problem(start_current_slave_angles_matrix, start_value_matrix, start_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute start slave state success!!!");
//        std::cout<<start_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute start slave state failed!!!");
//        return false;
//    }
//    if(solve_IK_problem(goal_current_slave_angles_matrix, goal_value_matrix, goal_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute goal slave state success!!!");
//        std::cout<<goal_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute goal slave state failed!!!");
//        return false;
//    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

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
            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(a_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_try_adjust(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(b_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_try_adjust(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
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

bool DualCBiRRT::plan_dense_collide_new(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){


    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);


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
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
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

    //计算 slave 相关变量
//    Eigen::Matrix<double, 7, 1> start_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> start_computed_slave_angles_matrix;
//    std::vector<double> start_slave_angles_vector;
//    Eigen::Matrix<double, 7, 1> goal_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> goal_computed_slave_angles_matrix;
//    std::vector<double> goal_slave_angles_vector;
//    start_state.copyJointGroupPositions(slave_group, start_slave_angles_vector);
//    goal_state.copyJointGroupPositions(slave_group, goal_slave_angles_vector);
//    for(size_t i=0; i<7; i++){
//        start_current_slave_angles_matrix[i] = start_slave_angles_vector[i];
//        goal_current_slave_angles_matrix[i] = goal_slave_angles_vector[i];
//    }
//    if(solve_IK_problem(start_current_slave_angles_matrix, start_value_matrix, start_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute start slave state success!!!");
//        std::cout<<start_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute start slave state failed!!!");
//        return false;
//    }
//    if(solve_IK_problem(goal_current_slave_angles_matrix, goal_value_matrix, goal_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute goal slave state success!!!");
//        std::cout<<goal_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute goal slave state failed!!!");
//        return false;
//    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

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
            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_dense_collide_new(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(a_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_dense_collide_new(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_dense_collide_new(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order);
            nearest_node_index = near_tree(b_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_dense_collide_new(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
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

bool DualCBiRRT::plan_adjust_extend_dir(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){
    
    for(size_t i=0; i<_slave_link_names.size(); i++){
        _master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        _slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }

    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> init_pair;
    _a_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);
    _b_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);


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
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
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

    //计算 slave 相关变量
//    Eigen::Matrix<double, 7, 1> start_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> start_computed_slave_angles_matrix;
//    std::vector<double> start_slave_angles_vector;
//    Eigen::Matrix<double, 7, 1> goal_current_slave_angles_matrix;
//    Eigen::Matrix<double, 7, 1> goal_computed_slave_angles_matrix;
//    std::vector<double> goal_slave_angles_vector;
//    start_state.copyJointGroupPositions(slave_group, start_slave_angles_vector);
//    goal_state.copyJointGroupPositions(slave_group, goal_slave_angles_vector);
//    for(size_t i=0; i<7; i++){
//        start_current_slave_angles_matrix[i] = start_slave_angles_vector[i];
//        goal_current_slave_angles_matrix[i] = goal_slave_angles_vector[i];
//    }
//    if(solve_IK_problem(start_current_slave_angles_matrix, start_value_matrix, start_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute start slave state success!!!");
//        std::cout<<start_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute start slave state failed!!!");
//        return false;
//    }
//    if(solve_IK_problem(goal_current_slave_angles_matrix, goal_value_matrix, goal_computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
//        ROS_INFO("Compute goal slave state success!!!");
//        std::cout<<goal_computed_slave_angles_matrix<<std::endl;
//    }
//    else{
//        ROS_INFO("Compute goal slave state failed!!!");
//        return false;
//    }

    Eigen::Matrix<double, 14, 1> tmp_matrix_element;
    tmp_matrix_element.head(7) = start_value_matrix;
    tmp_matrix_element.tail(7) = slave_start_value_matrix;
    _a_rrt_tree_matrix.push_back(tmp_matrix_element);
    tmp_matrix_element.head(7) = goal_value_matrix;
    tmp_matrix_element.tail(7) = slave_goal_value_matrix;
    _b_rrt_tree_matrix.push_back(tmp_matrix_element);

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
            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_adjust_extend_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true);
            nearest_node_index = near_tree(a_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_adjust_extend_dir(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample(goal_state, random_state, random_state_value_matrix, planning_group, rng);

            nearest_node_index = near_tree(random_state_value_matrix, nearest_node_matrix, extend_order);
            constraint_extend_tree_adjust_extend_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true);
            nearest_node_index = near_tree(b_tree_reached_matrix, nearest_node_matrix, !extend_order);
            constraint_extend_tree_adjust_extend_dir(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false);

            if((a_tree_reached_matrix - b_tree_reached_matrix).norm() < 0.05)
            {
                ROS_INFO("Success!!!");
                //先添加前半部分路径点
                size_t back_track = _a_rrt_tree_state.size()-1;
                while(back_track != -1){
                    planning_result.push_back(_a_rrt_tree_state[back_track].first);
                    back_track = _a_rrt_tree_state[back_track].second;
                }
                std::reverse(planning_result.begin(), planning_result.end());

                //添加后半部分路径点
                back_track = _b_rrt_tree_state[_b_rrt_tree_state.size()-1].second;
                while(back_track != -1){
                    planning_result.push_back(_b_rrt_tree_state[back_track].first);
                    back_track = _b_rrt_tree_state[back_track].second;
                }
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

bool DualCBiRRT::plan_task_space_dir(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){

    for(size_t i=0; i<_slave_link_names.size(); i++){
        _master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        _slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }

    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> init_pair;
    _a_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);
    _b_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);


    _a_rrt_tree_task_space_state.push_back(start_state.getGlobalLinkTransform("left_gripper").translation());
    _b_rrt_tree_task_space_state.push_back(goal_state.getGlobalLinkTransform("left_gripper").translation());
    std::cout<<"b_tree:: "<<goal_state.getGlobalLinkTransform("left_gripper").translation()<<std::endl;

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
    Eigen::Vector3d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector3d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector3d a_tree_reached_vector;
    Eigen::Vector3d b_tree_reached_vector;
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
            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, a_tree_reached_vector, a_tree_reached_index);
            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);

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
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);

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

bool DualCBiRRT::plan_task_space_dir_new(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){

    for(size_t i=0; i<_slave_link_names.size(); i++){
        _master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        _slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }

    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> init_pair;
    _a_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);
    _b_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);


    _a_rrt_tree_task_space_state.push_back(start_state.getGlobalLinkTransform("left_gripper").translation());
    _b_rrt_tree_task_space_state.push_back(goal_state.getGlobalLinkTransform("left_gripper").translation());


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
    Eigen::Vector3d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector3d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector3d a_tree_reached_vector;
    Eigen::Vector3d b_tree_reached_vector;
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
            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_new(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, a_tree_reached_vector, a_tree_reached_index);
            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_new(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);

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
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_new(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_new(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);

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

bool DualCBiRRT::plan_task_space_dir_dense_collide(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){

    for(size_t i=0; i<_slave_link_names.size(); i++){
        _master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        _slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }

    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> init_pair;
    _a_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);
    _b_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);


    _a_rrt_tree_task_space_state.push_back(start_state.getGlobalLinkTransform("left_gripper").translation());
    _b_rrt_tree_task_space_state.push_back(goal_state.getGlobalLinkTransform("left_gripper").translation());


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

    const std::vector<std::string>& master_joint_names = planning_group->getVariableNames();
    std::pair<std::vector<double>, std::vector<double>> master_joint_pos_bounds;
    std::vector<double> master_joint_max_bounds;
    std::vector<double> master_joint_min_bounds;
    for(size_t i=0; i<master_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = baxter_robot_model_ptr->getVariableBounds(master_joint_names[i]);
        master_joint_max_bounds.push_back(tmp_bounds.max_position_);
        master_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    master_joint_pos_bounds.first = master_joint_max_bounds;
    master_joint_pos_bounds.second = master_joint_min_bounds;


    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState a_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState b_tree_reached_state = planning_scene_ptr->getCurrentStateNonConst();


    Eigen::Matrix<double, 7, 1> random_state_value_matrix;
    Eigen::Vector3d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector3d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector3d a_tree_reached_vector;
    Eigen::Vector3d b_tree_reached_vector;
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
            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_dense_collide(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, master_joint_pos_bounds, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, a_tree_reached_vector, a_tree_reached_index);
            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_dense_collide(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, master_joint_pos_bounds, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);

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
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_dense_collide(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group,master_joint_pos_bounds, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_dense_collide(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group,master_joint_pos_bounds, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);

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

bool DualCBiRRT::plan_task_space_dir_try_adjust(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){

    for(size_t i=0; i<_slave_link_names.size(); i++){
        _master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
        _slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
    }

    //创建一个确定的随机数生成器
    random_numbers::RandomNumberGenerator rng(_seed);
    //创建一个碰撞检测器
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_ptr->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_ptr->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);


    std::pair<robot_state::RobotState, size_t> a_tree_init_pair(start_state, -1);
    std::pair<robot_state::RobotState, size_t> b_tree_init_pair(goal_state, -1);
    _a_rrt_tree_state.push_back(a_tree_init_pair);
    _b_rrt_tree_state.push_back(b_tree_init_pair);

    std::pair<std::vector<double>, std::vector<Eigen::Matrix<double ,7,1>>> init_pair;
    _a_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);
    _b_rrt_tree_adjust_weight_and_joints_master.push_back(init_pair);


    _a_rrt_tree_task_space_state.push_back(start_state.getGlobalLinkTransform("left_gripper").translation());
    _b_rrt_tree_task_space_state.push_back(goal_state.getGlobalLinkTransform("left_gripper").translation());


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
    Eigen::Vector3d random_task_state_vector;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Vector3d nearest_node_task_state;
    Eigen::Matrix<double, 7, 1> a_tree_reached_matrix;
    Eigen::Matrix<double, 7, 1> b_tree_reached_matrix;
    Eigen::Vector3d a_tree_reached_vector;
    Eigen::Vector3d b_tree_reached_vector;
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
            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, a_tree_reached_vector, a_tree_reached_index);
            nearest_node_index = near_tree_task_space(a_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_try_adjust(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, b_tree_reached_vector, b_tree_reached_index);

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
                return true;
            }
            else{
                extend_order = false;
            }
        }
        else{

            sample_task_state(goal_state, random_state, random_state_value_matrix, planning_group, rng, random_task_state_vector);

            nearest_node_index = near_tree_task_space(random_task_state_vector, nearest_node_task_state, nearest_node_matrix, extend_order);
            constraint_extend_task_space_dir_try_adjust(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, extend_order, true, b_tree_reached_vector, b_tree_reached_index);
            nearest_node_index = near_tree_task_space(b_tree_reached_vector, nearest_node_task_state, nearest_node_matrix, !extend_order);
            constraint_extend_task_space_dir_try_adjust(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds, perdex_one_sample, worldFcl, robot, !extend_order, false, a_tree_reached_vector, a_tree_reached_index);

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

bool DualCBiRRT::solve_IK_problem(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds , PerformanceIndexOneExtend & perdex_one_extend, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot){
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
    Eigen::Vector3d master_euler = master_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d master_end_pos = master_end_pose.translation();
    //********************************************************************************************************

    //*********************利用 RobotState 得到 slave 的位置向量、欧拉角向量以及雅克比矩阵、雅克比伪逆矩阵*************************************
    const Eigen::Affine3d & slave_end_pose = slave_state.getGlobalLinkTransform("right_gripper");
    auto slave_end_rot_matrix = slave_end_pose.rotation();
    Eigen::Vector3d slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d slave_end_pos = slave_end_pose.translation();
    Eigen::MatrixXd slave_end_jacobian;
    Eigen::MatrixXd slave_end_jacobian_mp_inverse;
    slave_end_jacobian = slave_state.getJacobian(slave_group);
    //********************************************************************************************************

    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************
    //最开始的闭环约束方式
    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] - 3.1415926);
    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(0, 0, _left_right_distance);
    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    //*****************************************************************************************

    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
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
        if(std::abs(pos_error[0]) < 0.01 && std::abs(pos_error[1])< 0.01 && std::abs(pos_error[2])< 0.01 &&  std::abs(euler_error[0]) < 0.1 && std::abs(euler_error[1]) < 0.1 && std::abs(euler_error[2]) < 0.1){
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
            slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose() + 0.0001 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
            //计算关节增量
            joint_delta_vector = slave_end_jacobian_mp_inverse * stack_error;
            joint_delta_vector *= 0.01;

//            std::cout<<"joint_delta_vector.norm()  " <<joint_delta_vector.norm() <<std::endl;
            if(joint_delta_vector.norm() < 0.01){
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

                if(slave_state.satisfiesBounds(slave_group, 0.05)){
                    //更新末端误差
                    const Eigen::Affine3d & slave_end_pose_tmp = slave_state.getGlobalLinkTransform("right_gripper");
                    slave_end_rot_matrix = slave_end_pose_tmp.rotation();
                    slave_end_pos = slave_end_pose_tmp.translation();
                    slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
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

bool DualCBiRRT::solve_IK_problem_no_plan(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds){
    //用于iktest 求解一个左臂点对应的右臂点
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
    Eigen::Vector3d master_euler = master_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d master_end_pos = master_end_pose.translation();
    //********************************************************************************************************

    //*********************利用 RobotState 得到 slave 的位置向量、欧拉角向量以及雅克比矩阵、雅克比伪逆矩阵*************************************
    const Eigen::Affine3d & slave_end_pose = slave_state.getGlobalLinkTransform("right_gripper");
    auto slave_end_rot_matrix = slave_end_pose.rotation();
    Eigen::Vector3d slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d slave_end_pos = slave_end_pose.translation();
    Eigen::MatrixXd slave_end_jacobian;
    Eigen::MatrixXd slave_end_jacobian_mp_inverse;
    slave_end_jacobian = slave_state.getJacobian(slave_group);


    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************
    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] - 3.1415926);
    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(0, 0, _left_right_distance);
    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    //*****************************************************************************************

    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
    Eigen::Vector3d pos_error = slave_goal_pos - slave_end_pos;
    Eigen::Matrix3d rot_error_matrix = slave_end_rot_matrix * slave_goal_rot_matrix.inverse();
    Eigen::AngleAxisd rot_error_axis_angle(rot_error_matrix);
    double rot_error_angle = rot_error_axis_angle.angle();
    Eigen::Vector3d rot_error_3vector = rot_error_angle * rot_error_axis_angle.axis();
    Eigen::Vector3d euler_error = slave_goal_euler - slave_euler;//画图测试一下欧拉角误差和轴角误差

    Eigen::Vector3d last_pos_error = pos_error;
    double last_rot_error_angle = rot_error_angle;
    Eigen::Matrix<double, 6, 1>  stack_error;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    Eigen::Matrix<double, 7, 1> null_space_joint_delta_vector;
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
        std::cout<<"computing IK "<<count<<std::endl;
        if(count > 2000){
            std::cout<<"computing IK fail"<<std::endl;
            return false;
        }
        else{
            count++;
            if(std::abs(pos_error[0]) < 0.01 && std::abs(pos_error[1])< 0.01 && std::abs(pos_error[2])< 0.01 &&  std::abs(euler_error[0]) < 0.1 && std::abs(euler_error[1]) < 0.1 && std::abs(euler_error[2]) < 0.1){
                result_state_value_matrix = slave_state_value_matrix;
                return true;
            }
            else{

//                //保存作图数据
//                euler_error_draw.emplace_back(euler_error.transpose());
//                rot_error_angle_draw.emplace_back(rot_error_angle);

                //计算消除末端在任务空间的误差所需要的速度
                rot_error_3vector = rot_error_axis_angle.angle() * rot_error_axis_angle.axis();
                stack_error.head(3) = pos_error;
                stack_error.tail(3) = rot_error_3vector;

                stack_error = (0.5 * stack_error) / 0.01;
                //更新雅克比矩阵
                for(size_t i=0; i<7; i++){
                    slave_joint_value_vector[i] = slave_state_value_matrix[i];
                }
                slave_state.setJointGroupPositions(slave_group, slave_joint_value_vector);

                slave_end_jacobian = slave_state.getJacobian(slave_group);

//                Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_end_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
//                Eigen::MatrixXd U = svd_holder.matrixU();
//                Eigen::MatrixXd V = svd_holder.matrixV();
//                Eigen::MatrixXd D = svd_holder.singularValues();
//                std::cout<<"singularValues\n  "<<D.transpose() <<std::endl;



                slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose()).inverse())).eval();

                //计算优化性能指标
//                for(size_t i=0; i<7; i++){
//                    delta_H[i] = 0.25 * max_min_square[i] * (2*slave_state_value_matrix[i]-slave_joint_pos_bounds.first[i]-slave_joint_pos_bounds.second[i]) / (  pow((slave_joint_pos_bounds.first[i]-slave_state_value_matrix[i]),2) * pow((slave_joint_pos_bounds.second[i]-slave_state_value_matrix[i]),2)  );
//                }

                //计算关节增量

//                joint_delta_vector =  (slave_end_jacobian_mp_inverse * stack_error  - (Eigen::Matrix<double, 7, 7>::Identity() - slave_end_jacobian_mp_inverse*slave_end_jacobian) * delta_H) * 0.01;
//                joint_delta_vector = (slave_end_jacobian_mp_inverse * stack_error);
                joint_delta_vector = (slave_end_jacobian_mp_inverse * stack_error);
                slave_state_value_matrix +=  0.01 * joint_delta_vector;

                //更新 slave RobotState, state 更新肯定没有问题
                for(size_t i=0; i<7; i++){
                    slave_joint_value_vector[i] = slave_state_value_matrix[i];
                }
                slave_state.setJointGroupPositions(slave_group, slave_joint_value_vector);
                //更新末端误差
                const Eigen::Affine3d & slave_end_pose_tmp = slave_state.getGlobalLinkTransform("right_gripper");
                slave_end_rot_matrix = slave_end_pose_tmp.rotation();
                slave_end_pos = slave_end_pose_tmp.translation();
                slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);

                last_pos_error = pos_error;
                last_rot_error_angle = rot_error_angle;
                pos_error = slave_goal_pos - slave_end_pos;
                rot_error_matrix = slave_goal_rot_matrix * slave_end_rot_matrix.inverse();
                rot_error_axis_angle = rot_error_matrix;
                rot_error_angle = rot_error_axis_angle.angle();
                euler_error = slave_goal_euler - slave_euler;

                for(size_t i=0; i<3; i++) {
                    if (last_pos_error[i] > 0) {
                        if (pos_error[i] - last_pos_error[i] > 1) {
                            std::cout << "computing IK1 fail" << std::endl;
                            return false;
                        }
                    } else {
                        if (pos_error[i] - last_pos_error[i] < -1) {
                            std::cout << "computing IK2 fail" << std::endl;

                            return false;
                        }
                    }
                }
                if(last_rot_error_angle > 0){
                    if(rot_error_angle - last_rot_error_angle > 1)
                    {
                        std::cout<<"computing IK3 fail"<<std::endl;
                        return false;
                    }
                }
                else{
                    if(rot_error_angle - last_rot_error_angle < -1)
                    {
                        std::cout<<"computing IK4 fail"<<std::endl;

                        return false;
                    }
                }

            }
        }
    }

}

bool DualCBiRRT::solve_IK_problem_dense_collide_new(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds , PerformanceIndexOneExtend & perdex_one_extend, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot){
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

    //************************************计算距离障碍物最近距离信息**************************************
    const std::set<const robot_model::LinkModel*> planning_link_model = slave_group->getUpdatedLinkModelsSet();

    collision_detection::DistanceResultsData min_distance;
    collision_detection::DistanceMap dis_map;
    double minimum_dis_value;
    Eigen::Vector3d away_from_collision_normal;
    Eigen::Vector3d robot_minimum_dis_point;
    std::string link_name;
    Eigen::MatrixXd minimum_collision_dis_jac;
    Eigen::MatrixXd minimum_collision_dis_jac_pinv;
    Eigen::MatrixXd minimum_collision_dis_jac_up_3;
    Eigen::MatrixXd minimum_collision_dis_jac_up_3_pinv;
    Eigen::Matrix<double, 3, 1> avoid_obstacle_vel;
    const std::vector<std::string> link_names = slave_group->getLinkModelNamesWithCollisionGeometry();
    std::string obstacle_name = "env1_table";
    std::vector<std::pair<std::string, std::string>> obstacle_link_pair_names;
    for (size_t i =0; i<link_names.size() ;i++){
        std::pair<std::string, std::string> tmp(obstacle_name, link_names[i]);
        obstacle_link_pair_names.push_back(tmp);
    }

    //****************************************************************************************************

    //*********************利用 RobotState 得到 master 的位置向量以及欧拉角向量*************************************
    const Eigen::Affine3d & master_end_pose = master_state.getGlobalLinkTransform("left_gripper");
    auto master_end_rot_matrix = master_end_pose.rotation();
    Eigen::Vector3d master_euler = master_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d master_end_pos = master_end_pose.translation();
    //********************************************************************************************************

    //*********************利用 RobotState 得到 slave 的位置向量、欧拉角向量以及雅克比矩阵、雅克比伪逆矩阵*************************************
    const Eigen::Affine3d & slave_end_pose = slave_state.getGlobalLinkTransform("right_gripper");
    auto slave_end_rot_matrix = slave_end_pose.rotation();
    Eigen::Vector3d slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d slave_end_pos = slave_end_pose.translation();
    Eigen::MatrixXd slave_end_jacobian;
    Eigen::MatrixXd slave_end_jacobian_mp_inverse;
    Eigen::MatrixXd slave_end_jacobian_null_space;
    slave_end_jacobian = slave_state.getJacobian(slave_group);


    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************
    //最开始的闭环约束方式
    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] - 3.1415926);
    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(0, 0, _left_right_distance);
    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    //*****************************************************************************************

    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
    Eigen::Vector3d pos_error = slave_goal_pos - slave_end_pos;
    Eigen::Matrix3d rot_error_matrix = slave_end_rot_matrix * slave_goal_rot_matrix.inverse();
    Eigen::AngleAxisd rot_error_axis_angle(rot_error_matrix);
    double rot_error_angle = rot_error_axis_angle.angle();
    Eigen::Vector3d rot_error_3vector = rot_error_angle * rot_error_axis_angle.axis();
    Eigen::Vector3d euler_error = slave_goal_euler - slave_euler;//画图测试一下欧拉角误差和轴角误差

    Eigen::Vector3d last_pos_error = pos_error;
    Eigen::Vector3d last_euler_error = euler_error;
    double last_rot_error_angle = rot_error_angle;
    Eigen::Matrix<double, 6, 1>  stack_error;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    Eigen::Matrix<double, 7, 1> null_space_joint_delta_vector;
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
    bool flag = false;
    Eigen::MatrixXd U;
    Eigen::MatrixXd V;
    Eigen::MatrixXd D;
    Eigen::MatrixXd U1;
    Eigen::MatrixXd V1;
    Eigen::MatrixXd D1;
    while (true){
        count++;
        if(std::abs(pos_error[0]) < 0.01 && std::abs(pos_error[1])< 0.01 && std::abs(pos_error[2])< 0.01 &&  euler_error.norm() < 0.03){
            result_state_value_matrix = slave_state_value_matrix;
            perdex_one_extend.ik_project_times = count;
            return true;
        }
        else{
//            std::cout<<"\n\n"<<std::endl;
            std::cout<<"computing IK "<<count<<std::endl;

            collision_detection::DistanceRequest dis_req;
            collision_detection::DistanceResult dis_res;
            dis_req.group_name = "right_arm";
            dis_req.active_components_only = &planning_link_model;
            dis_req.enable_nearest_points = true;
            dis_req.type = collision_detection::DistanceRequestType::SINGLE;

            std::vector<double> tmp_value;
            slave_state.copyJointGroupPositions(slave_group, tmp_value);
            world_FCL.distanceRobot(dis_req, dis_res, *robot, slave_state);
            min_distance = dis_res.minimum_distance;
            minimum_dis_value = min_distance.distance;
            dis_map = dis_res.distances;
            away_from_collision_normal = min_distance.normal;
            robot_minimum_dis_point = min_distance.nearest_points[1];
            link_name = min_distance.link_names[1];

//            std::cout<<"nearest dis link pairs is "<<min_distance.link_names[0]<<" and "<<min_distance.link_names[1]<<", dis = "<<minimum_dis_value<<std::endl;
//            std::cout<<"first point "<<min_distance.nearest_points[0].transpose()<<" second point "<<min_distance.nearest_points[1].transpose()<<std::endl;

            //                for(size_t i = 0; i < obstacle_link_pair_names.size(); i++){
//                    std::cout<<"dis between "<<obstacle_link_pair_names[i].first<<" and "<<obstacle_link_pair_names[i].second<<" is ";
//                    std::cout<< dis_map[obstacle_link_pair_names[i]][0].distance<<std::endl;
//                }

            if(!(link_name =="")){
                const robot_state::LinkModel* collision_link = slave_state.getLinkModel(link_name);
                if(slave_state.getJacobian(slave_group, collision_link, robot_minimum_dis_point, minimum_collision_dis_jac)){
                }
                else{
                    exit(1);
                }
                minimum_collision_dis_jac_up_3 = minimum_collision_dis_jac.topRows(3);
                minimum_collision_dis_jac_up_3_pinv = minimum_collision_dis_jac_up_3.transpose()*((minimum_collision_dis_jac_up_3 * (minimum_collision_dis_jac_up_3.transpose()) + 0.01 * Eigen::Matrix<double,3,3>::Identity()).inverse());

                double alpha = 0.05/(1+std::exp((2*minimum_dis_value/0.03-1)*4));
                avoid_obstacle_vel = alpha * away_from_collision_normal;
            }
            else{

                minimum_collision_dis_jac_up_3 = Eigen::Matrix<double,3,7>::Zero();
                minimum_collision_dis_jac_up_3_pinv = Eigen::Matrix<double,7,3>::Zero();
                avoid_obstacle_vel = Eigen::Matrix<double,3,1>::Zero();
            }
//                //保存作图数据
//                euler_error_draw.emplace_back(euler_error.transpose());
//                rot_error_angle_draw.emplace_back(rot_error_angle);

            //计算消除末端在任务空间的误差所需要的速度
            rot_error_3vector = rot_error_axis_angle.angle() * rot_error_axis_angle.axis();
            stack_error.head(3) = pos_error;
            stack_error.tail(3) = rot_error_3vector;

            stack_error = (_error_coefficient * stack_error) / 0.01;
            slave_end_jacobian = slave_state.getJacobian(slave_group);

//                Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_end_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
//                U = svd_holder.matrixU();
//                V = svd_holder.matrixV();
//                D = svd_holder.singularValues();
//
//                Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder1(minimum_collision_dis_jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
//                U1 = svd_holder1.matrixU();
//                V1 = svd_holder1.matrixV();
//                D1 = svd_holder1.singularValues();


            slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose() + 0.01 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
            slave_end_jacobian_null_space = Eigen::Matrix<double, 7, 7>::Identity() - slave_end_jacobian_mp_inverse * slave_end_jacobian;


            joint_delta_vector = slave_end_jacobian_mp_inverse * stack_error;

            Eigen::Vector3d closest_point_vel;
            closest_point_vel = minimum_collision_dis_jac_up_3 * joint_delta_vector;
            if(closest_point_vel.dot(away_from_collision_normal)>0){
                ;
            }
            else{
                null_space_joint_delta_vector = slave_end_jacobian_null_space * minimum_collision_dis_jac_up_3_pinv * avoid_obstacle_vel;
                joint_delta_vector += null_space_joint_delta_vector;
            }

            for(size_t i=0; i<7; i++){
                delta_H[i] = -0.25 * max_min_square[i] * (2*slave_state_value_matrix[i]-slave_joint_pos_bounds.first[i]-slave_joint_pos_bounds.second[i]) / (  pow((slave_joint_pos_bounds.first[i]-slave_state_value_matrix[i]),2) * pow((slave_joint_pos_bounds.second[i]-slave_state_value_matrix[i]),2)  );
            }


            joint_delta_vector *= 0.01;

            std::cout<<"joint_delta_vector.norm()   "<<joint_delta_vector.norm()<<std::endl;
            std::cout<<"pos_error"<<pos_error.transpose()<<std::endl;
            std::cout<<"rot_error_angle_error  "<<rot_error_angle<<std::endl;
            if(joint_delta_vector.norm() < 0.01){
                perdex_one_extend.ik_project_times = count;
                std::cout<<"computing IK6 fail"<<std::endl;
                return false;
            }
            else{
                slave_state_value_matrix +=  joint_delta_vector;
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
                    slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);


                    pos_error = slave_goal_pos - slave_end_pos;
                    rot_error_matrix = slave_goal_rot_matrix * slave_end_rot_matrix.inverse();
                    rot_error_axis_angle = rot_error_matrix;
                    rot_error_angle = rot_error_axis_angle.angle();
                    euler_error = slave_goal_euler - slave_euler;

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

bool DualCBiRRT::solve_IK_problem_task_space(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds , PerformanceIndexOneExtend & perdex_one_extend, collision_detection::CollisionWorldFCL & world_FCL, const collision_detection::CollisionRobotConstPtr & robot){
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
    Eigen::Vector3d master_euler = master_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d master_end_pos = master_end_pose.translation();
    //********************************************************************************************************

    //*********************利用 RobotState 得到 slave 的位置向量、欧拉角向量以及雅克比矩阵、雅克比伪逆矩阵*************************************
    const Eigen::Affine3d & slave_end_pose = slave_state.getGlobalLinkTransform("right_gripper");
    auto slave_end_rot_matrix = slave_end_pose.rotation();
    Eigen::Vector3d slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
    Eigen::Vector3d slave_end_pos = slave_end_pose.translation();
    Eigen::MatrixXd slave_end_jacobian;
    Eigen::MatrixXd slave_end_jacobian_mp_inverse;
    slave_end_jacobian = slave_state.getJacobian(slave_group);
    //********************************************************************************************************

    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************
    //最开始的闭环约束方式
    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] - 3.1415926);
    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(0, 0, _left_right_distance);
    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    //*****************************************************************************************

    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
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
        if(std::abs(pos_error[0]) < 0.01 && std::abs(pos_error[1])< 0.01 && std::abs(pos_error[2])< 0.01 &&  std::abs(euler_error[0]) < 0.1 && std::abs(euler_error[1]) < 0.1 && std::abs(euler_error[2]) < 0.1){
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
            slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose() + 0.0001 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
            //计算关节增量
            joint_delta_vector = slave_end_jacobian_mp_inverse * stack_error;
            joint_delta_vector *= 0.01;

//            std::cout<<"joint_delta_vector.norm()  " <<joint_delta_vector.norm() <<std::endl;
            if(joint_delta_vector.norm() < 0.01){
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

                if(slave_state.satisfiesBounds(slave_group, 0.05)){
                    //更新末端误差
                    const Eigen::Affine3d & slave_end_pose_tmp = slave_state.getGlobalLinkTransform("right_gripper");
                    slave_end_rot_matrix = slave_end_pose_tmp.rotation();
                    slave_end_pos = slave_end_pose_tmp.translation();
                    slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
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

const std::vector<std::pair<robot_state::RobotState, size_t>> & DualCBiRRT::get_tree_state_vector(bool if_a_tree){
    if(if_a_tree){
        return _a_rrt_tree_state;
    }
    else{
        return _b_rrt_tree_state;
    }

}