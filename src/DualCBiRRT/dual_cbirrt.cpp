
#include <baxter_moveit_application/DualCBiRRT/dual_cbirrt.h>
#include <limits.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <Eigen/SVD>
#include <math.h>
#define PI 3.1415926

DualCBiRRT::DualCBiRRT(double probability):_random_distribution(probability){
    _random_engine.seed(time(0));
    _step_size = 0.03;
    _constrain_delta = 0.1; //弧度值
    _pitch_min =  1.56999 - 0.00025;
    _pitch_max =  1.56999 + 0.00025 ;
    _yaw_min = 2.94792  - 0.00025;
    _yaw_max = 2.94792  + 0.00025;

    _draw_count = 0;
}

DualCBiRRT::~DualCBiRRT(){}

void DualCBiRRT::sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group) {
    if(_random_distribution(_random_engine)){
        //伯努利分布
        random_state.setToRandomPositions(planning_group);
    }
    else{
        random_state = goal_state;
    }
    std::vector<double> random_state_value;
    random_state.copyJointGroupPositions(planning_group, random_state_value);
    for(size_t i=0; i<random_state_value.size(); i++) {
        random_state_value_matrix[i] = random_state_value[i];
    }
}

size_t DualCBiRRT::near_a_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix) {
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    for(size_t i=0; i<_a_rrt_tree_matrix.size(); i++){
        Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _a_rrt_tree_matrix[i].head(7);
        double distance = dis_matrix.norm();
        if(distance < minum_dis){
            minum_dis = distance;
            minum_index = i;
        }
    }
    nearest_node_matrix = _a_rrt_tree_matrix[minum_index].head(7);
    return minum_index;
}

size_t DualCBiRRT::near_b_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix) {
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    for(size_t i=0; i<_b_rrt_tree_matrix.size(); i++){
        Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _b_rrt_tree_matrix[i].head(7);
        double distance = dis_matrix.norm();
        if(distance < minum_dis){
            minum_dis = distance;
            minum_index = i;
        }
    }
    nearest_node_matrix = _b_rrt_tree_matrix[minum_index].head(7);
    return minum_index;
}

void DualCBiRRT::constraint_extend_a_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds){
    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 6, 1> task_delta_vector;
    task_delta_vector<< 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;

    std::cout<<"new_projecting"<<std::endl;
    std::cout<<"random_matrix"<<std::endl;
    std::cout<<random_state_value_matrix<<std::endl;

    std::cout<<"nearest_node_matrix"<<std::endl;
    std::cout<<nearest_node_matrix<<std::endl;

    std::cout<<"qs_matrix"<<std::endl;
    std::cout<<qs_matrix<<std::endl;

    std::cout<<"(qs_matrix == random_state_value_matrix)"<<std::endl;
    std::cout<<(qs_matrix - random_state_value_matrix).norm()<<std::endl;

    //计算 slave 相关变量
    Eigen::Matrix<double, 7, 1> current_slave_angles_matrix = _a_rrt_tree_matrix[nearst_node_index].tail(7);
    std::cout<<"current_slave_angles_matrix\n"<< current_slave_angles_matrix.transpose()<<std::endl;

    Eigen::Matrix<double, 7, 1> computed_slave_angles_matrix;
    std::vector<double> slave_angles_vector(7);

    Eigen::Matrix<double, 14, 1> matrix_tree_element;


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
                std::cout<<"????"<<std::endl;
                qs_matrix = qs_matrix + direction_matrix * norm;
            }
            std::cout<<"1"<<std::endl;
            std::cout<<qs_matrix<<std::endl;

            //**********************************************向约束空间投影***************************************************
            double yaw_error = 0;
            double pitch_error = 0;
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
            while (true){

                project_count++;
                std::cout<<"projecting"<<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);

                if(end_rot_eulerAngle[1] < _pitch_min){
                    pitch_error = _pitch_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _pitch_max){
                    pitch_error = _pitch_max - end_rot_eulerAngle[1];
                }
                else{
                    pitch_error = 0;
                }
                
                if(end_rot_eulerAngle[0] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[0];
                }
                else if(end_rot_eulerAngle[0] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[0];
                }
                else{
                    yaw_error = 0;
                }

                std::cout<<"pitch_error "<<pitch_error <<std::endl;
                std::cout<<"yaw_error "<<yaw_error <<std::endl;
                std::cout<<"total_error "<<total_error <<std::endl;

                total_error = sqrt(pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2];
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;

//                    std::cout<<"required_eulerAngle"<<std::endl;
//                    std::cout<<required_eulerAngle<<std::endl;

                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector[3] = error_axis[0] * error_angle;
                    task_delta_vector[4] = error_axis[1] * error_angle;
                    task_delta_vector[5] = error_axis[2] * error_angle;
                    task_delta_vector = task_delta_vector / 0.01;

                    end_jacobian = qs_state.getJacobian(planning_group);
//                    std::cout<<"end_jacobian"<<std::endl;
//                    std::cout<<end_jacobian<<std::endl;
                    joint_delta_vector = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse()) * task_delta_vector;

                    std::cout<<"angles change norm"<<std::endl;
                    std::cout<<joint_delta_vector * 0.01<<std::endl;

                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    std::cout<<"1"<<std::endl;
                    std::cout<<qs_matrix<<std::endl;
                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。

                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }

            }
            //如果投影成功，qs_state肯定更新过
            if(project_success){

//                if(planning_scene_ptr->isStateValid(qs_state, planning_group_name)) {
                if(true){

                        if(solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
                            std::cout<<"adding a state"<<std::endl;

                            for(size_t i=0; i<7; i++){
                                slave_angles_vector[i] = computed_slave_angles_matrix[i];
                            }
                            qs_state.setJointGroupPositions(slave_group, slave_angles_vector);

                            //将节点加入树中
                            matrix_tree_element.head(7) = qs_matrix;
                            matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                            _a_rrt_tree_matrix.push_back(matrix_tree_element);
                            current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                            std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                            _a_rrt_tree_state.push_back(tmp);
                            qs_old_index = _a_rrt_tree_state.size() - 1;
                        }

                }
                else{
                    reached_state_matrix = qs_old_matrix;
                    break;
                }
            }
            else{
                reached_state_matrix = qs_old_matrix;
                break;
            }
        }
    }
}

void DualCBiRRT::constraint_extend_b_tree(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr, const robot_state::JointModelGroup* slave_group, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds){
    size_t qs_old_index = nearst_node_index;
    Eigen::Matrix<double, 7, 1> qs_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> direction_matrix;
    Eigen::MatrixXd end_jacobian;
    robot_state::RobotState qs_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState qs_old_state = planning_scene_ptr->getCurrentStateNonConst();
    std::vector<double> qs_vector(7);
    std::vector<double> qs_old_vector(7);
    Eigen::Matrix<double, 6, 1> task_delta_vector;
    task_delta_vector<< 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;

    std::cout<<"new_projecting"<<std::endl;
    std::cout<<"random_matrix"<<std::endl;
    std::cout<<random_state_value_matrix<<std::endl;

    std::cout<<"nearest_node_matrix"<<std::endl;
    std::cout<<nearest_node_matrix<<std::endl;

    std::cout<<"qs_matrix"<<std::endl;
    std::cout<<qs_matrix<<std::endl;

    std::cout<<"(qs_matrix == random_state_value_matrix)"<<std::endl;
    std::cout<<(qs_matrix - random_state_value_matrix).norm()<<std::endl;

    //计算 slave 相关变量
    Eigen::Matrix<double, 7, 1> current_slave_angles_matrix = _b_rrt_tree_matrix[nearst_node_index].tail(7);
    Eigen::Matrix<double, 7, 1> computed_slave_angles_matrix;
    std::vector<double> slave_angles_vector(7);

    Eigen::Matrix<double, 14, 1> matrix_tree_element;


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
                std::cout<<"????"<<std::endl;
                qs_matrix = qs_matrix + direction_matrix * norm;
            }
            std::cout<<"1"<<std::endl;
            std::cout<<qs_matrix<<std::endl;

            //**********************************************向约束空间投影***************************************************
            double yaw_error = 0;
            double pitch_error = 0;
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
            while (true){

                project_count++;
                std::cout<<"projecting"<<std::endl;
                //先计算当前与约束的误差
                for(size_t i=0; i<7; i++){
                    qs_vector[i] = qs_matrix[i];
                }

                qs_state.setJointGroupPositions(planning_group, qs_vector);
                qs_state.update();

                const Eigen::Affine3d end_pose = qs_state.getGlobalLinkTransform("left_gripper");
                auto end_rot_matrix = end_pose.rotation();

                end_rot_eulerAngle = end_rot_matrix.eulerAngles(2,1,0);

                if(end_rot_eulerAngle[1] < _pitch_min){
                    pitch_error = _pitch_min - end_rot_eulerAngle[1];
                }
                else if(end_rot_eulerAngle[1] > _pitch_max){
                    pitch_error = _pitch_max - end_rot_eulerAngle[1];
                }
                else{
                    pitch_error = 0;
                }

                if(end_rot_eulerAngle[0] < _yaw_min){
                    yaw_error = _yaw_min - end_rot_eulerAngle[0];
                }
                else if(end_rot_eulerAngle[0] > _yaw_max){
                    yaw_error = _yaw_max - end_rot_eulerAngle[0];
                }
                else{
                    yaw_error = 0;
                }

                std::cout<<"pitch_error "<<pitch_error <<std::endl;
                std::cout<<"yaw_error "<<yaw_error <<std::endl;
                std::cout<<"total_error "<<total_error <<std::endl;

                total_error = sqrt(pitch_error*pitch_error + yaw_error*yaw_error);
                if(total_error < _constrain_delta){
                    project_success = true;
                    break;
                }
                else{
                    required_eulerAngle[2] = end_rot_eulerAngle[2];
                    required_eulerAngle[1] = end_rot_eulerAngle[1] + pitch_error;
                    required_eulerAngle[0] = end_rot_eulerAngle[0] + yaw_error;

//                    std::cout<<"required_eulerAngle"<<std::endl;
//                    std::cout<<required_eulerAngle<<std::endl;

                    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(required_eulerAngle[2], Eigen::Vector3d::UnitX()));
                    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(required_eulerAngle[1], Eigen::Vector3d::UnitY()));
                    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(required_eulerAngle[0], Eigen::Vector3d::UnitZ()));
                    required_rot_matrix = yaw_angle*pitch_angle*roll_angle;
                    error_rot_matrix = required_rot_matrix * (end_rot_matrix.inverse());
                    error_axis_angle.fromRotationMatrix(error_rot_matrix);
                    error_axis = error_axis_angle.axis();
                    error_angle = error_axis_angle.angle();
                    task_delta_vector[3] = error_axis[0] * error_angle;
                    task_delta_vector[4] = error_axis[1] * error_angle;
                    task_delta_vector[5] = error_axis[2] * error_angle;
                    task_delta_vector = task_delta_vector / 0.01;

                    end_jacobian = qs_state.getJacobian(planning_group);
//                    std::cout<<"end_jacobian"<<std::endl;
//                    std::cout<<end_jacobian<<std::endl;
                    joint_delta_vector = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse()) * task_delta_vector;

                    std::cout<<"angles change norm"<<std::endl;
                    std::cout<<joint_delta_vector * 0.01<<std::endl;

                    qs_matrix = qs_matrix + joint_delta_vector * 0.01;

                    std::cout<<"1"<<std::endl;
                    std::cout<<qs_matrix<<std::endl;
                    //因为本身qs相对于qs_old最大扩展了一个qstep,应该只是想让投影在一个qstep的范围以内，所以不要超过2*qstep，如果超过了说明距离太远，没有投影成功。

                    if ((qs_matrix - qs_old_matrix).norm() > 2*_step_size || !qs_state.satisfiesBounds(planning_group, 0.05)){
                        project_success = false;
                        break;
                    }
                }

            }
            //如果投影成功，qs_state肯定更新过
            if(project_success){

//                if(planning_scene_ptr->isStateValid(qs_state, planning_group_name)) {
                if(true){
                    if(solve_IK_problem(current_slave_angles_matrix, qs_matrix, computed_slave_angles_matrix, planning_group, slave_group, planning_scene_ptr, slave_joint_pos_bounds)){
                        std::cout<<"adding a state"<<std::endl;

                        for(size_t i=0; i<7; i++){
                            slave_angles_vector[i] = computed_slave_angles_matrix[i];
                        }
                        qs_state.setJointGroupPositions(slave_group, slave_angles_vector);

                        //将节点加入树中
                        matrix_tree_element.head(7) = qs_matrix;
                        matrix_tree_element.tail(7) = computed_slave_angles_matrix;
                        _b_rrt_tree_matrix.push_back(matrix_tree_element);
                        current_slave_angles_matrix = computed_slave_angles_matrix; //如果还在这个循环里执行，下一次计算IK的话slave从这个值为初始值开始计算

                        std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                        _b_rrt_tree_state.push_back(tmp);
                        qs_old_index = _b_rrt_tree_state.size() - 1;
                    }

                }
                else{
                    reached_state_matrix = qs_old_matrix;
                    break;
                }
            }
            else{
                reached_state_matrix = qs_old_matrix;
                break;
            }
        }
    }
}

bool DualCBiRRT::plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group){


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
    for(int count=0; count < 100000; count++){
        std::cout<<"count: "<<count<<std::endl;
        //先扩展a树
        if(extend_order){

            sample(goal_state, random_state, random_state_value_matrix, planning_group);

            nearest_node_index = near_a_tree(random_state_value_matrix, nearest_node_matrix);
            constraint_extend_a_tree(random_state_value_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds);
            nearest_node_index = near_b_tree(a_tree_reached_matrix, nearest_node_matrix);
            constraint_extend_b_tree(a_tree_reached_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds);

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

            sample(goal_state, random_state, random_state_value_matrix, planning_group);

            nearest_node_index = near_b_tree(random_state_value_matrix, nearest_node_matrix);
            constraint_extend_b_tree(random_state_value_matrix, nearest_node_matrix, nearest_node_index, b_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds);
            nearest_node_index = near_a_tree(b_tree_reached_matrix, nearest_node_matrix);
            constraint_extend_a_tree(b_tree_reached_matrix, nearest_node_matrix, nearest_node_index, a_tree_reached_matrix, planning_group, planning_group_name, planning_scene_ptr, slave_group, slave_joint_pos_bounds);

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

    }
    return false;
}

bool DualCBiRRT::solve_IK_problem(Eigen::Matrix<double, 7, 1> slave_state_value_matrix, Eigen::Matrix<double, 7, 1> & master_state_value_matrix, Eigen::Matrix<double, 7, 1> & result_state_value_matrix, const robot_state::JointModelGroup* planning_group, const robot_state::JointModelGroup* slave_group, planning_scene::PlanningScenePtr & planning_scene_ptr, std::pair<std::vector<double>, std::vector<double>>& slave_joint_pos_bounds){
    //************************************获取函数参数，当前的master的各个关节值以及slave的各个关节值，存储在 RobotState 中*************************************
    robot_state::RobotState master_state = planning_scene_ptr->getCurrentStateNonConst();//用来保存这次计算所参考的master的状态，函数中不会更改
    robot_state::RobotState slave_state = planning_scene_ptr->getCurrentStateNonConst(); //用来保存计算到的当前的slave的状态，循环中多次更改
    std::vector<double> master_joint_value_vector;
    std::vector<double> slave_joint_value_vector;
    for(size_t i=0; i<7; i++){
        master_joint_value_vector.push_back(master_state_value_matrix[i]);
        slave_joint_value_vector.push_back(slave_state_value_matrix[i]);
    }
    std::cout<<"master_state_value_matrix\n"<<master_state_value_matrix.transpose()<<std::endl;

    master_state.setJointGroupPositions(planning_group, master_joint_value_vector);
    slave_state.setJointGroupPositions(slave_group,slave_joint_value_vector);
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
    std::cout<<"slave_current_euler\n  "<<slave_euler.transpose() <<std::endl;


    //************************************计算 slave 的目标末端位置，欧拉角向量以及旋转矩阵************************************

    Eigen::Vector3d slave_goal_euler(master_euler[0], master_euler[1], master_euler[2] + 3.1415926);
    Eigen::Vector3d slave_goal_pos;
    Eigen::Vector3d distance(0, 0, 0.06);
    std::cout<<"distance\n  "<<distance.transpose() <<std::endl;
    std::cout<<"slave_goal_euler\n  "<<slave_goal_euler.transpose() <<std::endl;
    slave_goal_pos = master_end_rot_matrix * distance + master_end_pos;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    //*****************************************************************************************
    std::cout<<"master_euler\n  "<<master_euler.transpose() <<std::endl;
    std::cout<<"slave_euler\n  "<<slave_euler.transpose() <<std::endl;
    //*********************计算 slave 的目标末端位置，欧拉角向量误差，定义任务空间误差，关节角度增量**********************
    Eigen::Vector3d pos_error = slave_goal_pos - slave_end_pos;
    Eigen::Vector3d rot_error = slave_goal_euler - slave_euler;
    std::cout<<"init_slave_goal_pos\n  "<<slave_goal_pos.transpose() <<std::endl;
    std::cout<<"init_slave_end_pos\n  "<<slave_end_pos.transpose() <<std::endl;
    std::cout<<"init_pos_error\n  "<<pos_error.transpose() <<std::endl;
    std::cout<<"init_rot_error\n  "<<rot_error.transpose() <<std::endl;


    Eigen::Vector3d last_pos_error = pos_error;
    Eigen::Vector3d last_rot_error = rot_error;

    Eigen::Matrix3d rot_error_matrix;
    Eigen::AngleAxisd rot_error_axis_angle;
    Eigen::Vector3d rot_error_3vector;
    Eigen::Matrix<double, 6, 1>  stack_error;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;



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


    int count = 0;
    while (true){
        std::cout<<"computing IK "<<count<<std::endl;
        if(count > 2000){
            std::cout<<"computing IK fail"<<std::endl;
            return false;
        }
        else{
            count+=1;
            if(std::abs(pos_error[0]) < 0.01 && std::abs(pos_error[1])< 0.01 && std::abs(pos_error[2])< 0.01 &&  std::abs(rot_error[0]) < 0.1 && std::abs(rot_error[1]) < 0.1 && std::abs(rot_error[2]) < 0.1)
            {
                result_state_value_matrix = slave_state_value_matrix;
                std::cout<<"computing IK success"<<std::endl;
                std::cout<<"success_slave_euler\n"<<slave_euler.transpose()<<std::endl;
                return true;
            } 
            else{

                //保存作图数据
                pos_error_draw.emplace_back(pos_error.transpose());
                euler_error_draw.emplace_back(rot_error.transpose());
                joint_angles_draw.emplace_back(slave_state_value_matrix.transpose());
//                if(count == 1000){
//                    std::string filenum = std::to_string(_draw_count++);
//                    std::ofstream out1("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/"+filenum+"pos_error_draw.txt");
//                    if(out1){
//                        std::cout<<"Open file success"<<std::endl;
//                    }
//                    else{
//                        std::cout<<"Open file fail"<<std::endl;
//                    }
//                    std::ofstream out2("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/"+filenum+"euler_error_draw.txt");
//                    std::ofstream out3("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/"+filenum+"joint_angles_draw.txt");
//                    std::ofstream out4("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/"+filenum+"joint_bounds.txt");
//                    for (size_t i=0; i<pos_error_draw.size(); i++){
//                        out1<<pos_error_draw[i]<<std::endl;
//                        out2<<euler_error_draw[i]<<std::endl;
//                        out3<<joint_angles_draw[i]<<std::endl;
//                    }
//                    for (size_t i=0; i<slave_joint_pos_bounds.first.size();i++){
//                        if(i == slave_joint_pos_bounds.first.size()-1){
//                            out4 << slave_joint_pos_bounds.first[i] <<std::endl;
//                        }
//                        else{
//                            out4 << slave_joint_pos_bounds.first[i] <<" ";
//                        }
//                    }
//                    for (size_t i=0; i<slave_joint_pos_bounds.second.size();i++){
//                        if(i == slave_joint_pos_bounds.second.size()-1){
//                            out4 << slave_joint_pos_bounds.second[i] <<std::endl;
//                        }
//                        else{
//                            out4 << slave_joint_pos_bounds.second[i] <<" ";
//                        }
//                    }
//                    out1.close();
//                    out2.close();
//                    out3.close();
//                }
                //计算消除末端在任务空间的误差所需要的速度
                rot_error_matrix = slave_goal_rot_matrix * slave_end_rot_matrix.inverse();
                rot_error_axis_angle = rot_error_matrix;
                rot_error_3vector = rot_error_axis_angle.angle() * rot_error_axis_angle.axis();
                stack_error.head(3) = pos_error;
                stack_error.tail(3) = rot_error_3vector;
                std::cout<<"stack_error  "<<stack_error <<std::endl;

                stack_error = (0.5 * stack_error) / 0.01;
                //更新雅克比矩阵
                for(size_t i=0; i<7; i++){
                    slave_joint_value_vector[i] = slave_state_value_matrix[i];
                }
                slave_state.setJointGroupPositions(slave_group, slave_joint_value_vector);

                slave_end_jacobian = slave_state.getJacobian(slave_group);

                Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(slave_end_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::MatrixXd U = svd_holder.matrixU();
                Eigen::MatrixXd V = svd_holder.matrixV();
                Eigen::MatrixXd D = svd_holder.singularValues();
                std::cout<<"singularValues\n  "<<D.transpose() <<std::endl;



                slave_end_jacobian_mp_inverse = (slave_end_jacobian.transpose() * ((slave_end_jacobian * slave_end_jacobian.transpose()).inverse())).eval();

                //计算优化性能指标
                for(size_t i=0; i<7; i++){
                    delta_H[i] = 0.25 * max_min_square[i] * (2*slave_state_value_matrix[i]-slave_joint_pos_bounds.first[i]-slave_joint_pos_bounds.second[i]) / (  pow((slave_joint_pos_bounds.first[i]-slave_state_value_matrix[i]),2) * pow((slave_joint_pos_bounds.second[i]-slave_state_value_matrix[i]),2)  );
                }

                //计算关节增量

//                joint_delta_vector =  (slave_end_jacobian_mp_inverse * stack_error  - (Eigen::Matrix<double, 7, 7>::Identity() - slave_end_jacobian_mp_inverse*slave_end_jacobian) * delta_H) * 0.01;
                joint_delta_vector = (slave_end_jacobian_mp_inverse * stack_error);
                slave_state_value_matrix +=  0.01 * joint_delta_vector;
                std::cout<<"joint_delta_vector\n  "<<0.01 * joint_delta_vector.transpose() <<std::endl;

                std::cout<<"slave_state_value_matrix\n  "<<slave_state_value_matrix.transpose() <<std::endl;

                //更新 slave RobotState, state 更新肯定没有问题
                for(size_t i=0; i<7; i++){
                    slave_joint_value_vector[i] = slave_state_value_matrix[i];
                }
                slave_state.setJointGroupPositions(slave_group, slave_joint_value_vector);
                //更新末端误差
                const Eigen::Affine3d & slave_end_pose_tmp = slave_state.getGlobalLinkTransform("right_gripper");
                slave_end_rot_matrix = slave_end_pose_tmp.rotation();
                slave_euler = slave_end_rot_matrix.eulerAngles(2,1,0);
                slave_end_pos = slave_end_pose_tmp.translation();

                last_pos_error = pos_error;
                last_rot_error = rot_error;
                pos_error = slave_goal_pos - slave_end_pos;
                rot_error = slave_goal_euler - slave_euler;

                std::cout<<"last_pos_error\n  "<<last_pos_error.transpose() <<std::endl;
                std::cout<<"last_rot_error\n  "<<last_rot_error.transpose() <<std::endl;
                std::cout<<"pos_error\n  "<<pos_error.transpose() <<std::endl;
                std::cout<<"rot_error\n  "<<rot_error.transpose() <<std::endl;

                for(size_t i=0; i<3; i++){
                    if(last_pos_error[i] > 0){
                        if(pos_error[i] - last_pos_error[i] > 1)
                        {
                            std::cout<<"computing IK1 fail"<<std::endl;
                            return false;
                        }
                    }
                    else{
                        if(pos_error[i] - last_pos_error[i] < -1)
                        {
                            std::cout<<"computing IK2 fail"<<std::endl;
                            return false;
                        }
                    }
                    if(last_rot_error[i] > 0){
                        if(rot_error[i] - last_rot_error[i] > 1)
                        {
                            std::cout<<"computing IK3 fail"<<std::endl;
                            return false;
                        }
                    }
                    else{
                        if(rot_error[i] - last_rot_error[i] < -1)
                        {
                            std::cout<<"computing IK4 fail"<<std::endl;
                            return false;
                        }
                    }
                }
            }
        }
    }
    
}