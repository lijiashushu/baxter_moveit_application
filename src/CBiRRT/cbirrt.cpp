
#include <baxter_moveit_application/CBiRRT/cbirrt.h>
#include <limits.h>

CRRT::CRRT(double probability):_random_distribution(probability){
    _random_engine.seed(time(0));
    _step_size = 0.03;
    _constrain_delta = 0.3; //弧度值
    _pitch_min =  1.56999 - 0.00025;
    _pitch_max =  1.56999 + 0.00025 ;
    _yaw_min = 2.94792  - 0.00025;
    _yaw_max = 2.94792  + 0.00025;
}

CRRT::~CRRT(){}

void CRRT::sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group) {
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

size_t CRRT::near(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix) {
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    for(size_t i=0; i<_rrt_tree_matrix.size(); i++){
        Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _rrt_tree_matrix[i];
        double distance = dis_matrix.norm();
        if(distance < minum_dis){
            minum_dis = distance;
            minum_index = i;
        }
    }
    nearest_node_matrix = _rrt_tree_matrix[minum_index];
    return minum_index;
}

void CRRT::constraint_extend(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, size_t nearst_node_index, Eigen::Matrix<double, 7, 1> & reached_state_matrix, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr){
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

                if(planning_scene_ptr->isStateValid(qs_state, planning_group_name)) {
                    std::cout<<"adding a state"<<std::endl;
                    std::vector<double> tmptmp;
                    qs_state.copyJointGroupPositions(planning_group, tmptmp);
                    for(std::vector<double>::iterator it=tmptmp.begin(); it!=tmptmp.end();it++){
                        std::cout<<*it<<",";
                    }
                    std::cout<<std::endl;
                    _rrt_tree_matrix.push_back(qs_matrix);
                    std::pair<robot_state::RobotState, size_t> tmp(qs_state, qs_old_index);
                    _rrt_tree_state.push_back(tmp);
                    qs_old_index = _rrt_tree_state.size() - 1;
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



void CRRT::steer(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, double step_size, Eigen::Matrix<double, 7, 1> & new_node_matrix) {
    std::vector<double> random_state_value;
    Eigen::Matrix<double, 7, 1> direction_matrix = random_state_value_matrix - nearest_node_matrix;
    double norm = direction_matrix.norm();
    direction_matrix = direction_matrix / norm;
    if(norm < step_size){
        new_node_matrix = norm * direction_matrix + nearest_node_matrix;
    }
    else{
        new_node_matrix = step_size * direction_matrix + nearest_node_matrix;
    }
}

bool CRRT::collision_check(Eigen::Matrix<double, 7, 1> & new_node_matrix, robot_state::RobotState & new_state, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr) {
    std::vector<double> new_node_value;
    for(size_t i=0; i<new_node_matrix.rows(); i++){
        new_node_value.push_back(new_node_matrix[i]);
    }
    new_state.setJointGroupPositions(planning_group, new_node_value);
    planning_scene_ptr->setCurrentState(new_state);
    return planning_scene_ptr->isStateColliding(planning_group_name);
}

bool CRRT::plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group){
    std::pair<robot_state::RobotState, size_t> init_pair(start_state, -1);
    _rrt_tree_state.push_back(init_pair);


    robot_state::RobotState random_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState new_state = planning_scene_ptr->getCurrentStateNonConst();

    Eigen::Matrix<double, 7, 1> random_state_value_matrix;
    Eigen::Matrix<double, 7, 1> nearest_node_matrix;
    Eigen::Matrix<double, 7, 1> new_node_matrix;
    Eigen::Matrix<double, 7, 1> goal_value_matrix;
    Eigen::Matrix<double, 7, 1> start_value_matrix;
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
    _rrt_tree_matrix.push_back(start_value_matrix);

    std::cout<<"start_value_matrix"<<std::endl;
    std::cout<<start_value_matrix<<std::endl;

    for(int count=0; count < 100000; count++){
        std::cout<<"count: "<<count<<std::endl;
        sample(goal_state, random_state, random_state_value_matrix, planning_group);
        nearest_node_index = near(random_state_value_matrix, nearest_node_matrix);
        constraint_extend(random_state_value_matrix, nearest_node_matrix, nearest_node_index, new_node_matrix, planning_group, planning_group_name, planning_scene_ptr);

        if((new_node_matrix - goal_value_matrix).norm() < 0.05)
        {
            ROS_INFO("Success!!!");
            size_t back_track = _rrt_tree_state.size()-1;
            while(back_track != -1){
                planning_result.push_back(_rrt_tree_state[back_track].first);
                back_track = _rrt_tree_state[back_track].second;
            }
            return true;
        }
    }
    return false;
}

