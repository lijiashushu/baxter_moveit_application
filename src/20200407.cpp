#include <dual_rrt_star/dual_cbirrt_star.h>

#define PI 3.1415926


DualCBiRRT::DualCBiRRT(const ros::NodeHandle &nh, std::string base_frame):
_nh(nh), _visual_tools(base_frame)
{
    //*************获取 moveit 保存的当前的 Planning Scene***************
    _monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    _monitor_ptr->requestPlanningSceneState("/get_planning_scene");
    ros::Duration(1.0).sleep();
    planning_scene_monitor::LockedPlanningSceneRW ps(_monitor_ptr);
    _planning_scene_ptr = ps->diff();
    _planning_scene_ptr->decoupleParent();
    _collision_robot = _planning_scene_ptr->getCollisionRobot(); //机器人的碰撞模型
    _collision_world = _planning_scene_ptr->getCollisionWorld(); //环境碰撞模型
    _robot_model_ptr = _planning_scene_ptr->getRobotModel(); //
    _both_group = _robot_model_ptr->getJointModelGroup("both_arms");
    _planning_group = _robot_model_ptr->getJointModelGroup("left_arm");
    _slave_group = _robot_model_ptr->getJointModelGroup("right_arm");

    //*********************获取机器人的关节限制信息***********************
    const std::vector<std::string>& slave_joint_names = _slave_group->getVariableNames();
    std::vector<double> slave_joint_max_bounds;
    std::vector<double> slave_joint_min_bounds;
    for(size_t i=0; i<slave_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = _robot_model_ptr->getVariableBounds(slave_joint_names[i]);
        slave_joint_max_bounds.push_back(tmp_bounds.max_position_);
        slave_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    _slave_joint_pos_bounds.first = slave_joint_max_bounds;
    _slave_joint_pos_bounds.second = slave_joint_min_bounds;

    const std::vector<std::string>& master_joint_names = _planning_group->getVariableNames();
    std::vector<double> master_joint_max_bounds;
    std::vector<double> master_joint_min_bounds;
    for(size_t i=0; i<master_joint_names.size(); i++){
        const robot_state::VariableBounds tmp_bounds = _robot_model_ptr->getVariableBounds(master_joint_names[i]);
        master_joint_max_bounds.push_back(tmp_bounds.max_position_);
        master_joint_min_bounds.push_back(tmp_bounds.min_position_);
    }
    _master_joint_pos_bounds.first = master_joint_max_bounds;
    _master_joint_pos_bounds.second = master_joint_min_bounds;


    //************将各个机器人的各个 link 的模型组成一个set，作为碰撞检测函数的参数*************************
    for(size_t i=0; i<8; i++) {
        _master_link_model_set.insert(_robot_model_ptr->getLinkModel(_master_link_all_collision_names[i]));
        _slave_link_model_set.insert(_robot_model_ptr->getLinkModel(_slave_link_all_collision_names[i]));
    }

    //****************得到加入了环境中的碰撞物体信息， 记录其在空间的坐标原点，用于计算碰撞点的位置******************************
    std::vector<moveit_msgs::CollisionObject> collision_ob_info_msgs;
    _planning_scene_ptr->getCollisionObjectMsgs(collision_ob_info_msgs);
    std::string ob_name;
    Eigen::Vector3d ob_origin;
    for(size_t i=0; i<collision_ob_info_msgs.size(); i++){
        ob_name = collision_ob_info_msgs[i].id;
        std::cout<<ob_name<<std::endl;
        ob_origin(0) = collision_ob_info_msgs[i].primitive_poses[0].position.x;
        ob_origin(1) = collision_ob_info_msgs[i].primitive_poses[0].position.y;
        ob_origin(2) = collision_ob_info_msgs[i].primitive_poses[0].position.z;
        _collision_object_origin.insert(std::make_pair(ob_name, ob_origin));
    }
}

DualCBiRRT::~DualCBiRRT(){}

void DualCBiRRT::initPara(int seed,  double biProbility, int maxSampleTimes,  double taskStep, size_t constrainIndex,
                          const std::vector<double>& startAngles,
                          const std::vector<double>& goalAngles,
                          bool displayStartState,
                          bool if_manipuGrand,
                          bool if_rrt_star)
{
    //****************设置随机数信息*******************
    _seed = seed;
    _random_engine.seed(seed);
    _random_distribution = std::bernoulli_distribution(biProbility);
    _random_uniform = std::uniform_real_distribution<double>(-1, 1);

    _max_planning_times = maxSampleTimes;
    _test_pose_num = constrainIndex;
    _task_step_size = taskStep;

    _startAngles = startAngles;
    _goalAngles = goalAngles;

    _if_manipuGrad = if_manipuGrand;
    _if_rrt_star = if_rrt_star;

    _planning_result.clear();
    _planning_result_index.clear();
    _planning_result_task_state_vector.clear();
    for(int i=0; i<2; i++){
        _tree_robotstate[i].clear();
        _tree_joints_angle_matrix[i].clear();
        _tree_task_space_vector[i].clear();
    }

    _foundPathFlag = false;
    _waitRelesePtr.clear();

    //************change the robot to the start state in rviz for visualization**************
    if(displayStartState){
        robot_state::RobotState start_state(_robot_model_ptr);
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions("both_arms", _startAngles);
        start_state.update();

        _planning_scene_ptr->setCurrentState(start_state);
        moveit_msgs::PlanningScene planning_scene_msg;
        _planning_scene_ptr->getPlanningSceneMsg(planning_scene_msg);
        planning_scene_msg.is_diff = true;
        ros::WallDuration sleep_t(0.5);
        ros::Publisher planning_scene_diff_publisher = _nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
        while (planning_scene_diff_publisher.getNumSubscribers() < 1)
        {
            sleep_t.sleep();
        }
        planning_scene_diff_publisher.publish(planning_scene_msg);
        ros::Duration(1).sleep();
    }
}


void DualCBiRRT::sample(robot_state::RobotState & goal_state, random_numbers::RandomNumberGenerator& rng){
    robot_state::RobotState random_state(_robot_model_ptr);
    random_state.setToDefaultValues();
    if(_random_distribution(_random_engine)){
        //伯努利分布
        random_state.setToRandomPositions(_planning_group, rng);
    }
    else{
        random_state = goal_state;
    }
    random_state.update();

    std::vector<double> random_state_value;
    random_state.copyJointGroupPositions(_planning_group, random_state_value);
    for(size_t i=0; i<random_state_value.size(); i++) {
        _random_state_value_matrix[i] = random_state_value[i];
    }
    _random_state_vector = get_task_state_vector(random_state, true);
    //采样到的末端状态的 yaw 角会受限制
    if(_random_state_vector(3) > _yaw_max[_test_pose_num]){
        _random_state_vector(3) = _yaw_max[_test_pose_num];
    }
    else if(_random_state_vector(3) < _yaw_min[_test_pose_num]){
        _random_state_vector(3) = _yaw_min[_test_pose_num];
    }
    else{;}
}

void DualCBiRRT::sampleEllipsoid(double info_cBest){
    Eigen::Vector4d dirVector;
    for(int i = 0; i<4; i++){
        dirVector(i) =  _random_uniform(_random_engine);
    }
    dirVector.normalize();
    double scale = std::abs(_random_uniform(_random_engine));
    Eigen::Vector4d xBall = scale * dirVector;

    Eigen::Matrix4d info_L_M = Eigen::Matrix4d::Zero();
    info_L_M(0, 0) = info_cBest / 2.0;
    info_L_M(1, 1) = std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin) /2.0;
    info_L_M(2, 2) = std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin) /2.0;
    info_L_M(3, 3) = std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin) /2.0;

//    std::cout<<"_info_C_M"<<std::endl;
//    std::cout<<_info_C_M<<std::endl;
//    std::cout<<"info_L_M"<<std::endl;
//    std::cout<<info_L_M<<std::endl;
//    std::cout<<"xBall"<<std::endl;
//    std::cout<<xBall.transpose()<<std::endl;
//    std::cout<<"_info_centre"<<std::endl;
//    std::cout<<_info_centre.transpose()<<std::endl;

    _random_state_vector = _info_C_M * info_L_M * xBall + _info_centre;
//    std::cout<<"_random_state_vector  "<<_random_state_vector.transpose()<<std::endl;
}

void  DualCBiRRT::nearest(bool if_tree_a, std::vector<flann::Index<flann::L2<double>>>& flann_index)
{
    flann::Matrix<double > querry(new double[4], 1, 4);
    flann::Matrix<int > indices(new int[1], 1, 1);
    flann::Matrix<double > dist(new double[1], 1, 1);
    for(int i=0; i<4; i++){
        querry[0][i] = _random_state_vector[i];
    }
    flann_index[if_tree_a].knnSearch(querry, indices, dist, 1, flann::SearchParams());

    _nearest_index = indices[0][0];
    delete querry.ptr();
    delete indices.ptr();
    delete dist.ptr();
}

int DualCBiRRT::extend(bool if_tree_a, bool if_sample, double& totalExtendTime, long& totalExtendCounts,
                       std::vector<flann::Index<flann::L2<double>>>& flann_index)
{

    int reached_index = _nearest_index;

    robot_state::RobotState qs_state = _planning_scene_ptr->getCurrentStateNonConst();


    Eigen::Vector4d qs_task_state_vector;
    Eigen::Matrix<double, 7, 1> qs_matrix;
    Eigen::Vector4d qs_slave_task_state;
    Eigen::Matrix<double, 7, 1> qs_slave_matrix;


    qs_state = _tree_robotstate[if_tree_a][_nearest_index].first;
    qs_task_state_vector = _tree_task_space_vector[if_tree_a][_nearest_index].first;
    qs_matrix = _tree_joints_angle_matrix[if_tree_a][_nearest_index].first.head(7);
    qs_slave_matrix = _tree_joints_angle_matrix[if_tree_a][_nearest_index].first.tail(7);
    qs_slave_task_state = _tree_task_space_vector[if_tree_a][_nearest_index].second;


    robot_state::RobotState qs_old_state = qs_state;
    Eigen::Vector4d qs_old_task_state_vector = qs_task_state_vector;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = qs_matrix;
    Eigen::Vector4d qs_old_slave_task_state = qs_slave_task_state;
    Eigen::Matrix<double, 7, 1> qs_old_slave_matrix = qs_slave_matrix;

    //4-dimension goal state in one extension
    Eigen::Vector4d extend_goal_task_state_vector;
    Eigen::Vector4d extend_direction_vector;

    //four flags for indicating substeps states in one extension
    bool master_ik_flag = true;
    bool slave_ik_flag = true;
    bool master_collide_flag = true;
    bool slave_collide_flag = true;

    std::chrono::high_resolution_clock::time_point startExtendTime = std::chrono::high_resolution_clock::now();

    int extend_num = 0;
    //adding a state when extend enough length
    while (true){
//        ROS_INFO("extending");
        extend_num++;
        if(if_sample){
            //这里就是让另一个棵树在尝试连接时，最多只尝试扩展10次，保持更多的可能性。
            if(extend_num > 10){
                totalExtendTime += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-startExtendTime).count();
                return reached_index;
            }
        }
        if ((_random_state_vector - qs_task_state_vector).norm() < 0.005){
            totalExtendTime += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-startExtendTime).count();
            return reached_index;
        }
        else if((_random_state_vector - qs_task_state_vector).norm() > (_random_state_vector - qs_old_task_state_vector).norm()){

            //因为在加入节点的时候，qs_old_index 会更新为了下一次的扩展，所以这里需要找到上一个index
            reached_index = _tree_robotstate[if_tree_a][reached_index].second;
            _tree_joints_angle_matrix[if_tree_a].pop_back();
            _tree_robotstate[if_tree_a].pop_back();
            _tree_task_space_vector[if_tree_a].pop_back();

            totalExtendTime += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-startExtendTime).count();
            return reached_index;
        }
        else{

            qs_old_state = qs_state;
            qs_old_task_state_vector = qs_task_state_vector;
            qs_old_matrix = qs_matrix;
            qs_old_slave_task_state = qs_slave_task_state;
            qs_old_slave_matrix = qs_slave_matrix;

            //构建 master 的任务空间目标
            extend_direction_vector = _random_state_vector - qs_task_state_vector;
            double norm = extend_direction_vector.norm();
            extend_direction_vector = extend_direction_vector / norm;
            if(_task_step_size <= norm){
                extend_goal_task_state_vector = qs_task_state_vector + extend_direction_vector * _task_step_size;
            }
            else{
                extend_goal_task_state_vector = qs_task_state_vector + extend_direction_vector * norm;
            }

            KDL::Rotation master_goal_rot_kdl = KDL::Rotation::EulerZYX(extend_goal_task_state_vector(3), 0.0, 1.57);
            master_ik_flag = solveIK(extend_goal_task_state_vector, master_goal_rot_kdl, qs_state, qs_task_state_vector, qs_matrix, true);
            //如果投影成功，qs_state肯定更新过
            if(master_ik_flag){
//                ROS_INFO("master IK success");
                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult res;
                req.group_name = "left_arm";
                _collision_world->checkRobotCollision(req, res, *_collision_robot, qs_state);
                master_collide_flag = !(res.collision);

                if(master_collide_flag){
//                    ROS_INFO("master collide success");
                    //计算闭环约束，得到slave的目标姿态
                    KDL::Vector distance(_left_right_distance_x[_test_pose_num], 0, _left_right_distance_z[_test_pose_num]);
                    KDL::Rotation slave_goal_rot_kdl = KDL::Rotation::EulerZYX(extend_goal_task_state_vector(3) - _left_right_euler_distance[_test_pose_num], 0.0, -1.57);
                    KDL::Vector distance_world = master_goal_rot_kdl * distance;
                    Eigen::Vector4d slave_goal_vector(distance_world(0) + extend_goal_task_state_vector(0),
                                                      distance_world(1) + extend_goal_task_state_vector(1),
                                                      distance_world(2) + extend_goal_task_state_vector(2),
                                                      extend_goal_task_state_vector(3) - _left_right_euler_distance[_test_pose_num]);


                    slave_ik_flag = solveIK(slave_goal_vector, slave_goal_rot_kdl, qs_state, qs_slave_task_state, qs_slave_matrix, false);
                    if(slave_ik_flag){
//                        ROS_INFO("slave IK success");
                        collision_detection::CollisionRequest reqSlave;
                        reqSlave.group_name = "right_arm";
                        collision_detection::CollisionResult resSlave;
                        _collision_world->checkRobotCollision(reqSlave, resSlave, *_collision_robot, qs_state);
                        slave_collide_flag = (!resSlave.collision);

                        if(slave_collide_flag){
//                            ROS_INFO("slave collide success");
//                            ROS_INFO("adding a state");
                            totalExtendCounts++;

                            if(_if_rrt_star){
//                            if(extend_num % _extend_step_num == 0 || (qs_task_state_vector - _random_state_vector).norm() < 0.01){
                                if(true) {

                                    //find posible parents
                                    flann::Matrix<double> querry(new double[1 * 4], 1, 4);
                                    std::vector<std::vector<int>> indices; //outer indicates each querry, inner indicates founded points
                                    std::vector<std::vector<double>> dist;
                                    for (int i = 0; i < 4; i++) {
                                        querry[0][i] = qs_task_state_vector(i);
                                    }

                                    int minIndex = 0;
                                    double minCost = 0;
                                    int treeIndex;
                                    //                                    std::cout<<"qs_task_state "<<qs_task_state_vector.transpose()<<std::endl;
                                    //                                    std::cout<<"reached  "<<_a_rrt_tree_task_space_vector[reached_index].first.transpose()<<std::endl;
                                    //                                    std::cout<<"norm "<<(qs_task_state_vector - _a_rrt_tree_task_space_vector[reached_index].first).norm()<<"\n\n"<<std::endl;
                                    //***********************************ChooseParent***********************************
                                    flann_index[if_tree_a].radiusSearch(querry, indices, dist, 0.01, flann::SearchParams());
//                                    std::cout<<"nearest size  "<<dist[0].size()<<std::endl;
                                    for(int i = 0; i<dist[0].size(); i++){
                                        dist[0][i] = std::sqrt(dist[0][i]);
                                    }


                                    std::vector<int> extendCheckRecord(indices[0].size(), 0); //0 untested 1 feasible 2 infeasible
                                    int curParentIndexIndex =  std::find(indices[0].begin(), indices[0].end(), reached_index) -  indices[0].begin();
                                    minIndex = indices[0][curParentIndexIndex];
                                    minCost = dist[0][curParentIndexIndex] + _tree_joints_angle_matrix[if_tree_a][indices[0][curParentIndexIndex]].second;
                                    extendCheckRecord[curParentIndexIndex] = 1;

                                    std::vector<std::pair<double, int>> feasibleCheckStack; //只有比当前的cost小的才进栈
                                    
                                    for (int i = 0; i < indices[0].size(); i++) {
                                        treeIndex = indices[0][i];
                                        if (minCost > dist[0][i] + _tree_joints_angle_matrix[if_tree_a][treeIndex].second) {
                                            //小步长直接连接
//                                            minCost = dist[0][i] + _tree_joints_angle_matrix[if_tree_a][treeIndex].second;
//                                            minIndex = treeIndex;
                                            //大步长需要判断

                                            if (extend_check(if_tree_a, qs_task_state_vector, treeIndex)) {
                                                std::cout<<"check success!"<<std::endl;
                                                minCost = dist[0][i] + _tree_joints_angle_matrix[if_tree_a][treeIndex].second;
                                                minIndex = treeIndex;
                                                extendCheckRecord[i] = 1;
                                            } else {
                                                extendCheckRecord[i] = 2;
                                            }
                                        }
                                    }
                                    //***********************************Add State***********************************
                                    reached_index = addState(qs_matrix, qs_slave_matrix, qs_task_state_vector,
                                                             qs_slave_task_state, qs_state, minIndex, if_tree_a,
                                                             minCost);
                                    flann_index[if_tree_a].addPoints(querry);
                                    _waitRelesePtr.push_back(querry.ptr());

                                    //***********************************Rewire***********************************
                                    for (int i = 0; i < indices[0].size(); i++) {
                                        treeIndex = indices[0][i];
                                        if (_tree_joints_angle_matrix[if_tree_a][treeIndex].second > dist[0][i] + _tree_joints_angle_matrix[if_tree_a][reached_index].second) {
//                                            _tree_joints_angle_matrix[if_tree_a][treeIndex].second = dist[0][i] + _tree_joints_angle_matrix[if_tree_a][reached_index].second;
//                                            _tree_robotstate[if_tree_a][treeIndex].second = reached_index;

                                            if (extendCheckRecord[i] == 1) {
                                                _tree_joints_angle_matrix[if_tree_a][treeIndex].second = dist[0][i] + _tree_joints_angle_matrix[if_tree_a][reached_index].second;
                                                _tree_robotstate[if_tree_a][treeIndex].second = reached_index;
                                            }
                                            else if (extendCheckRecord[i] == 0) {
                                                if (extend_check(if_tree_a, qs_task_state_vector, treeIndex)) {
                                                    _tree_joints_angle_matrix[if_tree_a][treeIndex].second = dist[0][i] + _tree_joints_angle_matrix[if_tree_a][reached_index].second;
                                                    _tree_robotstate[if_tree_a][treeIndex].second = reached_index;
                                                }
                                            }
                                        }
                                    }
                                }
                            }//_if_rrt_star
                            else{
                                double cost = (qs_task_state_vector - _tree_task_space_vector[if_tree_a][reached_index].first).norm() + _tree_joints_angle_matrix[if_tree_a][reached_index].second;
                                reached_index = addState(qs_matrix, qs_slave_matrix, qs_task_state_vector, qs_slave_task_state, qs_state, reached_index, if_tree_a, cost);
                                flann::Matrix<double> newPoint(new double[1 * 4], 1, 4);
                                for (int i = 0; i < 4; i++) {
                                    newPoint[0][i] = qs_task_state_vector(i);
                                }
                                flann_index[if_tree_a].addPoints(newPoint);
                                _waitRelesePtr.push_back(newPoint.ptr());
                            }
                        }
                    }
                }
            }
            //如果正常的扩展出了一个新的节点，这些标志位都会被置为 true
            if(!(master_ik_flag && slave_ik_flag && master_collide_flag && slave_collide_flag)){
                totalExtendTime += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-startExtendTime).count();
                return reached_index;
            }
        }
    }//end while
}

bool DualCBiRRT::extend_check(bool if_tree_a, const Eigen::Vector4d& goal_state_vec, int try_index){
    Eigen::Vector4d masterWorkStateVec;
    Eigen::Vector4d slaveWorkStateVec;
    Eigen::Matrix<double,7,1> masterWorkJointAngles;
    Eigen::Matrix<double,7,1> slaveWorkJointAngles;

    robot_state::RobotState curState = _tree_robotstate[if_tree_a][try_index].first;
    masterWorkStateVec = _tree_task_space_vector[if_tree_a][try_index].first;
    slaveWorkStateVec = _tree_task_space_vector[if_tree_a][try_index].second;
    masterWorkJointAngles = _tree_joints_angle_matrix[if_tree_a][try_index].first.head(7);
    slaveWorkJointAngles = _tree_joints_angle_matrix[if_tree_a][try_index].first.tail(7);


    Eigen::Vector4d extendDir = (goal_state_vec - masterWorkStateVec).normalized();
    Eigen::Vector4d masterGoalStateVec;
    KDL::Rotation masterGoalRotKdl;
    Eigen::Vector4d slaveGoalStateVec;
    KDL::Rotation slaveGoalRotKdl;

    KDL::Vector distance(_left_right_distance_x[_test_pose_num], 0, _left_right_distance_z[_test_pose_num]);
    double error_norm;

    while(true){
        error_norm = (goal_state_vec - masterWorkStateVec).norm();
        if(error_norm < 0.005){
            return true;
        }
        else if(error_norm >= 0.005 && error_norm < _task_step_size){
            masterGoalStateVec = goal_state_vec;
            masterGoalRotKdl = KDL::Rotation::EulerZYX(masterGoalStateVec(3), 0.0, 1.57);
        }
        else{
            masterGoalStateVec = masterWorkStateVec + _task_step_size * extendDir;
            masterGoalRotKdl = KDL::Rotation::EulerZYX(masterGoalStateVec(3), 0.0, 1.57);
        }

        if(solveIK(masterGoalStateVec, masterGoalRotKdl, curState, masterWorkStateVec, masterWorkJointAngles, true)){
            collision_detection::CollisionRequest masterReq;
            collision_detection::CollisionResult masterRes;
            masterReq.group_name = "left_arm";
            _collision_world->checkRobotCollision(masterReq, masterRes, *_collision_robot, curState);
            if(!masterRes.collision){
                slaveGoalRotKdl = KDL::Rotation::EulerZYX(masterGoalStateVec(3) - _left_right_euler_distance[_test_pose_num], 0.0, -1.57);
                KDL::Vector tmp = masterGoalRotKdl * distance;
                slaveGoalStateVec(0) = tmp(0) + masterGoalStateVec(0);
                slaveGoalStateVec(1) = tmp(1) + masterGoalStateVec(1);
                slaveGoalStateVec(2) = tmp(2) + masterGoalStateVec(2);
                slaveGoalStateVec(3) = masterGoalStateVec(3) - _left_right_euler_distance[_test_pose_num];
                if(solveIK(slaveGoalStateVec, slaveGoalRotKdl, curState, slaveWorkStateVec, slaveWorkJointAngles, false)){
                    collision_detection::CollisionRequest slaveReq;
                    collision_detection::CollisionResult slaveRes;
                    slaveReq.group_name = "right_arm";
                    _collision_world->checkRobotCollision(slaveReq, slaveRes, *_collision_robot, curState);
                    if(!slaveRes.collision){
                        ;
                    }
                    else{
                        ROS_INFO("check slave collide");
                        return false;
                    }
                }
                else{
                    ROS_INFO("check slave IK fail");
                    return false;
                }
            }
            else{
                ROS_INFO("check master collide");
                return false;
            }
        }
        else{
            ROS_INFO("check master IK fail");
            return false;
        }
    }
}

bool DualCBiRRT::solveIK(const Eigen::Vector4d & goal_vector,
                         const KDL::Rotation & goal_rot,
                         robot_state::RobotState &qs_state,
                         Eigen::Vector4d &qs_task_state_vector,
                         Eigen::Matrix<double, 7, 1> &qs_matrix,
                         bool is_master)
{
    const robot_state::JointModelGroup* group;
    std::string gripper_name;
    if(is_master){
        group = _planning_group;
        gripper_name = "left_gripper";
    }
    else{
        group = _slave_group;
        gripper_name = "right_gripper";
    }

    //当前位置及姿态
    Eigen::MatrixXd rot_M = qs_state.getGlobalLinkTransform(gripper_name).rotation();
    KDL::Rotation qs_rot_kdl;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            qs_rot_kdl(i, j) = rot_M(i, j);
        }
    }
    Eigen::Vector4d state_error;
    Eigen::Vector3d pos_error;
    KDL::Rotation rot_error;
    KDL::Vector axis_error;
    double angle_error;

    //将任务空间的误差通过雅克比伪逆矩阵转换到关节空间的变量
    Eigen::Matrix<double, 6, 1> task_delta_vector;
    Eigen::Matrix<double, 7, 1> joint_delta_vector;
    Eigen::MatrixXd end_jacobian;
    Eigen::MatrixXd end_jacobian_pinv;

    //梯度投影法，零空间
    Eigen::Matrix<double, 7, 1> gradientProj = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::MatrixXd end_jacobian_null_space;
    double nullspace_scale_factor = 0.05;
    Eigen::Matrix<double, 7, 1> gradient;

    int project_count = 0;
    while (project_count<10){
        project_count++;
//        std::cout<<"projecting "<< project_count <<std::endl;
//        std::cout<<"seed: "<<_seed<<std::endl;
        //先计算当前与约束的误差判断是否需要继续进行投影操作。

        state_error = goal_vector - qs_task_state_vector;
        pos_error = state_error.head(3);
        rot_error = goal_rot * qs_rot_kdl.Inverse();
        angle_error = rot_error.GetRotAngle(axis_error);
        if((state_error.norm() < 0.005 &&  fabs(angle_error) < 0.005 )){
            return true;
        }
        else{
            //构建任务空间的增量
            task_delta_vector.head(3) = pos_error;
            task_delta_vector(3) = axis_error(0) * angle_error;
            task_delta_vector(4) = axis_error(1) * angle_error;
            task_delta_vector(5) = axis_error(2) * angle_error;

            task_delta_vector = _error_coefficient * task_delta_vector / 0.01;
            //计算关节空间的增量
            end_jacobian = qs_state.getJacobian(group);
            end_jacobian_pinv = end_jacobian.transpose() * ((end_jacobian * end_jacobian.transpose()).inverse());

            if(_if_manipuGrad){
                end_jacobian_null_space = Eigen::Matrix<double, 7, 7>::Identity() - end_jacobian_pinv * end_jacobian;
                gradient = manipuGradient(qs_state, is_master);
                gradientProj = end_jacobian_null_space * gradient;
            }

            joint_delta_vector =  end_jacobian_pinv  * task_delta_vector;
            qs_matrix = qs_matrix + joint_delta_vector * 0.01 + nullspace_scale_factor * gradientProj;

            //更新当前的RobotState
            qs_state.setJointGroupPositions(group, qs_matrix);
            qs_state.update();
            if (!qs_state.satisfiesBounds(group, 0.05)){
                return false;
            }
            else{
                qs_task_state_vector = get_task_state_vector(qs_state, is_master);
                rot_M = qs_state.getGlobalLinkTransform(gripper_name).rotation();
                for(int i=0; i<3; i++){
                    for(int j=0; j<3; j++){
                        qs_rot_kdl(i, j) = rot_M(i, j);
                    }
                }
            }
        }
    }
    return false;
}

bool DualCBiRRT::plan(){
    double  tmp1, tmp3, tmp4;
    int tmp2;
    return plan(tmp1, tmp2, tmp3, tmp4);
}

bool DualCBiRRT::plan(double &panningTime, int &sampleCounts, double &avgExtendTime, double &avgExtendCountPerSample) {
    random_numbers::RandomNumberGenerator rng(_seed);

    robot_state::RobotState start_state(_robot_model_ptr);
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions("both_arms", _startAngles);
    start_state.update();
    robot_state::RobotState goal_state(_robot_model_ptr);
    goal_state.setToDefaultValues();
    goal_state.setJointGroupPositions("both_arms", _goalAngles);
    goal_state.update();

    _tree_robotstate[0].push_back(std::make_pair(start_state, -1));
    _tree_robotstate[1].push_back(std::make_pair(goal_state, -1));

    _tree_task_space_vector[0].push_back(std::make_pair(get_task_state_vector(start_state, true), get_task_state_vector(start_state, false)));
    _tree_task_space_vector[1].push_back(std::make_pair(get_task_state_vector(goal_state, true), get_task_state_vector(goal_state, false)));

    _tree_joints_angle_matrix[0].push_back(std::make_pair(get_joint_angles_matrix(start_state), 0));
    _tree_joints_angle_matrix[1].push_back(std::make_pair(get_joint_angles_matrix(goal_state), 0));

    int tree_reached_index[2];
    int last_tree_size[2] = {1,1};

    bool extendOrder = false; //false indicates 0 tree_a

    std::chrono::high_resolution_clock::time_point startPlanningTime = std::chrono::high_resolution_clock::now();
    long totalExtendCounts = 0;
    double totalExtendTime = 0;
    std::cout<<"pose: "<<_test_pose_num<<"  seed： "<<_seed<<std::endl;


    //建立KD tree
    std::vector<flann::Index<flann::L2<double>>> flannIndex;
    flann::Matrix<double> a_initData(new double[1 * 4], 1, 4);
    flann::Matrix<double> b_initData(new double[1 * 4], 1, 4);
    for(int i=0; i<4; i++){
        a_initData[0][i] = _tree_task_space_vector[0][0].first(i);
        b_initData[0][i] = _tree_task_space_vector[1][0].first(i);
    }
    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<double>> a_index(a_initData, flann::KDTreeSingleIndexParams());
    flann::Index<flann::L2<double>> b_index(b_initData, flann::KDTreeSingleIndexParams());
    a_index.buildIndex();
    b_index.buildIndex();
    flannIndex.push_back(a_index);
    flannIndex.push_back(b_index);
    _waitRelesePtr.push_back(a_initData.ptr());
    _waitRelesePtr.push_back(b_initData.ptr());

    double pathCost = 0;
    int back_track[2];

    for(int count=1; count < _max_planning_times; count++){
        std::cout<<count<<std::endl;
        if(!_foundPathFlag){
            sample(goal_state, rng);
        }
        else{
            sampleEllipsoid(pathCost);
        }

        //*****************************extend one tree**********************************
        nearest(extendOrder, flannIndex);
        tree_reached_index[extendOrder] = extend(extendOrder, true, totalExtendTime, totalExtendCounts, flannIndex);
        _random_state_vector = _tree_task_space_vector[extendOrder][tree_reached_index[extendOrder]].first; //update _random_state_vector for the other tree
        //*****************************extend the other tree**********************************
        nearest(!extendOrder, flannIndex);
        tree_reached_index[!extendOrder] = extend(!extendOrder, false, totalExtendTime, totalExtendCounts, flannIndex);

        extendOrder = !extendOrder;

        if((_tree_task_space_vector[0][tree_reached_index[0]].first - _tree_task_space_vector[1][tree_reached_index[1]].first).norm() < 0.005)
        {
//            ROS_INFO("Success!!!");
//            std::cout<<"Path Cost: "<<_tree_joints_angle_matrix[0][tree_reached_index[0]].second + _tree_joints_angle_matrix[1][tree_reached_index[1]].second<<std::endl;
            //先添加前半部分路径点
            if(!_foundPathFlag){
                _foundPathFlag = true;
                pathCost = _tree_joints_angle_matrix[0][tree_reached_index[0]].second + _tree_joints_angle_matrix[1][tree_reached_index[1]].second;
                back_track[0] = tree_reached_index[0];
                back_track[1] = tree_reached_index[1];
                if(!_if_rrt_star){
                    break;
                }
                //initialize informed sample parameter
                _info_centre = (_tree_task_space_vector[0][0].first - _tree_task_space_vector[1][0].first) * 0.5;
                _info_cMin = (_tree_task_space_vector[0][0].first - _tree_task_space_vector[1][0].first).norm();

                Eigen::Vector4d info_a1 = (_tree_task_space_vector[0][0].first - _tree_task_space_vector[1][0].first).normalized();
                Eigen::Vector4d info_i1 = Eigen::Vector4d::Zero();
                info_i1(0) = 1;
                Eigen::Matrix4d info_M_M = info_a1 * info_i1.transpose();
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(info_M_M,  Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix4d info_tmp = Eigen::Matrix4d::Identity();
                info_tmp(3, 3) = svd.matrixU().determinant()*svd.matrixV().determinant();
                _info_C_M = svd.matrixU() * info_tmp * svd.matrixV().transpose();
            }
            else{
                if(pathCost > _tree_joints_angle_matrix[0][tree_reached_index[0]].second + _tree_joints_angle_matrix[1][tree_reached_index[1]].second){
                    pathCost = _tree_joints_angle_matrix[0][tree_reached_index[0]].second + _tree_joints_angle_matrix[1][tree_reached_index[1]].second;
                    back_track[0] = tree_reached_index[0];
                    back_track[1] = tree_reached_index[1];
                }
            }
        }
    }//end while

    //***********return 前的处理****************
    for(auto ptr:_waitRelesePtr){
        delete ptr;
    }
    avgExtendTime = totalExtendTime / totalExtendCounts;
    avgExtendCountPerSample = static_cast<double>(totalExtendCounts) / (_max_planning_times + 1);
    panningTime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - startPlanningTime).count();
    std::cout<<"panningTime: "<<panningTime<<std::endl;
    std::cout<<"pathCost: "<<pathCost<<std::endl;
    if(_foundPathFlag){
        while(back_track[0] != -1){
            _planning_result.push_back(_tree_robotstate[0][back_track[0]].first);
            _planning_result_index.push_back(_tree_robotstate[0][back_track[0]].second);
            _planning_result_task_state_vector.push_back(_tree_task_space_vector[0][back_track[0]]);
            back_track[0] = _tree_robotstate[0][back_track[0]].second;
        }
        std::reverse(_planning_result.begin(), _planning_result.end());
        std::reverse(_planning_result_index.begin(), _planning_result_index.end());
        std::reverse(_planning_result_task_state_vector.begin(), _planning_result_task_state_vector.end());

        //添加后半部分路径点
        while(back_track[1] != -1){
            _planning_result.push_back(_tree_robotstate[1][back_track[1]].first);
            _planning_result_index.push_back(_tree_robotstate[1][back_track[1]].second);
            _planning_result_task_state_vector.push_back(_tree_task_space_vector[1][back_track[1]]);
            back_track[1] = _tree_robotstate[1][back_track[1]].second;
        }
        return true;
    }
    else{
        return false;
    }
}

void DualCBiRRT::showPath(){
    _visual_tools.deleteAllMarkers();
    _visual_tools.trigger();

    std::vector<geometry_msgs::Pose> result_pose;
    moveit_msgs::RobotTrajectory result_msg;
    trajectory_msgs::JointTrajectory path_point_msg;
    trajectory_msgs::JointTrajectoryPoint path_point_position_msg; //只需要添加关节位置点
    for(size_t i=0; i<_planning_result.size(); i++){
        std::vector<double> tmp1;
        std::vector<double> tmp2;
        _planning_result[i].copyJointGroupPositions(_planning_group, tmp1);
        _planning_result[i].copyJointGroupPositions(_slave_group, tmp2);
        tmp1.insert(tmp1.end(), tmp2.begin(), tmp2.end());
        path_point_position_msg.positions = tmp1;
        path_point_msg.points.push_back(path_point_position_msg);

        //就这一部分是添加姿态
//        if(i%5==0) {
//            geometry_msgs::Pose tmp_pose_msg;
//            const Eigen::Affine3d end_pose = _planning_result[i].getGlobalLinkTransform("left_gripper");
//            Eigen::Quaterniond end_quaternion(end_pose.rotation());
//            tmp_pose_msg.position.x = end_pose(0, 3);
//            tmp_pose_msg.position.y = end_pose(1, 3);
//            tmp_pose_msg.position.z = end_pose(2, 3);
//            tmp_pose_msg.orientation.x = end_quaternion.x();
//            tmp_pose_msg.orientation.y = end_quaternion.y();
//            tmp_pose_msg.orientation.z = end_quaternion.z();
//            tmp_pose_msg.orientation.w = end_quaternion.w();
//            result_pose.push_back(tmp_pose_msg);
//        }
    }
    //*********************显示扩展成功的 master 的末端位置点*****************************
    ROS_INFO("a_tree_size: %ld", _tree_task_space_vector[0].size());
    ROS_INFO("b_tree_size: %ld", _tree_task_space_vector[1].size());

    std::vector<geometry_msgs::Point> extend_states_display_a_tree;
    std::vector<std_msgs::ColorRGBA> extend_states_display_color_a_tree;
    std::vector<geometry_msgs::Point> extend_states_display_b_tree;
    std::vector<std_msgs::ColorRGBA> extend_states_display_color_b_tree;
    for(auto i:_tree_task_space_vector[0]){
        geometry_msgs::Point p;
        p.x = i.first[0];
        p.y = i.first[1];
        p.z = i.first[2];
        extend_states_display_a_tree.push_back(p);
        std_msgs::ColorRGBA c;
        c.b = 1.0;
        c.a = 1.0;
        extend_states_display_color_a_tree.push_back(c);
    }
    for(auto i:_tree_task_space_vector[1]){
        geometry_msgs::Point p;
        p.x = i.first[0];
        p.y = i.first[1];
        p.z = i.first[2];
        extend_states_display_b_tree.push_back(p);
        std_msgs::ColorRGBA c;
        c.r = 1.0;
        c.a = 1.0;
        extend_states_display_color_b_tree.push_back(c);
    }
    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;
    _visual_tools.publishSpheres(extend_states_display_a_tree, extend_states_display_color_a_tree, scale);
    _visual_tools.trigger();
    _visual_tools.publishSpheres(extend_states_display_b_tree, extend_states_display_color_b_tree, scale);
    _visual_tools.trigger();

    //*********************显示规划的轨迹*****************************
    std::vector<std::string> joint_names = _planning_group->getVariableNames();
    joint_names.insert(joint_names.end(), _slave_group->getVariableNames().begin(), _slave_group->getVariableNames().end());

    path_point_msg.joint_names = joint_names;
    path_point_msg.header.stamp = ros::Time::now();
    result_msg.joint_trajectory = path_point_msg;

    _visual_tools.publishTrajectoryLine(result_msg, _planning_group);
    _visual_tools.trigger();

    //创建一个DisplayTrajectory msg 动画演示轨迹
    moveit_msgs::DisplayTrajectory display_traj_msg;
    moveit_msgs::RobotState start_state_msg;
    sensor_msgs::JointState start_angles_msg;
    //添加这个消息的第三个参数，开始的状态
    start_angles_msg.position = _startAngles;
    start_angles_msg.name = joint_names;
    start_state_msg.joint_state = start_angles_msg;
    display_traj_msg.trajectory_start = start_state_msg;
    //添加这个消息的第二个参数，开始的状态，可能显示多条轨迹所以是向量
    display_traj_msg.trajectory.push_back(result_msg);

    ros::WallDuration sleep_t(0.5);
    ros::Publisher display_traj_publisher = _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    while (display_traj_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    display_traj_publisher.publish(display_traj_msg);
    ros::Duration(1).sleep();
}

int DualCBiRRT::addState(const Eigen::Matrix<double, 7, 1> &masterAngles,
                         const Eigen::Matrix<double, 7, 1> &slaveAngles,
                         const Eigen::Vector4d &masterState,
                         const Eigen::Vector4d &slaveState,
                         const robot_state::RobotState &robot_state,
                         int parrentIndex, bool if_tree_a, double cost)
{
    Eigen::Matrix<double, 14, 1> matrix_tree_element;
    matrix_tree_element.head(7) = masterAngles;
    matrix_tree_element.tail(7) = slaveAngles;
    _tree_joints_angle_matrix[if_tree_a].push_back(std::make_pair(matrix_tree_element, cost));
    _tree_robotstate[if_tree_a].emplace_back(std::make_pair(robot_state, parrentIndex));
    _tree_task_space_vector[if_tree_a].push_back(std::make_pair(masterState, slaveState));
    return _tree_robotstate[if_tree_a].size() - 1;
}


Eigen::Vector4d DualCBiRRT::get_task_state_vector(robot_state::RobotState &state, bool is_master){
    Eigen::Vector4d output;
    std::string gripper_name;
    if(is_master){
        gripper_name = "left_gripper";
    }
    else{
        gripper_name = "right_gripper";
    }

    Eigen::Affine3d pose = state.getGlobalLinkTransform(gripper_name);
    output.head(3) = pose.translation();
    Eigen::Matrix3d tmp_rot_matrix = state.getGlobalLinkTransform(gripper_name).rotation();
    KDL::Rotation tmp;
    for(size_t i=0; i<3; i++){
        for(size_t j=0; j<3; j++){
            tmp(i,j) = tmp_rot_matrix(i,j);
        }
    }
    double foo1, foo2;
    tmp.GetEulerZYX(output(3), foo1, foo2);
    return output;
}

Eigen::Matrix<double, 14, 1> DualCBiRRT::get_joint_angles_matrix(const robot_state::RobotState &state){
    Eigen::Matrix<double, 14, 1> output;
    std::vector<double> goal_state_value; //用来判断是不是采样到了goal_state
    state.copyJointGroupPositions(_both_group, goal_state_value);
    for(size_t i=0; i<goal_state_value.size(); i++){
        output(i) = goal_state_value[i];
    }

    return output;
}

const std::vector<std::pair<robot_state::RobotState, size_t>> & DualCBiRRT::get_tree_state_vector(bool if_a_tree){
    return _tree_robotstate[if_a_tree];
}

Eigen::Matrix<double, 7, 1> DualCBiRRT::manipuGradient(robot_state::RobotState & robot_state, bool is_master){
    const robot_state::JointModelGroup* group;
    if(is_master){
        group = _planning_group;
    }
    else{
        group = _slave_group;
    }

    Eigen::VectorXd currentAngles;
    robot_state.copyJointGroupPositions(group, currentAngles);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(currentAngles.rows());
    double delta = 0.02;
    double forward = 0;
    double back = 0;

    for(int i=0; i<currentAngles.rows(); i++){
        currentAngles(i) += delta;
        robot_state.setJointGroupPositions(group, currentAngles);
        robot_state.update();
        forward = manipulability(robot_state, is_master);
        currentAngles(i) -= 2 * delta;
        robot_state.setJointGroupPositions(group, currentAngles);
        robot_state.update();
        back = manipulability(robot_state, is_master);
        gradient(i) = (forward - back) / (2 * delta);
        currentAngles(i) += delta;
    }
    return gradient;
}

double DualCBiRRT::manipulability(robot_state::RobotState & robot_state, bool is_master){
    Eigen::MatrixXd jac;
    Eigen::MatrixXd jjt;
    if(is_master){
        jac = robot_state.getJacobian(_planning_group);
    }
    else{
        jac = robot_state.getJacobian(_slave_group);
    }
    jjt = jac * jac.transpose();
    return  std::sqrt(jjt.determinant());

}
