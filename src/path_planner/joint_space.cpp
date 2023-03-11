#include <dual_rrt_star/path_planner/joint_space.h>
#include <fstream>

std::ofstream  debug_file;

PathPlanner* creatPathPlanner(JointSpacePlanner* jsp){
    return jsp;
}

JointSpacePlanner::JointSpacePlanner(const ros::NodeHandle &nh, std::string base_frame):
_nh(nh), _visual_tools(base_frame)
{
//    debug_file.open("/home/lijiashushu/debug.txt");


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

JointSpacePlanner::~JointSpacePlanner(){}

void JointSpacePlanner::initPara(int seed,
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
                                 bool if_informed)
{
    //****************设置随机数信息*******************
    _seed = seed;
    _random_engine.seed(seed);
    _random_distribution = std::bernoulli_distribution(biProbility);
    _random_uniform = std::uniform_real_distribution<double>(-1, 1);

    _max_planning_times = maxSampleTimes;
    _max_planning_time = maxPlanningTime;
    _test_pose_num = constrainIndex;
    _joint_step_size = taskStep;

    _startAngles = startAngles;
    _goalAngles = goalAngles;

    _if_manipuGrad = if_manipuGrand;
    _if_rrt_star = if_rrt_star;
    _if_informed = if_informed;

    _planning_result.clear();
    _planning_result_index.clear();
    _planning_result_joint_angles.clear();
    for(int i=0; i<2; i++){
        _tree_robotstate[i].clear();
        _tree_joints_angle_matrix[i].clear();
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


void JointSpacePlanner::sample(robot_state::RobotState & goal_state, random_numbers::RandomNumberGenerator& rng){
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
    Eigen::VectorXd tmp;
    random_state.copyJointGroupPositions(_planning_group, tmp);
    _random_joint_angles = tmp.head(7);
}

void JointSpacePlanner::sampleEllipsoid(double info_cBest){
//    std::cout<<"info_cBest "<<info_cBest<<std::endl;
//    std::cout<<"_info_cMin "<<_info_cMin<<std::endl;
    Eigen::Matrix<double, 7, 1> dirVector;
    for(int i = 0; i<7; i++){
        dirVector(i) =  _random_uniform(_random_engine);
    }
    dirVector.normalize();
    double scale = std::abs(_random_uniform(_random_engine));
    Eigen::Matrix<double, 7, 1> xBall = scale * dirVector;

    Eigen::Matrix<double, 7, 7> info_L_M = Eigen::Matrix<double, 7, 7>::Identity();
//    std::cout<<"informed sample info_L_M  "<<info_L_M<<std::endl;
//    std::cout<<"std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin)  "<<std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin)<<std::endl;
    info_L_M = info_L_M * std::sqrt(info_cBest*info_cBest - _info_cMin*_info_cMin) /2.0;
    info_L_M(0, 0) = info_cBest / 2.0;


//    std::cout<<"_info_C_M"<<std::endl;
//    std::cout<<_info_C_M<<std::endl;
//    std::cout<<"info_L_M"<<std::endl;
//    std::cout<<info_L_M<<std::endl;
//    std::cout<<"xBall"<<std::endl;
//    std::cout<<xBall.transpose()<<std::endl;
//    std::cout<<"_info_centre"<<std::endl;
//    std::cout<<_info_centre.transpose()<<std::endl;

    _random_joint_angles = _info_C_M * info_L_M * xBall + _info_centre;
//    std::cout<<"informed sample _info_C_M  "<<_info_C_M<<std::endl;
//    std::cout<<"informed sample info_L_M  "<<info_L_M<<std::endl;
//    std::cout<<"informed sample _info_centre  "<<_info_centre.transpose()<<std::endl;
//    std::cout<<"informed sample _random_joint_angles  "<<_random_joint_angles.transpose()<<std::endl;
}

void JointSpacePlanner::nearest(bool if_tree_a, std::vector<flann::Index<flann::L2<double>>>& flann_index)
{
    flann::Matrix<double > querry(new double[7], 1, 7);
    flann::Matrix<int > indices(new int[1], 1, 1);
    flann::Matrix<double > dist(new double[1], 1, 1);
    for(int i=0; i<7; i++){
        querry[0][i] = _random_joint_angles(i);
    }
    flann_index[if_tree_a].knnSearch(querry, indices, dist, 1, flann::SearchParams());
    _nearest_index = indices[0][0];
    delete querry.ptr();
    delete indices.ptr();
    delete dist.ptr();
}

int JointSpacePlanner::extend(bool if_tree_a, bool if_sample, double& totalExtendTime, long& totalExtendCounts,
                       std::vector<flann::Index<flann::L2<double>>>& flann_index)
{
    int reached_index = _nearest_index;

    robot_state::RobotState qs_state = _tree_robotstate[if_tree_a][_nearest_index].first;
    Eigen::Matrix<double, 7, 1> qs_matrix = _tree_joints_angle_matrix[if_tree_a][_nearest_index].first.head(7);
    Eigen::Matrix<double, 7, 1> qs_slave_matrix = _tree_joints_angle_matrix[if_tree_a][_nearest_index].first.tail(7);

    robot_state::RobotState qs_old_state = qs_state;
    Eigen::Matrix<double, 7, 1> qs_old_matrix = qs_matrix;
    Eigen::Matrix<double, 7, 1> qs_old_slave_matrix = qs_slave_matrix;

    //4-dimension goal state in one extension
    Eigen::Matrix<double, 7, 1> extend_goal_joint_angles;
    Eigen::Matrix<double, 7, 1> extend_direction_joint_angles;

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
        if ((_random_joint_angles - qs_matrix).norm() < 0.005){
            totalExtendTime += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-startExtendTime).count();
            return reached_index;
        }
        else{
            qs_old_state = qs_state;
            qs_old_matrix = qs_matrix;
            qs_old_slave_matrix = qs_slave_matrix;

            //构建 master 的任务空间目标
            extend_direction_joint_angles = _random_joint_angles - qs_matrix;
            double norm = extend_direction_joint_angles.norm();
            extend_direction_joint_angles = extend_direction_joint_angles / norm;
            if(_joint_step_size <= norm){
                extend_goal_joint_angles = qs_matrix + extend_direction_joint_angles * _joint_step_size;
            }
            else{
                extend_goal_joint_angles = qs_matrix + extend_direction_joint_angles * norm;
            }

            if(project(extend_goal_joint_angles)){
                qs_state.setJointGroupPositions("left_arm", extend_goal_joint_angles);
                qs_state.update();
//            std::cout<<"after before project norm deviation  "<<(qs_matrix - extend_goal_joint_angles).norm()<<std::endl;
//            std::cout<<"master state before extend "<<get_task_state_vector(qs_state, true).transpose()<<std::endl;
                if((_random_joint_angles - extend_goal_joint_angles).norm() < (_random_joint_angles - qs_matrix).norm()
                   && qs_state.satisfiesBounds(_planning_group, 0.05)){
                    master_ik_flag = true;
                    qs_matrix = extend_goal_joint_angles;
                }
                else{
                    master_ik_flag = false;
                }
            }
            else{
                master_ik_flag = false;
            }
                    
            //如果投影成功，qs_state肯定更新过
            if(master_ik_flag){
//                ROS_INFO("master IK success");
                master_collide_flag = extend_collision_check(if_tree_a, true, qs_matrix, reached_index);
                if(master_collide_flag){
//                    ROS_INFO("master collide success");
                    //计算master空间位置及姿态
                    Eigen::Vector3d master_pos = qs_state.getGlobalLinkTransform("left_gripper").translation();
                    Eigen::MatrixXd master_rot = qs_state.getGlobalLinkTransform("left_gripper").rotation();
                    KDL::Rotation master_rot_kdl;
                    for(int i=0; i<3; i++){
                        for(int j=0; j<3; j++){
                            master_rot_kdl(i, j) = master_rot(i, j);
                        }
                    }
                    double master_yaw, master_pitch, master_roll;
                    master_rot_kdl.GetEulerZYX(master_yaw, master_pitch, master_roll);

                    //计算闭环约束，得到slave的目标姿态
                    KDL::Vector distance(_left_right_distance_x[_test_pose_num], 0, _left_right_distance_z[_test_pose_num]);
                    KDL::Rotation slave_goal_rot_kdl = KDL::Rotation::EulerZYX(master_yaw - _left_right_euler_distance[_test_pose_num], 0.0, -1.57);
                    KDL::Vector distance_world = master_rot_kdl * distance;
                    Eigen::Vector4d slave_goal_vector(distance_world(0) + master_pos(0),
                                                      distance_world(1) + master_pos(1),
                                                      distance_world(2) + master_pos(2),
                                                      master_yaw - _left_right_euler_distance[_test_pose_num]);

                    Eigen::Vector4d qs_slave_task_state = get_task_state_vector(qs_state, false); //need to set to the current state
                    //solveIK 函数会将 qs_state 更新
                    slave_ik_flag = solveIK(slave_goal_vector, slave_goal_rot_kdl, qs_state, qs_slave_task_state, qs_slave_matrix, false);
                    if(slave_ik_flag){
//                        ROS_INFO("slave IK success");
                        slave_collide_flag = extend_collision_check(if_tree_a, false, qs_slave_matrix, reached_index);

                        if(slave_collide_flag){
//                            ROS_INFO("slave collide success");
//                            ROS_INFO("adding a state");
                            totalExtendCounts++;

                            if(_if_rrt_star && _foundPathFlag){
                                //find posible parents
                                flann::Matrix<double> querry(new double[1 * 7], 1, 7);
                                std::vector<std::vector<int>> indices; //outer indicates each querry, inner indicates founded points
                                std::vector<std::vector<double>> dist;
                                for (int i = 0; i < 7; i++) {
                                    querry[0][i] = qs_matrix(i);
                                }

                                int minIndex = 0;
                                double minCost = 0;
                                double segCost = 0;
                                double remain_dis = 0;
                                int treeIndex;
                                //***********************************ChooseParent***********************************
//                                    flann_index[if_tree_a].radiusSearch(querry, indices, dist, pow(_task_step_size * 1.9, 2), flann::SearchParams());
                                flann_index[if_tree_a].knnSearch(querry, indices, dist, 8, flann::SearchParams());

                                std::vector<double> cost_to_root(indices[0].size(), 0);
                                for(int i = 0; i<dist[0].size(); i++){
                                    dist[0][i] = std::sqrt(dist[0][i]);
                                    int curIndex = indices[0][i];
                                    while(curIndex != 0){
                                        cost_to_root[i] += _tree_joints_angle_matrix[if_tree_a][curIndex].second;
                                        curIndex = _tree_robotstate[if_tree_a][curIndex].second;
                                    }
                                }

                                std::vector<int> extendCheckRecord(indices[0].size(), 0); //0 untested 1 feasible 2 infeasible
//                                int curParentIndexIndex =  std::find(indices[0].begin(), indices[0].end(), reached_index) -  indices[0].begin();
//                                minIndex = indices[0][curParentIndexIndex];
//                                minCost = dist[0][curParentIndexIndex] + cost_to_root[curParentIndexIndex];
//                                segCost = dist[0][curParentIndexIndex];
//                                extendCheckRecord[curParentIndexIndex] = 1;
                                //knn 的搜索结果可能不包含当前正在被扩展的节点。。。。
                                minIndex = reached_index;
                                segCost = (qs_matrix - _tree_joints_angle_matrix[if_tree_a][reached_index].first.head(7)).norm();
                                int curIndex = reached_index;
                                while(curIndex != 0){
                                    minCost += _tree_joints_angle_matrix[if_tree_a][curIndex].second;
                                    curIndex = _tree_robotstate[if_tree_a][curIndex].second;
                                }
                                minCost += segCost;


                                std::vector<std::pair<double, int>> feasibleCheckStack; //只有比当前的cost小的才进栈

                                for (int i = 0; i < indices[0].size(); i++) {
                                    if (minCost > dist[0][i] + cost_to_root[i]) {
                                        feasibleCheckStack.push_back(std::make_pair(dist[0][i] + + cost_to_root[i], i));
                                    }
                                }
                                std::sort(feasibleCheckStack.begin(), feasibleCheckStack.end());


//                                std::cout<<"original cost: "<<minCost<<std::endl;
                                //从 cost 最小的开始
                                for( int i=0; i<feasibleCheckStack.size(); i++){
                                    treeIndex = indices[0][feasibleCheckStack[i].second];
//                                    std::cout<<"before tree index "<<treeIndex<<std::endl;
                                    if (extend_check(if_tree_a, qs_matrix, treeIndex, remain_dis, flann_index)){
                                        minIndex = treeIndex;
                                        minCost = feasibleCheckStack[i].first;
                                        segCost = remain_dis;
                                        extendCheckRecord[feasibleCheckStack[i].second] = 1;
//                                        std::cout<<"after tree index "<<treeIndex<<std::endl;
                                        break;
                                    }
                                    else {
                                        extendCheckRecord[feasibleCheckStack[i].second] = 2;
                                    }
                                }

                                //***********************************Add State***********************************
                                reached_index = addState(qs_matrix, qs_slave_matrix, qs_state, minIndex, if_tree_a, segCost);
                                flann_index[if_tree_a].addPoints(querry);
                                _waitRelesePtr.push_back(querry.ptr());

                                //***********************************Rewire***********************************
                                for (int i = 0; i < indices[0].size(); i++) {
                                    treeIndex = indices[0][i];
                                    if (extendCheckRecord[i] == 0 && (cost_to_root[i] > dist[0][i] + minCost)) {
                                        int rewire_start_index = reached_index; //即反过来扩展，以刚才扩展的新的节点为起点，尝试连接查找到的最邻近的点。
                                        if (extend_check(if_tree_a, _tree_joints_angle_matrix[if_tree_a][treeIndex].first.head(7), rewire_start_index, remain_dis, flann_index)){
                                            std::cout<<"rewire"<<std::endl;
                                            _tree_joints_angle_matrix[if_tree_a][treeIndex].second = remain_dis;
                                            _tree_robotstate[if_tree_a][treeIndex].second = rewire_start_index;
                                        }
                                    }
                                }

                            }//_if_rrt_star
                            else{
                                double cost = (qs_matrix - _tree_joints_angle_matrix[if_tree_a][reached_index].first.head(7)).norm();
                                reached_index = addState(qs_matrix, qs_slave_matrix, qs_state, reached_index, if_tree_a, cost);
                                flann::Matrix<double> newPoint(new double[1 * 7], 1, 7);
                                for (int i = 0; i < 7; i++) {
                                    newPoint[0][i] = qs_matrix(i);
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

bool JointSpacePlanner::extend_check(bool if_tree_a, const Eigen::MatrixXd& goal_master_angles, int& try_index, double& remain_dis,
                              std::vector<flann::Index<flann::L2<double>>>& flann_index){

    Eigen::Matrix<double,7,1> masterWorkJointAngles = _tree_joints_angle_matrix[if_tree_a][try_index].first.head(7);
    Eigen::Matrix<double,7,1> slaveWorkJointAngles = _tree_joints_angle_matrix[if_tree_a][try_index].first.tail(7);
    robot_state::RobotState curState = _tree_robotstate[if_tree_a][try_index].first;

    //master extend needed
    double extendDis;
    Eigen::Matrix<double,7,1> extendDir;
    Eigen::Matrix<double,7,1> extend_goal_joint_angles;

    //slave extend needed
    Eigen::Vector3d master_pos;
    Eigen::MatrixXd master_rot;
    KDL::Rotation master_rot_kdl;
    double master_yaw, master_pitch, master_roll;
    KDL::Vector distance(_left_right_distance_x[_test_pose_num], 0, _left_right_distance_z[_test_pose_num]);
    KDL::Rotation slave_goal_rot_kdl;
    KDL::Vector distance_world;
    Eigen::Vector4d slave_goal_vector;


    while(true){
        //*************计算当前的 joint angles 与目标的距离************************
        extendDis = (goal_master_angles - masterWorkJointAngles).norm();
        if(extendDis < _joint_step_size){
            remain_dis = extendDis;
            return true;
        }
        else{
            extendDir = (goal_master_angles - masterWorkJointAngles).normalized();
            extend_goal_joint_angles = extendDir * _joint_step_size;
            if(!project(extend_goal_joint_angles)){
                return false;
            }
            curState.setJointGroupPositions("left_arm", extend_goal_joint_angles);
            curState.update();
            if((goal_master_angles - extend_goal_joint_angles).norm() < (goal_master_angles - masterWorkJointAngles).norm()
                 && curState.satisfiesBounds(_planning_group, 0.05)) {

                masterWorkJointAngles = extend_goal_joint_angles;
                collision_detection::CollisionRequest masterReq;
                collision_detection::CollisionResult masterRes;
                masterReq.group_name = "left_arm";
                _collision_world->checkRobotCollision(masterReq, masterRes, *_collision_robot, curState);


                if(!masterRes.collision){
                    master_pos = curState.getGlobalLinkTransform("left_gripper").translation();
                    master_rot = curState.getGlobalLinkTransform("left_gripper").rotation();
                    for(int i=0; i<3; i++){
                        for(int j=0; j<3; j++){
                            master_rot_kdl(i, j) = master_rot(i, j);
                        }
                    }
                    master_rot_kdl.GetEulerZYX(master_yaw, master_pitch, master_roll);
                    slave_goal_rot_kdl = KDL::Rotation::EulerZYX(master_yaw - _left_right_euler_distance[_test_pose_num], 0.0, -1.57);
                    distance_world = master_rot_kdl * distance;
                    slave_goal_vector(0) = distance_world(0) + master_pos(0);
                    slave_goal_vector(1) = distance_world(1) + master_pos(1);
                    slave_goal_vector(2) = distance_world(2) + master_pos(2);
                    slave_goal_vector(3) = master_yaw - _left_right_euler_distance[_test_pose_num];

                    Eigen::Vector4d qs_slave_task_state = get_task_state_vector(curState, false); //need to set to the current state
                    if(solveIK(slave_goal_vector, slave_goal_rot_kdl, curState, qs_slave_task_state, slaveWorkJointAngles, false)){
                        collision_detection::CollisionRequest slaveReq;
                        collision_detection::CollisionResult slaveRes;
                        slaveReq.group_name = "right_arm";
                        _collision_world->checkRobotCollision(slaveReq, slaveRes, *_collision_robot, curState);
                        if(!slaveRes.collision){
                            double cost = (masterWorkJointAngles - _tree_joints_angle_matrix[if_tree_a][try_index].first.head(7)).norm();
                            try_index = addState(masterWorkJointAngles, slaveWorkJointAngles, curState, try_index, if_tree_a, cost);
                            flann::Matrix<double> querry(new double[1 * 7], 1, 7);
                            for(int i=0; i<7; i++){
                                querry[0][i] = masterWorkJointAngles(i);
                            }
                            flann_index[if_tree_a].addPoints(querry);
                            _waitRelesePtr.push_back(querry.ptr());
                        }
                        else{
//                            ROS_INFO("check slave collide");
                            return false;
                        }
                    }
                    else{
//                        ROS_INFO("slave IK fail");
                        return false;
                    }
                }
                else{
//                    ROS_INFO("check master collide");
                    return false;
                }
            }
            else{
//                ROS_INFO("master project fail");
                return false;
            }
        }
    }
}

bool JointSpacePlanner::extend_collision_check(bool if_tree_a, bool if_master, Eigen::Matrix<double, 7, 1> goal_matrix, int try_index)
{
    robot_state::RobotState workState = _tree_robotstate[if_tree_a][try_index].first;
    Eigen::Matrix<double, 7, 1> workMatrix;
    const robot_state::JointModelGroup* group;
    std::string groupName;
    if(if_master){
        workMatrix = _tree_joints_angle_matrix[if_tree_a][try_index].first.head(7);
        group = _planning_group;
        groupName = "left_arm";
    }
    else{
        workMatrix = _tree_joints_angle_matrix[if_tree_a][try_index].first.tail(7);
        group = _slave_group;
        groupName = "right_arm";
    }
    double dis = (goal_matrix - workMatrix).norm();
    if(_collision_check_step < dis){
        Eigen::Matrix<double, 7, 1> dir = (goal_matrix - workMatrix).normalized();
        int step = floor(dis / _collision_check_step);

        for(int i=0; i<step; i++){
            workState.setJointGroupPositions(group, workMatrix + _collision_check_step * dir);
            workState.update();
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = groupName;
            _collision_world->checkRobotCollision(req, res, *_collision_robot, workState);
            if(res.collision){
                return false;
            }
        }
    }
    workState.setJointGroupPositions(group, goal_matrix);
    workState.update();
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = groupName;
    _collision_world->checkRobotCollision(req, res, *_collision_robot, workState);
    if(res.collision){
        return false;
    }

    return true;
}

bool JointSpacePlanner::project(Eigen::Matrix<double, 7, 1> &joint_angles){
//    std::cout<<"projecting"<<std::endl;
    robot_state::RobotState rb_state(_robot_model_ptr);
    rb_state.setToDefaultValues();

    Eigen::Affine3d end_pose;
    Eigen::MatrixXd end_rot;
    KDL::Rotation end_rot_kdl;
    double yaw, pitch, roll, yaw_error, pitch_error, roll_error, yaw_proj, pitch_proj, roll_proj;

    Eigen::VectorXd task_space_error = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd jac;
    Eigen::Matrix3d to_zyx;
    Eigen::MatrixXd jac_pinv;
    Eigen::VectorXd joint_space_error;

    int count = 0;
    while(true){
        //**********************compute task space error*******************
        rb_state.setJointGroupPositions("left_arm", joint_angles);
        rb_state.update();
        if(!rb_state.satisfiesBounds(_planning_group, 0.5)){
//            std::cout<<"end projecting"<<std::endl;
            return false;
        }
        end_pose = rb_state.getGlobalLinkTransform("left_gripper");
        end_rot = end_pose.rotation();
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                end_rot_kdl(i, j) = end_rot(i, j);
            }
        }
        end_rot_kdl.GetEulerZYX(yaw, pitch, roll);
        yaw_error = 0 - yaw;
        pitch_error = 0 - pitch;
        roll_error = 1.57 - roll;
//        std::cout<<count++<<"  "<<pitch_error<<" "<<roll_error<<std::endl;
        if(std::abs(yaw_error) < _yaw_deviation){
            yaw_proj = 0;
        }
        else{
            yaw_proj = 0;
        }
        if(std::abs(pitch_error) < _pitch_deviation){
            pitch_proj = 0;
        }
        else{
            pitch_proj = pitch_error;
        }
        if(std::abs(roll_error) < _roll_deviation){
            roll_proj = 0;
        }
        else{
            roll_proj = roll_error;
        }

        //************************if satisfy the bound************************
        if(yaw_proj ==0 && pitch_proj == 0 && roll_proj == 0){
//            std::cout<<"end projecting"<<std::endl;
            return true;
        }
        else{
            KDL::Rotation goal_rot = KDL::Rotation::EulerZYX(yaw, 0, 1.57);
            KDL::Rotation err_rot = goal_rot * end_rot_kdl.Inverse();
            KDL::Vector axis;
            double angle = err_rot.GetRotAngle(axis);

//            task_space_error(3) = yaw_proj;
//            task_space_error(4) = pitch_proj;
//            task_space_error(5) = roll_proj;

            task_space_error(3) = angle * axis(0);
            task_space_error(4) = angle * axis(1);
            task_space_error(5) = angle * axis(2);
            task_space_error = _error_coefficient * task_space_error / 0.01;

            jac = rb_state.getJacobian(_planning_group);

            to_zyx(0, 0) = cos(yaw) * tan(pitch);
            to_zyx(0, 1) = sin(yaw) * tan(pitch);
            to_zyx(0, 2) = 1;
            to_zyx(1, 0) = -sin(yaw);
            to_zyx(1, 1) = cos(yaw);
            to_zyx(1, 2) = 0;
            to_zyx(2, 0) = cos(yaw) / cos(pitch);
            to_zyx(2, 1) = sin(yaw) / cos(pitch);
            to_zyx(2, 2) = 0;

//            std::cout<<"to_zyx"<<std::endl;
//            std::cout<<to_zyx<<std::endl;
//            jac.block(3, 0, 3, 7) = to_zyx * jac.block(3, 0, 3, 7);

            jac_pinv = jac.transpose() * ((jac * jac.transpose()).inverse());
            joint_space_error = jac_pinv * task_space_error;
            joint_angles += joint_space_error * 0.01;
        }
    }
}

bool JointSpacePlanner::solveIK(const Eigen::Vector4d & goal_vector,
                         const KDL::Rotation & goal_rot,
                         robot_state::RobotState &qs_state,
                         Eigen::Vector4d &qs_task_state_vector,
                         Eigen::Matrix<double, 7, 1> &qs_matrix,
                         bool is_master)
{
//    std::cout<<"slave ik"<<std::endl;
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
//        std::cout<<"seed: "<<_seed<<std::endl;
        //先计算当前与约束的误差判断是否需要继续进行投影操作。

        state_error = goal_vector - qs_task_state_vector;
        pos_error = state_error.head(3);
        rot_error = goal_rot * qs_rot_kdl.Inverse();
        angle_error = rot_error.GetRotAngle(axis_error);

        if((state_error.norm() < 0.005 &&  fabs(angle_error) < 0.005 )){
//            std::cout<<"end slave ik"<<std::endl;
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
            Eigen::VectorXd tmp;
            qs_state.copyJointGroupPositions(group, tmp);
            qs_matrix = qs_matrix + joint_delta_vector * 0.01 + nullspace_scale_factor * gradientProj;
            //更新当前的RobotState
            qs_state.setJointGroupPositions(group, qs_matrix);
            qs_state.update();
            if (!qs_state.satisfiesBounds(group, 0.05)){
//                std::cout<<"end slave ik"<<std::endl;
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
//    std::cout<<"end slave ik"<<std::endl;
    return false;
}

bool JointSpacePlanner::plan(){
    std::cout << "joint space planner  " << std::endl;
    std::ofstream tmp;
    return plan(tmp);
}

bool JointSpacePlanner::plan(std::ofstream &outputFile) {
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

    _tree_joints_angle_matrix[0].push_back(std::make_pair(get_joint_angles_matrix(start_state), 0));
    _tree_joints_angle_matrix[1].push_back(std::make_pair(get_joint_angles_matrix(goal_state), 0));

    int tree_reached_index[2];
    int last_tree_size[2] = {1,1};

    bool extendOrder = false; //false indicates 0 tree_a

    long totalExtendCounts = 0;
    double totalExtendTime = 0;
    std::cout<<"pose: "<<_test_pose_num<<"  seed： "<<_seed<<std::endl;


    //建立KD tree
    std::vector<flann::Index<flann::L2<double>>> flannIndex;
    flann::Matrix<double> a_initData(new double[1 * 7], 1, 7);
    flann::Matrix<double> b_initData(new double[1 * 7], 1, 7);
    for(int i=0; i<7; i++){
        a_initData[0][i] = _tree_joints_angle_matrix[0][0].first(i);
        b_initData[0][i] = _tree_joints_angle_matrix[1][0].first(i);
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
    double pathCostTask = 0;
    int back_track[2];

    _planning_time = 0.0;
    std::chrono::high_resolution_clock::time_point startPlanningTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point loopStartTime;
    while(_planning_time < _max_planning_time){
//    for(int count=1; count < _max_planning_times; count++){
//        std::cout<<count<<std::endl;
//        std::cout<<"planning time: "<<_planning_time<<std::endl;
        loopStartTime = std::chrono::high_resolution_clock::now();
        if(!_foundPathFlag){
            sample(goal_state, rng);
        }
        else{
            if(_if_informed){
                sampleEllipsoid(pathCost);
            }
            else{
                sample(goal_state, rng);
            }
        }

        //*****************************extend one tree**********************************
        nearest(extendOrder, flannIndex);
        tree_reached_index[extendOrder] = extend(extendOrder, true, totalExtendTime, totalExtendCounts, flannIndex);
        _random_joint_angles= _tree_joints_angle_matrix[extendOrder][tree_reached_index[extendOrder]].first.head(7); //update _random_state_vector for the other tree
        //*****************************extend the other tree**********************************
        nearest(!extendOrder, flannIndex);
        tree_reached_index[!extendOrder] = extend(!extendOrder, false, totalExtendTime, totalExtendCounts, flannIndex);
        extendOrder = !extendOrder;

        if(!_foundPathFlag){
            double res_tmp = (_tree_joints_angle_matrix[0][tree_reached_index[0]].first.head(7) - _tree_joints_angle_matrix[1][tree_reached_index[1]].first.head(7)).norm();
            if(res_tmp< _joint_step_size)
            {
                std::cout<<"a_tree: "<<_tree_joints_angle_matrix[0][tree_reached_index[0]].first.head(7).transpose()<<std::endl;
                std::cout<<"b_tree: "<<_tree_joints_angle_matrix[1][tree_reached_index[1]].first.head(7).transpose()<<std::endl;
                _foundPathFlag = true;
                back_track[0] = tree_reached_index[0];
                back_track[1] = tree_reached_index[1];

                pathCost += res_tmp;
                int curIndex = back_track[0];
                while(curIndex != 0){
                    pathCost += _tree_joints_angle_matrix[0][curIndex].second;
                    curIndex = _tree_robotstate[0][curIndex].second;
                }
                curIndex = back_track[1];
                while(curIndex != 0){
                    pathCost += _tree_joints_angle_matrix[1][curIndex].second;
                    curIndex = _tree_robotstate[1][curIndex].second;
                }

//                if(!_if_rrt_star){
//                    break;
//                }
                if(_if_informed){
                    //initialize informed sample parameter
                    _info_centre = (_tree_joints_angle_matrix[0][0].first.head(7) + _tree_joints_angle_matrix[1][0].first.head(7)) * 0.5;
                    _info_cMin = (_tree_joints_angle_matrix[0][0].first.head(7) - _tree_joints_angle_matrix[1][0].first.head(7)).norm();

                    Eigen::Matrix<double, 7, 1> info_a1 = (_tree_joints_angle_matrix[0][0].first.head(7) - _tree_joints_angle_matrix[1][0].first.head(7)).normalized();
                    Eigen::Matrix<double, 7, 1> info_i1 = Eigen::Matrix<double, 7, 1>::Zero();
                    info_i1(0) = 1;
                    Eigen::Matrix<double, 7, 7>  info_M_M = info_a1 * info_i1.transpose();
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(info_M_M,  Eigen::ComputeFullU | Eigen::ComputeFullV);
                    Eigen::Matrix<double, 7, 7> info_tmp = Eigen::Matrix<double, 7, 7>::Identity();
                    info_tmp(6, 6) = svd.matrixU().determinant()*svd.matrixV().determinant();
                    _info_C_M = svd.matrixU() * info_tmp * svd.matrixV().transpose();
                }

                _planning_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - loopStartTime).count();
                pathCostTask += (get_task_state_vector(_tree_robotstate[0][back_track[0]].first, true)
                                - get_task_state_vector(_tree_robotstate[1][back_track[1]].first, true)).norm();
                curIndex = back_track[0];
                int parentIndex = _tree_robotstate[0][curIndex].second;
                while(curIndex != 0){
                    pathCostTask += (get_task_state_vector(_tree_robotstate[0][curIndex].first, true)
                                     - get_task_state_vector(_tree_robotstate[0][parentIndex].first, true)).norm();
                    curIndex = parentIndex;
                    parentIndex = _tree_robotstate[0][curIndex].second;
                }

                curIndex = back_track[1];
                parentIndex = _tree_robotstate[1][curIndex].second;
                while(curIndex != 0){
                    pathCostTask += (get_task_state_vector(_tree_robotstate[1][curIndex].first, true)
                                     - get_task_state_vector(_tree_robotstate[1][parentIndex].first, true)).norm();
                    curIndex = parentIndex;
                    parentIndex = _tree_robotstate[1][curIndex].second;
                }
                loopStartTime = std::chrono::high_resolution_clock::now();
            }
            outputFile<<_planning_time<<" "<<3.0<<std::endl;
        }
        else{
            double res_tmp = (_tree_joints_angle_matrix[0][tree_reached_index[0]].first - _tree_joints_angle_matrix[1][tree_reached_index[1]].first).norm();
            double tmp_cost1 = 0;
            double tmp_cost2 = 0;
            if(res_tmp < _joint_step_size){
                tmp_cost1 += res_tmp;
                int curIndex = tree_reached_index[0];
                while(curIndex != 0){
                    tmp_cost1 += _tree_joints_angle_matrix[0][curIndex].second;
                    curIndex = _tree_robotstate[0][curIndex].second;
                }
                curIndex = tree_reached_index[1];
                while(curIndex != 0){
                    tmp_cost1 += _tree_joints_angle_matrix[1][curIndex].second;
                    curIndex = _tree_robotstate[1][curIndex].second;
                }
            }
            else{
                tmp_cost1 = 1e10;
            }

            tmp_cost2 += (_tree_joints_angle_matrix[0][back_track[0]].first - _tree_joints_angle_matrix[1][back_track[1]].first).norm();
            int curIndex = back_track[0];
            while(curIndex != 0){
                tmp_cost2 += _tree_joints_angle_matrix[0][curIndex].second;
                curIndex = _tree_robotstate[0][curIndex].second;
            }
            curIndex = back_track[1];
            while(curIndex != 0){
                tmp_cost2 += _tree_joints_angle_matrix[1][curIndex].second;
                curIndex = _tree_robotstate[1][curIndex].second;
            }

            if(tmp_cost1 < tmp_cost2){
                pathCost = tmp_cost1;
                back_track[0] = tree_reached_index[0];
                back_track[1] = tree_reached_index[1];
            }
            else{
                pathCost = tmp_cost2;
            }

            _planning_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - loopStartTime).count();
            pathCostTask = 0;
            pathCostTask += (get_task_state_vector(_tree_robotstate[0][back_track[0]].first, true)
                             - get_task_state_vector(_tree_robotstate[1][back_track[1]].first, true)).norm();
            curIndex = back_track[0];
            int parentIndex = _tree_robotstate[0][curIndex].second;
            while(curIndex != 0){
                pathCostTask += (get_task_state_vector(_tree_robotstate[0][curIndex].first, true)
                                 - get_task_state_vector(_tree_robotstate[0][parentIndex].first, true)).norm();
                curIndex = parentIndex;
                parentIndex = _tree_robotstate[0][curIndex].second;
            }
            curIndex = back_track[1];
            parentIndex = _tree_robotstate[1][curIndex].second;
            while(curIndex != 0){
                pathCostTask += (get_task_state_vector(_tree_robotstate[1][curIndex].first, true)
                                 - get_task_state_vector(_tree_robotstate[1][parentIndex].first, true)).norm();
                curIndex = parentIndex;
                parentIndex = _tree_robotstate[1][curIndex].second;
            }
            loopStartTime = std::chrono::high_resolution_clock::now();


            outputFile<<_planning_time<<" "<<pathCostTask<<std::endl;
//            std::cout<<"jonit_cost "<<pathCost<<"  task cost"<<pathCostTask<<std::endl;
        }
        _planning_time += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - loopStartTime).count();
    }//end while

    //***********return 前的处理****************
    for(auto ptr:_waitRelesePtr){
        delete ptr;
    }

    std::cout<<"seed: "<<_seed<<std::endl;
    std::cout<<"planningTime: "<<_planning_time<<std::endl;
    std::cout<<"pathCost: "<<pathCost<<std::endl;
    if(_foundPathFlag){
        while(back_track[0] != -1){
            _planning_result.push_back(_tree_robotstate[0][back_track[0]].first);
            _planning_result_index.push_back(_tree_robotstate[0][back_track[0]].second);
            _planning_result_joint_angles.push_back(_tree_joints_angle_matrix[0][back_track[0]].first);
            back_track[0] = _tree_robotstate[0][back_track[0]].second;
        }
        std::reverse(_planning_result.begin(), _planning_result.end());
        std::reverse(_planning_result_index.begin(), _planning_result_index.end());
        std::reverse(_planning_result_joint_angles.begin(), _planning_result_joint_angles.end());

        //添加后半部分路径点
        back_track[1] = _tree_robotstate[1][back_track[1]].second;
        while(back_track[1] != -1){
            _planning_result.push_back(_tree_robotstate[1][back_track[1]].first);
            _planning_result_index.push_back(_tree_robotstate[1][back_track[1]].second);
            _planning_result_joint_angles.push_back(_tree_joints_angle_matrix[1][back_track[1]].first);
            back_track[1] = _tree_robotstate[1][back_track[1]].second;
        }
        return true;
    }
    else{
        return false;
    }
}

void JointSpacePlanner::showPath(){
    robot_state::RobotState moveit_state(_robot_model_ptr);
    moveit_state.setToDefaultValues();


    _visual_tools.deleteAllMarkers();
    _visual_tools.trigger();
    ros::Duration(0.2).sleep();

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
    }
    //*********************显示扩展成功的 master 的末端位置点*****************************
    ROS_INFO("a_tree_size: %ld", _tree_joints_angle_matrix[0].size());
    ROS_INFO("b_tree_size: %ld", _tree_joints_angle_matrix[1].size());

    std::vector<geometry_msgs::Point> extend_states_display_a_tree;
    std::vector<std_msgs::ColorRGBA> extend_states_display_color_a_tree;
    std::vector<geometry_msgs::Point> extend_states_display_b_tree;
    std::vector<std_msgs::ColorRGBA> extend_states_display_color_b_tree;
    Eigen::VectorXd tmp; //转换 Eigen::Matirx<doube, 7, 1> otherwise the setJointGroupPositions func can not use
    Eigen::Affine3d end_pose;
    for(auto i:_tree_joints_angle_matrix[0]){
        moveit_state.setJointGroupPositions("both_arms", i.first);
        moveit_state.update();
        end_pose = moveit_state.getGlobalLinkTransform("left_gripper");
        geometry_msgs::Point p;
        p.x = end_pose.translation()[0];
        p.y = end_pose.translation()[1];
        p.z = end_pose.translation()[2];
        extend_states_display_a_tree.push_back(p);
        std_msgs::ColorRGBA c;
        c.b = 1.0;
        c.a = 1.0;
        extend_states_display_color_a_tree.push_back(c);
    }
    for(auto i:_tree_joints_angle_matrix[1]){
        moveit_state.setJointGroupPositions("both_arms", i.first);
        moveit_state.update();
        end_pose = moveit_state.getGlobalLinkTransform("left_gripper");
        geometry_msgs::Point p;
        p.x = end_pose.translation()[0];
        p.y = end_pose.translation()[1];
        p.z = end_pose.translation()[2];
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

    ros::Duration(0.5).sleep();

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

int JointSpacePlanner::addState(const Eigen::Matrix<double, 7, 1> &masterAngles,
                         const Eigen::Matrix<double, 7, 1> &slaveAngles,
                         const robot_state::RobotState &robot_state,
                         int parrentIndex, bool if_tree_a, double cost)
{
    Eigen::Matrix<double, 14, 1> matrix_tree_element;
    matrix_tree_element.head(7) = masterAngles;
    matrix_tree_element.tail(7) = slaveAngles;
    _tree_joints_angle_matrix[if_tree_a].push_back(std::make_pair(matrix_tree_element, cost));
    _tree_robotstate[if_tree_a].emplace_back(std::make_pair(robot_state, parrentIndex));
    return _tree_robotstate[if_tree_a].size() - 1;
}

Eigen::Vector4d JointSpacePlanner::get_task_state_vector(robot_state::RobotState &state, bool is_master){
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

Eigen::Matrix<double, 14, 1> JointSpacePlanner::get_joint_angles_matrix(const robot_state::RobotState &state){
    Eigen::Matrix<double, 14, 1> output;
    std::vector<double> goal_state_value; //用来判断是不是采样到了goal_state
    state.copyJointGroupPositions(_both_group, goal_state_value);
    for(size_t i=0; i<goal_state_value.size(); i++){
        output(i) = goal_state_value[i];
    }

    return output;
}

const std::vector<std::pair<robot_state::RobotState, size_t>> & JointSpacePlanner::get_tree_state_vector(bool if_a_tree){
    return _tree_robotstate[if_a_tree];
}

Eigen::Matrix<double, 7, 1> JointSpacePlanner::manipuGradient(robot_state::RobotState & robot_state, bool is_master){
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

double JointSpacePlanner::manipulability(robot_state::RobotState & robot_state, bool is_master){
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
