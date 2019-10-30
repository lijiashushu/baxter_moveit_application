
#include <baxter_moveit_application/MyRRT/my_rrt.h>
#include <limits.h>

RRT::RRT(double probability):_random_distribution(probability){
    _random_engine.seed(time(0));
}

RRT::~RRT(){}

void RRT::sample(robot_state::RobotState & goal_state, robot_state::RobotState & random_state, Eigen::Matrix<double, 7, 1> & random_state_value_matrix, const robot_state::JointModelGroup* planning_group) {
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

size_t RRT::near(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix) {
    double minum_dis = std::numeric_limits<double>::max();
    size_t minum_index = 0;
    for(size_t i=0; i<_rrt_tree_matrix.size(); i++){
        Eigen::Matrix<double, 7, 1> dis_matrix = random_state_value_matrix - _rrt_tree_matrix[i];
        double distance = dis_matrix.squaredNorm();
        if(distance < minum_dis){
            minum_dis = distance;
            minum_index = i;
        }
    }
    nearest_node_matrix = _rrt_tree_matrix[minum_index];
    return minum_index;
}

void RRT::steer(Eigen::Matrix<double, 7, 1> & random_state_value_matrix, Eigen::Matrix<double, 7, 1> & nearest_node_matrix, double step_size, Eigen::Matrix<double, 7, 1> & new_node_matrix) {
    std::vector<double> random_state_value;
    Eigen::Matrix<double, 7, 1> direction_matrix = random_state_value_matrix - nearest_node_matrix;
    double norm = direction_matrix.squaredNorm();
    direction_matrix = direction_matrix / norm;
    if(norm < step_size){
        new_node_matrix = norm * direction_matrix + nearest_node_matrix;
    }
    else{
        new_node_matrix = step_size * direction_matrix + nearest_node_matrix;
    }
}

bool RRT::collision_check(Eigen::Matrix<double, 7, 1> & new_node_matrix, robot_state::RobotState & new_state, const robot_state::JointModelGroup* planning_group, const std::string & planning_group_name, planning_scene::PlanningScenePtr & planning_scene_ptr) {
    std::vector<double> new_node_value;
    for(size_t i=0; i<new_node_matrix.rows(); i++){
        new_node_value.push_back(new_node_matrix[i]);
    }
    new_state.setJointGroupPositions(planning_group, new_node_value);
    planning_scene_ptr->setCurrentState(new_state);
    return planning_scene_ptr->isStateColliding(planning_group_name);
}

bool RRT::plan(robot_state::RobotState & goal_state, robot_state::RobotState & start_state, planning_scene::PlanningScenePtr& planning_scene_ptr, const std::string & planning_group_name, const robot_state::JointModelGroup* planning_group){
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

    for(int count=0; count<1000; count++){
        std::cout<<count<<std::endl;
        sample(goal_state, random_state, random_state_value_matrix, planning_group);
        nearest_node_index = near(random_state_value_matrix, nearest_node_matrix);
        steer(random_state_value_matrix, nearest_node_matrix, 0.05, new_node_matrix);

        if(collision_check(new_node_matrix, new_state, planning_group, planning_group_name, planning_scene_ptr)){
            continue;
        }
        else{
            _rrt_tree_matrix.push_back(new_node_matrix);
            std::pair<robot_state::RobotState, size_t> tmp_pair(new_state, nearest_node_index);
            _rrt_tree_state.push_back(tmp_pair);
        }

        if(new_node_matrix == goal_value_matrix)
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

