//
// Created by lijiashushu on 20-4-3.
//

#include <ros/ros.h>
#include <dual_rrt_star/dual_cbirrt_star.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "one_plan_node");
    ros::NodeHandle nh("~");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

    std::srand((unsigned)time(NULL)); //fisrt srand set the seed, then rand()get the num
    int max_sample_times, constrain_index, seed;
    double bi_probility, task_step;
    bool if_rrt_star;
    nh.param("dualrrt/bi_probility",  bi_probility, 0.8);
    nh.param("dualrrt/max_sample_times",  max_sample_times, 1000);
    nh.param("dualrrt/task_step",  task_step, 0.02);
    nh.param("dualrrt/constrain_index",  constrain_index, 2);
    nh.param("dualrrt/seed",  seed, 0);
    nh.param("dualrrt/star",  if_rrt_star, false);
    if(seed == 0){
        seed = rand();
//        seed = 671710851;
    }

    //提前测量好的4个不同约束姿势的初始角度
    std::vector<double> start_pose0_master = {-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> start_pose0_slave = {-0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> start_pose1_master = {0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722};
    std::vector<double> start_pose1_slave = {-0.517204,-0.49348,1.79396,1.6679,0.302716,0.602833,-2.87906};
    std::vector<double> start_pose2_master = {0.75793,-0.888533,-2.01998,2.09217,-0.226114,-0.224376,2.39712};
    std::vector<double> start_pose2_slave = {-0.633959,-0.671778,2.03325,2.11988,0.716528,-0.334106,-3.05835};
    std::vector<double> start_pose3_master = {0.119411,-0.743597,-2.05219,2.14562,-0.840575,-1.39347,3.05791};
    std::vector<double> start_pose3_slave = {-0.119205,-0.74178,2.05281,2.14571,0.840017,-1.39411,-3.058};

    start_pose0_master.insert(start_pose0_master.end(),start_pose0_slave.begin(), start_pose0_slave.end());
    start_pose1_master.insert(start_pose1_master.end(),start_pose1_slave.begin(), start_pose1_slave.end());
    start_pose2_master.insert(start_pose2_master.end(),start_pose2_slave.begin(), start_pose2_slave.end());
    start_pose3_master.insert(start_pose3_master.end(),start_pose3_slave.begin(), start_pose3_slave.end());
    std::vector<std::vector<double>> default_start_vec;
    default_start_vec.push_back(start_pose0_master);
    default_start_vec.push_back(start_pose1_master);
    default_start_vec.push_back(start_pose2_master);
    default_start_vec.push_back(start_pose3_master);

    //提前测量好的4个不同约束姿势的目标角度
    std::vector<double> goal_pose0_master = {0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> goal_pose0_slave = {-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> goal_pose1_master = {0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696,3.05881};
    std::vector<double> goal_pose1_slave = {-0.640635,0.334855,1.43792,2.03636,-0.432353,0.425846,-3.05814};
    std::vector<double> goal_pose2_master = {0.378964,-0.235001,-1.09593,2.54013,0.629455,-0.946823,2.56304};
    std::vector<double> goal_pose2_slave = {-0.495711,0.18897,1.20076,2.51116,-0.470866,-0.757521,-3.05796};
    std::vector<double> goal_pose3_master = {0.188932,1.04619,-1.20316,2.47039,0.862578,-1.41834,-2.53144};
    std::vector<double> goal_pose3_slave = {-0.108729,1.0458,1.23816,2.45137,-0.881861,-1.45554,2.53844};

    goal_pose0_master.insert(goal_pose0_master.end(),goal_pose0_slave.begin(), goal_pose0_slave.end());
    goal_pose1_master.insert(goal_pose1_master.end(),goal_pose1_slave.begin(), goal_pose1_slave.end());
    goal_pose2_master.insert(goal_pose2_master.end(),goal_pose2_slave.begin(), goal_pose2_slave.end());
    goal_pose3_master.insert(goal_pose3_master.end(),goal_pose3_slave.begin(), goal_pose3_slave.end());

    std::vector<std::vector<double>> default_goal_vec;
    default_goal_vec.push_back(goal_pose0_master);
    default_goal_vec.push_back(goal_pose1_master);
    default_goal_vec.push_back(goal_pose2_master);
    default_goal_vec.push_back(goal_pose3_master);


    DualCBiRRT *planner = new DualCBiRRT(nh);
    planner->initPara(seed, bi_probility, max_sample_times, task_step, constrain_index,
            default_start_vec[constrain_index], default_goal_vec[constrain_index], false, true, if_rrt_star);

    if(planner->plan()){
        planner->showPath();
    }
    else{
        ROS_WARN("no plan!!");
    }
    delete planner;
    return 0;
}