//
// Created by lijiashushu on 20-4-22.
//

#include <ros/ros.h>
#include "dual_rrt_star/path_planner/joint_space.h"
#include <fstream>

int main(int argc, char** argv){
    ros::init(argc, argv, "compare_node");
    ros::NodeHandle nh("~");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

    std::srand((unsigned)time(NULL)); //fisrt srand set the seed, then rand()get the num
    int constrain_index;
    int seed;
    int test_times;
    int max_sample_times = 500;
    double bi_probility = 0.8;
    double task_step;
    double max_planning_time;

    nh.param("dualrrt/nums",  test_times, 3);
    nh.param("dualrrt/max_planning_time",  max_planning_time, 5.0);
    nh.param("dualrrt/task_step",  task_step, 0.1);
    nh.param("dualrrt/constrain_index",  constrain_index, 1);

    std::vector<std::vector<double>> default_start_vec;
    std::vector<std::vector<double>> default_goal_vec;
    std::vector<double> start_pose;
    std::vector<double> goal_pose;


    int experiment_num = 2;
    //提前测量好的4个不同约束姿势的初始角度
    if(experiment_num == 1){
        std::vector<double> start_pose0_master = {-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> start_pose0_slave = {-0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> start_pose1_master = {0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722};
        std::vector<double> start_pose1_slave = {-0.517204,-0.49348,1.79396,1.6679,0.302716,0.602833,-2.87906};
        std::vector<double> start_pose2_master = {0.75793,-0.888533,-2.01998,2.09217,-0.226114,-0.224376,2.39712};
        std::vector<double> start_pose2_slave = {-0.633959,-0.671778,2.03325,2.11988,0.716528,-0.334106,-3.05835};
        std::vector<double> start_pose3_master = {0.119411,-0.743597,-2.05219,2.14562,-0.840575,-1.39347,3.05791};
        std::vector<double> start_pose3_slave = {-0.119205,-0.74178,2.05281,2.14571,0.840017,-1.39411,-3.058};

        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose0_master.begin(), start_pose0_master.end());
        start_pose.insert(start_pose.end(), start_pose0_slave.begin(), start_pose0_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose1_master.begin(), start_pose1_master.end());
        start_pose.insert(start_pose.end(), start_pose1_slave.begin(), start_pose1_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose2_master.begin(), start_pose2_master.end());
        start_pose.insert(start_pose.end(), start_pose2_slave.begin(), start_pose2_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose3_master.begin(), start_pose3_master.end());
        start_pose.insert(start_pose.end(), start_pose3_slave.begin(), start_pose3_slave.end());
        default_start_vec.push_back(start_pose);

        std::vector<double> goal_pose0_master = {0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> goal_pose0_slave = {-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> goal_pose1_master = {0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696,3.05881};
        std::vector<double> goal_pose1_slave = {-0.640635,0.334855,1.43792,2.03636,-0.432353,0.425846,-3.05814};
        std::vector<double> goal_pose2_master = {0.378964,-0.235001,-1.09593,2.54013,0.629455,-0.946823,2.56304};
        std::vector<double> goal_pose2_slave = {-0.495711,0.18897,1.20076,2.51116,-0.470866,-0.757521,-3.05796};
        std::vector<double> goal_pose3_master = {0.188932,1.04619,-1.20316,2.47039,0.862578,-1.41834,-2.53144};
        std::vector<double> goal_pose3_slave = {-0.108729,1.0458,1.23816,2.45137,-0.881861,-1.45554,2.53844};

        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose0_master.begin(), goal_pose0_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose0_slave.begin(), goal_pose0_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose1_master.begin(), goal_pose1_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose1_slave.begin(), goal_pose1_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose2_master.begin(), goal_pose2_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose2_slave.begin(), goal_pose2_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose3_master.begin(), goal_pose3_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose3_slave.begin(), goal_pose3_slave.end());
        default_goal_vec.push_back(goal_pose);
    }
    else{
        //**************只用 pose1 能用 其他都是上一个的 ********************
        std::vector<double> start_pose0_master = {-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> start_pose0_slave = {-0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> start_pose1_master = {0.588793,0.459951,-1.6498,2.06105,-2.23376,-0.509111,-0.586783};
        std::vector<double> start_pose1_slave = {-0.588793,0.459951,1.6498,2.06105,2.23376,-0.509111,0.586783};
        std::vector<double> start_pose2_master = {0.75793,-0.888533,-2.01998,2.09217,-0.226114,-0.224376,2.39712};
        std::vector<double> start_pose2_slave = {-0.633959,-0.671778,2.03325,2.11988,0.716528,-0.334106,-3.05835};
        std::vector<double> start_pose3_master = {-0.420181,0.32788,-2.81828,1.76005,-0.253116,-1.34156,-1.82713};
        std::vector<double> start_pose3_slave = {0.943985,0.0770524,2.45456,1.32517,-0.174816,-1.36827,2.30339};

        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose0_master.begin(), start_pose0_master.end());
        start_pose.insert(start_pose.end(), start_pose0_slave.begin(), start_pose0_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose1_master.begin(), start_pose1_master.end());
        start_pose.insert(start_pose.end(), start_pose1_slave.begin(), start_pose1_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose2_master.begin(), start_pose2_master.end());
        start_pose.insert(start_pose.end(), start_pose2_slave.begin(), start_pose2_slave.end());
        default_start_vec.push_back(start_pose);
        start_pose.clear();
        start_pose.insert(start_pose.end(), start_pose3_master.begin(), start_pose3_master.end());
        start_pose.insert(start_pose.end(), start_pose3_slave.begin(), start_pose3_slave.end());
        default_start_vec.push_back(start_pose);

        std::vector<double> goal_pose0_master = {0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> goal_pose0_slave = {-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
        std::vector<double> goal_pose1_master = {0.134243,0.469875,-1.94033,1.40118,-2.33495,-0.759089,-0.357254};
        std::vector<double> goal_pose1_slave = {-0.134243,0.469875, 1.94033,1.40118, 2.33495,-0.759089, 0.357254};
        std::vector<double> goal_pose2_master = {0.378964,-0.235001,-1.09593,2.54013,0.629455,-0.946823,2.56304};
        std::vector<double> goal_pose2_slave = {-0.495711,0.18897,1.20076,2.51116,-0.470866,-0.757521,-3.05796};
        std::vector<double> goal_pose3_master = {-0.728359,0.978506,-2.05522,1.86281,0.820482,-1.30827,-2.61229};
        std::vector<double> goal_pose3_slave = {-0.486375,1.04586,1.643,2.22476,-0.254866,-0.820848,2.28068};

        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose0_master.begin(), goal_pose0_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose0_slave.begin(), goal_pose0_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose1_master.begin(), goal_pose1_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose1_slave.begin(), goal_pose1_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose2_master.begin(), goal_pose2_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose2_slave.begin(), goal_pose2_slave.end());
        default_goal_vec.push_back(goal_pose);
        goal_pose.clear();
        goal_pose.insert(goal_pose.end(), goal_pose3_master.begin(), goal_pose3_master.end());
        goal_pose.insert(goal_pose.end(), goal_pose3_slave.begin(), goal_pose3_slave.end());
        default_goal_vec.push_back(goal_pose);
    }


    std::string outputDir = "/home/lijiashushu/ros_ws/src/dual_rrt_star/result/experiment2/";
    std::string seedFileName = "seed.txt";
//    std::string dualFileDir = "/dual/";
//    std::string dualStarFileDir = "/dual_star/";
    std::string dualInformdStarFileDir = "/joint_space/";
    std::ifstream seedFile;
    std::ofstream dualFile, dualStarFile, dualInformedStarFile;

    JointSpacePlanner *planner = new JointSpacePlanner(nh);
    seedFile.open(outputDir + seedFileName);
    std::string number;
    int i=0;
    while(!seedFile.eof()){

        getline(seedFile, number);
//    for(int i=0; i<test_times; i++){
        seed = std::stoi(number);
        std::cout<<seed<<std::endl;
        //dual_informed_star
        planner->initPara(seed,
                          bi_probility,
                          max_sample_times,
                          max_planning_time,
                          task_step,
                          constrain_index,
                          default_start_vec[constrain_index],
                          default_goal_vec[constrain_index],
                          false,
                          false,
                          true,
                          true);
        dualInformedStarFile.open(outputDir + dualInformdStarFileDir + std::to_string(i) + ".txt");
        ROS_WARN("num %d", i++);
        planner->plan(dualInformedStarFile);
        dualInformedStarFile.close();
    }
    seedFile.close();

    delete planner;

    return 0;
}