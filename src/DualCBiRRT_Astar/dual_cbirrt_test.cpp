#include <ros/ros.h>
#include <baxter_moveit_application/DualCBiRRT_Astar/dual_cbirrt.h>

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

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <algorithm>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include "baxter_moveit_application/DualCBiRRT_Astar/Astar_searcher.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "cbirrt_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::srand((unsigned)time(NULL));

    DualCBiRRT my_planner(1.0 , rand(), 0.00);

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

    robot_state::RobotState start_state = planning_scene_for_operate->getCurrentStateNonConst();
    //Z:180 Y:90 X:-90   2.94792  1.56999 -1.76536


// 这些点都有奇异问题
//    std::vector<double> test_start_value = {0.178307,-1.36637,-0.718743,2.32057,-1.28874,1.62442,2.4651};//有障碍时手臂平方位置
//    std::vector<double> test_start_value = {0.17109754, -0.87923624, -0.08423487,  1.712199,   -0.81049842,  2.09320188,  2.58848987}; //无障碍时手臂平方位置
//    std::vector<double> slave_test_start_value = {0.0633710, 0.118378, 1.5027523, 2.2347026,-0.579105, 0.054547, -1.11615}; //无障碍时手臂平方位置
//    std::vector<double> test_start_value = {0.178307,-1.36637,-0.718743,2.32057,-1.28874,1.62442,2.4651}; //有障碍物测试
//    std::vector<double> slave_test_start_value = {-0.644661 , 0.255123 ,  1.83284 ,  2.19888,  -0.36092  , 0.90258  , -1.1066}; //有障碍物测试
//    std::vector<double> test_start_value = {-0.202391,-1.01283,-0.709538,1.16068,-1.21936,1.51294,1.59967}; //narrow障碍物的起始左臂位置
//    std::vector<double> slave_test_start_value = {-0.0273947,-0.113638,2.14442,0.981496,-0.31,1.45411,-1.02899};//narrow障碍物的起始右臂位置

    std::vector<double> test_start_value = {-0.375463,-1.09228,-0.440484,1.20106,1.76769,-1.57028,0.0672226}; //没有奇异、narrow障碍物的起始左臂位置
    std::vector<double> slave_test_start_value = {-0.115289,-0.393004,1.72106,1.01171,-2.93258,-1.39411,0.332235};//没有奇异、narrow障碍物的起始右臂位置


    std::vector<double> both_start_value;
    const robot_state::JointModelGroup* planning_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* slave_group = start_state.getJointModelGroup("right_arm"); //
    const robot_state::JointModelGroup* both_group = start_state.getJointModelGroup("both_arms"); //


    //获取当前的关节角度
    std::vector<double> tmp_display;
    start_state.copyJointGroupPositions(both_group, tmp_display);
    for(size_t i=0; i<tmp_display.size();i++){
        std::cout<<tmp_display[i]<<",";
    }
    std::cout<<std::endl;

    start_state.setJointGroupPositions(planning_group, test_start_value);
    start_state.setJointGroupPositions(slave_group, slave_test_start_value);
    start_state.copyJointGroupPositions(both_group, both_start_value);
    Eigen::Matrix<double ,14, 1 >both_start_value_matrix;
    for(size_t i=0; i<14; i++){
        both_start_value_matrix[i] = both_start_value[i];
    }
    std::cout<<"both_start_value_matrix\n"<<both_start_value_matrix.transpose()<<std::endl;

    const Eigen::Affine3d & left_end_pose_tmp = start_state.getGlobalLinkTransform("left_gripper");
    auto left_end_rot_matrix_tmp = left_end_pose_tmp.rotation();
    auto left_euler_tmp = left_end_rot_matrix_tmp.eulerAngles(2,1,0);
    auto left_end_pos = left_end_pose_tmp.translation();
    std::cout<<"left_euler  "<<left_euler_tmp.transpose()<<std::endl;
    std::cout<<"left_end_pos  "<<left_end_pos.transpose()<<std::endl;

    const Eigen::Affine3d & right_end_pose_tmp = start_state.getGlobalLinkTransform("right_gripper");
    auto right_end_rot_matrix_tmp = right_end_pose_tmp.rotation();
    auto right_euler_tmp = right_end_rot_matrix_tmp.eulerAngles(2,1,0);
    auto right_end_pos = right_end_pose_tmp.translation();
    std::cout<<"right_euler  "<<right_euler_tmp.transpose()<<std::endl;
    std::cout<<"right_end_pos  "<<right_end_pos.transpose()<<std::endl;



    planning_scene_for_operate->setCurrentState(start_state);
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(1).sleep();


    robot_state::RobotState goal_state = planning_scene_for_operate->getCurrentStateNonConst();

//    std::vector<double> test_goal_value = {-0.53121395, -1.14663671 , 0.21698349  ,2.33939883 ,-1.17448029  ,1.81105335,  2.82284528};//无障碍时手臂平方的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.64966, 0.0056597, 1.453030, 2.2167859, 0.0142739, 0.7887366, -1.69753346};//无障碍时手臂平方的目标右臂位置
//    std::vector<double> test_goal_value = {0.0511426,-0.422846,-0.602817,1.92707,-0.888771,1.20479,2.70597}; //平板类似桌子的障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.614005,  0.611334 ,  1.40829,   1.80571, -0.631447,   1.11582,  -1.56488}; //平板类似桌子的障碍物的目标右臂位置
//    std::vector<double> test_goal_value = {-0.0813673,-0.68199,-0.637715,1.9482,-1.13503,1.24992,2.59584};//narrow障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.485412,0.487359,1.66579,1.6767,-0.522427,1.24843,-1.42944};//narrow障碍物的目标右臂位置

    std::vector<double> test_goal_value = {-0.233357,-0.754374,-0.490762,1.95377,1.90675,-1.34839,1.06295};//没有奇异、narrow障碍物的目标左臂位置
    std::vector<double> slave_test_goal_value = {-0.446697,-0.0863082,1.24614,1.77273,-2.93228,-1.08041,-0.381265};//没有奇异、narrow障碍物的目标右臂位置



    goal_state.setJointGroupPositions(planning_group, test_goal_value);
    goal_state.setJointGroupPositions(slave_group, slave_test_goal_value);

    auto left_test = goal_state.getGlobalLinkTransform("left_gripper");
    auto left_rotation = left_test.rotation();
    auto left_euler = left_rotation.eulerAngles(2, 1, 0);
    auto left_pos = left_test.translation();
    std::cout<<"left_euler\n"<<left_euler.transpose()<<std::endl;
    std::cout<<"left_pos\n"<<left_pos.transpose()<<std::endl;

    auto right_test = goal_state.getGlobalLinkTransform("right_gripper");
    auto right_rotation = right_test.rotation();
    auto right_euler = right_rotation.eulerAngles(2, 1, 0);
    auto right_pos = right_test.translation();
    std::cout<<"right_euler\n"<<right_euler.transpose()<<std::endl;
    std::cout<<"right_pos\n"<<right_pos.transpose()<<std::endl;

    Eigen::Vector3d slave_goal_euler(left_euler[0], left_euler[1] + 3.1415926, left_euler[2] - 3.1415926);
    std::cout<<"slave_goal_euler\n"<<slave_goal_euler.transpose()<<std::endl;
    Eigen::AngleAxisd goal_roll_angle(Eigen::AngleAxisd(slave_goal_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd goal_pitch_angle(Eigen::AngleAxisd(slave_goal_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd goal_yaw_angle(Eigen::AngleAxisd(slave_goal_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_goal_rot_matrix;
    slave_goal_rot_matrix = goal_yaw_angle*goal_pitch_angle*goal_roll_angle;
    Eigen::Vector3d slave_goal_euler2 = slave_goal_rot_matrix.eulerAngles(2,1,0);
    std::cout<<"slave_goal_euler2\n"<<slave_goal_euler2.transpose()<<std::endl;



    //***************************Astar*******************************************
    double _resolution, _inv_resolution, _cloud_margin;
    double _x_size, _y_size, _z_size;
    Eigen::Vector3d _start_pt;
    Eigen::Vector3d _map_lower, _map_upper;
    int _max_x_id, _max_y_id, _max_z_id;
    nh.param("map/resolution",    _resolution,   0.05);

    nh.param("map/x_size",        _x_size, 1.50);
    nh.param("map/y_size",        _y_size, 2.0);
    nh.param("map/z_size",        _z_size, 1.50 );

    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << 0.0, - _y_size/2.0,     0.0;  //地图的上下界限
    _map_upper << _x_size, + _y_size/2.0, _z_size;

    _inv_resolution = 1.0 / _resolution;

    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Publisher _obstacle_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("obstacle_nodes_vis",1);
    double safe_margin_x = 0.1;
    double safe_margin_y = 0.05;
    double safe_margin_z = 0.05;

    std::vector<moveit_msgs::CollisionObject> object_in_the_world;
    planning_scene_for_operate->getCollisionObjectMsgs(object_in_the_world);

    std::cout<<"object_in_the_world size:  "<<object_in_the_world.size()<<std::endl;
    for(auto it=object_in_the_world.begin(); it!=object_in_the_world.end();it++){
        std::vector<shape_msgs::SolidPrimitive> all_shape_items = it->primitives;
        std::vector<geometry_msgs::Pose> all_shape_items_poses = it->primitive_poses;
        for(size_t i =0; i<all_shape_items.size(); i++){
            visualization_msgs::Marker obstacle_vis;
            obstacle_vis.header.frame_id = "base";
            obstacle_vis.header.stamp = ros::Time::now();
            obstacle_vis.type = visualization_msgs::Marker::CUBE_LIST;
            obstacle_vis.action = visualization_msgs::Marker::ADD;
            obstacle_vis.id = 1;
            obstacle_vis.ns = "demo_node/obstacle_grids";
            obstacle_vis.pose.orientation.x = 0.0;
            obstacle_vis.pose.orientation.y = 0.0;
            obstacle_vis.pose.orientation.z = 0.0;
            obstacle_vis.pose.orientation.w = 1.0;

            obstacle_vis.color.a = 0.3;
            obstacle_vis.color.r = 0.0;
            obstacle_vis.color.g = 0.0;
            obstacle_vis.color.b = 1.0;

            obstacle_vis.scale.x = _resolution;
            obstacle_vis.scale.y = _resolution;
            obstacle_vis.scale.z = _resolution;


            Eigen::Vector3d ob_max_pos;
            Eigen::Vector3d ob_min_pos;
            if(all_shape_items[i].type == shape_msgs::SolidPrimitive::BOX){
                double center_x = all_shape_items_poses[i].position.x;
                double center_y = all_shape_items_poses[i].position.y;
                double center_z = all_shape_items_poses[i].position.z;
                double x_length = all_shape_items[i].dimensions[shape_msgs::SolidPrimitive::BOX_X];
                double y_length = all_shape_items[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                double z_length = all_shape_items[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
                std::cout<<"positioin:  "<<center_x<<" "<<center_y<<" "<<center_z<<std::endl;
                std::cout<<"size:  "<<x_length<<" "<<y_length<<" "<<z_length<<std::endl;
                ob_max_pos << center_x + 0.5*x_length, center_y + 0.5*y_length , center_z + 0.5*z_length;
                ob_min_pos << center_x - 0.5*x_length, center_y - 0.5*y_length , center_z - 0.5*z_length;
                std::cout<<"ob_max:  "<<ob_max_pos.transpose()<<std::endl;
                std::cout<<"ob_min:  "<<ob_min_pos.transpose()<<std::endl;


                ob_max_pos[0] += safe_margin_x;
                ob_max_pos[1] += safe_margin_y;
                ob_max_pos[2] += safe_margin_z;
                ob_min_pos[0] -= safe_margin_x;
                ob_min_pos[1] -= safe_margin_y;
                ob_min_pos[2] -= safe_margin_z;
                std::cout<<"ob_max:  "<<ob_max_pos.transpose()<<std::endl;
                std::cout<<"ob_min:  "<<ob_min_pos.transpose()<<std::endl;
                ob_min_pos = _astar_path_finder->coordRounding(ob_min_pos);
                ob_max_pos = _astar_path_finder->coordRounding(ob_max_pos);
            }
            geometry_msgs::Point pt;
            Eigen::Vector3d coord;
            Eigen::Vector3d coord_round;
            for(double ii = ob_min_pos[0]; ii<ob_max_pos[0]; ii += _resolution){
                for(double jj = ob_min_pos[1]; jj<ob_max_pos[1]; jj += _resolution){
                    for(double kk = ob_min_pos[2]; kk<ob_max_pos[2]; kk += _resolution){
                        _astar_path_finder->setObs(ii, jj ,kk);
//                        coord(0) = ii;
//                        coord(1) = jj;
//                        coord(2) = kk;
//                        coord_round = _astar_path_finder->coordRounding(coord);
//                        pt.x = coord_round(0);
//                        pt.y = coord_round(1);
//                        pt.z = coord_round(2);
                        pt.x = ii;
                        pt.y = jj;
                        pt.z = kk;
                        obstacle_vis.points.push_back(pt);
                    }
                }
            }
            _obstacle_nodes_vis_pub.publish(obstacle_vis);
        }
    }

    //*********************************Astar end line***********************************************

    ros::Time time_1 = ros::Time::now();
//    if(my_planner.plan_task_space_dir(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group)){
    if(my_planner.plan_task_space_dir_astar(goal_state, start_state, planning_scene_for_operate, "left_arm", planning_group, slave_group, _astar_path_finder)){
    std::cout<<"???"<<std::endl;
    }
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("Planning time  is %f ms", (time_2 - time_1).toSec() * 1000.0);

    my_planner.output_perdex();
    std::vector<robot_state::RobotState> result = my_planner.planning_result;
    std::vector<size_t> result_index = my_planner.planning_result_index;
    std::vector<Eigen::Vector3d> result_state_vector = my_planner.planning_result_task_state_vector;
    ROS_INFO("waypoints num is %d", int(result.size()));


    //*********************整体都是在添加轨迹显示的消息内容太*****************************
    std::vector<geometry_msgs::Pose> result_pose;
    moveit_msgs::RobotTrajectory result_msg;
    trajectory_msgs::JointTrajectory path_point_msg;
    trajectory_msgs::JointTrajectoryPoint path_point_position_msg; //只需要添加关节位置点
    for(size_t i=0; i<result.size(); i++){
        std::vector<double> tmp1;
        std::vector<double> tmp2;
        result[i].copyJointGroupPositions(planning_group, tmp1);
        result[i].copyJointGroupPositions(slave_group, tmp2);
        tmp1.insert(tmp1.end(), tmp2.begin(), tmp2.end());
        path_point_position_msg.positions = tmp1;
        path_point_msg.points.push_back(path_point_position_msg);

//        if(i<result.size()-1){
//            std::cout<<"norm: "<<(result_state_vector[i+1] - result_state_vector[i]).norm()<<std::endl;
//            std::cout<<"index1: "<<result_index[i] <<" index2 "<< result_index[i+1] <<std::endl;
//            std::cout<<"vec1: "<<result_state_vector[i].transpose() <<" vec2 "<< result_state_vector[i+1].transpose() <<std::endl;
//        }


        //就这一部分是添加姿态
        if(i%5==0) {
            geometry_msgs::Pose tmp_pose_msg;
            const Eigen::Affine3d end_pose = result[i].getGlobalLinkTransform("left_gripper");
            Eigen::Quaterniond end_quaternion(end_pose.rotation());
            tmp_pose_msg.position.x = end_pose(0, 3);
            tmp_pose_msg.position.y = end_pose(1, 3);
            tmp_pose_msg.position.z = end_pose(2, 3);
            tmp_pose_msg.orientation.x = end_quaternion.x();
            tmp_pose_msg.orientation.y = end_quaternion.y();
            tmp_pose_msg.orientation.z = end_quaternion.z();
            tmp_pose_msg.orientation.w = end_quaternion.w();
            result_pose.push_back(tmp_pose_msg);
        }
    }
    //********************************************************************

    //*********************显示扩展成功的 master 的末端位置点*****************************




    const std::vector<std::pair<robot_state::RobotState, size_t>> & a_state_tree = my_planner.get_tree_state_vector(true);
    int a_tree_size = a_state_tree.size();
    const std::vector<std::pair<robot_state::RobotState, size_t>> & b_state_tree = my_planner.get_tree_state_vector(false);
    int b_tree_size = b_state_tree.size();

    int display_num = a_tree_size + b_tree_size;
    int step_size = 0;
    step_size = 1;
    std::vector<geometry_msgs::Point> extend_states_displaya;
    std::vector<std_msgs::ColorRGBA> extend_states_display_colora;
    std::vector<geometry_msgs::Point> extend_states_displayb;
    std::vector<std_msgs::ColorRGBA> extend_states_display_colorb;
    ROS_INFO("a_tree_size: %d", a_tree_size);
    ROS_INFO("b_tree_size: %d", b_tree_size);
    int count = 0;
    Eigen::Vector3d tmp;
    while(count <  a_tree_size){
        tmp = a_state_tree[count].first.getGlobalLinkTransform("left_gripper").translation();
        geometry_msgs::Point tmp_point_msg;
        tmp_point_msg.x = tmp[0];
        tmp_point_msg.y = tmp[1];
        tmp_point_msg.z = tmp[2];
        extend_states_displaya.push_back(tmp_point_msg);
        std_msgs::ColorRGBA tmp_color;
        tmp_color.b = 1.0;
        tmp_color.a = 1.0;
        extend_states_display_colora.push_back(tmp_color);
        count += step_size;
    }
    count = 0;
    while(count <  b_tree_size){
        tmp = b_state_tree[count].first.getGlobalLinkTransform("left_gripper").translation();
        geometry_msgs::Point tmp_point_msg;
        tmp_point_msg.x = tmp[0];
        tmp_point_msg.y = tmp[1];
        tmp_point_msg.z = tmp[2];
        extend_states_displayb.push_back(tmp_point_msg);
        std_msgs::ColorRGBA tmp_color;
        tmp_color.r = 1.0;
        tmp_color.a = 1.0;
        extend_states_display_colorb.push_back(tmp_color);
        count += step_size;
    }


    //*********************显示规划的轨迹*****************************
    const std::vector<std::string>& master_joint_names = planning_group->getVariableNames();
    for(size_t i=0; i<master_joint_names.size(); i++){
        path_point_msg.joint_names.push_back(master_joint_names[i]);
    }
    const std::vector<std::string>& slave_joint_names = slave_group->getVariableNames();
    for(size_t i=0; i<slave_joint_names.size(); i++){
        path_point_msg.joint_names.push_back(slave_joint_names[i]);
    }

    path_point_msg.header.stamp = ros::Time::now();
    result_msg.joint_trajectory = path_point_msg;

    visualization_msgs::Marker delete_all_markers;
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(result_msg, planning_group);
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Simple RRT", rvt::WHITE, rvt::XLARGE);
    for (std::size_t i = 0; i < result_pose.size(); ++i)
        visual_tools.publishAxisLabeled(result_pose[i], "pt" + std::to_string(i), rvt::SMALL);

    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;
    visual_tools.publishSpheres(extend_states_displaya, extend_states_display_colora, scale);
    visual_tools.publishSpheres(extend_states_displayb, extend_states_display_colorb, scale);
    visual_tools.trigger();



    // 创建一个DisplayTrajectory msg
    moveit_msgs::DisplayTrajectory display_traj_msg;
    moveit_msgs::RobotState start_state_msg;
    sensor_msgs::JointState start_angles_msg;

    //添加这个消息的第三个参数，开始的状态
    for(size_t i=0; i<master_joint_names.size(); i++){
        start_angles_msg.name.push_back(master_joint_names[i]);
        start_angles_msg.position.push_back(test_start_value[i]);
    }
    for(size_t i=0; i<slave_joint_names.size(); i++){
        start_angles_msg.name.push_back(slave_joint_names[i]);
        start_angles_msg.position.push_back(slave_test_start_value[i]);
    }
    start_state_msg.joint_state = start_angles_msg;
    display_traj_msg.trajectory_start = start_state_msg;

    //添加这个消息的第二个参数，开始的状态，可能显示多条轨迹所以是向量
    display_traj_msg.trajectory.push_back(result_msg);

    ros::Publisher display_traj_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    while (display_traj_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    display_traj_publisher.publish(display_traj_msg);
    ros::Duration(1).sleep();


    return 0;
}