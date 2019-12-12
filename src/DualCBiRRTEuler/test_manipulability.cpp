#include <ros/ros.h>
#include <baxter_moveit_application/DualCBiRRTEuler/dual_cbirrt.h>

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
#include <moveit/collision_detection/collision_tools.h>
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
#include <moveit/collision_detection/collision_common.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <kdl/frames.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "cbirrt_test");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    ros::Publisher collision_point_visual_publisher = n.advertise<visualization_msgs::MarkerArray>("/collision_point",1);

    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1 or collision_point_visual_publisher.getNumSubscribers()<1)
    {
        std::cout<<"\ndfsdfs\n"<<std::endl;
        std::cout<<"waiting.."<<std::endl;

        sleep_t.sleep();
    }


    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

    robot_state::RobotState robot_state = planning_scene_for_operate->getCurrentStateNonConst();

//    std::vector<double> left_test_value = {0.0010190456195,-0.541413503883,-8.95068892985e-05,0.748570361585,-0.000972739830137,1.25329285604,-7.44821020824e-05};
//    std::vector<double> right_test_value = {-0.00144350975696,-0.541413503826,-0.000113662662422,0.748576314717,-0.000463683012766,1.25329915969,0.0001414542156};

    std::vector<double> left_test_value = {0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722};
    std::vector<double> right_test_value = {-0.517204,-0.49348,1.79496,1.66956,0.302716,0.602833,-2.87906};

    const robot_state::JointModelGroup* left_group = robot_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* right_group = robot_state.getJointModelGroup("right_arm"); //

    //获取当前的关节角度

    robot_state.setJointGroupPositions(left_group, left_test_value);
    robot_state.setJointGroupPositions(right_group, right_test_value);
    robot_state.update();
    
    planning_scene_for_operate->setCurrentState(robot_state);

    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(1).sleep();

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = "both_arms";
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;
    c_req.distance = true;
    planning_scene_for_operate->checkCollision(c_req, c_res, robot_state);

    std::cout<<"???  "<<std::endl;
    std::cout<<"dis  "<<c_res.distance<<std::endl;
    std::cout<<"contact  "<<c_res.contacts.size()<<std::endl;
    DualCBiRRT my_planner(1.0, 0, 0.00, 0, planning_scene_for_operate, left_group, right_group);


    std::cout<<"???  "<<std::endl;
    std::cout<<"dis  "<<c_res.contacts.size()<<std::endl;
    collision_detection::DistanceRequest collision_req_master_world;
    collision_req_master_world.max_contacts_per_body = 1;
    collision_req_master_world.distance_threshold = 0.5;
    collision_req_master_world.enable_nearest_points = true;
    collision_req_master_world.enable_signed_distance = true;
    collision_req_master_world.active_components_only = &my_planner._master_link_model_set;
    collision_req_master_world.group_name = "left_arm";
    collision_req_master_world.type = collision_detection::DistanceRequestType::LIMITED;
    collision_detection::DistanceResult collision_res_master_world;
    my_planner.collision_world->distanceRobot(collision_req_master_world, collision_res_master_world, *my_planner.collision_robot, robot_state);
    collision_detection::DistanceMap master_dis_map = collision_res_master_world.distances;


    collision_detection::DistanceRequest collision_req_slave_world;
    collision_req_slave_world.max_contacts_per_body = 1;
    collision_req_slave_world.distance_threshold = 0.5;
    collision_req_slave_world.enable_nearest_points = true;
    collision_req_slave_world.enable_signed_distance = true;
    collision_req_slave_world.active_components_only = &my_planner._slave_link_model_set;
    collision_req_slave_world.group_name = "right_arm";
    collision_req_slave_world.type = collision_detection::DistanceRequestType::LIMITED;
    collision_detection::DistanceResult collision_res_slave_world;
    my_planner.collision_world->distanceRobot(collision_req_slave_world, collision_res_slave_world, *my_planner.collision_robot, robot_state);
    collision_detection::DistanceMap slave_dis_map = collision_res_slave_world.distances;

    Eigen::Vector4d move_dir(0, 0, -1, 0);
    double master_manipulability, slave_manipulability;


    visualization_msgs::MarkerArray robot_collision_points;
    visualization_msgs::MarkerArray object_collision_points;
    visualization_msgs::MarkerArray robot_object_lines;
    

    my_planner.init_all_dir_manipulability_compute_all_obstacle_one_dir_test(robot_state, master_dis_map, slave_dis_map, master_manipulability, slave_manipulability, move_dir, robot_collision_points, object_collision_points);
    std::cout<<"master_manipulability:  "<<master_manipulability<<std::endl;
    std::cout<<"slave_manipulability:  "<<slave_manipulability<<std::endl;
    
    for(size_t i=0; i<robot_collision_points.markers.size(); i++){
        visualization_msgs::Marker robot_object_line;
        robot_object_line.header.frame_id = "base";
        robot_object_line.header.stamp = ros::Time::now();
        robot_object_line.type = visualization_msgs::Marker::LINE_STRIP;
        robot_object_line.action = visualization_msgs::Marker::ADD;
        robot_object_line.id = 100+i;
        robot_object_line.ns = "collision_point";
        
        robot_object_line.color.a = 1.0;
        robot_object_line.color.r = 1.0;
        robot_object_line.scale.x = 0.005;
        
        geometry_msgs::Point robot_pt;
        robot_pt.x = robot_collision_points.markers[i].pose.position.x;
        robot_pt.y = robot_collision_points.markers[i].pose.position.y;
        robot_pt.z = robot_collision_points.markers[i].pose.position.z;
        geometry_msgs::Point object_pt;
        object_pt.x = object_collision_points.markers[i].pose.position.x;
        object_pt.y = object_collision_points.markers[i].pose.position.y;
        object_pt.z = object_collision_points.markers[i].pose.position.z;

        robot_object_line.points.push_back(robot_pt);
        robot_object_line.points.push_back(object_pt);

        robot_object_lines.markers.push_back(robot_object_line);
    }

    collision_point_visual_publisher.publish(robot_collision_points);
    ros::Duration(0.5).sleep();
    collision_point_visual_publisher.publish(object_collision_points);
    ros::Duration(0.5).sleep();
    collision_point_visual_publisher.publish(robot_object_lines);
    ros::Duration(0.5).sleep();
    return 0;
}
