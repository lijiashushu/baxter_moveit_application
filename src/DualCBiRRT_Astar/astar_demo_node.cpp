#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "baxter_moveit_application/DualCBiRRT_Astar/Astar_searcher.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
using namespace std;
using namespace Eigen;


namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _obstacle_nodes_vis_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes );
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);


void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    ROS_INFO("[node] ready to search");
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    visGridPath (grid_path, false);
    visVisitedNode(visited_nodes);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    _obstacle_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("obstacle_nodes_vis",1);

    ros::WallDuration sleep_t(0.5);
    while (_grid_path_vis_pub.getNumSubscribers() < 1 or _obstacle_nodes_vis_pub.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

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


    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);


    double safe_margin_x = 0.1;
    double safe_margin_y = 0.05;
    double safe_margin_z = 0.05;

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();

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

    Vector3d start_pt;
    start_pt<<0.717443, -0.0286898,    0.66986;
    Vector3d target_pt;
    target_pt<<0.580433, -0.0316257,   0.284839;

    pathFinding(start_pt, target_pt);


    delete _astar_path_finder;
    return 0;
}

void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "base";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}