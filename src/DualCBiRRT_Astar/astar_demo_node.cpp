
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
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
#include <kdl/frames.hpp>

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


std::vector<double> left_euler_offset_angles = {0, 0.52, 1.05, 1.57};
double _error_coefficient = 0.5;


void rcvWaypointsCallback(const nav_msgs::Path & wp);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes );
void pathFinding(const Vector3d start_pt, const Vector3d target_pt, vector<Vector3d>& astar_path);
void set_scene_object_as_obstacle(planning_scene::PlanningScenePtr & planning_scene_ptr);


void pathFinding(const Vector3d start_pt, const Vector3d target_pt, vector<Vector3d>& astar_path)
{
    ROS_INFO("[node] ready to search");
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);

    //Retrieve the path
    astar_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    visGridPath (astar_path, false);
    visVisitedNode(visited_nodes);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();
}

bool solve_dual_arm_IK(vector<Vector3d> & astar_path, planning_scene::PlanningScenePtr & planning_scene_ptr, std::vector<std::vector<Eigen::Matrix<double, 14, 1>>> & ik_results_along_the_path_all_posture){
    Eigen::Vector3d left_ref_eluer(0, 0, 1.57);

    std::vector<Eigen::Matrix<double, 14, 1>> ik_results_along_the_path_for_one_path_point;
//    std::vector<std::vector<Eigen::Matrix<double, 14, 1>>> ik_results_along_the_path_all_postures;
    robot_state::RobotState start_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState left_state = planning_scene_ptr->getCurrentStateNonConst();
    robot_state::RobotState right_state = planning_scene_ptr->getCurrentStateNonConst();

    std::vector<double> current_joint_angles_vector;
    Eigen::Matrix<double, 14, 1> current_joint_angles_matrix;
    start_state.copyJointGroupPositions("both_arms", current_joint_angles_vector);
    for(size_t i=0; i<14; i++){
        current_joint_angles_matrix(i) = current_joint_angles_vector[i];
    }
    const robot_state::JointModelGroup* left_group = start_state.getJointModelGroup("left_arm"); //
    const robot_state::JointModelGroup* right_group = start_state.getJointModelGroup("right_arm");
    
    //*********左臂相关变量定义**************
    std::vector<double> left_joint_angles_vector(7);
    Eigen::Matrix<double ,7,1>  left_joint_angles_matrix;
    Eigen::Matrix<double ,7,1>  left_next_joint_angles_matrix;
    left_joint_angles_matrix = current_joint_angles_matrix.head(7);
    left_next_joint_angles_matrix = left_joint_angles_matrix;

    Eigen::Vector3d left_target_pos;
    Eigen::Vector3d left_target_euler;
    Eigen::Matrix3d left_target_rot_matrix;
    
    Eigen::Vector3d left_current_pos;
    Eigen::Vector3d left_current_euler;
    Eigen::Matrix3d left_current_rot_matrix;
    Eigen::Vector3d left_next_pos;
    Eigen::Vector3d left_next_euler;
    Eigen::Matrix3d left_next_rot_matrix;
    
    Eigen::Vector3d left_error_pos;
    Eigen::Vector3d left_error_euler;
    Eigen::Matrix3d left_rot_error_matrix;
    Eigen::AngleAxisd left_rot_error_axis_angle;
    double left_rot_error_angle;
    Eigen::Vector3d left_rot_error_3vector;
    Eigen::Matrix<double ,6,1> left_stacked_vector;

    Eigen::MatrixXd left_end_jacobian;
    Eigen::MatrixXd left_end_jacobian_mp_inverse;
    
    const Eigen::Affine3d & left_end_pose_tmp = left_state.getGlobalLinkTransform("left_gripper");

    left_current_pos = left_end_pose_tmp.translation();
    left_current_rot_matrix = left_end_pose_tmp.rotation();
//    left_current_euler = left_current_rot_matrix.eulerAngles(2,1,0);
    KDL::Rotation left_tmp_conversion(left_current_rot_matrix(0,0), left_current_rot_matrix(0,1), left_current_rot_matrix(0,2),
                                left_current_rot_matrix(1,0), left_current_rot_matrix(1,1), left_current_rot_matrix(1,2),
                                left_current_rot_matrix(2,0), left_current_rot_matrix(2,1), left_current_rot_matrix(2,2));
    left_tmp_conversion.GetEulerZYX(left_current_euler(0), left_current_euler(1), left_current_euler(2));
    
    
    left_next_pos = left_current_pos;
    left_next_euler = left_current_euler;
    left_next_rot_matrix = left_current_rot_matrix;

    Eigen::Matrix<double ,7,1>  left_joint_delta_vector;
    bool left_ik_success;
    //*****************************************************************************************************************************
    //*********右臂相关变量定义**********************************************************************************************************
    std::vector<double> right_joint_angles_vector(7);
    Eigen::Matrix<double ,7,1>  right_joint_angles_matrix;
    Eigen::Matrix<double ,7,1>  right_next_joint_angles_matrix;
    right_joint_angles_matrix = current_joint_angles_matrix.tail(7);
    right_next_joint_angles_matrix = right_joint_angles_matrix;
    

    Eigen::Vector3d right_target_pos;
    Eigen::Vector3d right_target_euler;
    Eigen::Matrix3d right_target_rot_matrix;

    Eigen::Vector3d right_current_pos;
    Eigen::Vector3d right_current_euler;
    Eigen::Matrix3d right_current_rot_matrix;
    Eigen::Vector3d right_next_pos;
    Eigen::Vector3d right_next_euler;
    Eigen::Matrix3d right_next_rot_matrix;
    
    Eigen::Vector3d right_error_pos;
    Eigen::Vector3d right_error_euler;
    Eigen::Matrix3d right_rot_error_matrix;
    Eigen::AngleAxisd right_rot_error_axis_angle;
    double right_rot_error_angle;
    Eigen::Vector3d right_rot_error_3vector;
    Eigen::Matrix<double ,6,1> right_stacked_vector;

    Eigen::MatrixXd right_end_jacobian;
    Eigen::MatrixXd right_end_jacobian_mp_inverse;
    
    const Eigen::Affine3d & right_end_pose_tmp = right_state.getGlobalLinkTransform("right_gripper");
    
    right_current_pos = right_end_pose_tmp.translation();
    right_current_rot_matrix = right_end_pose_tmp.rotation();
//    right_current_euler = right_current_rot_matrix.eulerAngles(2,1,0);
    KDL::Rotation right_tmp_conversion(right_current_rot_matrix(0,0), right_current_rot_matrix(0,1), right_current_rot_matrix(0,2),
                                      right_current_rot_matrix(1,0), right_current_rot_matrix(1,1), right_current_rot_matrix(1,2),
                                      right_current_rot_matrix(2,0), right_current_rot_matrix(2,1), right_current_rot_matrix(2,2));
    right_tmp_conversion.GetEulerZYX(right_current_euler(0), right_current_euler(1), right_current_euler(2));
    right_next_pos = right_current_pos;
    right_next_euler = right_current_euler;
    right_next_rot_matrix = right_current_rot_matrix;
    
    Eigen::Matrix<double ,7,1>  right_joint_delta_vector;
    bool right_ik_success;
    //*********************************

    Eigen::Vector3d distance(0, 0, 0.06);
    for(size_t ii=0; ii<astar_path.size(); ii++){
        for(size_t j=0; j<left_euler_offset_angles.size(); j++){
            //先尝试一下每一个都算出来准确的角度值（但这样的话是不是可能会没有解呀，所以还是先计算左臂到可行的约束再计算右臂？？？？？）
            
            left_target_pos = astar_path[ii];
            left_target_euler = left_ref_eluer;
            left_target_euler[0] +=  left_euler_offset_angles[j];
            Eigen::AngleAxisd left_goal_roll_angle(Eigen::AngleAxisd(left_target_euler[2], Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd left_goal_pitch_angle(Eigen::AngleAxisd(left_target_euler[1], Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd left_goal_yaw_angle(Eigen::AngleAxisd(left_target_euler[0], Eigen::Vector3d::UnitZ()));
            left_target_rot_matrix = left_goal_yaw_angle*left_goal_pitch_angle*left_goal_roll_angle;
            
            left_current_pos = left_next_pos;
            left_current_euler = left_next_euler;
            left_current_rot_matrix = left_next_rot_matrix;
            left_joint_angles_matrix = left_next_joint_angles_matrix;
            std::cout<<"left_joint_angles_matrix\n"<<left_joint_angles_matrix.transpose()<<std::endl;

//            std::cout<<"left_target_pos "<<left_target_pos.transpose()<<std::endl;
//            std::cout<<"left_target_euler "<<left_target_euler.transpose()<<std::endl;
            std::cout<<"\nnew loop!"<<std::endl;
            std::cout<<"left_target_euler\n"<<left_target_euler.transpose()<<std::endl;


            while (true){

                left_error_pos = left_target_pos - left_current_pos;
                left_error_euler = left_target_euler - left_current_euler;
                std::cout<<"left_error_euler\n"<<left_error_euler.transpose()<<std::endl;
                left_rot_error_matrix = left_target_rot_matrix * (left_current_rot_matrix.inverse());
                left_rot_error_axis_angle = left_rot_error_matrix;
                std::cout<<"angle:  "<<left_rot_error_axis_angle.angle()<<std::endl;

                if(left_error_pos.norm() < 0.01 && (left_rot_error_axis_angle.angle()<0.01 || left_error_euler.norm()<0.01)){
                    left_next_pos = left_current_pos;
                    left_next_euler = left_current_euler;
                    left_next_rot_matrix = left_current_rot_matrix;
                    left_next_joint_angles_matrix = left_joint_angles_matrix;
                    
                    left_ik_success = true;
                    current_joint_angles_matrix.head(7) = left_joint_angles_matrix;
                    std::cout<<"left computing IK success"<<std::endl;
                    break;
                }
                else{
                    
                    left_rot_error_3vector = left_rot_error_axis_angle.angle() * left_rot_error_axis_angle.axis();
                    left_stacked_vector.head(3) = left_error_pos;
                    left_stacked_vector.tail(3) = left_rot_error_3vector;
                    left_stacked_vector = (_error_coefficient * left_stacked_vector) / 0.01;

                    left_end_jacobian = left_state.getJacobian(left_group);
                    left_end_jacobian_mp_inverse = (left_end_jacobian.transpose() * ((left_end_jacobian * left_end_jacobian.transpose() + 0.0001 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
                    left_joint_delta_vector = left_end_jacobian_mp_inverse * left_stacked_vector;
                    left_joint_delta_vector *= 0.01;
                    if(left_joint_delta_vector.norm() < 0.00001){
                        std::cout<<"left computing IK fail, norm too small!!"<<std::endl;
                        left_ik_success = false;
                        current_joint_angles_matrix.head(7) = Eigen::Matrix<double,7,1>::Zero();
                        break;
                    }
                    else{
                        left_joint_angles_matrix += left_joint_delta_vector;
                        for(size_t i=0; i<7; i++){
                            left_joint_angles_vector[i] = left_joint_angles_matrix[i];
                        }
                        left_state.setJointGroupPositions(left_group, left_joint_angles_vector);
                        left_state.update();

                        if(left_state.satisfiesBounds(left_group, 0.05)){
                            //更新状态
                            const Eigen::Affine3d & left_end_pose_tmp = left_state.getGlobalLinkTransform("left_gripper");
                            left_current_rot_matrix = left_end_pose_tmp.rotation();
                            left_current_pos = left_end_pose_tmp.translation();
                            KDL::Rotation left_tmp_conversion(left_current_rot_matrix(0,0), left_current_rot_matrix(0,1), left_current_rot_matrix(0,2),
                                                              left_current_rot_matrix(1,0), left_current_rot_matrix(1,1), left_current_rot_matrix(1,2),
                                                              left_current_rot_matrix(2,0), left_current_rot_matrix(2,1), left_current_rot_matrix(2,2));
                            left_tmp_conversion.GetEulerZYX(left_current_euler(0), left_current_euler(1), left_current_euler(2));
                        }
                        else{
                            std::cout<<"left computing IK fail, nexceed joint limits!!"<<std::endl;
                            left_ik_success = false;
                            current_joint_angles_matrix.head(7) = Eigen::Matrix<double,7,1>::Zero();
                            break;
                        }
                    }
                }
            }

            distance(0) = -0.06 * std::sin(left_euler_offset_angles[j]);
            distance(1) = 0;
            distance(2) = 0.06 * std::cos(left_euler_offset_angles[j]);
            
            //如果求解成功，就使用解对应的位置，如果失败就是用原来的目标位置
            if(left_ik_success){
                right_target_pos = left_current_rot_matrix * distance + left_current_pos;
                right_target_euler[0] = left_current_euler[0] - 2 * left_euler_offset_angles[j];
                right_target_euler[1] = left_current_euler[1];
                right_target_euler[2] = left_current_euler[2] - 3.14;
            }
            else{
                right_target_pos = left_target_rot_matrix * distance + left_target_pos;
                right_target_euler[0] = left_target_euler[0] - 2 * left_euler_offset_angles[j];
                right_target_euler[1] = left_target_euler[1];
                right_target_euler[2] = left_target_euler[2] - 3.14;
            }
            Eigen::AngleAxisd right_goal_roll_angle(Eigen::AngleAxisd(right_target_euler[2], Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd right_goal_pitch_angle(Eigen::AngleAxisd(right_target_euler[1], Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd right_goal_yaw_angle(Eigen::AngleAxisd(right_target_euler[0], Eigen::Vector3d::UnitZ()));
            right_target_rot_matrix = right_goal_yaw_angle*right_goal_pitch_angle*right_goal_roll_angle;
            
            right_current_pos = right_next_pos;
            right_current_euler = right_next_euler;
            right_current_rot_matrix = right_next_rot_matrix;
            right_joint_angles_matrix = right_next_joint_angles_matrix;
            std::cout<<"right_joint_angles_matrix\n"<<right_joint_angles_matrix.transpose()<<std::endl;
//            std::cout<<"right_target_pos "<<right_target_pos.transpose()<<std::endl;
//            std::cout<<"right_target_euler "<<right_target_euler.transpose()<<std::endl;
            while (true){

                right_error_pos = right_target_pos - right_current_pos;
                right_error_euler = right_target_euler - right_current_euler;
                right_rot_error_matrix = right_target_rot_matrix * (right_current_rot_matrix.inverse());
                right_rot_error_axis_angle = right_rot_error_matrix;


                if(right_error_pos.norm() < 0.03 && (right_rot_error_axis_angle.angle()<0.01 || right_error_euler.norm()<0.01)){
                    right_next_pos = right_current_pos;
                    right_next_euler = right_current_euler;
                    right_next_rot_matrix = right_current_rot_matrix;
                    right_next_joint_angles_matrix = right_joint_angles_matrix;
                    right_ik_success = true;
                    current_joint_angles_matrix.tail(7) = right_joint_angles_matrix;
                    std::cout<<"right computing IK success"<<std::endl;
                    break;
                }
                else{
                    
                    right_rot_error_3vector = right_rot_error_axis_angle.angle() * right_rot_error_axis_angle.axis();
                    right_stacked_vector.head(3) = right_error_pos;
                    right_stacked_vector.tail(3) = right_rot_error_3vector;
                    right_stacked_vector = (_error_coefficient * right_stacked_vector) / 0.01;

//                    std::cout<<"right_target_euler "<<right_target_euler.transpose()<<std::endl;
//                    std::cout<<"right_current_euler "<<right_current_euler.transpose()<<std::endl;
//                    std::cout<<"right_error_pos "<<right_error_pos.transpose()<<std::endl;
//                    std::cout<<"right_error_euler "<<right_error_euler.transpose()<<std::endl;
//                    std::cout<<"right_rot_error_axis_angle.angle() "<<right_rot_error_axis_angle.angle()<<std::endl;

                    right_end_jacobian = right_state.getJacobian(right_group);
                    right_end_jacobian_mp_inverse = (right_end_jacobian.transpose() * ((right_end_jacobian * right_end_jacobian.transpose() + 0.0001 * Eigen::Matrix<double,6,6>::Identity()).inverse())).eval();
                    right_joint_delta_vector = right_end_jacobian_mp_inverse * right_stacked_vector;
                    right_joint_delta_vector *= 0.01;
                    if(right_joint_delta_vector.norm() < 0.00001){
                        std::cout<<"right computing IK fail, norm too small!!"<<std::endl;
                        right_ik_success = false;
                        current_joint_angles_matrix.tail(7) = Eigen::Matrix<double,7,1>::Zero();
                        break;
                    }
                    else{
//                        std::cout<<"right_joint_angles_matrix\n"<<right_joint_angles_matrix.transpose()<<std::endl;
                        right_joint_angles_matrix += right_joint_delta_vector;
                        for(size_t i=0; i<7; i++){
                            right_joint_angles_vector[i] = right_joint_angles_matrix[i];
                        }
                        right_state.setJointGroupPositions(right_group, right_joint_angles_vector);
                        right_state.update();
                        if(right_state.satisfiesBounds(right_group, 0.05)){
                            //更新状态
                            const Eigen::Affine3d & right_end_pose_tmp = right_state.getGlobalLinkTransform("right_gripper");
                            right_current_rot_matrix = right_end_pose_tmp.rotation();
                            right_current_pos = right_end_pose_tmp.translation();
                            KDL::Rotation right_tmp_conversion(right_current_rot_matrix(0,0), right_current_rot_matrix(0,1), right_current_rot_matrix(0,2),
                                                              right_current_rot_matrix(1,0), right_current_rot_matrix(1,1), right_current_rot_matrix(1,2),
                                                              right_current_rot_matrix(2,0), right_current_rot_matrix(2,1), right_current_rot_matrix(2,2));
                            right_tmp_conversion.GetEulerZYX(right_current_euler(0), right_current_euler(1), right_current_euler(2));
                        }
                        else{
                            std::cout<<"right computing IK fail, nexceed joint limits!!"<<std::endl;
                            right_ik_success = false;
                            current_joint_angles_matrix.tail(7) = Eigen::Matrix<double,7,1>::Zero();
                            break;
                        }
                    }
                }
            }
            ik_results_along_the_path_all_posture[j].push_back(current_joint_angles_matrix);
        }
    }
}

void set_scene_object_as_obstacle(planning_scene::PlanningScenePtr & planning_scene_ptr){
    double safe_margin_x = 0.1;
    double safe_margin_y = 0.05;
    double safe_margin_z = 0.05;
    
    std::vector<moveit_msgs::CollisionObject> object_in_the_world;
    planning_scene_ptr->getCollisionObjectMsgs(object_in_the_world);

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

    nh.param("map/resolution",    _resolution,   0.1);
    
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


    

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    planning_scene::PlanningScenePtr planning_scene_for_operate = ps->diff();
    planning_scene_for_operate->decoupleParent();
    robot_state::RobotState robot_state = planning_scene_for_operate->getCurrentStateNonConst();
    robot_state::RobotState robot_state_original = planning_scene_for_operate->getCurrentStateNonConst();

    set_scene_object_as_obstacle(planning_scene_for_operate);
    
    Vector3d start_pt;
    start_pt<<0.717443, -0.0286898,    0.66986;
    Vector3d target_pt;
    target_pt<<0.580433, -0.0316257,   0.284839;

    vector<Vector3d> astar_path;
    pathFinding(start_pt, target_pt, astar_path);
    
    std::vector<std::vector<Eigen::Matrix<double, 14, 1>>> ik_results_along_the_path_all_posture(4);

    solve_dual_arm_IK(astar_path, planning_scene_for_operate, ik_results_along_the_path_all_posture);

    ROS_WARN("ik_results_along_the_path_all_posture.size %d", int(ik_results_along_the_path_all_posture.size()));
    ROS_WARN("ik_results_along_the_path_all_posture[0].size %d", int(ik_results_along_the_path_all_posture[0].size()));
    ROS_WARN("ik_results_along_the_path_all_posture[1].size %d", int(ik_results_along_the_path_all_posture[1].size()));
    ROS_WARN("ik_results_along_the_path_all_posture[2].size %d", int(ik_results_along_the_path_all_posture[2].size()));
    ROS_WARN("ik_results_along_the_path_all_posture[3].size %d", int(ik_results_along_the_path_all_posture[3].size()));

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    std::vector<double> both_arm_angles(14);
    Eigen::Matrix<double, 14, 1> tmp_matrix;
    for(size_t i=0; i<4; i++){
        ROS_WARN("posture %d", int(i));
        for(size_t j=0; j<ik_results_along_the_path_all_posture[i].size();j++){
            ROS_WARN("path point %d", int(j));
            tmp_matrix = ik_results_along_the_path_all_posture[i][j];
            std::cout<<tmp_matrix.transpose()<<std::endl;
            for(size_t k=0; k<14; k++){
                both_arm_angles[k] = tmp_matrix[k];
            }
            robot_state.setJointGroupPositions("both_arms", both_arm_angles);
            robot_state.update();
            planning_scene_for_operate->setCurrentState(robot_state);
            moveit_msgs::PlanningScene planning_scene_msg;
            planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
            planning_scene_msg.is_diff = true;

            planning_scene_diff_publisher.publish(planning_scene_msg);
            ros::Duration(1).sleep();
        }
    }

    planning_scene_for_operate->setCurrentState(robot_state_original);
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;

    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(1).sleep();

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