#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test220191025");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);

    const robot_model::RobotModelConstPtr & robot_model_operate = ps->getRobotModel();
    robot_state::RobotState & robot_state_write = ps->getCurrentStateNonConst();
    const robot_state::JointModelGroup* two_arm = robot_state_write.getJointModelGroup("both_arms");
    std::vector<double> original_values;
    robot_state_write.copyJointGroupPositions(two_arm, original_values);
    for(std::vector<double>::iterator it=original_values.begin(); it!=original_values.end();it++){
        std::cout<<*it<<",";
    }
    std::cout<<std::endl;

    std::vector<double> test_values = {-0.670559,0.847365,-2.18348,1.09974,0.9986,-1.48732,1.56971,0.421259,0.36132,1.78139,0.905517,2.71258,1.30217,1.63765};
    robot_state_write.setJointGroupPositions(two_arm, test_values);
    robot_state_write.update();
    std::vector<double> new_values;
    robot_state_write.copyJointGroupPositions(two_arm, new_values);
    for(std::size_t i=0; i<new_values.size(); i++){
        ROS_INFO("new value %d is %f", int(i), new_values[i]);
    }
    moveit_msgs::PlanningScene planning_scene;
    ps->getPlanningSceneMsg(planning_scene);
    planning_scene.is_diff = true;
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene);
    ros::Duration(1).sleep();

    planning_scene::PlanningScenePtr ps1 = ps->diff();
    ps1->decoupleParent();

    collision_detection::CollisionRobotFCL robotFcl(robot_model_operate);

    collision_detection::DistanceRequest dis_req;
    collision_detection::DistanceResult dis_res;
    dis_req.group_name = "both_arms";
    dis_req.enable_nearest_points = true;
    dis_req.type = collision_detection::DistanceRequestType::SINGLE;
    robotFcl.distanceSelf(dis_req, dis_res, robot_state_write);

    ROS_INFO("The test state is %s in collision", dis_res.collision? "" : "not");
    collision_detection::DistanceMap collisiion_pairs = dis_res.distances;

    collision_detection::DistanceResultsData min_distance = dis_res.minimum_distance;
    ROS_INFO("The minimum distance is %f", min_distance.distance);
    ROS_INFO("%s collides with %s", min_distance.link_names[0].c_str(), min_distance.link_names[1].c_str());
    Eigen::Vector3d first_point = min_distance.nearest_points[0];
    Eigen::Vector3d second_point = min_distance.nearest_points[1];
    Eigen::Vector3d normal = min_distance.normal;
    ROS_INFO("first point");
    std::cout<<first_point<<std::endl;
    ROS_INFO("second point");
    std::cout<<second_point<<std::endl;
    ROS_INFO("normal");
    std::cout<<normal<<std::endl;
    const robot_state::JointModelGroup* right_arm = robot_state_write.getJointModelGroup("right_arm");
    const robot_state::LinkModel* collision_link = robot_state_write.getLinkModel(min_distance.link_names[1]);
    Eigen::MatrixXd result_jac;
    second_point[2]+=0.1;
    if(robot_state_write.getJacobian(right_arm, collision_link, second_point, result_jac)){
        ROS_INFO("Computed jacobian succesully!!");
        std::cout<<result_jac<<std::endl;
    }
    else{
        ROS_INFO("fail to compute jacobian!!");
    }

//    std::map<const std::pair<std::string, std::string>, std::vector<collision_detection::DistanceResultsData>>::iterator it;
//    for(it = dis_res.distances.begin(); it!=dis_res.distances.end(); it++){
//        const std::pair<std::string, std::string> tmp_pair = it->first;
//
//    }



    return 0;
}