#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

    robot_state::RobotState & robot_state_write = ps->getCurrentStateNonConst();
    const std::vector< std::string > & varible_names = robot_state_write.getVariableNames();
    for(std::size_t i=0; i<varible_names.size(); i++){
        ROS_INFO("%d name is %s", i, varible_names[i].c_str());
    }
    const robot_state::JointModelGroup* two_arm = robot_state_write.getJointModelGroup("both_arms");
    std::vector<double> original_values;
    robot_state_write.copyJointGroupPositions(two_arm, original_values);
    for(std::size_t i=0; i<original_values.size(); i++){
        ROS_INFO("original value %d is %f", i, original_values[i]);
    }
    robot_state_write.setToRandomPositions(two_arm);
    std::vector<double> new_values;
    robot_state_write.copyJointGroupPositions(two_arm, new_values);
    for(std::size_t i=0; i<new_values.size(); i++){
        ROS_INFO("new value %d is %f", i, new_values[i]);
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
//    planning_scene_diff_publisher.publish(planning_scene);
//    ros::Duration(0.5).sleep();




    const planning_scene::PlanningSceneConstPtr & parent_scene = ps->getParent ();
//    moveit_msgs::PlanningScene now_scene;
//    moveit_msgs::CollisionObject now_object;
//    ps->getPlanningSceneMsg(now_scene);
//
////    读取到加载到环境中的物体信息
//    if(now_scene.world.collision_objects.size() > 0) {
//        now_object = now_scene.world.collision_objects[0];
//        ROS_INFO("object id is %s", now_object.id.c_str());
//        ROS_INFO("object solid primitive type is %d", now_object.primitives[0].type);
//        ROS_INFO("object solid primitive size is x=%f, y=%f, z=%f", now_object.primitives[0].dimensions[0], now_object.primitives[0].dimensions[1], now_object.primitives[0].dimensions[2]);
//        ROS_INFO("object solid primitive poses length is %d", now_object.primitive_poses.size());
//        ROS_INFO("object solid primitive locate at x=%f, y=%f, z=%f", now_object.primitive_poses[0].position.x, now_object.primitive_poses[0].position.y, now_object.primitive_poses[0].position.z);
//        ROS_INFO("object frame_id is %s", now_object.header.frame_id.c_str());
//    }
//    else
//        ROS_INFO("no object");

//    改变一个机器人位置到达碰撞位置，查看碰撞检测的情况





    return 0;
}