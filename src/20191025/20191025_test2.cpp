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

int main(int argc, char** argv){
    ros::init(argc, argv, "test220191025");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    monitor_ptr->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr);
    robot_state::RobotState& current_robot_state = ps->getCurrentStateNonConst();

    robot_model::RobotModelConstPtr robot_kinematic_model = ps->getRobotModel();
    const robot_state::JointModelGroup* joint_model_group = robot_kinematic_model->getJointModelGroup("both_arms");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    current_robot_state.copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i<joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        joint_values[i] = 0.0;
        current_robot_state.setJointGroupPositions(joint_model_group, joint_values);
    }
    planning_scene::PlanningScenePtr scene = ps->diff();
    current_robot_state = scene->getCurrentState();
    ROS_INFO("set values to zeros");
    for(std::size_t i=0; i<joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    scene->decoupleParent();
    scene->setCurrentState(current_robot_state);
    if(monitor_ptr->updatesScene(scene))
        ROS_INFO("update success");
    else
        ROS_INFO("update fail");

    sleep(1);
    monitor_ptr->updatesScene(scene);

//    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
//    ros::WallDuration sl eep_t(0.5);
//    while (planning_scene_diff_publisher.getNumSubscribers() < 1){
//        sleep_t.sleep();
//    }


//    moveit_msgs::AttachedCollisionObject attached_object;
//    attached_object.link_name = "l_gripper_l_finger";
//    attached_object.object.header.frame_id = "l_gripper_l_finger";
//    attached_object.object.id = "box";
//
//    geometry_msgs::Pose pose;
//    pose.orientation.w = 1.0;
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = 0.1;
//    primitive.dimensions[1] = 0.1;
//    primitive.dimensions[2] = 0.1;
//
//    attached_object.object.primitives.push_back(primitive);
//    attached_object.object.primitive_poses.push_back(pose);
//    attached_object.object.operation = attached_object.object.ADD;
//
//    moveit_msgs::PlanningScene command_planning_scene_msg;
//    command_planning_scene_msg.world.collision_objects.push_back(attached_object.object);
//    command_planning_scene_msg.robot_state.joint_state.position = joint_values;
//    command_planning_scene_msg.is_diff = true;
//    planning_scene_diff_publisher.publish(command_planning_scene_msg);
//    sleep(1);
//    planning_scene_diff_publisher.publish(command_planning_scene_msg);
//    sleep(1);
//    planning_scene_diff_publisher.publish(command_planning_scene_msg);
//    sleep(1);



////    ros::Publisher joint_states_publisher = n.advertise<sensor_msgs::JointState>("user_motion_command",1);
//    ros::Rate loop_rate(10);
////    sensor_msgs::JointState command_joint_positions_msg;
////    command_joint_positions_msg.header.stamp = ros::Time::now();
////    command_joint_positions_msg.name = joinkmt_names;
////    command_joint_positions_msg.position = joint_values;
////    joint_states_publisher.publish(command_joint_positions_msg);
////    sleep(1);
////    joint_states_publisher.publish(command_joint_positions_msg);
////    sleep(1);
////    joint_states_publisher.publish(command_joint_positions_msg);
////    sleep(1);
////    joint_states_publisher.publish(command_joint_positions_msg);
//    while(ros::ok()){
//        planning_scene_diff_publisher.publish(command_planning_scene_msg);
//        loop_rate.sleep();
//    }






    return 0;
}