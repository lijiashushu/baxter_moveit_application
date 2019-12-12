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
    std::srand((unsigned)time(NULL));

    int if_try_adjust = 1;
    int seed = 1607533959;//625114190   1344077138
    ROS_WARN("seed: %d", seed);
    size_t  test_pose_num = 1;

    Eigen::Matrix3d test123;
    test123<< 1, 0, 0,
           0, 1, 1,
           0, 0.5, 1;
    std::cout<<"test123:  "<<(test123*test123.transpose()).determinant()<<std::endl;
    Eigen::Matrix3d test456;
    test456<< 1, 0, 0,
            0, 1, 1,
            0, 0.9, 1;
    std::cout<<"test456:  "<<(test456*test456.transpose()).determinant()<<std::endl;

    Eigen::Matrix3d rotmatrix;
    rotmatrix<<0.999998 ,-0.00148166 , 0.00139371, -0.00139265, 0.000715737 ,   0.999999, -0.00148265 ,  -0.999999, 0.000713672;
    Eigen::Vector3d euler;
    euler = rotmatrix.eulerAngles(2,1,0);
    std::cout<<"euler  "<<euler.transpose()<<std::endl;

    KDL::Rotation test(0.999998 ,-0.00148166 , 0.00139371, -0.00139265, 0.000715737 ,   0.999999, -0.00148265 ,  -0.999999, 0.000713672);
    Eigen::Vector3d euler2;
    test.GetEulerZYX(euler2(0), euler2(1), euler2(2));
    std::cout<<"euler2  "<<euler2.transpose()<<std::endl;

    Eigen::Vector3d slave_www_euler;
    slave_www_euler <<3.1402, 3.14011,1.57151;
//    slave_www_euler <<0.00027487, 0.00016706, -1.56988;
    Eigen::AngleAxisd www_roll_angle(Eigen::AngleAxisd(slave_www_euler[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd www_pitch_angle(Eigen::AngleAxisd(slave_www_euler[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd www_yaw_angle(Eigen::AngleAxisd(slave_www_euler[0], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d slave_www_rot_matrix;
    slave_www_rot_matrix=www_yaw_angle*www_pitch_angle*www_roll_angle;
    std::cout<<slave_www_rot_matrix<<std::endl;




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

//    std::vector<double> test_start_value = {-0.375463,-1.09228,-0.440484,1.20106,1.76769,-1.57028,0.0672226}; //没有奇异、narrow障碍物的起始左臂位置，大桌子
//    std::vector<double> slave_test_start_value = {-0.115289,-0.393004,1.72106,1.01171,-2.93258,-1.39411,0.332235};//没有奇异、narrow障碍物的起始右臂位置，大桌子

//    std::vector<double> test_start_value = {-0.0137274,-0.648781,-1.1192,0.880775,2.37693,-1.56809,-0.11713}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体
//    std::vector<double> slave_test_start_value = {-0.0361471,-0.345621,1.6738,0.869939,-2.96241,-1.47801,0.298402};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体


    std::vector<double> test_start_value1 = {-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> slave_test_start_value1 = {-0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> test_start_value2 = {0.514998,-0.487572,-1.79923,1.6679,-0.28682,0.603706,2.86722};
    std::vector<double> slave_test_start_value2 = {-0.517204,-0.49348,1.79496,1.66956,0.302716,0.602833,-2.87906};
    std::vector<double> test_start_value3 = {0.75793,-0.888533,-2.01998,2.09217,-0.226114,-0.224376,2.39712};
    std::vector<double> slave_test_start_value3 = {-0.633959,-0.671778,2.03325,2.11988,0.716528,-0.334106,-3.05835};
    std::vector<double> test_start_value4 = {0.119411,-0.743597,-2.05219,2.14562,-0.840575,-1.39347,3.05791};
    std::vector<double> slave_test_start_value4 = {-0.119205,-0.74178,2.05281,2.14571,0.840017,-1.39411,-3.058};

    std::vector<std::vector<double>> four_start_master;
    four_start_master.push_back(test_start_value1);
    four_start_master.push_back(test_start_value2);
    four_start_master.push_back(test_start_value3);
    four_start_master.push_back(test_start_value4);
    std::vector<std::vector<double>> four_start_slave;
    four_start_slave.push_back(slave_test_start_value1);
    four_start_slave.push_back(slave_test_start_value2);
    four_start_slave.push_back(slave_test_start_value3);
    four_start_slave.push_back(slave_test_start_value4);



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

    start_state.setJointGroupPositions(planning_group, four_start_master[test_pose_num]);
    start_state.setJointGroupPositions(slave_group, four_start_slave[test_pose_num]);
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

    start_state.update();
    std::cout<<"isStateColliding 1  "<<planning_scene_for_operate->isStateColliding(start_state, "both_arms")<<std::endl;
    std::vector<std::string> collide_link1;
    planning_scene_for_operate->getCollidingLinks(collide_link1);
    for(size_t i=0; i<collide_link1.size(); i++) {
        std::cout << collide_link1[i] << std::endl;
    }

    planning_scene_for_operate->setCurrentState(start_state);
    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::CollisionObject collision_object;
    if(planning_scene_for_operate->getCollisionObjectMsg(collision_object, "object")){std::cout<<"has object!!"<<std::endl;}
    else{std::cout<<"no object!!"<<std::endl;}
    attached_object.link_name="left_gripper_base";
    attached_object.object = collision_object;
    std::cout<<"collision_object"<<std::endl;
    std::cout<<collision_object.primitives.size()<<std::endl;
    attached_object.touch_links = std::vector<std::string>{"l_gripper_l_finger_tip", "l_gripper_l_finger", "l_gripper_r_finger_tip", "l_gripper_r_finger",
                                                           "r_gripper_l_finger_tip", "r_gripper_l_finger", "r_gripper_r_finger_tip", "r_gripper_r_finger",
    };
//    std::cout<<"id "<<attached_object.object.id<<std::endl;
//    std::cout<<"attached_object"<<std::endl;
//    std::cout<<attached_object.object.primitives[0].dimensions[0]<<std::endl;
//    std::cout<<attached_object.object.primitives[0].dimensions[1]<<std::endl;
//    std::cout<<attached_object.object.primitives[0].dimensions[2]<<std::endl;

    moveit_msgs::PlanningScene planning_scene_msg;


//    planning_scene_msg.robot_state.attached_collision_objects.push_back()
    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;
    collision_object.operation = collision_object.REMOVE;
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_msg.world.collision_objects.push_back(collision_object);



    collision_detection::AllowedCollisionMatrix acm1 = planning_scene_for_operate->getAllowedCollisionMatrix();
    collision_detection::AllowedCollisionMatrix acm2 = acm1;

    DualCBiRRT my_planner(1.0, seed, 0.00, test_pose_num, planning_scene_for_operate, planning_group, slave_group);
    robot_model::RobotModelConstPtr robot_model_ptr = planning_scene_for_operate->getRobotModel();
//    std::string mesh_links[2] = {"torso", "pedestal"};
//    for(size_t i=0; i<2; i++){
//        const robot_model::LinkModel* tmp_link_ptr  =robot_model_ptr->getLinkModel(mesh_links[i]);
//        std::string link_mesh_file_name = tmp_link_ptr->getVisualMeshFilename();
//        Eigen::Affine3d link_mesh_origin = tmp_link_ptr->getVisualMeshOrigin();
//        Eigen::Vector3d link_mesh_scale = tmp_link_ptr->getVisualMeshScale();
//        std::vector<shapes::ShapeConstPtr> test;
//        test = tmp_link_ptr->getShapes();
//        std::cout<<"shapes len  "<<test.size()<<std::endl;
//        std::cout<<"shapes type  "<<test[0]->type<<std::endl;
//
//
//        moveit_msgs::CollisionObject co;
//        co.id = mesh_links[i];
//
//        std_msgs::Header head;
//        head.frame_id = "base";
//        co.header = head;
//
//        shapes::Mesh* m = shapes::createMeshFromResource(link_mesh_file_name, link_mesh_scale);
//        shape_msgs::Mesh mesh;
//        shapes::ShapeMsg mesh_msg;
//        shapes::constructMsgFromShape(m, mesh_msg);
//        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
//
//        geometry_msgs::Pose mesh_pose;
//        mesh_pose.position.x = link_mesh_origin.translation()(0);
//        mesh_pose.position.y = link_mesh_origin.translation()(1);
//        mesh_pose.position.z = link_mesh_origin.translation()(2);
//
//        Eigen::Quaterniond quaternion(link_mesh_origin.rotation());
//        mesh_pose.orientation.x = quaternion.x();
//        mesh_pose.orientation.y = quaternion.y();
//        mesh_pose.orientation.z = quaternion.z();
//        mesh_pose.orientation.w = quaternion.w();
//
//        co.meshes.push_back(mesh);
//        co.mesh_poses.push_back(mesh_pose);
//        co.operation = co.ADD;
//
//        planning_scene_msg.world.collision_objects.push_back(co);
//    }
////    begin**************************把机器人自身的一部分作为障碍物，由于显示之后可以不用继续在程序里读取，直接通过导出的文件进行读取即可**********************************
//    moveit_msgs::CollisionObject co;
//    co.id = "approximate_torso";
//
//    std_msgs::Header head;
//    head.frame_id = "base";
//    co.header = head;
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.CYLINDER;
//    primitive.dimensions.resize(2);
//    primitive.dimensions[0] = 1.2;
//    primitive.dimensions[1] = 0.16;
//
//    geometry_msgs::Pose primitive_pose;
//    primitive_pose.position.x = 0;
//    primitive_pose.position.y = 0;
//    primitive_pose.position.z = 0;
//
//    primitive_pose.orientation.x = 0;
//    primitive_pose.orientation.y = 0;
//    primitive_pose.orientation.z = 0;
//    primitive_pose.orientation.w = 1;
//
//    co.primitives.push_back(primitive);
//    co.primitive_poses.push_back(primitive_pose);
//    co.operation = co.ADD;
//    planning_scene_msg.world.collision_objects.push_back(co);
//
//    std::string primitive_links[2] = {"collision_head_link_1", "collision_head_link_2"};
//    for(size_t i=0; i<1; i++) {
//        const robot_model::LinkModel *tmp_link_ptr = robot_model_ptr->getLinkModel(primitive_links[i]);
//        std::vector<shapes::ShapeConstPtr> test;
//        test = tmp_link_ptr->getShapes();
//        std::cout<<"shapes len  "<<test.size()<<std::endl;
//        std::cout<<"shapes type  "<<test[0]->type<<std::endl;
//        std::cout<<"are identity  "<<tmp_link_ptr->areCollisionOriginTransformsIdentity()[0]<<std::endl;
//        moveit_msgs::CollisionObject co;
//        co.id = primitive_links[i];
//
//        std_msgs::Header head;
//        head.frame_id = "base";
//        co.header = head;
//
//        shape_msgs::SolidPrimitive primitive;
//        primitive.type = primitive.SPHERE;
//        primitive.dimensions.resize(1);
//        primitive.dimensions[0] = 0.15;
//
//        EigenSTL::vector_Affine3d pose_offset = tmp_link_ptr->getCollisionOriginTransforms();
//        Eigen::Affine3d pose = tmp_link_ptr->getJointOriginTransform();
//        geometry_msgs::Pose primitive_pose;
//        primitive_pose.position.x = (pose).translation()(0);
//        primitive_pose.position.y = (pose).translation()(1);
//        primitive_pose.position.z = (pose).translation()(2);
//
//        Eigen::Quaterniond quaternion((pose).rotation());
//        primitive_pose.orientation.x = quaternion.x();
//        primitive_pose.orientation.y = quaternion.y();
//        primitive_pose.orientation.z = quaternion.z();
//        primitive_pose.orientation.w = quaternion.w();
//
//        co.primitives.push_back(primitive);
//        co.primitive_poses.push_back(primitive_pose);
//        co.operation = co.ADD;
//
//        planning_scene_msg.world.collision_objects.push_back(co);
//    }
//
//    std_msgs::ColorRGBA color;
//    color.b = 1;
//    color.a = 0.1;
//
//    moveit_msgs::ObjectColor ob_color1;
//    ob_color1.id = "approximate_torso";
//    ob_color1.color = color;
//    planning_scene_msg.object_colors.push_back(ob_color1);
//    moveit_msgs::ObjectColor ob_color2;
//    ob_color2.id = "collision_head_link_1";
//    ob_color2.color = color;
//    planning_scene_msg.object_colors.push_back(ob_color2);
//    moveit_msgs::ObjectColor ob_color3;
//    ob_color3.id = "collision_head_link_2";
//    ob_color3.color = color;
//    planning_scene_msg.object_colors.push_back(ob_color3);
////  end**************************把机器人自身的一部分作为障碍物，由于显示之后可以不用继续在程序里读取，直接通过导出的文件进行读取即可**********************************

    
    
    ros::WallDuration sleep_t(0.5);
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(1).sleep();

    planning_scene_for_operate->setPlanningSceneMsg(planning_scene_msg);
    start_state = planning_scene_for_operate->getCurrentStateNonConst();
//    std::vector<std::string> acm_names;
//    acm1.getAllEntryNames(acm_names);
//    std::cout<<"ACM"<<std::endl;
//    for(auto it=acm_names.begin(); it!=acm_names.end();it++){
//        std::cout<<*it<<std::endl;
//    }

    start_state.update();
    robot_state::RobotState goal_state = planning_scene_for_operate->getCurrentStateNonConst();

//    planning_scene_for_operate->getCollisionObjectMsgs()

    std::cout<<"isStateColliding 2  "<<planning_scene_for_operate->isStateColliding(start_state, "both_arms")<<std::endl;
    std::vector<std::string> collide_link2;
    planning_scene_for_operate->getCollidingLinks(collide_link2, start_state);
    for(size_t i=0; i<collide_link2.size(); i++)
    {
        std::cout<<collide_link2[i]<<std::endl;

    }
//    std::set<const moveit::core::LinkModel *> master_link_model_set;
//    std::set<const moveit::core::LinkModel *> slave_link_model_set;
//    for(size_t i=0; i<_slave_link_names.size(); i++){
//        master_link_model_set.insert(planning_group->getLinkModel(_master_link_names[i]));
//        slave_link_model_set.insert(slave_group->getLinkModel(_slave_link_names[i]));
//    }
    const collision_detection::CollisionRobotConstPtr robot = planning_scene_for_operate->getCollisionRobot();
    const collision_detection::WorldPtr world = planning_scene_for_operate->getWorldNonConst();
    collision_detection::CollisionWorldFCL worldFcl(world);
    collision_detection::DistanceRequest collision_req_master;
    collision_req_master.enable_nearest_points = true;
    collision_req_master.enable_signed_distance = true;
//    collision_req_master.active_components_only = &master_link_model_set;
    collision_req_master.group_name = "both_arm";
    collision_req_master.type = collision_detection::DistanceRequestType::LIMITED;
    collision_detection::DistanceResult collision_res_master;
    collision_detection::DistanceResult new_collision_res_master;
    worldFcl.distanceRobot(collision_req_master, collision_res_master, *robot, start_state);
    std::cout<<"if collision  "<<collision_res_master.collision<<std::endl;
    collision_detection::DistanceResultsData master_min_dis = collision_res_master.minimum_distance;
    std::cout<<"master_min_dis.distance  "<<master_min_dis.distance<<std::endl;
    std::cout<<master_min_dis.link_names[0]<<std::endl;
    std::cout<<master_min_dis.link_names[1]<<std::endl;
    std::cout<<master_min_dis.nearest_points[0].transpose()<<std::endl;
    std::cout<<master_min_dis.nearest_points[1].transpose()<<std::endl;
    std::cout<<master_min_dis.normal.transpose()<<std::endl;
    std::cout<<master_min_dis.body_types[0]<<std::endl;
    std::cout<<master_min_dis.body_types[1]<<std::endl;

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.distance = true;
    worldFcl.checkCollision(req,res,*robot, start_state);
    std::cout<<"if collision  "<<res.collision<<std::endl;
    std::cout<<" distance  "<<res.distance<<std::endl;
    std::cout<<"isStateValid 3  "<<planning_scene_for_operate->isStateValid(start_state, "left_arm")<<std::endl;

//    std::vector<double> test_goal_value = {-0.53121395, -1.14663671 , 0.21698349  ,2.33939883 ,-1.17448029  ,1.81105335,  2.82284528};//无障碍时手臂平方的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.64966, 0.0056597, 1.453030, 2.2167859, 0.0142739, 0.7887366, -1.69753346};//无障碍时手臂平方的目标右臂位置
//    std::vector<double> test_goal_value = {0.0511426,-0.422846,-0.602817,1.92707,-0.888771,1.20479,2.70597}; //平板类似桌子的障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.614005,  0.611334 ,  1.40829,   1.80571, -0.631447,   1.11582,  -1.56488}; //平板类似桌子的障碍物的目标右臂位置
//    std::vector<double> test_goal_value = {-0.0813673,-0.68199,-0.637715,1.9482,-1.13503,1.24992,2.59584};//narrow障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.485412,0.487359,1.66579,1.6767,-0.522427,1.24843,-1.42944};//narrow障碍物的目标右臂位置

//    std::vector<double> test_goal_value = {-0.233357,-0.754374,-0.490762,1.95377,1.90675,-1.34839,1.06295};//没有奇异、narrow障碍物的目标左臂位置
//    std::vector<double> slave_test_goal_value = {-0.446697,-0.0863082,1.24614,1.77273,-2.93228,-1.08041,-0.381265};//没有奇异、narrow障碍物的目标右臂位置


//    std::vector<double> test_goal_value = {0.104215,-0.367542,-0.866707,1.42165,2.45671,-1.38136,0.587156};//没有奇异、narrow障碍物的目标左臂位置，大桌子，增加抓取物体
//    std::vector<double> slave_test_goal_value = {-0.260604,0.0155082,1.32256,1.4155,-3.02382,-1.21494,-0.264714};//没有奇异、narrow障碍物的目标右臂位置,大桌子，增加抓取物体

//    std::vector<double> test_goal_value = {0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696,3.05881};//没有奇异、narrow障碍物的目标左臂位置，大桌子，增加抓取物体，新姿态
//    std::vector<double> slave_test_goal_value = {-0.640635,0.334855,1.43792,2.03636,-0.432353,0.425846,-3.05814};//没有奇异、narrow障碍物的目标右臂位置,大桌子，增加抓取物体，新姿态

    std::vector<double> test_goal_value1 = {0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954}; //没有奇异、narrow障碍物的起始左臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> slave_test_goal_value1 = {-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414};//没有奇异、narrow障碍物的起始右臂位置，大桌子，增加抓取物体，新姿态
    std::vector<double> test_goal_value2 = {0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696,3.05881};
    std::vector<double> slave_test_goal_value2 = {-0.640635,0.334855,1.43792,2.03636,-0.432353,0.425846,-3.05814};
    std::vector<double> test_goal_value3 = {0.378964,-0.235001,-1.09593,2.54013,0.629455,-0.946823,2.56304};
    std::vector<double> slave_test_goal_value3 = {-0.495711,0.18897,1.20076,2.51116,-0.470866,-0.757521,-3.05796};
    std::vector<double> test_goal_value4 = {0.188932,1.04619,-1.20316,2.47039,0.862578,-1.41834,-2.53144};
    std::vector<double> slave_test_goal_value4 = {-0.108729,1.0458,1.23816,2.45137,-0.881861,-1.45554,2.53844};

    std::vector<std::vector<double>> four_goal_master;
    four_goal_master.push_back(test_goal_value1);
    four_goal_master.push_back(test_goal_value2);
    four_goal_master.push_back(test_goal_value3);
    four_goal_master.push_back(test_goal_value4);
    std::vector<std::vector<double>> four_goal_slave;
    four_goal_slave.push_back(slave_test_goal_value1);
    four_goal_slave.push_back(slave_test_goal_value2);
    four_goal_slave.push_back(slave_test_goal_value3);
    four_goal_slave.push_back(slave_test_goal_value4);


//    std::cout<<"four pose start manipulability"<<std::endl;
//    for(size_t i=0; i<4; i++){
//        double l_m, r_m;
//        my_planner.basic_manipulability_compute_once(four_start_master[i], four_start_slave[i], l_m, r_m);
//        std::cout<<l_m <<" "<<r_m<<" "<<l_m + r_m<<std::endl;
//    }
//    std::cout<<"four pose goal manipulability"<<std::endl;
//    for(size_t i=0; i<4; i++){
//        double l_m, r_m;
//        my_planner.basic_manipulability_compute_once(four_goal_master[i], four_goal_slave[i], l_m, r_m);
//        std::cout<<l_m <<" "<<r_m<<" "<<l_m + r_m<<std::endl;
//    }
//
//    std::cout<<"four pose start goal dir manipulability"<<std::endl;
//    for(size_t i=0; i<4; i++){
//        double s_l_m, s_r_m, g_l_m, g_r_m;
//        my_planner.dir_manipulability_compute_once(four_start_master[i], four_start_slave[i], four_goal_master[i], four_goal_slave[i],s_l_m, s_r_m, g_l_m, g_r_m);
//        std::cout<<s_l_m <<" "<<s_r_m<<" "<<g_l_m <<" "<<g_r_m<<std::endl;
//    }


    goal_state.setJointGroupPositions(planning_group, four_goal_master[test_pose_num]);
    goal_state.setJointGroupPositions(slave_group, four_goal_slave[test_pose_num]);
    goal_state.update();

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

    ros::Time time_1 = ros::Time::now();
    if(if_try_adjust){
        if(my_planner.plan_task_space_dir_try_adjust(goal_state, start_state)){
            std::cout<<"??? try ????"<<std::endl;
        }
    }
    else{
        if(my_planner.plan_task_space_dir(goal_state, start_state)){
            std::cout<<"??? no try ?????"<<std::endl;
        }
    }
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("planning_time  %f", (time_2 - time_1).toSec());
    my_planner.output_perdex();
    std::vector<robot_state::RobotState> result = my_planner.planning_result;
    std::vector<size_t> result_index = my_planner.planning_result_index;
    std::vector<Eigen::Vector4d> result_state_vector = my_planner.planning_result_task_state_vector;
    ROS_INFO("waypoints num is %d", int(result.size()));

    std::cout<<"\n\nhahha"<<std::endl;
    for(size_t i=0; i<my_planner._new_manipulability_mini_vec_exploit.size();i ++){
        std::cout<<my_planner._new_manipulability_mini_vec_exploit[i]<<std::endl;
    }

    std::ofstream abc;
    abc.open("/home/lijiashushu/ros_ws/src/baxter_moveit_application/draw_data/manipulability/fuck.txt", std::ios::out | std::ios::trunc);
    my_planner.output_new_manipulability(abc);
    abc.close();
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
//    visual_tools.publishTrajectoryLine(result_msg, planning_group);
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
        start_angles_msg.position.push_back(four_start_master[test_pose_num][i]);
    }
    for(size_t i=0; i<slave_joint_names.size(); i++){
        start_angles_msg.name.push_back(slave_joint_names[i]);
        start_angles_msg.position.push_back(four_start_slave[test_pose_num][i]);
    }
    start_state_msg.joint_state = start_angles_msg;
    start_state_msg.attached_collision_objects.push_back(attached_object);
    display_traj_msg.trajectory_start = start_state_msg;

    //添加这个消息的第二个参数，开始的状态，可能显示多条轨迹所以是向量
    display_traj_msg.trajectory.push_back(result_msg);

    ros::Publisher display_traj_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    while (display_traj_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    display_traj_publisher.publish(display_traj_msg);
    ros::Duration(1).sleep();

    planning_scene_for_operate->getPlanningSceneMsg(planning_scene_msg);
    planning_scene_msg.is_diff = true;
    collision_object.operation = collision_object.ADD;
    attached_object.object.operation = attached_object.object.REMOVE;
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(2).sleep();

    return 0;
}