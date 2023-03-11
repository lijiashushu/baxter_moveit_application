//
// Created by lijiashushu on 20-4-8.
//按照梯形 profile来设置优化变量，每一段有7个时间，每一段每个自由度有4个jerk值。 优化变量的个数为 segment_num * (7+4 * dimension).
//因为梯形 profile 的速度、加速度曲线都是单调的，所以只要检查边界点满足约束就可以。
//
#include "dual_rrt_star/trajectory_planner/time_dual_constraint_planner/time_dual_constraint_planner.h"

#define INF 1e10
//#define MY_OPTIMIZATION_DEBUG

double softConstraintRelWeight; //3个设置成一样的值
double step;
int _constraint_index;
double _left_right_distance_z[4] = {0.2 * std::cos(0.0), 0.2 * std::cos(0.52), 0.2 * std::cos(1.05), 0.2 * std::cos(1.57)};
double _left_right_distance_x[4] = {-0.2 * std::sin(0.0), -0.2 * std::sin(0.52), -0.2 * std::sin(1.05), -0.2 * std::sin(1.57)};
double _left_right_euler_distance[4] = {0, 2*0.52, 2*1.05, 2*1.57};

int segment_num = 2;
int dimension = 14;
int iterationCount = 0;

KDL::Chain leftChain;
KDL::Chain rightChain;

void TimeDualConstraintPlanner::viewTrajectory(const Eigen::MatrixXd& allSegCoeff, const Eigen::MatrixXd &pathPoints, ros::NodeHandle &nh){
    KDL::ChainJntToJacSolver leftJacSolver(leftChain);
    KDL::ChainFkSolverPos_recursive leftFKSolver(leftChain);
    KDL::JntArray kdlAngles(7);
    KDL::Frame frame;

    ros::WallDuration sleep_t(0.5);
    ros::Publisher _traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    while (_traj_vis_pub.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    visualization_msgs::Marker _traj_vis;
    double _vis_traj_width = 0.005;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/base";

    _traj_vis.ns = "trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 1.0;
    _traj_vis.color.b = 0.0;

    _traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    int jerkIndex;
    Eigen::MatrixXd angle(1, dimension);
    Eigen::MatrixXd velocity(1, dimension);
    Eigen::MatrixXd acceleration(1, dimension);
    Eigen::MatrixXd pStart = Eigen::MatrixXd::Zero(1, dimension);
    Eigen::MatrixXd vStart = Eigen::MatrixXd::Zero(1, dimension);
    Eigen::MatrixXd aStart = Eigen::MatrixXd::Zero(1, dimension);

    for(int i=0; i<allSegCoeff.rows(); i++){
        Eigen::VectorXd x = allSegCoeff.row(i);
        pStart = pathPoints.row(i);
        for(int k=0; k<7; k++){
            for(double_t t=0; t<x[k]; t+=0.01){
                for(int j=0; j<dimension; j++){
                    jerkIndex = 7 + j*7 + k;
                    acceleration(j) = aStart(j) + x[jerkIndex] * t;
                    velocity(j) =  vStart(j) + aStart(j)*t + 0.5*x[jerkIndex]*pow(t,2);
                    angle(j) = pStart(j) + vStart(j)*t + 0.5*aStart(j)*pow(t,2) + x[jerkIndex]*pow(t,3)/6;
                }
                kdlAngles.data = angle.block(0, 0, 1, 7).transpose();
                leftFKSolver.JntToCart(kdlAngles, frame);
                pt.x = frame.p(0);
                pt.y = frame.p(1);
                pt.z = frame.p(2);
                _traj_vis.points.push_back(pt);
            }
            //这部分很重要，少了这部分轨迹就完全错误了
            for(int j=0; j<dimension; j++){
                jerkIndex = 7 + j*7 + k;
                acceleration(j) = aStart(j) + x[jerkIndex] * x[k];
                velocity(j)     = vStart(j) + aStart(j)*x[k] + 0.5*x[jerkIndex]*pow(x[k],2);
                angle(j)        = pStart(j) + vStart(j)*x[k] + 0.5*aStart(j)*pow(x[k],2) + x[jerkIndex]*pow(x[k],3)/6;
            }
            kdlAngles.data = angle.block(0, 0, 1, 7).transpose();
            leftFKSolver.JntToCart(kdlAngles, frame);
            pt.x = frame.p(0);
            pt.y = frame.p(1);
            pt.z = frame.p(2);
            _traj_vis.points.push_back(pt);

            pStart = angle;
            vStart = velocity;
            aStart = acceleration;
        }
    }

    _traj_vis_pub.publish(_traj_vis);
    ros::Duration(3).sleep();
    cout<<"after publish"<<_traj_vis.points.size()<<endl;
}

void TimeDualConstraintPlanner::outputTrajToFile(const Eigen::MatrixXd &allSegCoeff, const Eigen::MatrixXd &pathPoints, string path){
    ofstream outFile0, outFile1, outFile2, outFile3;
    outFile0.open(path + "_time.txt");
    outFile1.open(path + "_angle.txt");
    outFile2.open(path + "_velocity.txt");
    outFile3.open(path + "_acceleration.txt");

    if(allSegCoeff.rows() == 1 && allSegCoeff.cols() == 1){
        outFile0<<0.0<<endl;
        outFile0.close();
        outFile1.close();
        outFile2.close();
        outFile3.close();
        return;
    }

    double time = 0;
    double record_time = 0;
    int jerkIndex;
    Eigen::MatrixXd angle(1, dimension);
    Eigen::MatrixXd velocity(1, dimension);
    Eigen::MatrixXd acceleration(1, dimension);
    Eigen::MatrixXd pStart = Eigen::MatrixXd::Zero(1, dimension);
    Eigen::MatrixXd vStart = Eigen::MatrixXd::Zero(1, dimension);
    Eigen::MatrixXd aStart = Eigen::MatrixXd::Zero(1, dimension);

    for(int i=0; i<allSegCoeff.rows(); i++){
        Eigen::VectorXd x = allSegCoeff.row(i);
        pStart = pathPoints.row(i);

#ifdef MY_OPTIMIZATION_DEBUG
        cout<<"seg i"<<endl;
        cout<<vStart<<endl;
        cout<<aStart<<endl;
#endif

        for(int k=0; k<7; k++){
            record_time = 0;
            for(double_t t=0; t<x[k]; t+=0.01){
                for(int j=0; j<dimension; j++){
                    jerkIndex = 7 + j*7 + k;
                    acceleration(j) = aStart(j) + x[jerkIndex] * t;
                    velocity(j) =  vStart(j) + aStart(j)*t + 0.5*x[jerkIndex]*pow(t,2);
                    angle(j) = pStart(j) + vStart(j)*t + 0.5*aStart(j)*pow(t,2) + x[jerkIndex]*pow(t,3)/6;
                }
                time += 0.01;
                record_time += 0.01;

                outFile0<<time<<endl;
                outFile1<<angle<<endl;
                outFile2<<velocity<<endl;
                outFile3<<acceleration<<endl;
            }
            time += x[k] - record_time;
            for(int j=0; j<dimension; j++){
                jerkIndex = 7 + j*7 + k;
                acceleration(j) = aStart(j) + x[jerkIndex] * x[k];
                velocity(j)     = vStart(j) + aStart(j)*x[k] + 0.5*x[jerkIndex]*pow(x[k],2);
                angle(j)        = pStart(j) + vStart(j)*x[k] + 0.5*aStart(j)*pow(x[k],2) + x[jerkIndex]*pow(x[k],3)/6;
            }

            outFile0<<time<<endl;
            outFile1<<angle<<endl;
            outFile2<<velocity<<endl;
            outFile3<<acceleration<<endl;
            pStart = angle;
            vStart = velocity;
            aStart = acceleration;
        }
    }
    outFile0.close();
    outFile1.close();
    outFile2.close();
    outFile3.close();
}

double TimeDualConstraintPlanner::getAllSegCoeff(Eigen::MatrixXd pathPoints, Eigen::MatrixXd &allSegCoeff, double softWeight, double gradStep, int constraint_index){
//    std::cout<<"in pathPoints"<<std::endl;
//    std::cout<<pathPoints<<std::endl;
    std::chrono::high_resolution_clock::time_point startPlanningTime = std::chrono::high_resolution_clock::now();
    _constraint_index = constraint_index;
    softConstraintRelWeight = softWeight;
    step = gradStep;
    //*****************************************KDL tree define********************************
    KDL::Tree kdlTree;
    kdl_parser::treeFromParam("robot_description", kdlTree);
    kdlTree.getChain("base", "left_gripper", leftChain);
    kdlTree.getChain("base", "right_gripper", rightChain);

    //***************************************kinematic constraint********************************
    Eigen::MatrixXd deravativeLimit(3, 14);
    deravativeLimit<<2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
            2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
            10, 10, 10, 10, 20, 20, 20, 10, 10, 10, 10, 20, 20, 20;

    //*****************************************optimizier define********************************
    int varibleNum = segment_num * 7 +  segment_num * dimension *7;
    int equalityNum = (2* segment_num + 2) * dimension;//每段3个jerk=0,a3=0,p[7]=pathpoints(i);each segment end v=0,a=0
    int inequalityNum = 4 * 7 * segment_num * dimension;

    double x[varibleNum]; //initial value be input in
    double lb[varibleNum];
    double ub[varibleNum];
    for(int i=0; i<segment_num*7; i++){
        lb[i] = 0;
        ub[i] = INF;
    }
    for(int i=0; i<dimension; i++){
        for(int j=0; j<segment_num; j++){
            for(int k=0; k<7; k++){
                if(k==1 || k==3 || k==5){
                    lb[segment_num*7 + i*segment_num*7 + j*7 + k] = 0;
                    ub[segment_num*7 + i*segment_num*7 + j*7 + k] = 0;
                }
                else{
                    lb[segment_num*7 + i*segment_num*7 + j*7 + k] = -deravativeLimit(2, i);
                    ub[segment_num*7 + i*segment_num*7 + j*7 + k] = deravativeLimit(2, i);
                }
            }
        }
    }
    double eqTol[equalityNum];
    for(int i=0; i<equalityNum; i++){
        eqTol[i] = 1e-4;
    }
    double ineqTol[inequalityNum];
    for(int i=0; i<inequalityNum; i++){
        ineqTol[i] = 1e-4;
    }


    //*****************************************start to roll**********************************************
    Eigen::MatrixXd epsilon = Eigen::MatrixXd::Ones(1, 7) * 1e-3;

    Eigen::MatrixXd inputEqualityConstraint = Eigen::MatrixXd::Zero(5, dimension);
    Eigen::MatrixXd inputInequalityConstraint = Eigen::MatrixXd::Zero(5, dimension);
    inputInequalityConstraint.block(0, 0, 3, dimension) = deravativeLimit;
    //优化变量的
    //set initial value for the first segment
    Eigen::MatrixXd segTrajCoeff = getThirdOrderMultidofZeroSync(pathPoints.row(0), Eigen::VectorXd::Zero(14), Eigen::VectorXd::Zero(14), pathPoints.row(1), deravativeLimit);
    segTrajCoeff.block(0, 0, 1, 7) = segTrajCoeff.block(0, 0, 1, 7) + epsilon;
    for(int j=0; j<7; j++) {
        x[j] = segTrajCoeff(0, j);//only take the time assignment for the first dimension as initial value
        for(int k=0; k<dimension; k++){
            x[segment_num*7 + k*segment_num*7 + j] =  segTrajCoeff(1, k*7+j);
        }
    }

#ifdef MY_OPTIMIZATION_DEBUG
    ofstream jlt_init_value;
    jlt_init_value.open("/home/lijiashushu/optimise_debug/jlt_init.txt");
    jlt_init_value<<"0"<<endl;
    jlt_init_value<<segTrajCoeff.transpose()<<endl<<endl;
#endif

    for(int i=1; i<pathPoints.rows()-1; i++){
        //set initial value for the second segment
        segTrajCoeff = getThirdOrderMultidofZeroSync(pathPoints.row(i), Eigen::VectorXd::Zero(14), Eigen::VectorXd::Zero(14), pathPoints.row(i+1), deravativeLimit);
        segTrajCoeff.block(0, 0, 1, 7) = segTrajCoeff.block(0, 0, 1, 7) + epsilon;
        for(int j=0; j<7; j++) {
            x[j+7] = segTrajCoeff(0, j);//only take the time assignment for the first dimension as initial value
            for(int k=0; k<dimension; k++){
                x[segment_num*7 + k*segment_num*7 + 7 + j] =  segTrajCoeff(1, k*7+j);
            }
        }

#ifdef MY_OPTIMIZATION_DEBUG
        jlt_init_value<<i<<endl;
        jlt_init_value<<segTrajCoeff.transpose()<<endl<<endl;
        cout<<"varibleNum "<<varibleNum<<endl;
        cout<<"equalityNum "<<equalityNum<<endl;
        cout<<"inequalityNum "<<inequalityNum<<endl;
#endif

        nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP, varibleNum);
        nlopt_set_lower_bounds(opter, lb);//设置优化变量的下限
        nlopt_set_upper_bounds(opter, ub);
        nlopt_set_xtol_rel(opter, 1e-3);
        nlopt_set_ftol_rel(opter, 1e-3);
        inputEqualityConstraint.block(0, 0, 3, dimension) = pathPoints.block(i-1, 0, 3, dimension);

        nlopt_set_min_objective(opter, objectiveFuncNew, &inputEqualityConstraint); //设置目标函数
        nlopt_add_equality_mconstraint(opter, equalityNum,  equalityConstraintFunc, &inputEqualityConstraint, eqTol);// 等式约束
        nlopt_add_inequality_mconstraint(opter, inequalityNum, inequalityConstraintFunc, &inputInequalityConstraint, ineqTol); // 不等式约束；

#ifdef MY_OPTIMIZATION_DEBUG
        cout<<"不等式约束"<<endl;
        for (int i = 0; i < varibleNum; ++i)
            if (lb[i] > ub[i] || x[i] < lb[i] || x[i] > ub[i]) {
                cout<<i<<endl;
                cout<<"x[i] "<<x[i]<<endl;
                cout<<"lb[i] "<<lb[i]<<endl;
                cout<<"ub[i] "<<ub[i]<<endl;
                cout<<"知道了"<<endl;
            }
#endif
        double f_min; //目标函数的值
        iterationCount = 0;
        nlopt_result result = nlopt_optimize(opter, x, &f_min);
        if (result < 0) {
            printf("nlopt failed!  %d\n", result);
            exit(100);
            return 0.0;
        }
        else {
            std::cout<<"time "<<i<<" success"<<std::endl;
            //record first segment coeff
            Eigen::MatrixXd saveSegCoeff(1, 7 * (dimension + 1));
            for(int j=0; j<7; j++){
                saveSegCoeff(0, j) = x[j];
                for(int k=0; k<dimension; k++){
                    saveSegCoeff(0, k*7+7+j) =  x[segment_num*7 + k*segment_num*7 + j];
                }
            }
            allSegCoeff.row(i-1) = saveSegCoeff;

            if(i == pathPoints.rows()-2){
                //the last optimization time
                for(int j=0; j<7; j++){
                    saveSegCoeff(0, j) = x[j+7];
                    for(int k=0; k<dimension; k++){
                        saveSegCoeff(0, k*7+7+j) =  x[segment_num*7 + k*segment_num*7 + 7 + j];
                    }
                }
                allSegCoeff.row(i) = saveSegCoeff;
                double total_traj_time = 0;
                for(int i=0; i<allSegCoeff.rows(); i++){
                    for(int j=0; j<7; j++){
                        total_traj_time += allSegCoeff(i, j);
                    }
                }
//                cout<<saveSegCoeff<<std::endl;
                cout<<"total trajectory time: "<<total_traj_time<<std::endl;

#ifdef MY_OPTIMIZATION_DEBUG
                jlt_init_value.close();
#endif
            }
            else{
                //the second segment becomes the first
                for(int j=0; j<7; j++){
                    x[j] = x[j+7];
                    for(int k=0; k<dimension; k++){
                        x[segment_num*7 + k*segment_num*7 + j] = x[segment_num*7 + k*segment_num*7 + 7 +j];
                    }
                }
                //compute next initial velocity and acceleration

                double v, a, last_v, last_a;
                int jerkIndex, timeIndex;
                for(int j=0; j<dimension; j++){
                    last_v = inputEqualityConstraint(3, j);
                    last_a = inputEqualityConstraint(4, j);
                    for(int k=0; k<7; k++){
                        jerkIndex = 7 + j*7 + k;
                        timeIndex = k;
                        a = last_a + saveSegCoeff(0, jerkIndex) * saveSegCoeff(0, timeIndex);     //每个维度有7个j，每一段有7个公用的t
                        v = last_v + last_a * saveSegCoeff(0, timeIndex) + 0.5 * saveSegCoeff(0, jerkIndex) * pow(saveSegCoeff(0, timeIndex), 2);
                        last_a = a;
                        last_v = v;
                    }
                    inputEqualityConstraint(3, j) = v;
                    inputEqualityConstraint(4, j) = a;
                    inputInequalityConstraint(3, j) = v;
                    inputInequalityConstraint(4, j) = a;

                }

#ifdef MY_OPTIMIZATION_DEBUG
                cout<<"update initial velocity and acceleration"<<endl;
                cout<<inputEqualityConstraint.block(3, 0, 2, dimension)<<endl;
#endif
            }
        }
        nlopt_destroy(opter);
    }
    return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - startPlanningTime).count();
}

double TimeDualConstraintPlanner::objectiveFuncNew(unsigned n, const double *x, double *grad, void *f_data)
{

    double cost=0;
    for(int i=0; i<segment_num*7; i++){
        cost+= x[i];
    }

    if (grad) {
        int i=0;
        // first segment_num*7 variables are time intervals
        for(;i<segment_num*7;i++){
            grad[i] = 1 * step * 2 * cost;
        }
        // jerk for each time interval of each dimension
        for(;i<n;i++){
            grad[i] = 0;
        }
    }


    Eigen::MatrixXd *pathPoints = (Eigen::MatrixXd *)f_data;
    int jerkStartIndex,jerkIndex, timeIndex;

    KDL::ChainJntToJacSolver leftJacSolver(leftChain);
    KDL::ChainJntToJacSolver rightJacSolver(rightChain);
    KDL::Jacobian leftJacobian(7);
    KDL::Jacobian rightJacobian(7);

    KDL::ChainFkSolverPos_recursive leftFKSolver(leftChain);
    KDL::ChainFkSolverPos_recursive rightFKSolver(rightChain);
    KDL::JntArray leftAngleArrKDL(7);
    KDL::JntArray rightAngleArrKDL(7);
    KDL::Frame leftFrame;
    KDL::Frame rightFrame;
    KDL::Vector leftPos;
    KDL::Vector rightPos;
    KDL::Rotation leftRot;
    KDL::Rotation rightRot;


    int prevTimeIntervalNum;
    double tmp1;
    double cost_pos = 0;
    double left_cost_rot = 0, right_cost_rot = 0;
    double posWeightRel = 50;
    double leftRotWeightRel = 50;
    double rightRotWeightRel = 50;

    double p[8][dimension],v[8][dimension], a[8][dimension];
    for(int j=0; j<dimension; j++){
        p[0][j] = (*pathPoints)(0, j);
        v[0][j] = (*pathPoints)(3, j);
        a[0][j] = (*pathPoints)(4, j);
    }

    //***************************把timeInterval 的数量扩大一倍*************************
//    double double_x[2 * n];
//    for(int i=0, j=0; i<segment_num*7; i++, j+=2){
//        double_x[j] = 0.5 * x[i];
//        double_x[j+1] = 0.5 * x[i];
//    }
//    for(int i=0; i<dimension; i++){
//        for(int j=0, k=0; j<7*segment_num; j++, k+=2){
//            double_x[segment_num * 7 * 2 + i * segment_num * 7 * 2 + k] = x[segment_num * 7 + i * segment_num * 7 + j];
//            double_x[segment_num * 7 * 2 + i * segment_num * 7 * 2 + k + 1] = x[segment_num * 7 + i * segment_num * 7 + j];
//        }
//    }

    for(int i=0; i<segment_num; i++){
        for(int k=0; k<7; k++){
            //*****************************compute all joint angles after k time intervals**********************
            for(int j=0; j<dimension; j++){
                jerkIndex = segment_num*7 + j*(segment_num * 7) + i*7 + k;
                timeIndex = i*7 + k;
                a[k+1][j] = a[k][j] + x[jerkIndex] *x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
                v[k+1][j] = v[k][j] + a[k][j] * x[timeIndex] + 0.5 * x[jerkIndex] * pow(x[timeIndex], 2);
                p[k+1][j] = p[k][j] + v[k][j] * x[timeIndex] + 0.5 * a[k][j] *pow(x[timeIndex], 2) + x[jerkIndex] * pow(x[timeIndex], 3) / 6;
            }
            //***************************compute cartesian position and rotation of both arms**********************
            for(int j=0; j<7; j++) {
                leftAngleArrKDL(j) = p[k + 1][j];
                rightAngleArrKDL(j) = p[k + 1][j + 7];
            }

            leftFKSolver.JntToCart(leftAngleArrKDL, leftFrame);
            rightFKSolver.JntToCart(rightAngleArrKDL, rightFrame);
            //假设当前 left 的 xyz 和 yaw 角是正确的。
            leftPos = leftFrame.p;
            rightPos = rightFrame.p;
            leftRot = leftFrame.M;
            rightRot = rightFrame.M;

            //******************compute cartesian error, take current left(master) arm's xyz and yaw as reference. cur minus goal**********************
            Eigen::Vector3d leftEulerVec;
            leftRot.GetEulerZYX(leftEulerVec(0), leftEulerVec(1), leftEulerVec(2));
            Eigen::Vector3d leftGoalEulerVec(leftEulerVec(0), 0, 1.57);
            Eigen::Vector3d leftEulerErr = leftEulerVec - leftGoalEulerVec;

            KDL::Rotation leftGoalRot = KDL::Rotation::EulerZYX(leftEulerVec(0), 0, 1.57);
            KDL::Vector distance(_left_right_distance_x[_constraint_index], 0, _left_right_distance_z[_constraint_index]);
            KDL::Vector rightGoalPos = leftGoalRot * distance + leftPos;
            KDL::Vector rightPosErr = rightPos - rightGoalPos;

            Eigen::Vector3d rightEulerVec;
            rightRot.GetEulerZYX(rightEulerVec(0), rightEulerVec(1), rightEulerVec(2));
            Eigen::Vector3d rightGoalEulerVec(leftEulerVec(0)-_left_right_euler_distance[_constraint_index], 0, -1.57);
            Eigen::Vector3d rightEulerErr = rightEulerVec - rightGoalEulerVec;

            cost_pos += pow(rightPosErr.Norm(), 2);
            left_cost_rot += pow(leftEulerErr.norm(), 2);
            right_cost_rot += pow(rightEulerErr.norm(),2);

            //根据误差的数量级来调整权重
            posWeightRel = softConstraintRelWeight / cost_pos;
            leftRotWeightRel = softConstraintRelWeight / left_cost_rot;
            rightRotWeightRel = softConstraintRelWeight / right_cost_rot;


            //因为要用到中间得到的 goal rot 矩阵
            if(grad){
                //************compute joint angles dot all variables, get a dimension * n matrix********************
                Eigen::MatrixXd jointDotX = Eigen::MatrixXd::Zero(dimension, n); //这个时刻的每个关节角度对优化变量的导数
                prevTimeIntervalNum = i*7 + k+1; //外层的k只用来计算经历了多少个 timeInterval
                for(int j=0; j<dimension; j++){
                    jerkStartIndex = segment_num*7 + j*segment_num*7;
                    for(int k=0; k<prevTimeIntervalNum; k++){
                        //************************************对时间求导*********************************
                        //初始速度项
                        jointDotX(j, k) = (*pathPoints)(3, j);
                        //初始加速度项
                        tmp1 = 0;
                        for(int l=0; l<prevTimeIntervalNum; l++){
                            tmp1 += x[l];
                        }
                        jointDotX(j, k) += (*pathPoints)(4, j)*tmp1;

                        //正常项
                        jointDotX(j, k) += x[jerkStartIndex + k] * pow(x[k], 2) / 2;
                        //第一个复合项
                        if(k<prevTimeIntervalNum-1){
                            tmp1 = 0;
                            for(int l=k+1; l<prevTimeIntervalNum; l++){
                                tmp1+= x[l];
                            }
                            jointDotX(j, k) += x[jerkStartIndex+k] * pow(tmp1, 2) / 2;
                        }
                        if(k-1 >= 0){
                            for(int l=0; l<k; l++){
                                tmp1 = 0;
                                for(int m=l+1; m<prevTimeIntervalNum; m++){
                                    tmp1 += x[m];
                                }
                                jointDotX(j, k) += tmp1 * x[jerkStartIndex+l] * x[l];
                            }
                        }
                        //第二个复合项
                        if(k<prevTimeIntervalNum-1){
                            tmp1 = 0;
                            for(int l=k+1; l<prevTimeIntervalNum; l++){
                                tmp1+=x[l];
                            }
                            jointDotX(j, k) += x[jerkStartIndex+k]*x[k]*tmp1;
                        }
                        if(k-1 >=0){
                            for(int m=0; m<k; m++){
                                jointDotX(j, k) += x[jerkStartIndex+m]*pow(x[m], 2) / 2;
                            }
                        }

                        //************************************对jerk求导*********************************
                        jointDotX(j, jerkStartIndex + k) = pow(x[k], 3)/ 6;
                        if(k<prevTimeIntervalNum-1){
                            tmp1 = 0;
                            for(int l=k+1; l<prevTimeIntervalNum; l++){
                                tmp1+= x[l];
                            }
                            jointDotX(j, jerkStartIndex + k) += pow(x[k], 2) * tmp1 / 2 ;
                            jointDotX(j, jerkStartIndex + k) += x[k] * pow(tmp1, 2) / 2;
                        }
                    }
                }
                //**************************compute cartesian error dot joint angles*******************************
                leftJacSolver.JntToJac(leftAngleArrKDL, leftJacobian);
                rightJacSolver.JntToJac(rightAngleArrKDL, rightJacobian);
                Eigen::MatrixXd leftJacMatrix = leftJacobian.data;
                Eigen::MatrixXd rightJacMatrix = rightJacobian.data;

                Eigen::Matrix3d leftToEulerVel;
                leftToEulerVel(0, 0) = cos(leftEulerVec(0)) * tan(leftEulerVec(1));
                leftToEulerVel(0, 1) = sin(leftEulerVec(0)) * tan(leftEulerVec(1));
                leftToEulerVel(0, 2) = 1;
                leftToEulerVel(1, 0) = -sin(leftEulerVec(0));
                leftToEulerVel(1, 1) = cos(leftEulerVec(0));
                leftToEulerVel(1, 2) = 0;
                leftToEulerVel(2, 0) = cos(leftEulerVec(0)) / cos(leftEulerVec(1));
                leftToEulerVel(2, 1) = sin(leftEulerVec(0)) / cos(leftEulerVec(1));
                leftToEulerVel(2, 2) = 0;

                Eigen::Matrix3d rightToEulerVel;
                rightToEulerVel(0, 0) = cos(rightEulerVec(0)) * tan(rightEulerVec(1));
                rightToEulerVel(0, 1) = sin(rightEulerVec(0)) * tan(rightEulerVec(1));
                rightToEulerVel(0, 2) = 1;
                rightToEulerVel(1, 0) = -sin(rightEulerVec(0));
                rightToEulerVel(1, 1) = cos(rightEulerVec(0));
                rightToEulerVel(1, 2) = 0;
                rightToEulerVel(2, 0) = cos(rightEulerVec(0)) / cos(rightEulerVec(1));
                rightToEulerVel(2, 1) = sin(rightEulerVec(0)) / cos(rightEulerVec(1));
                rightToEulerVel(2, 2) = 0;

                leftJacMatrix.block(3, 0, 3, 7) = leftToEulerVel * leftJacMatrix.block(3, 0, 3, 7);
                rightJacMatrix.block(3, 0, 3, 7) = rightToEulerVel * rightJacMatrix.block(3, 0, 3, 7);

                //**************************compute cartesian error dot all variables *******************************
                Eigen::MatrixXd leftCartDotX = leftJacMatrix * jointDotX.block(0, 0, 7, n);
                Eigen::MatrixXd rightCartDotX = rightJacMatrix * jointDotX.block(7, 0, 7, n);
                for(int i=0; i<n; i++){
                    grad[i] += 2 * leftEulerErr(0) * leftCartDotX(3, i) * leftRotWeightRel;
                    grad[i] += 2 * leftEulerErr(1) * leftCartDotX(4, i) * leftRotWeightRel;
                    grad[i] += 2 * leftEulerErr(2) * leftCartDotX(5, i) * leftRotWeightRel;
                    grad[i] += 2 * rightPosErr(0) * (rightCartDotX(0, i) - leftCartDotX(0, i)) * posWeightRel;
                    grad[i] += 2 * rightPosErr(1) * (rightCartDotX(1, i) - leftCartDotX(1, i)) * posWeightRel;
                    grad[i] += 2 * rightPosErr(2) * (rightCartDotX(2, i) - leftCartDotX(2, i)) * posWeightRel;
                    grad[i] += 2 * rightEulerErr(0) * (rightCartDotX(3, i)- leftCartDotX(3, i))* rightRotWeightRel;
                    grad[i] += 2 * rightEulerErr(1) * rightCartDotX(4, i) * rightRotWeightRel;
                    grad[i] += 2 * rightEulerErr(2) * rightCartDotX(5, i) * rightRotWeightRel;
                }
            }// end grad
        }//end 7 time interval loop
        for(int j=0; j<dimension; j++){
            p[0][j] = p[7][j];
            v[0][j] = v[7][j];
            a[0][j] = a[7][j];
        }
    }//end segment loop

    if(grad){
        for(int i=0; i<n; i++){
            grad[i] *= step;
        }
    }

#ifdef MY_OPTIMIZATION_DEBUG
    std::cout<<"itre "<<iterationCount++<<std::endl;
    std::cout<<"cost time "<<cost<<std::endl;
    std::cout<<"cost pos error "<<cost_pos<<std::endl;
    std::cout<<"left cost rot error "<<left_cost_rot<<std::endl;
    std::cout<<"right cost rot error "<<right_cost_rot<<std::endl;
#endif

    return pow(cost, 2) + posWeightRel*cost_pos + leftRotWeightRel*left_cost_rot + rightRotWeightRel*right_cost_rot;
}

void TimeDualConstraintPlanner::equalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    Eigen::MatrixXd *pathPoints = (Eigen::MatrixXd *)f_data;

#ifdef MY_OPTIMIZATION_DEBUG
    cout<<"initial velocity and acceleration"<<endl;
    cout<<(*pathPoints).block(3, 0, 2, dimension)<<endl;
#endif

    int curIndex, jerkStartIndex, timeIndex, jerkIndex;
    if(grad){
        for(int i=0; i<m*n; i++){
            grad[i] = 0;
        }
        double tmp1 = 0;
        curIndex = 0;
        //需要把所有segment的j和t都考虑
        int prevTimeIntervalNum = 0;
        for(int j=0; j<dimension; j++){
            jerkStartIndex = segment_num*7 + j*(segment_num*7); //jerk start index for this dimension

            //*********************每一段的中间加速度、jerk，到达位置****************************************
            for(int i=0; i<segment_num; i++){
                //这个segment的加速度约束
                prevTimeIntervalNum = i*7+3;
                for(int k=0; k<prevTimeIntervalNum; k++){
                    grad[curIndex*n+k] = x[jerkStartIndex + k];
                    grad[curIndex*n +jerkStartIndex+k] = x[k];
                }
                curIndex++;

                //这个segment 结束的位置等于下一个pathPoint
                prevTimeIntervalNum =i*7+7;
                for(int k=0; k<prevTimeIntervalNum; k++){
                    //************************************对时间求导*********************************
                    //初始速度项
                    grad[curIndex*n + k] = (*pathPoints)(3, j);
                    //初始加速度项
                    tmp1 = 0;
                    for(int l=0; l<prevTimeIntervalNum; l++){
                        tmp1 += x[l];
                    }
                    grad[curIndex*n + k] += (*pathPoints)(4, j) * tmp1;

                    //正常项
                    grad[curIndex*n + k] += x[jerkStartIndex + k] * pow(x[k], 2) / 2;
                    //第一个复合项
                    if(k<prevTimeIntervalNum-1){
                        tmp1 = 0;
                        for(int l=k+1; l<prevTimeIntervalNum; l++){
                            tmp1+= x[l];
                        }
                        grad[curIndex*n + k] += x[jerkStartIndex+k] * pow(tmp1, 2) / 2;
                    }
                    if(k-1 >= 0){
                        for(int l=0; l<k; l++){
                            tmp1 = 0;
                            for(int m=l+1; m<prevTimeIntervalNum; m++){
                                tmp1 += x[m];
                            }
                            grad[curIndex*n + k] += tmp1 * x[jerkStartIndex+l] * x[l];
                        }
                    }
                    //第二个复合项
                    if(k<prevTimeIntervalNum-1){
                        tmp1 = 0;
                        for(int l=k+1; l<prevTimeIntervalNum; l++){
                            tmp1+=x[l];
                        }
                        grad[curIndex*n+k] += x[jerkStartIndex+k]*x[k]*tmp1;
                    }
                    if(k-1 >=0){
                        for(int m=0; m<k; m++){
                            grad[curIndex*n + k] += x[jerkStartIndex+m]*pow(x[m], 2) / 2;
                        }
                    }

                    //************************************对jerk求导*********************************
                    grad[curIndex*n + jerkStartIndex + k] += pow(x[k], 3)/ 6;
                    if(k<prevTimeIntervalNum-1){
                        tmp1 = 0;
                        for(int l=k+1; l<prevTimeIntervalNum; l++){
                            tmp1+= x[l];
                        }
                        grad[curIndex*n + jerkStartIndex + k] += pow(x[k], 2) * tmp1 / 2 ;
                        grad[curIndex*n + jerkStartIndex + k] += x[k] * pow(tmp1, 2) / 2;
                    }
                }
                curIndex++;
            }//end segment num loop

//          ************************************最后一段的速度****************************************
            prevTimeIntervalNum = segment_num * 7;
            for(int k=0; k<prevTimeIntervalNum; k++){
                //对t求导
                //初始加速度项
                grad[curIndex*n+k] = (*pathPoints)(4, j);
                //正常项
                grad[curIndex*n+k] += x[jerkStartIndex +k]*x[k];
                //复合项
                if(k<prevTimeIntervalNum-1){
                    tmp1=0;
                    for(int l=k+1; l<prevTimeIntervalNum; l++){
                        tmp1+=x[l];
                    }
                    grad[curIndex*n+k] += x[jerkStartIndex +k] * tmp1;
                }
                if(k-1>=0){
                    for(int m=0; m<k; m++){
                        grad[curIndex*n+k] += x[jerkStartIndex + m]*x[m];
                    }
                }
                //对j求导
                grad[curIndex*n + jerkStartIndex + k] = pow(x[k], 2) / 2;
                if(k<prevTimeIntervalNum-1){
                    tmp1 = 0;
                    for(int l=k+1; l<prevTimeIntervalNum; l++){
                        tmp1+=x[l];
                    }
                    grad[curIndex*n + jerkStartIndex + k]+=x[k]*tmp1;
                }
            }
            curIndex++;
            //*********************************最后一段的加速度****************************************
            prevTimeIntervalNum = segment_num * 7;
            for(int k=0; k<prevTimeIntervalNum; k++){
                grad[curIndex*n + k] = x[jerkStartIndex + k];
                grad[curIndex*n + jerkStartIndex + k] = x[k];
            }
            curIndex++;
        }//end dimension loop
        for(int i=0; i<m*n; i++){
            grad[i] *= step;
        }
    }//end grad


    for(int i=0; i<m; i++){
        result[i] = 0;
    }
    //每一段的位置约束，主要是计算到最后需要等于下一段的开头
    Eigen::MatrixXd recordMidVelocity(2, dimension);
    curIndex = 0;
    double p[8],v[8], a[8];
    for(int j=0; j<dimension; j++){
        jerkStartIndex = segment_num*7 + j*(segment_num * 7);
        p[0] = (*pathPoints)(0, j);
        v[0] = (*pathPoints)(3, j);
        a[0] = (*pathPoints)(4, j);
        for(int i=0; i<segment_num; i++){
             for(int k=0; k<7; k++){
                jerkIndex = jerkStartIndex + i*7 + k;
                timeIndex = i*7 + k;
                a[k+1] = a[k] + x[jerkIndex] * x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
                v[k+1] = v[k] + a[k] * x[timeIndex] + 0.5 * x[jerkIndex] * pow(x[timeIndex], 2);
                p[k+1] = p[k] + v[k] * x[timeIndex] + 0.5 * a[k] *pow(x[timeIndex], 2) + x[jerkIndex] * pow(x[timeIndex], 3) / 6;
            }
            result[curIndex++] = a[3];
            result[curIndex++] = p[7] - (*pathPoints)(i+1, j); //第7小段到达的位置等于下一段的位置
            //记录这一个segment达到的速度和加速度（其实可以直接保存下来用于不等式约束的计算。 ）
            p[0] = p[7];
            v[0] = v[7];
            a[0] = a[7];
            if(i==0){
                recordMidVelocity(0, j) = v[7];
                recordMidVelocity(1, j) = a[7];
            }
        }
        result[curIndex++] = v[7];
        result[curIndex++] = a[7];
    }
    //********************************************debug information**********************************
#ifdef MY_OPTIMIZATION_DEBUG
    cout<<"record mid velocity and acceleration"<<endl;
    cout<<recordMidVelocity<<endl;
    cout<<"equality constraint result"<<endl;
    for(int i=0; i<dimension; i++){
        for(int j=0; j<(2*segment_num + 2); j++){
            cout<<result[i*(2*segment_num + 2)+j]<<" ";
        }
        cout<<endl<<endl;
    }
#endif
}

void TimeDualConstraintPlanner::inequalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    Eigen::MatrixXd *jointLimits = (Eigen::MatrixXd *)f_data;
    int curIndex, jerkStartIndex,jerkIndex, timeIndex;
    if(grad){
        for(size_t i=0; i<m*n; i++){
            grad[i] = 0;
        }
        double tmp;
        int prevTimeIntervalNum;
        curIndex = 0;
        for(int j=0; j<dimension; j++){
            prevTimeIntervalNum = 0;
            jerkStartIndex = segment_num*7 + j*(segment_num*7);
            for(int i=0; i<segment_num; i++){
                for(int k=0; k<7; k++){
                    prevTimeIntervalNum++; //每一个都需要考虑之前的所有的，直接记录这是第几段
                    for(int l=0; l<prevTimeIntervalNum; l++){
                        //t
                        //初始加速度项
                        grad[curIndex*n + l] = (*jointLimits)(4, j);
                        //
                        grad[curIndex*n + l] += x[jerkStartIndex + l] * x[l]; //jiti
                        if(l<prevTimeIntervalNum-1){
                            tmp = 0;
                            for(int m=l+1; m<prevTimeIntervalNum; m++){
                                tmp+=x[m];
                            }
                            grad[curIndex*n + l]+= x[jerkStartIndex + l] *tmp;
                        }
                        if(l-1>=0){
                            for(int m=0; m<l; m++){
                                grad[curIndex*n + l]+= x[jerkStartIndex + m]*x[m];
                            }
                        }
                        grad[(curIndex+1)*n + l] = -grad[curIndex*n + l];//直接把下界的也直接得到
                        //j
                        grad[curIndex*n + jerkStartIndex + l] = 0.5 * pow(x[l], 2);
                        if(l<prevTimeIntervalNum-1){
                            tmp = 0;
                            for(int m=l+1; m<prevTimeIntervalNum; m++){
                                tmp += x[m];
                            }
                            grad[curIndex*n + jerkStartIndex + l] += x[l]*tmp;
                        }
                        grad[(curIndex+1)*n + jerkStartIndex + l] = -grad[curIndex*n + jerkStartIndex + l];//直接把下界的也直接得到
                    }
                    curIndex+=2;


                    for(int l=0; l<prevTimeIntervalNum; l++){
                        grad[curIndex*n + l] = x[jerkStartIndex + l];
                        grad[curIndex*n + jerkStartIndex + l] = x[l];
                        grad[(curIndex+1)*n + l] = -grad[curIndex*n + l];
                        grad[(curIndex+1)*n + jerkStartIndex + l] = - grad[curIndex*n + jerkStartIndex + l];
                    }
                    curIndex+=2;
                }
            }//end segment loop
        }//end dimension loop
        for(size_t i=0; i<m*n; i++){
            grad[i] *= step;
        }
    }//end grad

    double v, a, last_v=0, last_a=0;
    for(int i=0; i<m; i++){
        result[i] = 0;
    }
    curIndex = 0;
    for(int j=0; j<dimension; j++){
        last_v = (*jointLimits)(3, j);
        last_a = (*jointLimits)(4, j);
        jerkStartIndex = segment_num*7 + j*(segment_num*7);
        for(int i=0; i<segment_num; i++){
            for(int k=0; k<7; k++){
                jerkIndex = jerkStartIndex + i*7 + k;
                timeIndex = i*7 + k;
                a = last_a + x[jerkIndex] * x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
                v = last_v + last_a * x[timeIndex] + 0.5 * x[jerkIndex] * pow(x[timeIndex], 2);
                last_a = a;
                last_v = v;
                result[curIndex++] = v - (*jointLimits)(0, j);
                result[curIndex++] = -v - (*jointLimits)(0, j);
                result[curIndex++] = a - (*jointLimits)(1, j);
                result[curIndex++] = -a - (*jointLimits)(1, j);
            }
        }
    }
}


