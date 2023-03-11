//
// Created by lijiashushu on 20-4-17.
//
#include "dual_rrt_star/trajectory_planner/time_dual_constraint_planner/time_dual_constraint_planner.h"
#include <ros/ros.h>

#define INF 1e10
#define MY_OPTIMIZATION_DEBUG


#ifdef THIS_FILE
void equalityConstraintFuncNew(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    Eigen::MatrixXd *pathPoints = (Eigen::MatrixXd *)f_data;
    int curIndex, gradCurIndex, jerkStartIndex,jerkIndex, timeIndex;

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
    for(int i=0; i<m; i++){
        result[i] = 0;
    }
    //每一段的位置约束，主要是计算到最后需要等于下一段的开头
    curIndex = 0;
    gradCurIndex = 0;
    double p[8][dimension],v[8][dimension], a[8][dimension];
    for(int j=0; j<dimension; j++){
        p[0][j] = (*pathPoints)(0, j);
        v[0][j] = 0;
        a[0][j] = 0;
    }
    for(int i=0; i<segment_num; i++){
        for(int k=0; k<7; k++){
            //*****************************compute all joint angles after k time intervals**********************
            for(int j=0; j<dimension; j++){
                jerkIndex = segment_num*7 + j*(segment_num * 7) + i*7 + k;
                timeIndex = i*7 + k;
                a[k+1][j] = a[k][j] + x[jerkIndex] * x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
                v[k+1][j] = v[k][j] + a[k][j] * x[timeIndex] + 0.5 * x[jerkIndex] * pow(x[timeIndex], 2);
                p[k+1][j] = p[k][j] + v[k][j] * x[timeIndex] + 0.5 * a[k][j] *pow(x[timeIndex], 2) + x[jerkIndex] * pow(x[timeIndex], 3) / 6;
            }
            //***************************compute cartesian position and rotation of both arms**********************
            for(int j=0; j<dimension/2; j++) {
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
            double left_yaw, left_pitch, left_roll;
            leftRot.GetEulerZYX(left_yaw, left_pitch, left_roll);
            KDL::Rotation leftGoalRot = KDL::Rotation::EulerZYX(left_yaw, 0, 1.57);
            KDL::Rotation leftErrRot = leftGoalRot * leftRot.Inverse();
            KDL::Vector leftAxis;
            double leftAngleErr = -leftErrRot.GetRotAngle(leftAxis);

            KDL::Vector distance(0, 0, 0.2);
            KDL::Rotation rightGoalRot = KDL::Rotation::EulerZYX(left_yaw, 0.0, -1.57);
            KDL::Vector rightGoalPos = leftGoalRot * distance + leftPos;
            KDL::Vector rightPosErr = rightPos - rightGoalPos;
            KDL::Rotation rightErrRot = rightGoalRot * rightRot.Inverse();
            KDL::Vector rightAxis;
            double rightAngleErr = -rightErrRot.GetRotAngle(rightAxis);

//            result[curIndex++] = leftAngleErr * leftAxis(0);
//            result[curIndex++] = leftAngleErr * leftAxis(1);
//            result[curIndex++] = leftAngleErr * leftAxis(2);
            result[curIndex++] = rightPosErr(0);
            result[curIndex++] = rightPosErr(1);
            result[curIndex++] = rightPosErr(2);
//            result[curIndex++] = rightAngleErr * rightAxis(0);
//            result[curIndex++] = rightAngleErr * rightAxis(1);
//            result[curIndex++] = rightAngleErr * rightAxis(2);

            //因为要用到中间得到的 goal rot 矩阵
            if(grad){
                //************compute joint angles dot all variables, get a dimension * n matrix********************
                Eigen::MatrixXd jointDotX = Eigen::MatrixXd::Zero(dimension, n); //这个时刻的每个关节角度对优化变量的导数
                prevTimeIntervalNum = i*7 + k+1; //外层的k只用来计算经历了多少个 timeInterval
                for(int j=0; j<dimension; j++){
                    jerkStartIndex = segment_num*7 + j*segment_num*7;
                    for(int k=0; k<prevTimeIntervalNum; k++){
                        //************************************对时间求导*********************************
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
                        jointDotX(j, jerkStartIndex + k) += pow(x[k], 3)/ 6;
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
                KDL::Rotation leftGoalRotInv = leftGoalRot.Inverse();
                KDL::Rotation rightGoalRotInv = rightGoalRot.Inverse();
                Eigen::MatrixXd leftGoalRotInvEigen(3, 3), rightGoalRotInvEigen(3, 3);
                for(int i=0; i<3; i++){
                    for(int j=0; j<3; j++){
                        leftGoalRotInvEigen(i, j) = leftGoalRotInv(i, j);
                        rightGoalRotInvEigen(i, j) = rightGoalRotInv(i, j);
                    }
                }
                leftJacMatrix.block(3, 0, 3, 7) = leftGoalRotInvEigen * leftJacMatrix.block(3, 0, 3, 7);
                rightJacMatrix.block(3, 0, 3, 7) = rightGoalRotInvEigen * rightJacMatrix.block(3, 0, 3, 7);
                //**************************compute cartesian error dot all variables *******************************
                Eigen::MatrixXd leftCartDotX = leftJacMatrix.block(3, 0, 3, 7) * jointDotX.block(0, 0, 7, n);
                Eigen::MatrixXd rightCartDotX = rightJacMatrix * jointDotX.block(7, 0, 7, n);
                for(int i=0; i<n; i++){
//                    grad[i] += 2 * leftEulerErr(0) * leftCartDotX(3, i) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(1) * leftCartDotX(4, i) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(2) * leftCartDotX(5, i) * leftRotWeightRel;
//                    grad[i] += 2 * rightPosErr(0) * (rightCartDotX(0, i) - leftCartDotX(0, i)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(1) * (rightCartDotX(1, i) - leftCartDotX(1, i)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(2) * (rightCartDotX(2, i) - leftCartDotX(2, i)) * posWeightRel;
//                    grad[i] += 2 * rightEulerErr(0) * (rightCartDotX(3, i)- leftCartDotX(3, i))* rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(1) * rightCartDotX(4, i) * rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(2) * rightCartDotX(5, i) * rightRotWeightRel;
//                    grad[gradCurIndex*n+i] = leftCartDotX(0, i);
//                    grad[(gradCurIndex        +1)*n+i] = leftCartDotX(1, i);
//                    grad[(gradCurIndex+2)*n+i] = leftCartDotX(2, i);
                    grad[(gradCurIndex+0)*n+i] = rightCartDotX(0, i) - leftCartDotX(0, i);
                    grad[(gradCurIndex+1)*n+i] = rightCartDotX(1, i) - leftCartDotX(1, i);
                    grad[(gradCurIndex+2)*n+i] = rightCartDotX(2, i) - leftCartDotX(2, i);
//                    grad[(gradCurIndex+6)*n+i] = rightCartDotX(3, i);
//                    grad[(gradCurIndex+7)*n+i] = rightCartDotX(4, i);
//                    grad[(gradCurIndex+8)*n+i] = rightCartDotX(5, i);
                }
                gradCurIndex += 3;
            }// end grad

        }//end 7 time interval loop
        for(int j=0; j<dimension; j++){
            p[0][j] = p[7][j];
            v[0][j] = v[7][j];
            a[0][j] = a[7][j];
        }
    }//end segment loop


//    cout<<"variable"<<endl;
//    for(int i=0; i<segment_num*7; i++){
//        cout<<x[i]<<" ";`
//    }
//    cout<<endl;
//    for(int i=0; i<dimension; i++){
//        for(int j=0; j<segment_num*7; j++){
//            cout<<x[segment_num*7 + i*segment_num*7 + j]<<" ";
//        }
//        cout<<endl;
//    }
//    cout<<endl<<endl;

//    cout<<"new equality constrain grad"<<endl;
//    for(int i=0; i<m; i++){
//        for(int j=0; j<dimension+1; j++){
//            for(int k=0; k<segment_num*7; k++){
//                cout<<grad[i*n+j*segment_num*7 + k]<<" ";
//            }
//            cout<<endl;
//        }
//        cout<<endl<<endl;
//    }

    cout<<"new equality constraint result !!"<<endl;
    for(int i=0; i<segment_num * 7; i++){
        for(int j=0; j<9; j++){
            cout<<result[i*9 + j]<<" ";
        }
        cout<<endl<<endl;
    }
}
#endif

int main(int argc, char**argv){
    ros::init(argc, argv, "one_plan_node");
    ros::NodeHandle nh("~");

    Eigen::MatrixXd pathPoints(7, 14);
    pathPoints<<
0.514998, -0.487572,-1.79923,1.6679,-0.28682,0.603706, 2.86722, -0.517204,-0.49348, 1.79396,1.6679,0.302716,0.602833,-2.87906,
0.695107, -0.388971,-1.83187, 1.83449, -0.277127,0.661101, 2.92719, -0.633855, -0.449291,1.8943, 2.01821,0.336925,0.329849,-2.92437,
0.730323, -0.285182,-1.80333,2.1025, -0.264298,0.448463, 3.03204, -0.758369, -0.287821, 1.82707, 2.16735,0.273008,0.359966,-3.02602,
0.756027,-0.00105752,-1.73229, 2.29707,0.269464,0.314336,2.7458, -0.791898, 0.0141751, 1.73216, 2.27622, -0.257883,0.366565,-2.76635,
0.604934, 0.0631528, -1.6692, 2.12547,0.309917,0.367043, 2.81296, -0.721268, 0.0661433, 1.67025, 2.15963, -0.296116,0.387328,-2.82283,
0.616999,0.157597,-1.60399, 2.05248,0.349111,0.427176, 2.89569,-0.61261,0.146656, 1.59655, 2.07604, -0.345805,0.385839,-2.89427,
0.665724,0.337947,-1.44691,2.0171,0.430279,0.472696, 3.05881, -0.640635,0.334855, 1.43792, 2.03636, -0.432353,0.425846,-3.05814;

    TimeDualConstraintPlanner planner;
    Eigen::MatrixXd allSegCoeff(pathPoints.rows()-1, 7*(14+1));
    planner.getAllSegCoeff(pathPoints, allSegCoeff, 0, 1, 1);
//    planner.outputTrajToFile(allSegCoeff, pathPoints, "/home/lijiashushu/ros_ws/src/dual_rrt_star/data");

    //******************************************单独测试***************************************************
//    using namespace Eigen;
//    using namespace std;
//    ros::init(argc, argv, "one_plan_node");
//    ros::NodeHandle nh("~");
//
//    KDL::Tree kdlTree;
//    kdl_parser::treeFromParam("robot_description", kdlTree);
//    kdlTree.getChain("base", "left_gripper", leftChain);
//    kdlTree.getChain("base", "right_gripper", rightChain);
//
//    MatrixXd startPoint(1,14);
//    MatrixXd midPoint(1,14);
//    MatrixXd endPoint(1,14);
//    //    startPoint<<-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081, -0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128;
//    //    midPoint<<0.270022,-0.740675,-1.19372,1.23129,2.32521,-1.51671,0.0311289,-0.150274, -0.33512,  1.70708,  1.23217, -2.93672, -1.26642, 0.243694;
//    //    endPoint<<0.513621,-0.791709,-1.2481,  1.45756,2.3027, -1.53165, 0.141405,-0.199454,-0.320086,  1.72987,  1.49196, -2.92022, -1.09762,  0.19719;
//
//    startPoint<<0.294272,-0.604898, -1.03688,  2.01083,  2.47495, -1.29819, 0.699967,  -1.1442,-0.135134,  1.36257,  2.41997, -2.99699,-0.859209,-0.317002;
//    midPoint<<0.148715,-0.428094,-0.894495,  1.77455,   2.5583, -1.34528, 0.718425,-0.950577,-0.022152,  1.20183,  2.25282, -3.07185,-0.820592,-0.413582;
//    endPoint<<0.115122,-0.320197,-0.812176,  1.55132,  2.59481, -1.45057, 0.709246,-0.716477,0.0776586,  1.16501,  2.07835,  -3.0906, -0.78808,-0.447044;
//
//    Eigen::MatrixXd deravativeLimit(3, 14);
//    deravativeLimit<<2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
//                    2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
//                    2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4;
//    deravativeLimit = deravativeLimit;
//    //get path points
//    MatrixXd pathPoints =  MatrixXd::Zero(segment_num+1, 14);
//    pathPoints.row(0) = startPoint;
//    pathPoints.row(1) = midPoint;
//    pathPoints.row(2) = endPoint;
//
//    //********************************JLT generate initial value *******************************
//    vector<double> initValue(7*segment_num + 7*segment_num*dimension);
//    for(int i=0; i<segment_num; i++){
//        Eigen::MatrixXd segTrajCoeff = getThirdOrderMultidofZeroSync(pathPoints.row(i), Eigen::VectorXd::Zero(14), Eigen::VectorXd::Zero(14), pathPoints.row(i+1), deravativeLimit);
//        for(int j=0; j<7; j++){
//            initValue[i*7+j] = segTrajCoeff(0, j);
//            for(int k=0; k<dimension; k++){
//                initValue[segment_num*7 + k*segment_num*7 + i*7 + j] =  segTrajCoeff(1, k*7+j);
//            }
//        }
//    }
//
//    //**********************************nlopt求解时间最优轨迹**************************************
//    int varibleNum = segment_num * 7 +  segment_num * dimension *7;
//    int equalityNum = (2* segment_num + 2) * dimension;//每段3个jerk=0,a3=0,p[7]=pathpoints(i);each segment end v=0,a=0
//    int inequalityNum = 4 * 7 * segment_num * dimension;
//    int newEqualityNum = 3 * segment_num * 7;
//
//    double x[varibleNum]; //initial val
////    ifstream inFile;
////    inFile.open("/home/lijiashushu/initial_value_JLT.txt");
////    string number;
////    int count = 0;
////    while(getline(inFile,number))
////    {
////        x[count++] = stod(number);
////    }
//
//    for(int i=0; i<initValue.size();i++){
//        x[i] = initValue[i];
//    }
//
//    double lb[varibleNum];
//    double ub[varibleNum];
//    for(int i=0; i<segment_num*7; i++){
//        lb[i] = -0.0001;
////        lb[i] = -INF;
//        ub[i] = INF;
//    }
//    for(int i=0; i<dimension; i++){
//        for(int j=0; j<segment_num; j++){
//            for(int k=0; k<7; k++){
//                if(k==1 || k==3 || k==5){
//                    lb[segment_num*7 + i*segment_num*7 + j*7 + k] = 0;
//                    ub[segment_num*7 + i*segment_num*7 + j*7 + k] = 0;
//                }
//                else{
//                    lb[segment_num*7 + i*segment_num*7 + j*7 + k] = -deravativeLimit(2, i);
//                    ub[segment_num*7 + i*segment_num*7 + j*7 + k] = deravativeLimit(2, i);
//                }
//            }
//        }
//    }
//    double eqTol[equalityNum];
//    for(int i=0; i<equalityNum; i++){
//        eqTol[i] = 1e-4;
//    }
//    double newEqTol[newEqualityNum];
//    for(int i=0; i<newEqualityNum; i++){
//        newEqTol[i] = 1e-4;
//    }
//    double ineqTol[inequalityNum];
//    for(int i=0; i<inequalityNum; i++){
//        ineqTol[i] = 1e-4;
//    }
//
//    nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP/*NLOPT_LD_MMA*/, varibleNum);
//    nlopt_set_lower_bounds(opter, lb);//设置优化变量的下限
//    nlopt_set_upper_bounds(opter, ub);
//    nlopt_set_min_objective(opter, objectiveFuncNew, &pathPoints); //设置目标函数
//    nlopt_add_equality_mconstraint(opter, equalityNum,  equalityConstraintFunc, &pathPoints, eqTol);// 等式约束
//    nlopt_add_inequality_mconstraint(opter, inequalityNum, inequalityConstraintFunc, &deravativeLimit, ineqTol);// 不等式约束；
//
//    nlopt_set_xtol_rel(opter, 1e-4);
//    nlopt_set_ftol_rel(opter, 1e-4);
//
//    double f_min; //目标函数的值
//    int result = nlopt_optimize(opter, x, &f_min);
//    if (result < 0) {
//        printf("nlopt failed!  %d\n", result);
//    }
//    else {
//        cout<<"found minimum "<<endl;
//        cout<<"cost: "<<f_min<<endl;
//        ofstream outFile;
//        outFile.open("/home/lijiashushu/optimal_para.txt");
//        for(int i=0; i<varibleNum; i++){
//            outFile<<x[i]<<endl;
//        }
//        outFile.close();
//        outputTrajToFile(x, pathPoints);
//    }
//    nlopt_destroy(opter);
//
    return 0;
}






//记得修改 p v a 数组的大小
//void tmp(){
//    for(int i=0; i<segment_num; i++){
//        for(int k=0; k<14; k++){
//            //*****************************compute all joint angles after k time intervals**********************
//            for(int j=0; j<dimension; j++){
//                jerkIndex = 2*segment_num*7 + j*2*(segment_num * 7) + i*2*7 + k;
//                timeIndex = i*2*7 + k;
//                a[k+1][j] = a[k][j] + double_x[jerkIndex] *double_x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
//                v[k+1][j] = v[k][j] + a[k][j] * double_x[timeIndex] + 0.5 * double_x[jerkIndex] * pow(double_x[timeIndex], 2);
//                p[k+1][j] = p[k][j] + v[k][j] * double_x[timeIndex] + 0.5 * a[k][j] *pow(double_x[timeIndex], 2) + double_x[jerkIndex] * pow(double_x[timeIndex], 3) / 6;
//            }
//            //***************************compute cartesian position and rotation of both arms**********************
//            for(int j=0; j<dimension/2; j++) {
//                leftAngleArrKDL(j) = p[k + 1][j];
//                rightAngleArrKDL(j) = p[k + 1][j + 7];
//            }
//
//            leftFKSolver.JntToCart(leftAngleArrKDL, leftFrame);
//            rightFKSolver.JntToCart(rightAngleArrKDL, rightFrame);
//            //假设当前 left 的 xyz 和 yaw 角是正确的。
//            leftPos = leftFrame.p;
//            rightPos = rightFrame.p;
//            leftRot = leftFrame.M;
//            rightRot = rightFrame.M;
//
//            //******************compute cartesian error, take current left(master) arm's xyz and yaw as reference. cur minus goal**********************
//            Eigen::Vector3d leftEulerVec;
//            leftRot.GetEulerZYX(leftEulerVec(0), leftEulerVec(1), leftEulerVec(2));
//            Eigen::Vector3d leftGoalEulerVec(leftEulerVec(0), 0, 1.57);
//            Eigen::Vector3d leftEulerErr = leftEulerVec - leftGoalEulerVec;
//
//            KDL::Rotation leftGoalRot = KDL::Rotation::EulerZYX(leftEulerVec(0), 0, 1.57);
//            KDL::Vector distance(0, 0, 0.2);
//            KDL::Vector rightGoalPos = leftGoalRot * distance + leftPos;
//            KDL::Vector rightPosErr = rightPos - rightGoalPos;
//
//            Eigen::Vector3d rightEulerVec;
//            rightRot.GetEulerZYX(rightEulerVec(0), rightEulerVec(1), rightEulerVec(2));
//            Eigen::Vector3d rightGoalEulerVec(leftEulerVec(0), 0, -1.57);
//            Eigen::Vector3d rightEulerErr = rightEulerVec - rightGoalEulerVec;
//
//            cost_pos += pow(rightPosErr.Norm(), 2);
//            left_cost_rot += pow(leftEulerErr.norm(), 2);
//            right_cost_rot += pow(rightEulerErr.norm(),2);
//
//            //根据误差的数量级来调整权重
//            posWeightRel = softConstraintRelWeight / cost_pos;
//            leftRotWeightRel = softConstraintRelWeight / left_cost_rot;
//            rightRotWeightRel = softConstraintRelWeight / right_cost_rot;
//
//
//            //因为要用到中间得到的 goal rot 矩阵
//            if(grad){
//                //************compute joint angles dot all variables, get a dimension * n matrix********************
//                Eigen::MatrixXd jointDotX = Eigen::MatrixXd::Zero(dimension, 2*n); //这个时刻的每个关节角度对优化变量的导数
//                prevTimeIntervalNum = i*7*2 + k+1; //外层的k只用来计算经历了多少个 timeInterval
//                for(int j=0; j<dimension; j++){
//                    jerkStartIndex = 2*segment_num*7 + 2*j*segment_num*7;
//                    for(int k=0; k<prevTimeIntervalNum; k++){
//                        //************************************对时间求导*********************************
//                        //初始速度项
//                        jointDotX(j, k) += (*pathPoints)(3, j);
//                        //初始加速度项
//                        tmp1 = 0;
//                        for(int l=0; l<prevTimeIntervalNum; l++){
//                            tmp1 += double_x[l];
//                        }
//                        jointDotX(j, k) += (*pathPoints)(4, j)*tmp1;
//
//                        //正常项
//                        jointDotX(j, k) += double_x[jerkStartIndex + k] * pow(double_x[k], 2) / 2;
//                        //第一个复合项
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+= double_x[l];
//                            }
//                            jointDotX(j, k) += double_x[jerkStartIndex+k] * pow(tmp1, 2) / 2;
//                        }
//                        if(k-1 >= 0){
//                            for(int l=0; l<k; l++){
//                                tmp1 = 0;
//                                for(int m=l+1; m<prevTimeIntervalNum; m++){
//                                    tmp1 += double_x[m];
//                                }
//                                jointDotX(j, k) += tmp1 * double_x[jerkStartIndex+l] * double_x[l];
//                            }
//                        }
//                        //第二个复合项
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+=double_x[l];
//                            }
//                            jointDotX(j, k) += double_x[jerkStartIndex+k]*double_x[k]*tmp1;
//                        }
//                        if(k-1 >=0){
//                            for(int m=0; m<k; m++){
//                                jointDotX(j, k) += double_x[jerkStartIndex+m]*pow(double_x[m], 2) / 2;
//                            }
//                        }
//
//                        //************************************对jerk求导*********************************
//                        jointDotX(j, jerkStartIndex + k) += pow(double_x[k], 3)/ 6;
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+= double_x[l];
//                            }
//                            jointDotX(j, jerkStartIndex + k) += pow(double_x[k], 2) * tmp1 / 2 ;
//                            jointDotX(j, jerkStartIndex + k) += double_x[k] * pow(tmp1, 2) / 2;
//                        }
//                    }
//                }
//                //**************************compute cartesian error dot joint angles*******************************
//                leftJacSolver.JntToJac(leftAngleArrKDL, leftJacobian);
//                rightJacSolver.JntToJac(rightAngleArrKDL, rightJacobian);
//                Eigen::MatrixXd leftJacMatrix = leftJacobian.data;
//                Eigen::MatrixXd rightJacMatrix = rightJacobian.data;
//
//                Eigen::Matrix3d leftToEulerVel;
//                leftToEulerVel(0, 0) = cos(leftEulerVec(0)) * tan(leftEulerVec(1));
//                leftToEulerVel(0, 1) = sin(leftEulerVec(0)) * tan(leftEulerVec(1));
//                leftToEulerVel(0, 2) = 1;
//                leftToEulerVel(1, 0) = -sin(leftEulerVec(0));
//                leftToEulerVel(1, 1) = cos(leftEulerVec(0));
//                leftToEulerVel(1, 2) = 0;
//                leftToEulerVel(2, 0) = cos(leftEulerVec(0)) / cos(leftEulerVec(1));
//                leftToEulerVel(2, 1) = sin(leftEulerVec(0)) / cos(leftEulerVec(1));
//                leftToEulerVel(2, 2) = 0;
//
//                Eigen::Matrix3d rightToEulerVel;
//                rightToEulerVel(0, 0) = cos(rightEulerVec(0)) * tan(rightEulerVec(1));
//                rightToEulerVel(0, 1) = sin(rightEulerVec(0)) * tan(rightEulerVec(1));
//                rightToEulerVel(0, 2) = 1;
//                rightToEulerVel(1, 0) = -sin(rightEulerVec(0));
//                rightToEulerVel(1, 1) = cos(rightEulerVec(0));
//                rightToEulerVel(1, 2) = 0;
//                rightToEulerVel(2, 0) = cos(rightEulerVec(0)) / cos(rightEulerVec(1));
//                rightToEulerVel(2, 1) = sin(rightEulerVec(0)) / cos(rightEulerVec(1));
//                rightToEulerVel(2, 2) = 0;
//
//                leftJacMatrix.block(3, 0, 3, 7) = leftToEulerVel * leftJacMatrix.block(3, 0, 3, 7);
//                rightJacMatrix.block(3, 0, 3, 7) = rightToEulerVel * rightJacMatrix.block(3, 0, 3, 7);
//
//                //**************************compute cartesian error dot all variables *******************************
//                Eigen::MatrixXd leftCartDotX = leftJacMatrix * jointDotX.block(0, 0, 7, 2*n);
//                Eigen::MatrixXd rightCartDotX = rightJacMatrix * jointDotX.block(7, 0, 7, 2*n);
//                for(int i=0, j=0; i<n; i++, j+=2){
//                    grad[i] += 2 * leftEulerErr(0) * (leftCartDotX(3, j) + leftCartDotX(3, j+1)) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(1) * (leftCartDotX(4, j) + leftCartDotX(4, j+1)) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(2) * (leftCartDotX(5, j) + leftCartDotX(5, j+1))* leftRotWeightRel;
//                    grad[i] += 2 * rightPosErr(0) * (rightCartDotX(0, j) - leftCartDotX(0, j) + rightCartDotX(0, j+1) - leftCartDotX(0, j+1)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(1) * (rightCartDotX(1, j) - leftCartDotX(1, j) + rightCartDotX(1, j+1) - leftCartDotX(1, j+1)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(2) * (rightCartDotX(2, j) - leftCartDotX(2, j) + rightCartDotX(2, j+1) - leftCartDotX(2, j+1)) * posWeightRel;
//                    grad[i] += 2 * rightEulerErr(0) * (rightCartDotX(3, j)- leftCartDotX(3, j) + rightCartDotX(3, j+1)- leftCartDotX(3, j+1))* rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(1) * (rightCartDotX(4, j) + rightCartDotX(4, j+1)) * rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(2) * (rightCartDotX(5, j) + rightCartDotX(5, j+1)) * rightRotWeightRel;
//                }
//            }// end grad
//        }//end 7 time interval loop
//        for(int j=0; j<dimension; j++){
//            p[0][j] = p[14][j];
//            v[0][j] = v[14][j];
//            a[0][j] = a[14][j];
//        }
//    }//end segment loop
//}




//***************************正常七段
//void a(){
//    for(int i=0; i<segment_num; i++){
//        for(int k=0; k<7; k++){
//            //*****************************compute all joint angles after k time intervals**********************
//            for(int j=0; j<dimension; j++){
//                jerkIndex = segment_num*7 + j*(segment_num * 7) + i*7 + k;
//                timeIndex = i*7 + k;
//                a[k+1][j] = a[k][j] + x[jerkIndex] *x[timeIndex];     //每个维度有7个j，每一段有7个公用的t
//                v[k+1][j] = v[k][j] + a[k][j] * x[timeIndex] + 0.5 * x[jerkIndex] * pow(x[timeIndex], 2);
//                p[k+1][j] = p[k][j] + v[k][j] * x[timeIndex] + 0.5 * a[k][j] *pow(x[timeIndex], 2) + x[jerkIndex] * pow(x[timeIndex], 3) / 6;
//            }
//            //***************************compute cartesian position and rotation of both arms**********************
//            for(int j=0; j<dimension/2; j++) {
//                leftAngleArrKDL(j) = p[k + 1][j];
//                rightAngleArrKDL(j) = p[k + 1][j + 7];
//            }
//
//            leftFKSolver.JntToCart(leftAngleArrKDL, leftFrame);
//            rightFKSolver.JntToCart(rightAngleArrKDL, rightFrame);
//            //假设当前 left 的 xyz 和 yaw 角是正确的。
//            leftPos = leftFrame.p;
//            rightPos = rightFrame.p;
//            leftRot = leftFrame.M;
//            rightRot = rightFrame.M;
//
//            //******************compute cartesian error, take current left(master) arm's xyz and yaw as reference. cur minus goal**********************
//            Eigen::Vector3d leftEulerVec;
//            leftRot.GetEulerZYX(leftEulerVec(0), leftEulerVec(1), leftEulerVec(2));
//            Eigen::Vector3d leftGoalEulerVec(leftEulerVec(0), 0, 1.57);
//            Eigen::Vector3d leftEulerErr = leftEulerVec - leftGoalEulerVec;
//
//            KDL::Rotation leftGoalRot = KDL::Rotation::EulerZYX(leftEulerVec(0), 0, 1.57);
//            KDL::Vector distance(0, 0, 0.2);
//            KDL::Vector rightGoalPos = leftGoalRot * distance + leftPos;
//            KDL::Vector rightPosErr = rightPos - rightGoalPos;
//
//            Eigen::Vector3d rightEulerVec;
//            rightRot.GetEulerZYX(rightEulerVec(0), rightEulerVec(1), rightEulerVec(2));
//            Eigen::Vector3d rightGoalEulerVec(leftEulerVec(0), 0, -1.57);
//            Eigen::Vector3d rightEulerErr = rightEulerVec - rightGoalEulerVec;
//
//            cost_pos += pow(rightPosErr.Norm(), 2);
//            left_cost_rot += pow(leftEulerErr.norm(), 2);
//            right_cost_rot += pow(rightEulerErr.norm(),2);
//
//            //根据误差的数量级来调整权重
//            posWeightRel = softConstraintRelWeight / cost_pos;
//            leftRotWeightRel = softConstraintRelWeight / left_cost_rot;
//            rightRotWeightRel = softConstraintRelWeight / right_cost_rot;
//
//
//            //因为要用到中间得到的 goal rot 矩阵
//            if(grad){
//                //************compute joint angles dot all variables, get a dimension * n matrix********************
//                Eigen::MatrixXd jointDotX = Eigen::MatrixXd::Zero(dimension, n); //这个时刻的每个关节角度对优化变量的导数
//                prevTimeIntervalNum = i*7 + k+1; //外层的k只用来计算经历了多少个 timeInterval
//                for(int j=0; j<dimension; j++){
//                    jerkStartIndex = segment_num*7 + j*segment_num*7;
//                    for(int k=0; k<prevTimeIntervalNum; k++){
//                        //************************************对时间求导*********************************
//                        //初始速度项
//                        jointDotX(j, k) += (*pathPoints)(3, j);
//                        //初始加速度项
//                        tmp1 = 0;
//                        for(int l=0; l<prevTimeIntervalNum; l++){
//                            tmp1 += x[l];
//                        }
//                        jointDotX(j, k) += (*pathPoints)(4, j)*tmp1;
//
//                        //正常项
//                        jointDotX(j, k) += x[jerkStartIndex + k] * pow(x[k], 2) / 2;
//                        //第一个复合项
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+= x[l];
//                            }
//                            jointDotX(j, k) += x[jerkStartIndex+k] * pow(tmp1, 2) / 2;
//                        }
//                        if(k-1 >= 0){
//                            for(int l=0; l<k; l++){
//                                tmp1 = 0;
//                                for(int m=l+1; m<prevTimeIntervalNum; m++){
//                                    tmp1 += x[m];
//                                }
//                                jointDotX(j, k) += tmp1 * x[jerkStartIndex+l] * x[l];
//                            }
//                        }
//                        //第二个复合项
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+=x[l];
//                            }
//                            jointDotX(j, k) += x[jerkStartIndex+k]*x[k]*tmp1;
//                        }
//                        if(k-1 >=0){
//                            for(int m=0; m<k; m++){
//                                jointDotX(j, k) += x[jerkStartIndex+m]*pow(x[m], 2) / 2;
//                            }
//                        }
//
//                        //************************************对jerk求导*********************************
//                        jointDotX(j, jerkStartIndex + k) += pow(x[k], 3)/ 6;
//                        if(k<prevTimeIntervalNum-1){
//                            tmp1 = 0;
//                            for(int l=k+1; l<prevTimeIntervalNum; l++){
//                                tmp1+= x[l];
//                            }
//                            jointDotX(j, jerkStartIndex + k) += pow(x[k], 2) * tmp1 / 2 ;
//                            jointDotX(j, jerkStartIndex + k) += x[k] * pow(tmp1, 2) / 2;
//                        }
//                    }
//                }
//                //**************************compute cartesian error dot joint angles*******************************
//                leftJacSolver.JntToJac(leftAngleArrKDL, leftJacobian);
//                rightJacSolver.JntToJac(rightAngleArrKDL, rightJacobian);
//                Eigen::MatrixXd leftJacMatrix = leftJacobian.data;
//                Eigen::MatrixXd rightJacMatrix = rightJacobian.data;
//
//                Eigen::Matrix3d leftToEulerVel;
//                leftToEulerVel(0, 0) = cos(leftEulerVec(0)) * tan(leftEulerVec(1));
//                leftToEulerVel(0, 1) = sin(leftEulerVec(0)) * tan(leftEulerVec(1));
//                leftToEulerVel(0, 2) = 1;
//                leftToEulerVel(1, 0) = -sin(leftEulerVec(0));
//                leftToEulerVel(1, 1) = cos(leftEulerVec(0));
//                leftToEulerVel(1, 2) = 0;
//                leftToEulerVel(2, 0) = cos(leftEulerVec(0)) / cos(leftEulerVec(1));
//                leftToEulerVel(2, 1) = sin(leftEulerVec(0)) / cos(leftEulerVec(1));
//                leftToEulerVel(2, 2) = 0;
//
//                Eigen::Matrix3d rightToEulerVel;
//                rightToEulerVel(0, 0) = cos(rightEulerVec(0)) * tan(rightEulerVec(1));
//                rightToEulerVel(0, 1) = sin(rightEulerVec(0)) * tan(rightEulerVec(1));
//                rightToEulerVel(0, 2) = 1;
//                rightToEulerVel(1, 0) = -sin(rightEulerVec(0));
//                rightToEulerVel(1, 1) = cos(rightEulerVec(0));
//                rightToEulerVel(1, 2) = 0;
//                rightToEulerVel(2, 0) = cos(rightEulerVec(0)) / cos(rightEulerVec(1));
//                rightToEulerVel(2, 1) = sin(rightEulerVec(0)) / cos(rightEulerVec(1));
//                rightToEulerVel(2, 2) = 0;
//
//                leftJacMatrix.block(3, 0, 3, 7) = leftToEulerVel * leftJacMatrix.block(3, 0, 3, 7);
//                rightJacMatrix.block(3, 0, 3, 7) = rightToEulerVel * rightJacMatrix.block(3, 0, 3, 7);
//
//                //**************************compute cartesian error dot all variables *******************************
//                Eigen::MatrixXd leftCartDotX = leftJacMatrix * jointDotX.block(0, 0, 7, n);
//                Eigen::MatrixXd rightCartDotX = rightJacMatrix * jointDotX.block(7, 0, 7, n);
//                for(int i=0; i<n; i++){
//                    grad[i] += 2 * leftEulerErr(0) * leftCartDotX(3, i) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(1) * leftCartDotX(4, i) * leftRotWeightRel;
//                    grad[i] += 2 * leftEulerErr(2) * leftCartDotX(5, i) * leftRotWeightRel;
//                    grad[i] += 2 * rightPosErr(0) * (rightCartDotX(0, i) - leftCartDotX(0, i)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(1) * (rightCartDotX(1, i) - leftCartDotX(1, i)) * posWeightRel;
//                    grad[i] += 2 * rightPosErr(2) * (rightCartDotX(2, i) - leftCartDotX(2, i)) * posWeightRel;
//                    grad[i] += 2 * rightEulerErr(0) * (rightCartDotX(3, i)- leftCartDotX(3, i))* rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(1) * rightCartDotX(4, i) * rightRotWeightRel;
//                    grad[i] += 2 * rightEulerErr(2) * rightCartDotX(5, i) * rightRotWeightRel;
//                }
//            }// end grad
//        }//end 7 time interval loop
//        for(int j=0; j<dimension; j++){
//            p[0][j] = p[7][j];
//            v[0][j] = v[7][j];
//            a[0][j] = a[7][j];
//        }
//    }//e
//}