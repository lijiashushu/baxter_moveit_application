//
// Created by lijiashushu on 20-4-8.
// 每个自由度在每一段的运行时间都相同，一共有 segment_num（时间） + segment_num*numPolyCoeff * dimension(每个自由度在每一段的多项式) 个优化变量。
// 以五次多项式的系数，以及各段的时间t作为优化变量，只在多项式的边界点检查动力学约束。以OOQP的 minimum jerk 作为初始值，可以得到结果，但是得到的结果在多项式的中间加速度往往会超出限制。
// 没有很好的办法来确保其一定不会超出限制。
//
#include "dual_rrt_star/trajectory_generator.h"
#include <nlopt.h>

#define INF 1e10

int segment_num;
int numCoeff = 6;
int dimension = 14;
int timeOrder = 6;
int iterationCount = 0;
//Eigen::MatrixXd pathPoints = Eigen::MatrixXd::Zero(segment_num, dimension);
//目标函数 各段的时间求和；
double objectiveFunc(unsigned n, const double *x, double *grad, void *data)
{


    if (grad) {
        for(int i=0; i<n; i++){
            grad[i] = 0;
        }
        for(int i=0; i<segment_num; i++){
            grad[i] = 1;
        }
    }
    double cost = 0;
    for(int i=0; i<segment_num; i++){
        cost+=x[i];
    }

    std::cout<<"itre "<<iterationCount++<<std::endl;
    std::cout<<"cost "<<cost<<"\n"<<std::endl;
    return cost;
}
//先假设不设置的地方都默认为0
//waypoint的位置约束，以及连续性约束。
void equalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    Eigen::MatrixXd *pathPointPtr = (Eigen::MatrixXd *)f_data;
    int coffeStartIndex = segment_num;
    int curIndex = 0;
    if(grad){
        for(int i=0; i<m*n; i++){
            grad[i] = 0;
        }
        curIndex = 0;
        //每一段 0 时刻的位置约束。
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                grad[curIndex*n + coffeStartIndex + i*numCoeff*dimension + j*numCoeff] = 1;
                curIndex++;
            }
        }
        //每一段 t 时刻的位置约束
        for(int i=0; i<segment_num; i++) {
            for (int j = 0; j < dimension; j++) {
                //对系数求导
                for (int k = 0; k < timeOrder; k++) {
                    grad[curIndex * n + coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] = pow(x[i], k);
                }
                //对时间求导
                for (int k = 1; k < timeOrder; k++) {
                    grad[curIndex * n + i] += x[coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] * pow(x[i], k - 1) * k;
                }
                curIndex++;
            }
        }

        //速度连续约束
        for(int i=0; i<segment_num-1; i++){
            for(int j=0; j<dimension; j++){
                //对系数求导
                for(int k = 1; k<timeOrder; k++){
                    grad[curIndex*n+ coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] = pow(x[i], k-1) * k;
                }
                grad[curIndex*n+ coffeStartIndex + (i+1)*numCoeff*dimension + j*numCoeff + 1] = -1;
                //对时间求导
                for(int k=2; k<timeOrder; k++){
                    grad[curIndex*n + i] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-2) * k * (k-1);
                }
                curIndex++;
            }
        }

        //加速度连续约束
        for(int i=0; i<segment_num-1; i++){
            for(int j=0; j<dimension; j++){
                //对系数求导
                for(int k = 2; k<timeOrder; k++){
                    grad[curIndex*n+ coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] = pow(x[i], k-2) * k * (k-1);
                }
                grad[curIndex*n+ coffeStartIndex + (i+1)*numCoeff*dimension + j*numCoeff + 2] = -2;
                //对时间求导
                for(int k=3; k<timeOrder; k++){
                    grad[curIndex*n + i] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-3) * k * (k-1) * (k-2);
                }
                curIndex++;
            }
        }

        //起始点速度、加速度约束
        for(int i=0; i<dimension; i++){
            grad[curIndex*n + coffeStartIndex + i*numCoeff + 1] = 1;
            curIndex++;
        }
        for(int i=0; i<dimension; i++){
            grad[curIndex*n + coffeStartIndex + i*numCoeff + 2] = 1;
            curIndex++;
        }
        //终点速度、加速度约束
        for(int i=0; i<dimension; i++){
            //对系数求导
            for(int j=1; j<timeOrder; j++){
                grad[curIndex*n + coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] = pow(x[segment_num-1], j-1) * j;
            }
            //对时间求导
            for(int j=2; j<timeOrder; j++){
                grad[curIndex*n + segment_num-1] += x[coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] * pow(x[segment_num-1], j-2) * j * (j-1);
            }
            curIndex++;
        }
        for(int i=0; i<dimension; i++){
            //对系数求导
            for(int j=2; j<timeOrder; j++){
                grad[curIndex*n + coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] = pow(x[segment_num-1], j-2) * j * (j-1);
            }
            //对时间求导
            for(int j=3; j<timeOrder; j++){
                grad[curIndex*n + segment_num-1] += x[coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] * pow(x[segment_num-1], j-3) * j * (j-1) * (j-2);
            }
            curIndex++;
        }

    }//end grad

    for(int i=0; i<m; i++){
        result[i] = 0;
    }
    curIndex = 0;
    //每一段 0 时刻的位置约束。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            result[curIndex] = x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff] - (*pathPointPtr)(i, j);
            curIndex++;
        }
    }
    //每一段 t 时刻的位置约束
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k = 0; k<timeOrder; k++){
                result[curIndex] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k);
            }
            result[curIndex]-= (*pathPointPtr)(i+1, j);
            curIndex++;
        }
    }
    //速度连续约束
    for(int i=0; i<segment_num-1; i++){
        for(int j=0; j<dimension; j++){
            for(int k = 1; k<timeOrder; k++){
                result[curIndex] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-1) * k;
            }
            result[curIndex] -= x[coffeStartIndex + (i+1)*numCoeff*dimension + j*numCoeff + 1];
            curIndex++;
        }
    }

    //加速度连续约束
    for(int i=0; i<segment_num-1; i++){
        for(int j=0; j<dimension; j++){
            for(int k = 2; k<timeOrder; k++){
                result[curIndex] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-2) * k * (k-1);
            }
            result[curIndex] -= x[coffeStartIndex + (i+1)*numCoeff*dimension + j*numCoeff + 2] * 2;
            curIndex++;
        }
    }

    //起始点速度、加速度约束
    for(int i=0; i<dimension; i++){
        result[curIndex] = x[coffeStartIndex + i*numCoeff + 1];
        curIndex++;
    }
    for(int i=0; i<dimension; i++){
        result[curIndex] = x[coffeStartIndex + i*numCoeff + 2];
        curIndex++;
    }
    //终点速度、加速度约束
    for(int i=0; i<dimension; i++){
        for(int j=1; j<timeOrder; j++){
            result[curIndex] += x[coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] * pow(x[segment_num-1], j-1) * j;
        }
        curIndex++;
    }
    for(int i=0; i<dimension; i++){
        for(int j=2; j<timeOrder; j++){
            result[curIndex] += x[coffeStartIndex + (segment_num-1)*numCoeff*dimension + i*numCoeff + j] * pow(x[segment_num-1], j-2) * j * (j-1);
        }
        curIndex++;
    }
}

//速度加速度限制，先只在0和t时刻进行限制。
void inequalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    Eigen::MatrixXd *derivativeLimit = (Eigen::MatrixXd *)f_data; //第一行速度限制，第二行加速度限制
    int coffeStartIndex = segment_num;
    int curIndex = 0;
    if(grad){
        for(int i=0; i<m*n; i++){
            grad[i] = 0;
        }
        curIndex = 0;
        //************************时刻点的速度限制*************************
        //每一段 0 时刻的速度上限。
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                grad[curIndex*n + coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 1] = 1;
                curIndex++;
            }
        }
        //每一段 0 时刻的速度下限。
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                grad[curIndex*n + coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 1] = -1;
                curIndex++;
            }
        }
        //每一段 t 时刻的速度上限
        for(int i=0; i<segment_num; i++) {
            for (int j = 0; j < dimension; j++) {
                //对系数求导
                for (int k = 1; k < timeOrder; k++) {
                    grad[curIndex * n + coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] = pow(x[i], k-1) * k;
                }
                //对时间求导
                for (int k = 2; k < timeOrder; k++) {
                    grad[curIndex * n + i] += x[coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] * pow(x[i], k - 2) * k * (k-1);
                }
                curIndex++;
            }
        }
        //每一段 t 时刻的速度下限
        for(int i=0; i<segment_num; i++) {
            for (int j = 0; j < dimension; j++) {
                //对系数求导
                for (int k = 1; k < timeOrder; k++) {
                    grad[curIndex * n + coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] =  - pow(x[i], k-1) * k;
                }
                //对时间求导
                for (int k = 2; k < timeOrder; k++) {
                    grad[curIndex * n + i] += -x[coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] * pow(x[i], k - 2) * k * (k-1);
                }
                curIndex++;
            }
        }
        //************************时刻点的加速度限制*************************
        //每一段 0 时刻的加速度上限。
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                grad[curIndex*n + coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 2] = 2;
                curIndex++;
            }
        }
        //每一段 0 时刻的加速度下限。
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                grad[curIndex*n + coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 2] = -2;
                curIndex++;
            }
        }
        //每一段 t 时刻的加速度上限
        for(int i=0; i<segment_num; i++) {
            for (int j = 0; j < dimension; j++) {
                //对系数求导
                for (int k = 2; k < timeOrder; k++) {
                    grad[curIndex * n + coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] = pow(x[i], k-2) * k * (k-1);
                }
                //对时间求导
                for (int k = 3; k < timeOrder; k++) {
                    grad[curIndex * n + i] += x[coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] * pow(x[i], k - 3) * k * (k-1) * (k-2);
                }
                curIndex++;
            }
        }
        //每一段 t 时刻的加速度下限
        for(int i=0; i<segment_num; i++) {
            for (int j = 0; j < dimension; j++) {
                //对系数求导
                for (int k = 2; k < timeOrder; k++) {
                    grad[curIndex * n + coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] = -pow(x[i], k-2) * k * (k-1);
                }
                //对时间求导
                for (int k = 3; k < timeOrder; k++) {
                    grad[curIndex * n + i] += -x[coffeStartIndex + i * numCoeff * dimension + j * numCoeff + k] * pow(x[i], k - 3) * k * (k-1) * (k-2);
                }
                curIndex++;
            }
        }

    }//end grad

    for(int i=0; i<m; i++){
        result[i] = 0;
    }
    curIndex = 0;
    //************************时刻点的速度限制*************************
    //每一段 0 时刻点的速度上限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            result[curIndex++] = x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 1] - (*derivativeLimit)(0, j);
        }
    }
    //每一段 0 时刻点的速度下限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            result[curIndex++] = -x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 1] - (*derivativeLimit)(0, j);
        }
    }
    //每一段 t 时刻点的速度上限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k=1; k<timeOrder; k++){
                result[curIndex] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-1) * k;
            }
            result[curIndex++] += -(*derivativeLimit)(0, j);
        }
    }
    //每一段 t 时刻点的速度下限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k=1; k<timeOrder; k++){
                result[curIndex] += -x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-1) * k;
            }
            result[curIndex++] += -(*derivativeLimit)(0, j);
        }
    }
    //************************时刻点的加速度限制*************************
    //每一段 0 时刻点的加速度上限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            result[curIndex++] = x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 2] - (*derivativeLimit)(1, j);
        }
    }
    //每一段 0 时刻点的速度下限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            result[curIndex++] = -x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + 2] - (*derivativeLimit)(1, j);
        }
    }
    //每一段 t 时刻点的加速度上限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k=2; k<timeOrder; k++){
                result[curIndex] += x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-2) * k * (k-1);
            }
            result[curIndex++] += -(*derivativeLimit)(1, j);
        }
    }
    //每一段 t 时刻点的加速度上限。
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k=2; k<timeOrder; k++){
                result[curIndex] += -x[coffeStartIndex + i*numCoeff*dimension + j*numCoeff + k] * pow(x[i], k-2) * k * (k-1);
            }
            result[curIndex++] += -(*derivativeLimit)(1, j);
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "one_plan_node");
    ros::NodeHandle nh("~");

    KDL::Tree kdlTree;
    kdl_parser::treeFromString("/home/lijiashushu/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", kdlTree);
    KDL::Chain leftChain;
    kdlTree.getChain("base", "left_gripper", leftChain);
    KDL::ChainJntToJacSolver leftJacSolver(leftChain);
    KDL::JntArray angles(7);

    TrajectoryGenerator traj_generator;

    MatrixXd startPoint(1,14);
    startPoint<<-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081, -0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128;
    MatrixXd endPoint(1,14);
    endPoint<<0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954,-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414;

//    endPoint<<-0.396905522331,-0.326393998119, -1.0608541408,1.27634968272,2.35823683771,-0.911276650603,0.45783256801;
    segment_num = 4;
    //get path points
    double norm = (endPoint - startPoint).norm();
    MatrixXd dir = (endPoint - startPoint).normalized();
    double step = norm / segment_num;
    MatrixXd pathPoints =  MatrixXd::Zero(segment_num+1, 14);
    pathPoints.row(0) = startPoint;
    pathPoints.row(segment_num) = endPoint;
    for(int i=1; i<segment_num; i++){
        pathPoints.row(i) = step * i * dir + startPoint;
    }
    //get segment times
    VectorXd times = VectorXd::Ones(segment_num) * 1.5;
    //get segment jacobians
    vector<MatrixXd> pathJacobians;
    for(int i=0; i<segment_num; i++){
        angles.data = pathPoints.row(i).head(7);
        KDL::Jacobian jac(7);
        leftJacSolver.JntToJac(angles, jac);
        pathJacobians.push_back(jac.data);
    }

    std::cout<<pathPoints<<std::endl;
    Eigen::MatrixXd ooqpSolu = traj_generator.generatePolyQP(pathPoints, times, pathJacobians);
//    traj_generator.outputTrajToFile("/home/lijiashushu", pathPoints);

    traj_generator.outputTrajToFile("/home/lijiashushu", ooqpSolu, times);
    //*********************nlopt求解时间最优轨迹*********************************
    Eigen::MatrixXd deravativeLimit(2, 14);
    deravativeLimit<<2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
                     2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4;

    int varibleNum = segment_num + segment_num * numCoeff * dimension;
    int equalityNum = (2 * segment_num + 2 * (segment_num - 1) + 4 ) * dimension;
    int inequalityNum = 8 * segment_num * dimension;

    double x[varibleNum]; //initial val
    for(int i=0; i<segment_num; i++){
        x[i] = times[i];
    }
    for(int i=0; i<segment_num; i++){
        for(int j=0; j<dimension; j++){
            for(int k=0; k<numCoeff; k++){
                x[segment_num + i*dimension*numCoeff + j*numCoeff + k] = ooqpSolu(i, j*numCoeff+k);
            }
        }
    }

    double lb[varibleNum];
    for(int i=0; i<segment_num; i++){
        lb[i] = 0;
    }
    for(int i=segment_num; i<varibleNum; i++){
        lb[i] = -INF;
    }

    double eqTol[equalityNum];
    for(int i=0; i<equalityNum; i++){
        eqTol[i] = 1e-4;
    }
    double ineqTol[inequalityNum];
    for(int i=0; i<inequalityNum; i++){
        ineqTol[i] = 1e-4;
    }

    nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP/*NLOPT_LD_MMA*/, varibleNum);
    nlopt_set_lower_bounds(opter, lb);//设置优化变量的下限

    nlopt_set_min_objective(opter, objectiveFunc, NULL); //设置目标函数

    // 等式约束
    nlopt_add_equality_mconstraint(opter, equalityNum,  equalityConstraintFunc, &pathPoints, eqTol);
    // 不等式约束；
    nlopt_add_inequality_mconstraint(opter, inequalityNum, inequalityConstraintFunc, &deravativeLimit, ineqTol);

    nlopt_set_xtol_rel(opter, 1e-4);


    double f_min; //目标函数的值
    if (nlopt_optimize(opter, x, &f_min) < 0) {
        printf("nlopt failed!\n");
    }
    else {
        cout<<"found minimum "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<endl;
        cout<<"cost: "<<f_min<<endl;

        Eigen::VectorXd newTimes(segment_num);
        for(int i=0; i<segment_num; i++){
            newTimes(i) = x[i];
        }
        Eigen::MatrixXd newPolyCoeff(segment_num, numCoeff * dimension);
        for(int i=0; i<segment_num; i++){
            for(int j=0; j<dimension; j++){
                for(int k=0; k<numCoeff; k++){
                    newPolyCoeff(i, j*numCoeff+k) = x[segment_num + i*dimension*numCoeff + j*numCoeff + k];
                }
            }
        }
        traj_generator.viewTrajectory(newPolyCoeff, newTimes, nh);
        traj_generator.outputTrajToFile("/home/lijiashushu", newPolyCoeff, newTimes);
    }

    nlopt_destroy(opter);


    return 0;
}