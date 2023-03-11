//
// Created by lijiashushu on 20-4-9.
//

#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nlopt.h>

#define INF 1e10

int segment_num = 5;
int numCoeff = 6;
int dimension = 14;
int timeOrder = 6;
//Eigen::MatrixXd pathPoints = Eigen::MatrixXd::Zero(segment_num, dimension);
//目标函数 各段的时间求和；
double objectiveFunc(unsigned n, const double *x, double *grad, void *data)
{
    if (grad) {
        int i=0;
        for(; i<segment_num; i++){
            grad[i] = 1;
        }
    }
    double cost = 0;
    for(int i=0; i<segment_num; i++){
        cost+=x[i];
    }

    return cost;
}

//先假设不设置的地方都默认为0
//waypoint的位置约束，以及连续性约束。
void equalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data){
    Eigen::MatrixXd *pathPointPtr = (Eigen::MatrixXd *)f_data;
    int coffeStartIndex = segment_num;
    int curIndex = 0;
    if(grad){
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

    Eigen::MatrixXd pathPoints;
    Eigen::MatrixXd deravativeLimit;

    int varibleNum = segment_num + segment_num * numCoeff * dimension;
    int equalityNum = (2 * segment_num + 2 * (segment_num - 1) + 4 ) * dimension;
    int inequalityNum = 8 * segment_num * dimension;

    double lb[varibleNum] = {-INF};
    double ub[varibleNum] = { INF};
    for(int i=0; i<segment_num; i++){
        lb[i] = 0;
    }

    double f_min; //目标函数的值
    double eqTol[equalityNum] = {1e-4};
    double ineqTol[inequalityNum] = {1e-4};


    nlopt_opt opter = nlopt_create(NLOPT_LD_SLSQP/*NLOPT_LD_MMA*/, varibleNum);
    nlopt_set_lower_bounds(opter, lb);//设置优化变量的下限

    nlopt_set_min_objective(opter, objectiveFunc, NULL); //设置目标函数

    // 等式约束
    nlopt_add_equality_mconstraint(opter, equalityNum,  equalityConstraintFunc, &pathPoints, eqTol);
    // 不等式约束；
    nlopt_add_inequality_mconstraint(opter, inequalityNum, inequalityConstraintFunc, &deravativeLimit, ineqTol);

    nlopt_destroy(opter);
    return  0;
}