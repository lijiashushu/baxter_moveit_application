//
// Created by lijiashushu on 20-4-7.
//

#include <Eigen/Dense>
#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>

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



using namespace std;
using namespace Eigen;

int Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

//获得某一个时间点的坐标,小系数为低阶项
void getPosPoly( MatrixXd polyCoeff, int k, double t, VectorXd& pos, VectorXd& vel, VectorXd& acc)
{
    for ( int dim = 0; dim < 14; dim++ )
    {
        VectorXd posCoeff = (polyCoeff.row(k)).segment( dim * 6, 6 ); //从 dim * 6 开始的 6 个数
        VectorXd velCoeff = VectorXd::Zero(6);
        VectorXd accCoeff = VectorXd::Zero(6);
        for(int i=1; i<6; i++){
            velCoeff(i) =  posCoeff(i) * i;
        }
        for(int i=2; i<6; i++){
            accCoeff(i) =  posCoeff(i) * i * (i-1);
        }

        VectorXd posTimePow  = VectorXd::Ones(6);
        VectorXd velTimePow  = VectorXd::Ones(6);
        VectorXd accTimePow  = VectorXd::Ones(6);
        for(int j = 1; j < 6; j++){
            posTimePow(j) = pow(t, j);
        }
        for(int j = 2; j < 6; j++){
            velTimePow(j) = pow(t, j-1);
        }
        for(int j = 3; j < 6; j++){
            accTimePow(j) = pow(t, j-2);
        }

        pos(dim) = posCoeff.dot(posTimePow);
        vel(dim) = velCoeff.dot(velTimePow);
        acc(dim) = accCoeff.dot(accTimePow);
    }
}

void getTraj( MatrixXd polyCoeff, VectorXd times)
{
    ofstream outFile1, outFile2, outFile3;
    outFile1.open("/home/lijiashushu/minimum_jerk_pos.txt", ios::trunc);
    outFile2.open("/home/lijiashushu/minimum_jerk_vel.txt", ios::trunc);
    outFile3.open("/home/lijiashushu/minimum_jerk_acc.txt", ios::trunc);
    VectorXd angle(14);
    VectorXd velocity(14);
    VectorXd acceleration(14);
    for(int i = 0; i < times.size(); i++ )
    {
        for (double t = 0.0; t <= times(i); t += 0.01)
        {
            getPosPoly(polyCoeff, i, t, angle, velocity, acceleration);
            outFile1<<angle.transpose()<<endl;
            outFile2<<velocity.transpose()<<endl;
            outFile3<<acceleration.transpose()<<endl;
            //最后少了0.01s的数据，所以最后加速度、速度的值看起来和0差了一点。
        }
    }

    getPosPoly(polyCoeff, times.size()-1, times[times.size()-1], angle, velocity, acceleration);
    cout<<angle.transpose()<<endl;
    cout<<velocity.transpose()<<endl;
    cout<<acceleration.transpose()<<endl;

    outFile1.close();
    outFile2.close();
    outFile3.close();
}



MatrixXd generatePolyQP(MatrixXd pathPoints, VectorXd times, const vector<MatrixXd>& pathJacobians){
    int segment_num = pathPoints.rows() - 1;
    int dim = pathPoints.cols();
    int d_order = 3;
    int pOrder = 2 * d_order - 1;
    int numPolyCoff = pOrder + 1;

    int rowIndex = 0;
    int curIndex = 0;

    MatrixXd polyCoff = MatrixXd::Zero(segment_num, dim * numPolyCoff);

    //*****************优化变量*******************
    int nx = numPolyCoff * segment_num * dim;
    double  xupp[nx] = {0};
    char   ixupp[nx] = {0};
    double  xlow[nx] = {0};
    char   ixlow[nx]= {0};
    double c[nx] = {0}; //目标函数线性项系数

    //*****************等式约束********************

    //边界点各阶条件，中间的点的位置条件，中间的导数连续性约束
    int my = (2 * d_order + (segment_num - 1) * (d_order + 1)) * dim; //my 是等式约束的个数
    double b[my];
    curIndex = 0;
    for(int i=0; i < dim; i++){
        b[curIndex++] = pathPoints(0, i);
        b[curIndex++] = 0;
        b[curIndex++] = 0;
        b[curIndex++] = pathPoints(segment_num, i);
        b[curIndex++] = 0;
        b[curIndex++] = 0;

        for( int j = 0; j< segment_num-1; j++) {
            b[curIndex++] = pathPoints(j+1, i);
        }
        for(int j = 0; j< d_order; j++){
            for(int k = 0; k< segment_num-1; k++){
                b[curIndex++] = 0;
            }
        }
    }

    int nnzA =dim * (numPolyCoff * (segment_num  -1 + 2 * d_order) +     //两个边界各d_ordr各条件，中间n-1个位置条件，只用到一段多项式
                        d_order * (segment_num - 1) * 2 * numPolyCoff);  //中间的连续性条件，每个用到两段多项式
    std::cout<<"nnzA= "<<nnzA<<std::endl;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];

    rowIndex = 0;
    curIndex = 0;
    for(int i=0; i<dim; i++){
        //起点条件，有3个方程
        for(int j = 0; j < d_order; j++){
            for(int k=0; k< numPolyCoff; k++){
                irowA[curIndex] = rowIndex;
                jcolA[curIndex] = segment_num * numPolyCoff * i + k ;
                if(k == j){
                    dA[curIndex] = Factorial(k);
                }
                else{
                    dA[curIndex] = 0;
                }
                curIndex++;
            }
            rowIndex++;
        }
        //终点，有3个方程
        for(int j = 0; j < d_order; j++){
            for(int k=0; k< numPolyCoff; k++){
                irowA[curIndex] = rowIndex;
                jcolA[curIndex] = segment_num  * numPolyCoff * i + numPolyCoff * (segment_num -1) + k;
                if(k < j){
                    dA[curIndex] = 0;
                }
                else{
                    dA[curIndex] = Factorial(k) * pow(times(segment_num-1), k- j) / Factorial(k- j);
                }
                curIndex++;
            }
            rowIndex++;
        }
        //中间点的位置条件，全部用后一个多项式在 0 时刻的系数来表示
        for(int j = 0; j < segment_num-1; j++){
            for(int k=0; k<numPolyCoff; k++){
                irowA[curIndex] = rowIndex;
                jcolA[curIndex] = segment_num * numPolyCoff * i + (j+1) * numPolyCoff + k;
                if(k==0){
                    dA[curIndex] = 1;
                }
                else{
                    dA[curIndex] = 0;
                }
                curIndex++;
            }
            rowIndex++;
        }
        //中间点各阶连续性条件
        for(int j=0; j <  d_order; j++) {
            for (int k = 0; k < segment_num-1; k++) {
                for (int l = 0; l < numPolyCoff; l++) {
                    irowA[curIndex] = rowIndex;
                    jcolA[curIndex] = segment_num * numPolyCoff * i + k * numPolyCoff + l;
                    if (l < j) {
                        dA[curIndex] = 0;
                    }
                    else {
                        dA[curIndex] = -Factorial(l) * pow(times[k], l - j) / Factorial(l - j);
                    }
                    curIndex++;
                }
                for (int l = 0; l < numPolyCoff; l++) {
                    irowA[curIndex] = rowIndex;
                    jcolA[curIndex] = segment_num * numPolyCoff * i + (k+1) * numPolyCoff + l;

                    if (l == j) {
                        dA[curIndex] = Factorial(l);
                    } else {
                        dA[curIndex] = 0;
                    }
                    curIndex++;
                }
                rowIndex++;
            }
        }
        std::cout<<"??"<<std::endl;
    }
    std::cout<<"??"<<std::endl;
    //*****************不等式约束*********************
    int mz = 0;
    int nnzC = 0;
    int *irowC = 0 ;
    double *dC = 0;
    int *jcolC = 0;
    double  *cupp = 0;
    char   *icupp = 0;
    double  *clow = 0;
    char   *iclow = 0;

    //每一段以开头的关节角度作为雅克比矩阵
//    int mz = (segment_num + segment_num -1) * 2;
//    double  cupp[mz];
//    char   icupp[mz];
//    double  clow[mz];
//    char   iclow[mz];
//
//    for(int i=0; i<mz; i++){
//        cupp[i] = 0.01;
//        icupp[i] = 1;
//        clow[i] = -0.01;
//        iclow[i] = 1;
//    }
//
//    int nnzC = (segment_num + segment_num -1) * 2 * numPolyCoff * dim;
//    int irowC[nnzC];
//    double dC[nnzC];
//    int jcolC[nnzC];
//
//    curIndex = 0;
//    rowIndex = 0;
//
//    for(int jacRow = 3; jacRow<=4; jacRow++) {
//        for (int i = 0; i < dim; i++) {
//            for (int k = 0; k < numPolyCoff; k++) {
//                irowC[curIndex] = rowIndex;
//                jcolC[curIndex] = i * segment_num * numPolyCoff + k;
//                if (k == 0) {
//                    dC[curIndex] = 0;
//                } else {
//                    dC[curIndex] = pathJacobians[0](jacRow, i) * Factorial(k) * pow(times[0] / 2, k - 1) / Factorial(k - 1);
//                }
//                curIndex++;
//            }
//        }
//        rowIndex++;
//    }
//
//    for(int j=1; j<segment_num; j++){
//        for(int jacRow = 3; jacRow<=4; jacRow++) {
//            for(int k=0; k<numPolyCoff; k++) {
//                for (int i = 0; i < dim; i++) {
//                    irowC[curIndex] = rowIndex;
//                    jcolC[curIndex] = i * segment_num * numPolyCoff + j * numPolyCoff + k;
//                    if (k == 1) {
//                        dC[curIndex] = pathJacobians[j](jacRow, i);
//                    } else {
//                        dC[curIndex] = 0;
//                    }
//                    curIndex++;
//                }
//            }
//            rowIndex++;
//            for(int k=0; k<numPolyCoff; k++) {
//                for (int i = 0; i < dim; i++) {
//                    irowC[curIndex] = rowIndex;
//                    jcolC[curIndex] = i* segment_num *numPolyCoff + j * numPolyCoff + k;
//                    if(k==0){
//                        dC[curIndex] = 0;
//                    }
//                    else{
//                        dC[curIndex] = pathJacobians[j](jacRow, i) * Factorial(k) * pow(times[j]/2, k - 1) / Factorial(k-1);
//                    }
//                    curIndex++;
//                }
//            }
//            rowIndex++;
//        }
//    }

    //*****************目标函数********************

    //一定要注意，目标函数的矩阵Q是对称矩阵，并且这里要求只输入Q的下三角元素。
    int nnzQ = (numPolyCoff + 1) * numPolyCoff * segment_num * dim / 2;
    int irowQ[nnzQ];
    double dQ[nnzQ];
    int jcolQ[nnzQ];

    rowIndex = 0;
    curIndex = 0;
    for(int i=0; i<dim; i++){
        for(int j=0; j<segment_num; j++) {
            for (int k = 0; k < numPolyCoff; k++) {
                for (int l = 0; l < k+1; l++) {
                    irowQ[curIndex] = rowIndex;
                    jcolQ[curIndex] = segment_num * numPolyCoff * i + j * numPolyCoff + l;
                    if (k < d_order || l < d_order) {
                        dQ[curIndex] = 0;
                    } else {
                        dQ[curIndex] = k * (k - 1) * (k - 2) * l * (l - 1) * (l - 2) *  pow(times(j), k + l - 5) / (k + l - 5);
                    }
                    curIndex++;
                }
                rowIndex++;
            }
        }
    }




    //nx表示优化变量的个数，my表示等式约束的个数，mz表示不等式约束的个数，nnzAQC，都代表矩阵中不为0的元素的个数
    QpGenSparseMa27 * qp  = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );

    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,
            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars  = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid = (QpGenResiduals *) qp->makeResiduals( prob );
    GondzioSolver  * s     = new GondzioSolver( qp, prob );

    std::cout<<" herehere "<<std::endl;
    // Turn Off/On the print of the solving process
//    s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);

    if( ierr == 0 )
    {
        cout<< "solved!!.\n";
        double d_var[nx];
        vars->x->copyIntoArray(d_var);
        curIndex = 0;
        for(int i=0; i<dim; i++){
            for(int j=0; j<segment_num; j++){
                for(int k=0; k<numPolyCoff; k++){
                    polyCoff(j, i * numPolyCoff + k) = d_var[curIndex];
                    curIndex++;
                }
            }
        }
    }
    else if( ierr == 3)
        cout << "The program is provably infeasible, check the formulation.\n";
    else if (ierr == 4)
        cout << "Ther program is very slow in convergence, may have numerical issue.\n";
    else
        cout << "Don't know what the fuck it is, should not happen.\n";


    return polyCoff;
}


int main(int argc, char** argv){
    KDL::Tree kdlTree;
    kdl_parser::treeFromString("/home/lijiashushu/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf", kdlTree);
    KDL::Chain leftChain;
    kdlTree.getChain("base", "left_gripper", leftChain);
    KDL::ChainJntToJacSolver leftJacSolver(leftChain);
    KDL::JntArray angles(7);

    MatrixXd startPoint(1,14);
    startPoint<<-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081, -0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128;
    MatrixXd endPoint(1,14);
    endPoint<<0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954,-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414;

//    endPoint<<-0.396905522331,-0.326393998119, -1.0608541408,1.27634968272,2.35823683771,-0.911276650603,0.45783256801;
    int segment_num = 4;
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
    VectorXd times = VectorXd::Ones(segment_num) * 2;
    //get segment jacobians
    vector<MatrixXd> pathJacobians;
    for(int i=0; i<segment_num; i++){
        angles.data = pathPoints.row(i).head(7);
        KDL::Jacobian jac(7);
        leftJacSolver.JntToJac(angles, jac);
        pathJacobians.push_back(jac.data);
    }

    std::cout<<pathPoints<<std::endl;
    MatrixXd polyCoeff = generatePolyQP(pathPoints, times, pathJacobians);
    getTraj(polyCoeff, times);

    return 0;
}

