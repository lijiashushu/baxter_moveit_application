//
// Created by lijiashushu on 20-4-7.
//

#include "dual_rrt_star/trajectory_planner/minimum_jerk_planner/minimum_jerk_planner.h"


int MinimumJerkPlanner::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

//获得某一个时间点的坐标,小系数为低阶项
void MinimumJerkPlanner::getPosPoly( MatrixXd polyCoeff, int k, double t, VectorXd& pos, VectorXd& vel, VectorXd& acc)
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

MatrixXd MinimumJerkPlanner::generatePolyQP(MatrixXd pathPoints, VectorXd times){
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

void MinimumJerkPlanner::outputTrajToFile(string path, const MatrixXd& polyCoeff, const VectorXd &times){
    ofstream outFile1, outFile2, outFile3;
    outFile1.open(path + "/minimum_jerk_pos.txt", ios::trunc);
    outFile2.open(path + "/minimum_jerk_vel.txt", ios::trunc);
    outFile3.open(path + "/minimum_jerk_acc.txt", ios::trunc);
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

void MinimumJerkPlanner::viewTrajectory(const MatrixXd& polyCoeff, const VectorXd &times, ros::NodeHandle &nh){
    KDL::Tree kdlTree;
    kdl_parser::treeFromParam("robot_description", kdlTree);
    KDL::Chain leftChain;
    kdlTree.getChain("base", "left_gripper", leftChain);
    KDL::ChainJntToJacSolver leftJacSolver(leftChain);
    KDL::ChainFkSolverPos_recursive leftFKSolver(leftChain);
    KDL::JntArray kdlAngles(7);
    KDL::Frame frame;

    ros::WallDuration sleep_t(0.5);
    ros::Publisher _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    while (_wp_traj_vis_pub.getNumSubscribers() < 1)
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
    VectorXd angle(14);
    VectorXd velocity(14);
    VectorXd accelaration(14);
    Vector3d pos;
    geometry_msgs::Point pt;

    int count = 0;
    for(int i = 0; i < times.size(); i++ )
    {
        for (double t = 0.0; t < times(i); t += 0.01, count += 1)
        {

            getPosPoly(polyCoeff, i, t,angle, velocity, accelaration);
            kdlAngles.data = angle.head(7);
            leftFKSolver.JntToCart(kdlAngles, frame);
            pt.x = frame.p(0);
            pt.y = frame.p(1);
            pt.z = frame.p(2);
            _traj_vis.points.push_back(pt);
        }
    }
    _wp_traj_vis_pub.publish(_traj_vis);
    ros::Duration(1).sleep();
    cout<<"after publish"<<endl;
}

