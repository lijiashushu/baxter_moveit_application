//
// Created by lijiashushu on 20-4-14.
//
#include "dual_rrt_star/trajectory_planner/time_dual_constraint_planner/trapezoid_profile.h"

using namespace std;

Eigen::MatrixXd getSecondOrderCoeff(double v0, double a0, double v_goal, double a_max, double j_max){
    int d_a;
    double a_cruise, j_acc, j_dec, dt1, dt2, dt3, dv1, dv2, dv3, v_end;
    if(a0 == 0){
        d_a = (v_goal >= v0) ? 1 : -1;
    }
    else{
        v_end = v0 + 0.5 * a0 * abs(a0) / j_max;
        d_a = (v_goal - v_end)>0 ? 1 : -1;
    }
    a_cruise = d_a * a_max;
    dt1 =  abs(a_cruise-a0) / j_max;
    j_acc = (a_cruise - a0) >= 0 ? j_max : -j_max; // identify sign of the j in first time interval
    dv1 = a0 *dt1 + 0.5 * j_acc * pow(dt1, 2);
    dt3 = abs(a_cruise) / j_max;
    j_dec = (a_cruise >= 0) ? -j_max : j_max;
    dv3 = a_cruise * dt3 + 0.5 * j_dec * pow(dt3, 2);
    dv2 = v_goal - (v0 + dv1 + dv3);
    if(abs(dv2) < 1e-3){
        dt2 = 0;
    }
    else{
        dt2 = dv2 / a_cruise;
    }
    if(dt2 < 0){
        a_cruise = d_a * sqrt(d_a * j_max * (v_goal - v0) + 0.5 * pow(a0, 2));
        dt1 = abs(a_cruise-a0) / j_max;
        dt2 = 0;
        dt3 = abs(a_cruise) / j_max;
    }
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(2, 3);
    output(0, 0) = dt1;
    output(0, 1) = dt2;
    output(0, 2) = dt3;
    output(1, 0) = j_acc;
    output(1, 1) = 0;
    output(1, 2) = j_dec;
    return output;
}

//return the vector of p v a at time t
Eigen::Vector3d getStateFromVelocityCoeff(double p0, double v0, double a0, double t, const Eigen::MatrixXd& velocityCoeff){
    int i=0;
    while(t > velocityCoeff(0, i) && i<2){
        t-= velocityCoeff(0, i);
        i++;
    }
    if(i>=3){
        cout<<"time t exceeds the velocityCoeff total time"<<endl;
        exit(123);
    }
    double p,v,a;
    double last_p = p0;
    double last_v = v0;
    double last_a = a0;
    for(int j=0; j<i; j++){
        a = last_a + velocityCoeff(0, j) * velocityCoeff(1,j);
        v = last_v + last_a * velocityCoeff(0, j) + 0.5 * velocityCoeff(1, j) * pow(velocityCoeff(0, j), 2);
        p = last_p + last_v * velocityCoeff(0, j) + 0.5 * last_a * pow(velocityCoeff(0, j), 2) + velocityCoeff(1, j) * pow(velocityCoeff(0, j), 3) / 6;
        last_a = a;
        last_v = v;
        last_p = p;
    }
    a = last_a + velocityCoeff(1, i) * t;
    v = last_v + last_a * t + 0.5 * velocityCoeff(1, i) * pow(t, 2);
    p = last_p + last_v * t + 0.5 * last_a * pow(t, 2) + velocityCoeff(1, i) * pow(t, 3) / 6;

    Eigen::Vector3d output;
    output(0) = p;
    output(1) = v;
    output(2) = a;
    return output;
}


Eigen::MatrixXd getThirdOrderCoeff(double p0, double v0, double a0, double p_goal, double v_max, double a_max, double j_max){
    int d_v;
    double tmp_t, error;
    double v_cruise, p_cruise, t_cruise;
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(2, 7);
    if(v0 == 0 && a0 == 0){
        d_v = (p_goal >= p0) ? 1 : -1;
    }
    else{
        Eigen::MatrixXd directToZeroCoeff = getSecondOrderCoeff(v0, a0, 0, a_max, j_max);
        tmp_t = directToZeroCoeff(0, 0) + directToZeroCoeff(0, 1) + directToZeroCoeff(0, 2);
        Eigen::Vector3d directToZeroState = getStateFromVelocityCoeff(p0, v0, a0, tmp_t, directToZeroCoeff);
        d_v  = (p_goal >= directToZeroState(0)) ? 1:-1;
    }
    v_cruise = d_v * v_max;
    Eigen::MatrixXd toVmaxCoeff = getSecondOrderCoeff(v0, a0, v_cruise, a_max, j_max);
//    cout<<"toVmaxCoeff "<<toVmaxCoeff<<endl;
    tmp_t = toVmaxCoeff(0, 0) + toVmaxCoeff(0, 1) + toVmaxCoeff(0, 2);
    Eigen::Vector3d toVmaxState = getStateFromVelocityCoeff(p0, v0, a0, tmp_t, toVmaxCoeff);
//    cout<<"toVmaxState "<<toVmaxState.transpose()<<endl;

    Eigen::MatrixXd vmaxToZeroCoeff = getSecondOrderCoeff(v_cruise, 0, 0, a_max, j_max);
//    cout<<"vmaxToZeroCoeff "<<vmaxToZeroCoeff<<endl;
    tmp_t = vmaxToZeroCoeff(0, 0) + vmaxToZeroCoeff(0, 1) + vmaxToZeroCoeff(0, 2);
    Eigen::Vector3d vmaxToZeroState = getStateFromVelocityCoeff(toVmaxState(0), toVmaxState(1), toVmaxState(2),tmp_t, vmaxToZeroCoeff);
//    cout<<"vmaxToZeroState "<<vmaxToZeroState.transpose()<<endl;
    p_cruise = p_goal - vmaxToZeroState(0);
    if(abs(p_cruise) < 1e-4 ){
        t_cruise = 0;
    }
    else{
        t_cruise = p_cruise / v_cruise;
    }
    if(t_cruise < 0){
        double low = 0;
        double high = v_cruise;
        double mid;
        int count = 0;
        for(; count<20; count++){
            mid = low + (high - low) / 2;
            Eigen::MatrixXd toMidCoeff = getSecondOrderCoeff(v0, a0, mid, a_max, j_max);
            tmp_t = toMidCoeff(0, 0) + toMidCoeff(0, 1) + toMidCoeff(0, 2);
            Eigen::Vector3d toMidState = getStateFromVelocityCoeff(p0, v0, a0, tmp_t, toMidCoeff);

            Eigen::MatrixXd midToZeroCoeff = getSecondOrderCoeff(toMidState(1), toMidState(2), 0, a_max, j_max);
            tmp_t = midToZeroCoeff(0, 0) + midToZeroCoeff(0, 1) + midToZeroCoeff(0, 2);
            Eigen::Vector3d midToZeroState = getStateFromVelocityCoeff(toMidState(0), toMidState(1), toMidState(2), tmp_t, midToZeroCoeff);

            error = midToZeroState(0) - p_goal;
            if(abs(error) < 1e-3){
                //eigen block operation
                output.block(0, 0, 2, 3) = toMidCoeff;
                output(0, 3) = 0;
                output(1, 3) = 0;
                output.block(0, 4, 2, 3) = midToZeroCoeff;
                break;
            }
            else{
                if(d_v > 0){
                   if(error <= 0){
                        low = mid;
                   }
                   else{
                        high = mid;
                   }
                }
                else{
                    if(error <= 0){
                        high = mid;
                    }
                    else{
                        low = mid;
                    }
                }
            }
        }
        if(count>=20){
            cout<<"exceed max bisection times!!"<<endl;
            exit(2);
        }

    }
    else{
        output.block(0, 0, 2, 3) = toVmaxCoeff;
        output(0, 3) = t_cruise;
        output(1, 3) = 0;
        output.block(0, 4, 2, 3) = vmaxToZeroCoeff;
    }
    return output;
}


Eigen::Vector3d getStateFromPositionCoeff(double p0, double v0, double a0, double t, const Eigen::MatrixXd positionCoeff){
    int i=0;
    while(t > positionCoeff(0, i)){
        t-= positionCoeff(0, i);
        i++;
    }
    if(i>=7){
        cout<<"time t exceeds the positionCoeff total time"<<endl;
        exit(456);
    }
    double p,v,a;
    double last_p = p0;
    double last_v = v0;
    double last_a = a0;
    for(int j=0; j<i; j++){
        a = last_a + positionCoeff(0, j) * positionCoeff(1,j);
        v = last_v + last_a * positionCoeff(0, j) + 0.5 * positionCoeff(1, j) * pow(positionCoeff(0, j), 2);
        p = last_p + last_v * positionCoeff(0, j) + 0.5 * last_a * pow(positionCoeff(0, j), 2) + positionCoeff(1, j) * pow(positionCoeff(0, j), 3) / 6;
        last_a = a;
        last_v = v;
        last_p = p;
    }
    a = last_a + positionCoeff(1, i) * t;
    v = last_v + last_a * t + 0.5 * positionCoeff(1, i) * pow(t, 2);
    p = last_p + last_v * t + 0.5 * last_a * pow(t, 2) + positionCoeff(1, i) * pow(t, 3) / 6;

    Eigen::Vector3d output;
    output(0) = p;
    output(1) = v;
    output(2) = a;
    return output;
}

//v_goal and a_goal are all equals to zero
Eigen::MatrixXd getThirdOrderMultidofNonSync(Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::VectorXd p_goal, Eigen::MatrixXd jointLimits){
    int dimension = p0.rows();
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(2, 7*dimension);
    for(int i=0; i<dimension; i++){
        output.block(0, i*7, 2, 7) = getThirdOrderCoeff(p0(i), v0(i), a0(i), p_goal(i), jointLimits(0, i), jointLimits(1,i), jointLimits(2,i));
    }
    return output;
}


Eigen::MatrixXd getThirdOrderMultidofZeroSync(Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::VectorXd p_goal, Eigen::MatrixXd jointLimits){
    int dimension = p0.rows();
    double scale_factor = 1.0;
    Eigen::MatrixXd output = Eigen::MatrixXd::Zero(2, 7*dimension);
    for(int i=0; i<dimension; i++){
        output.block(0, i*7, 2, 7) = getThirdOrderCoeff(p0(i), v0(i), a0(i), p_goal(i), jointLimits(0, i), jointLimits(1,i), scale_factor*jointLimits(2,i));
    }
    int max_time_index =0;
    double max_time = output(0, 6);
    for(int i=1; i<dimension; i++){
        if(output(0, i*7+6) > max_time){
            max_time = output(0, i*7+6);
            max_time_index = i;
        }
    }
    for(int i=0; i<dimension; i++){
        if(i != max_time_index){
            double ratio = pow(output(0, i*7+6) / max_time, 3);
            output.block(0, i*7, 2, 7) = getThirdOrderCoeff(p0(i), v0(i), a0(i), p_goal(i), jointLimits(0, i), jointLimits(1,i), scale_factor*ratio*jointLimits(2,i));
        }
    }
    //the first row is each intervals time for all dimensions
    //the seconde row is each intervals jerk for all dimensions
    return output;
}


void outputTrajToFile(string path, Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::MatrixXd trajCoeff){
    ofstream outFile1, outFile2, outFile3;
    outFile1.open(path+"_angle.txt");
    outFile2.open(path+"_velocity.txt");
    outFile3.open(path+"_acceleration.txt");


    int dimension = p0.rows();
    int dim_start_index;
    vector<Eigen::Vector3d> trajRecorder[dimension];
    double p,v,a;
    double last_p, last_v, last_a;

    for(int i=0; i<dimension; i++){
        last_p = p0(i);
        last_v = v0(i);
        last_a = a0(i);
        dim_start_index = i * 7;
        for(int j=0; j<7; j++){
            for(double t=0; t<trajCoeff(0, dim_start_index+j); t+=0.01){
                a = last_a + trajCoeff(1, dim_start_index+j) * t;
                v = last_v + last_a * t + 0.5 * trajCoeff(1, dim_start_index+j) * pow(t, 2);
                p = last_p + last_v * t + 0.5 * last_a * pow(t, 2) + trajCoeff(1, dim_start_index+j) * pow(t, 3) / 6;
                trajRecorder[i].push_back(Eigen::Vector3d(p, v, a));
            }
            a = last_a + trajCoeff(1, dim_start_index+j) * trajCoeff(0, dim_start_index+j);
            v = last_v + last_a * trajCoeff(0, dim_start_index+j) + 0.5 * trajCoeff(1, dim_start_index+j) * pow(trajCoeff(0, dim_start_index+j), 2);
            p = last_p + last_v * trajCoeff(0, dim_start_index+j) + 0.5 * last_a * pow(trajCoeff(0, dim_start_index+j), 2) + trajCoeff(1, dim_start_index+j) * pow(trajCoeff(0, dim_start_index+j), 3) / 6;
            trajRecorder[i].push_back(Eigen::Vector3d(p, v, a));
            last_a = a;
            last_v = v;
            last_p = p;
        }
    }

    int max_size = trajRecorder[0].size();
    int max_size_dim = 0;
    for(int i=1; i<dimension; i++){
        if(trajRecorder[i].size() > max_size){
            max_size = trajRecorder[i].size();
            max_size_dim = i;
        }
    }
    for(int i=0; i<dimension; i++){
        if(i!=max_size_dim){
            Eigen::Vector3d elem = trajRecorder[i][trajRecorder[i].size()-1];
            trajRecorder[i].insert(trajRecorder[i].end(), max_size-trajRecorder[i].size(), elem);
        }
    }

    for(int i=0; i<max_size; i++){
        for(int j=0; j<dimension; j++){
            outFile1<<trajRecorder[j][i](0)<<" ";
            outFile2<<trajRecorder[j][i](1)<<" ";
            outFile3<<trajRecorder[j][i](2)<<" ";
        }
        outFile1<<endl;
        outFile2<<endl;
        outFile3<<endl;
    }
    outFile1.close();
    outFile2.close();
    outFile3.close();
}


//int main(int argc, char** argv){
//    Eigen::VectorXd startPoint(14);
//    startPoint<<-0.0143575,-0.647576,-1.11934,0.879812,2.37683,-1.5691,-0.116081, -0.0385529,-0.34499,1.6744,0.874578,-2.96177,-1.47428,0.297128;
//
//    Eigen::VectorXd endPoint(14);
//    endPoint<<0.103837,-0.367915,-0.866577,1.42099,2.45727,-1.38169,0.585954,-0.260812,0.0160462,1.32291,1.41565,-3.02408,-1.21469,-0.264414;
//
//    Eigen::MatrixXd deravativeLimit(3, 14);
//    deravativeLimit<<2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
//            2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4,
//            2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4;
//
//    Eigen::MatrixXd trajCoeff = getThirdOrderMultidofZeroSync(startPoint, Eigen::VectorXd::Zero(14), Eigen::VectorXd::Zero(14), endPoint, deravativeLimit);
//
//    for(int i=0; i<14; i++){
//        double time = 0;
//        for(int j=0; j<7; j++){
//            cout<<trajCoeff(0, i*7+j)<<" ";
//            time+= trajCoeff(0, i*7+j);
//        }
//        cout<<endl;
//    }
//    outputTrajToFile("/home/lijiashushu/c++JLT", startPoint, Eigen::VectorXd::Zero(14), Eigen::VectorXd::Zero(14), trajCoeff);
//
//    return 0;
//}