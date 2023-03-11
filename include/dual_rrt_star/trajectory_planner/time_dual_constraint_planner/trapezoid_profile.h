//
// Created by lijiashushu on 20-4-14.
//

#ifndef TRAPEZOID_PROFILE_H
#define TRAPEZOID_PROFILE_H
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>


Eigen::MatrixXd getSecondOrderCoeff(double v0, double a0, double v_goal, double a_max, double j_max);
Eigen::Vector3d getStateFromVelocityCoeff(double p0, double v0, double a0, double t, const Eigen::MatrixXd& velocityCoeff);
Eigen::MatrixXd getThirdOrderCoeff(double p0, double v0, double a0, double p_goal, double v_max, double a_max, double j_max);
Eigen::Vector3d getStateFromPositionCoeff(double p0, double v0, double a0, double t, const Eigen::MatrixXd positionCoeff);
Eigen::MatrixXd getThirdOrderMultidofNonSync(Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::VectorXd p_goal, Eigen::MatrixXd jointLimits);
Eigen::MatrixXd getThirdOrderMultidofZeroSync(Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::VectorXd p_goal, Eigen::MatrixXd jointLimits);
void outputTrajToFile(std::string path, Eigen::VectorXd p0, Eigen::VectorXd v0, Eigen::VectorXd a0, Eigen::MatrixXd trajCoeff);


#endif //TRAPEZOID_PROFILE_H
