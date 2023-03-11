//
// Created by lijiashushu on 20-4-17.
//

#ifndef TIME_DULA_CONSTRAINT_PLANNER_H
#define TIME_DULA_CONSTRAINT_PLANNER_H
#include "dual_rrt_star/trajectory_planner/time_dual_constraint_planner/trapezoid_profile.h"
#include <nlopt.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

using namespace std;

class TimeDualConstraintPlanner{

public:
    TimeDualConstraintPlanner(){};
    ~TimeDualConstraintPlanner(){};

    /**
       *  @brief  Compute the optimization variable for the trajectory.
       *  @param  pathPoints  pathPoints matrix from path planner. (segment + 1) * dimension
       *  @param  allSegCoeff  output parameter. total segments parameters  segment * (7 + dimension * 7) matrix, the first 7 are time of each time interval
       *          , then the jerk of each interval of each dimension.
       *  @param  softWeight the relative weight of cartisian error
       *  @param  gradStep the scale factor for the computed grad in each optimization step. (generally 1, intended for debug)
       */
    double getAllSegCoeff(Eigen::MatrixXd pathPoints,
                          Eigen::MatrixXd &allSegCoeff,
                          double softWeight,
                          double gradStep,
                          int constraint_index);

    /**
       *  @brief  output the generated trajectory angle, velocity, acceleration to the file.
       *  @param  allSegCoeff  optimized trajectory parameters after calling getAllSegCoeff
       *  @param  pathPoints  pathPoints matrix from path planner. (segment + 1) * dimension
       *  @param  path the path of the storage folder
       */
    void outputTrajToFile(const Eigen::MatrixXd &allSegCoeff, const Eigen::MatrixXd &pathPoints, string path);

    /**
       *  @brief  show the generated trajectory in rviz.
       *  @param  allSegCoeff  optimized trajectory parameters after calling getAllSegCoeff
       *  @param  pathPoints  pathPoints matrix from path planner. (segment + 1) * dimension
       *  @param  nh ros node handler
       */
    void viewTrajectory(const Eigen::MatrixXd& allSegCoeff, const Eigen::MatrixXd &pathPoints, ros::NodeHandle &nh);


private:

    /**
       *  @brief  optimization object function
       */
    static double objectiveFuncNew(unsigned n, const double *x, double *grad, void *f_data);

    /**
       *  @brief  optimization equality constraint function
       */
    static void equalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

    /**
       *  @brief  optimization inequality constraint function
       */
    static void inequalityConstraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

};



#endif //TIME_DULA_CONSTRAINT_PLANNER_H
