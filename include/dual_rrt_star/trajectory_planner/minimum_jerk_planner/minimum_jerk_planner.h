//
// Created by lijiashushu on 20-4-8.
//

#ifndef MINIMUM_JERK_PLANNER_H
#define MINIMUM_JERK_PLANNER_H
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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

//each segment of each dof uses one 5th order polynomial
class MinimumJerkPlanner{
public:
    MinimumJerkPlanner(){};
    ~MinimumJerkPlanner(){};

    /**
       *  @brief  output the generated trajectory angle, velocity, acceleration to the file.
       *  @param  path the path of the storage folder
       *  @param  polyCoeff  optimized coeffients of polynomial after calling generatePolyQP
       *  @param  times time of each segment which needs to be set before the optimization.
       */
    void outputTrajToFile(string path, const MatrixXd& polyCoeff, const VectorXd &time);

    /**
       *  @brief  show the generated trajectory in rviz
       *  @param  polyCoeff  optimized coeffients of polynomial after calling generatePolyQP
       *  @param  times time of each segment which needs to be set before the optimization.
       *  @param  nh ros node handler for creating publisher
       */
    void viewTrajectory(const MatrixXd& polyCoeff, const VectorXd &times, ros::NodeHandle &nh);

    /**
       *  @brief  Compute the optimization variable for the trajectory.
       *  @param  pathPoints  pathPoints matrix from path planner. (segment + 1) * dimension
       *  @param  times time of each segment which needs to be set before the optimization.
       *  @return  segment * (dimension * 6) matrix
       */
    MatrixXd generatePolyQP(MatrixXd pathPoints, VectorXd times);
    

private:

    /**
       *  @brief  get the pos, vel, acc of segment k at time t
       *  @param  polyCoeff optimized coeffients of polynomial after calling generatePolyQP
       *  @param  k segment index
       *  @param  t time at this segment, each segment start from 0
       *  @param  pos output
       *  @param  vel output
       *  @param  acc output
       */
    void getPosPoly( MatrixXd polyCoeff, int k, double t, VectorXd& pos, VectorXd& vel, VectorXd& acc);

    /**
       *  @brief  Calculate the factorial of x
       *  @param  x input number
       */
    int Factorial(int x);


};


#endif //STAR_TRAJECTORY_GENERATOR_H
