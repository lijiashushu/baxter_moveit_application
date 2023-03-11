//
// Created by lijiashushu on 20-9-8.
//

#ifndef DUAL_RRT_STAR_PATH_PLANNER_H
#define DUAL_RRT_STAR_PATH_PLANNER_H

#include <vector>

class PathPlanner{
public:
    virtual bool plan() = 0;
    virtual void showPath() = 0;
    //感觉这种接口必须得提前想好所有可能传入的参数，或者类似于 Python 一样，通过字典传递参数
    virtual void initPara(int seed,
                           double biProbility,
                           int maxSampleTimes,
                           double maxPlanningTime,
                           double taskStep,
                           size_t constrainIndex,
                           const std::vector<double>& startAngles,
                           const std::vector<double>& goalAngles,
                           bool displayStartState,
                           bool if_manipuGrand,
                           bool if_rrt_star,
                           bool if_informed) = 0;

    std::vector<Eigen::Matrix<double, 14, 1>> _planning_result_joint_angles;
    std::vector<size_t> _planning_result_index;
    std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> _planning_result_task_state_vector;

public:
    virtual ~PathPlanner(){};
};

#endif //DUAL_RRT_STAR_PATH_PLANNER_H
