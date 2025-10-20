#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"
#include "segment.h"
#include "MyAStar.h"
#include <numeric>

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 

#define FLOAT_TOL .00001
#define GOAL_BIAS .05
#define MAX_CONNECT 0.5
#define MAX_CONNECT_PRM 2.0
#define CONNECT_STEP 0.03
#define GOAL_CONNECT 0.25
#define PATH_CHECKER_DISCRETIZATION_LEVELS 5


class PointCollisionChecker{
    public:
        PointCollisionChecker(std::vector<amp::Obstacle2D> obstacles, std::vector<double> r);

        bool check(Eigen::Vector2d point, int disk);

        bool check_segment(Eigen::Vector2d p1, Eigen::Vector2d p2, int disk);

        int check_agents(std::vector<Eigen::Vector2d> p1, std::vector<Eigen::Vector2d> p2);
        int check_agents2(std::vector<Eigen::Vector2d> p1, std::vector<Eigen::Vector2d> p2);

        bool findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter);

        //assuming translation, translation per robot
        bool checkXdPointCollision(Eigen::VectorXd test_v);
        bool checkXdSegmentCollision(Eigen::VectorXd test_v1, Eigen::VectorXd test_v2);

        std::vector<std::vector<Segment>> segments;

        std::vector<std::vector<std::vector<Eigen::Vector2d>>> vertices_cw;
        std::vector<std::vector<Eigen::Vector2d>> base_vertices_cw;

        std::vector<double> disk_radius;
};

class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 

        double get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);

        std::map<amp::Node, Eigen::VectorXd> nodes;

        double cost = -1;
        double valid = false;
        double elapsed_time;



};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        double get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);

        std::map<amp::Node, Eigen::VectorXd> nodes;

        double cost = -1;
        double valid = false;
        double elapsed_time;

};