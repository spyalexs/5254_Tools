#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

#include "segment.h"

#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <chrono> // Required for std::chrono



#define FLOAT_TOL .00001
#define GOAL_BIAS .05
#define MAX_CONNECT 0.5
#define MAX_CONNECT_PRM 2.0
#define CONNECT_STEP 0.01
#define GOAL_CONNECT 0.25
#define PATH_CHECKER_DISCRETIZATION_LEVELS 5


class PointCollisionChecker{
    public:
        PointCollisionChecker(std::vector<amp::Obstacle2D> obstacles);

        bool check(Eigen::Vector2d point);

        bool check_segment(Eigen::Vector2d p1, Eigen::Vector2d p2);

        bool findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter);

        std::vector<Segment> segments;

        std::vector<std::vector<Eigen::Vector2d>> vertices_cw;
};

class MyPRM : public amp::PRM2D {
    public:
        MyPRM(){
            graphPtr = std::make_shared<amp::Graph<double>>();
        }
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        amp::Path2D planXD(Eigen::VectorXd init_state, 
                Eigen::VectorXd goal_state, 
                PointCollisionChecker collision_checker,
                Eigen::VectorXd lower_bound, 
                Eigen::VectorXd upper_bound);

        double get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);

        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::VectorXd> nodes;

        double max_connect = MAX_CONNECT_PRM;
        double number_of_nodes = 200;

        double cost = -1;
        double valid = false;
        double elapsed_time;
 
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        double get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);

        amp::Path2D planXD(Eigen::VectorXd init_state, 
                Eigen::VectorXd goal_state, 
                PointCollisionChecker collision_checker,
                Eigen::VectorXd lower_bound, 
                Eigen::VectorXd upper_bound);

        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::VectorXd> nodes;

        double cost = -1;
        double valid = false;
        double elapsed_time;

};
