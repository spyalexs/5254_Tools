#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "segment.h"
#include <algorithm>
#include <stdexcept>

#define BUMP_DIST 0.001
#define FLOAT_TOL 0.000001 


/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBug1 : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        bool determineMoveToGoalIntersection(std::vector<Segment> segments, Eigen::Vector2d starting, Eigen::Vector2d ending, Segment *inter_seg, Eigen::Vector2d *inter);

        bool findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter);

        Eigen::Vector2d apply_tolerance(Eigen::Vector2d *previous_point, Eigen::Vector2d *path_point);

        double findClosestPointToGoal(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d goal, Eigen::Vector2d *closest);

    private:
        // Add any member variables here...
};