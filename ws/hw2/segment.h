#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

//little class to better represent line segments
struct Segment
{
    /* data */
    Eigen::Vector2d v1;
    Eigen::Vector2d v2;

    int obstacle_index;
    int intra_index;
    int index;
};
