#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

class MyManipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        MyManipulator2D();

        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;

        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        //apply a two D rotation matrix
        Eigen::Vector2d applyRotMat(const Eigen::Vector2d link, const double theta) const;
};