#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)

    if(state.rows() != 3 && state.rows() != 2){
        printf("Input State Vector has the wrong length!\n");
    }

    if(nLinks() != state.rows()){
        //printf("Wrong number of link lengths provided!\n");
    }

    //printf("FKin state: %f %f vals: %ld \n", state(0), state(1), state.rows());

    std::vector<Eigen::Vector2d> joint_positions;
    joint_positions.push_back(Eigen::Vector2d(0.0,0.0));

    Eigen::Vector2d previous_position(0.0, 0.0);
    double previous_angle = 0;
    for(int i = 0; i < state.rows(); i++){
        previous_position = previous_position + Eigen::Vector2d(applyRotMat(Eigen::Vector2d(m_link_lengths.at(i), 0.0), previous_angle + state(i)));
        previous_angle += state(i);

        joint_positions.push_back(previous_position);
        
        //printf("Position %d at x: %f %f\n", i, previous_position(0), previous_position(1));
    }

    // //do forward kinematics
    // Eigen::Vector2d p1 = applyRotMat(Eigen::Vector2d(m_link_lengths.at(0), 0.0), state(0));
    // Eigen::Vector2d p2 = applyRotMat(Eigen::Vector2d(m_link_lengths.at(0), 0.0), state(0))
    //  + applyRotMat(Eigen::Vector2d(m_link_lengths.at(1), 0.0), state(0) + state(1));

    // printf("Link Length 1: %f, pos_x: %f, pos_y: %f\n", m_link_lengths.at(0), p1(0), p1(1));
    // printf("Link Length 2: %f, pos_x: %f, pos_y: %f\n", m_link_lengths.at(1), p2(0), p2(1));

    // if(state.rows() == 3){
    //     Eigen::Vector2d p3 = applyRotMat(Eigen::Vector2d(m_link_lengths.at(0), 0.0), state(0))
    //     + applyRotMat(Eigen::Vector2d(m_link_lengths.at(1), 0.0), state(0) + state(1))
    //     + applyRotMat(Eigen::Vector2d(m_link_lengths.at(2), 0.0), state(0) + state(1) + state(2));

    //     printf("Link Length 3: %f, pos_x: %f, pos_y: %f\n", m_link_lengths.at(2), p3(0), p3(1));

    //     std::vector<Eigen::Vector2d> joint_positions = {p3, p2, p1};
    //     return joint_positions[joint_index];
    // }

    //std::vector<Eigen::Vector2d> joint_positions = {p2, p1};
    return joint_positions[joint_index];
}

Eigen::Vector2d MyManipulator2D::applyRotMat(const Eigen::Vector2d link, const double theta) const{
    Eigen::Vector2d result;

    result << cos(theta) * link(0) - sin(theta) * link(1), sin(theta) * link(0) + cos(theta) * link(1);

    return result;
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location, bool flipped) const {
    // Implement inverse kinematics here

    printf("RQ %f %f -- ", end_effector_location(0), end_effector_location(1));

    amp::ManipulatorState joint_angles(nLinks());
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {

        Eigen::Vector2d p2 = end_effector_location;

        //use the lecture equations to solve for the other two links
        double t2 = acos(1 / (2*(m_link_lengths.at(0) * m_link_lengths.at(1))) * ((p2(0) * p2(0) + p2(1) * p2(1)) - (m_link_lengths.at(0) * m_link_lengths.at(0) + m_link_lengths.at(1) * m_link_lengths.at(1))));
        if(flipped){
            t2 = -t2;
        }
        double t1 = acos(1 / ((p2(0) * p2(0) + p2(1) * p2(1))) * (p2(0) * (m_link_lengths.at(0) + m_link_lengths.at(1) * cos(t2)) + p2(1) * m_link_lengths.at(1) * sqrt(1 - cos(t2) * cos(t2))));

        joint_angles(0) = t1;
        joint_angles(1) = t2;

        if(fabs(getJointLocation(joint_angles, 2)(0) - end_effector_location(0)) > 0.001){
            t1 = -t1;
        }


        return joint_angles;

    } else if (nLinks() == 3) {

        //I am assuming that the links can never interfere with each other.

        //printf("Target Location: %f %f L1: %f L2: %f L3: %f\n", end_effector_location(0), end_effector_location(1), m_link_lengths.at(0), m_link_lengths.at(1), m_link_lengths.at(2));

        //make the first link always point towards the origin
        bool solution = false;
        double angle_from_origin = atan2(end_effector_location(1), end_effector_location(0));

        while(!solution){
            Eigen::Vector2d p2 = end_effector_location + applyRotMat(Eigen::Vector2d(-m_link_lengths.at(2), 0.0), angle_from_origin);

            //printf("%f\n", 1 / (2*(m_link_lengths.at(0) * m_link_lengths.at(1))) * ((p2(0) * p2(0) + p2(1) * p2(1)) - (m_link_lengths.at(0) * m_link_lengths.at(0) + m_link_lengths.at(1) * m_link_lengths.at(1))));

            if(fabs(1 / (2*(m_link_lengths.at(0) * m_link_lengths.at(1))) * ((p2(0) * p2(0) + p2(1) * p2(1)) - (m_link_lengths.at(0) * m_link_lengths.at(0) + m_link_lengths.at(1) * m_link_lengths.at(1)))) > 1){
                angle_from_origin += .01;
                continue;
            }

            solution = true;

            //use the lecture equations to solve for the other two links
            double t2 = acos(1 / (2*(m_link_lengths.at(0) * m_link_lengths.at(1))) * ((p2(0) * p2(0) + p2(1) * p2(1)) - (m_link_lengths.at(0) * m_link_lengths.at(0) + m_link_lengths.at(1) * m_link_lengths.at(1))));
            double t1 = acos(1 / ((p2(0) * p2(0) + p2(1) * p2(1))) * (p2(0) * (m_link_lengths.at(0) + m_link_lengths.at(1) * cos(t2)) + p2(1) * m_link_lengths.at(1) * sqrt(1 - cos(t2) * cos(t2))));

            double t3 = angle_from_origin - t2 - t1;

            joint_angles(0) = t1;
            joint_angles(1) = t2;
            joint_angles(2) = t3;

            if(fabs(getJointLocation(joint_angles, 3)(0) - end_effector_location(0)) > 0.001){
                t1 = -t1;
                t3 = angle_from_origin - t2 - t1;
                joint_angles(0) = t1;
                joint_angles(2) = t3;
            }

            

            //printf("p2_x: %f p2_y: %f t3: %f, t2: %f t1: %f\n", p2(0), p2(1), t3, t2, t1);

        }

        printf("End effector %f %f\n", getJointLocation(joint_angles, 3)(0), getJointLocation(joint_angles, 3)(1));
        //printf("Links %f %f %f \n", m_link_lengths.at(0), m_link_lengths.at(1), m_link_lengths.at(2));

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}