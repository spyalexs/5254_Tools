#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    //create a potential function
    func = new MyPotentialFunction(problem.q_goal, 2, 2, .1, 5, problem.obstacles);


    amp::Path2D path;

    path.waypoints.push_back(problem.q_init);


    int steps = 50000;
    Eigen::Vector2d current_position = problem.q_init ;//+ Eigen::Vector2d(-0.0000001,0.0);
    while(steps > 0){
        Eigen::Vector2d grad = func->getGradient(Eigen::Vector2d(current_position[1], current_position[0]));
        grad.normalize();
        current_position += 0.001 * Eigen::Vector2d(grad[0], grad[1]);
        path.waypoints.push_back(current_position);

        steps --;

        if((current_position - problem.q_goal).norm() < 0.25){
            path.waypoints.push_back(problem.q_goal);
            break;
        }
    }
    
    

    return path;
}

MyPotentialFunction::MyPotentialFunction(Eigen::Vector2d Goal, double dStar, double Zetta, double Q_star, double Eta, std::vector<amp::Obstacle2D> Obs){
    this->goal = Goal;
    this->d_star = dStar;
    this->zetta = Zetta;
    this->obstacles = Obs;
    this->q_star = Q_star;
    this->eta = Eta;
}

Eigen::Vector2d MyPotentialFunction::getGradient(const Eigen::Vector2d& q) const {
    //attract towards the goal

    Eigen::Vector2d qc(q[1], q[0]);
        
    double dx_attractive = -zetta * (qc[0] - goal[0]);
    double dy_attractive = -zetta * (qc[1] - goal[1]);
    
    if((qc - goal).norm() <= d_star){
        //if further that
        dx_attractive = -d_star * zetta * (qc[0] - this->goal[0]) / sqrt((qc[0] - this->goal[0]) * (qc[0] - this->goal[0]) + (qc[1] - this->goal[1]) * (qc[1] - this->goal[1]));
        dy_attractive = -d_star * zetta * (qc[1] - this->goal[1]) / sqrt((qc[0] - this->goal[0]) * (qc[0] - this->goal[0]) + (qc[1] - this->goal[1]) * (qc[1] - this->goal[1]));
    }

    //repell from obstacles
    double dx_repulsive = 0;
    double dy_repulsive = 0;
    for(int i = 0; i < obstacles.size(); i++){
        //determine the closest point to the obstalce
        Eigen::Vector2d obstacle_point = getClosestPointToObstacle(qc, obstacles.at(i));
        Eigen::Vector2d obstacle_center = getObstacleCentroid(obstacles.at(i));


        if((obstacle_point - qc).norm() < q_star){
            dx_repulsive += eta * (1/(obstacle_point - qc).norm() - 1 / q_star) / (obstacle_point - qc).norm() / (obstacle_point - qc).norm() / (obstacle_point - qc).norm() * (qc[0] - obstacle_point[0]);
            dy_repulsive += eta * (1/(obstacle_point - qc).norm() - 1 / q_star) / (obstacle_point - qc).norm() / (obstacle_point - qc).norm() / (obstacle_point - qc).norm() * (qc[1] - obstacle_point[1]);

            // if((obstacle_center - qc).norm() < q_star_c){
            //be sure to steer a little bit away from the center
            double dx = eta_c * (1/(obstacle_center - qc).norm() - 1 / q_star_c) / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() * (qc[0] - obstacle_center[0]);
            dx_repulsive += dx;
            double dy = eta_c * (1/(obstacle_center - qc).norm() - 1 / q_star_c) / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() * (qc[1] - obstacle_center[1]);
            dy_repulsive += dy;

                //printf("Repelling from obstalce at (%f, %f) with (%f, %f)\n",  dx_repulsive,  dy_repulsive, dx, dy);

            //}

        } 

         
        
        // if((obstacle_center - qc).norm() < q_star_c){
        //     double dx = eta_c * (1/(obstacle_center - qc).norm() - 1 / q_star_c) / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() * (qc[0] - obstacle_center[0]);
        //     dx_repulsive += dx;
        //     double dy = eta_c * (1/(obstacle_center - qc).norm() - 1 / q_star_c) / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() / (obstacle_center - qc).norm() * (qc[1] - obstacle_center[1]);
        //     dy_repulsive += dy;

        //     //printf("Repelling from obstalce at (%f, %f) with (%f, %f)\n",  obstacle_center[0],  obstacle_center[0], dx, dy);
        // } 

        //printf("Obstalce Closest Point: %f %f to q: %f %f \n", obstacle_point[0], obstacle_point[1], q[0], q[1]);

    }

    // dx_repulsive = std::clamp(dx_repulsive, -10.0, 10.0);
    // dy_repulsive = std::clamp(dy_repulsive, -10.0, 10.0);

    //printf("Goal %f, %f \n\n", goal[0],  goal[1]);

    

    return Eigen::Vector2d(dx_attractive + dx_repulsive,  dy_attractive + dy_repulsive);

}

Eigen::Vector2d MyPotentialFunction::getClosestPointToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obs) const{
    //return the closest point on each obstacle to the evaluation point q

    Eigen::Vector2d closest;
    double dist_closest = -1;
    for(int i = 0; i < obs.verticesCW().size(); i++){

        Eigen::Vector2d segment_closest = findClosestPoint(obs.verticesCW().at(i), obs.verticesCW().at((i + 1) % obs.verticesCW().size()), q);

        if(dist_closest < 0 || dist_closest > (segment_closest - q).norm()){
            dist_closest = (segment_closest - q).norm();
            closest = segment_closest;
        }
    }

    return closest;
}

Eigen::Vector2d MyPotentialFunction::getObstacleCentroid(const amp::Obstacle2D& obs) const{
    //return the closest point on each obstacle to the evaluation point q

    Eigen::Vector2d centroid(0,0);
    double dist_closest = -1;
    for(int i = 0; i < obs.verticesCW().size(); i++){
        centroid += obs.verticesCW().at(i);
    }

    centroid = centroid / obs.verticesCW().size();

    return centroid;
}

//I had chatgpt inspire me on the best way to go about this... I wrote myself
//Inspired from code I wrote from HW2
Eigen::Vector2d MyPotentialFunction::findClosestPoint(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d q) const{

    //find the point on the line in which the vector from the segment to the goal is perpendicular to the segment itself
    Eigen::Vector2d segment_diff = v2 - v1;
    Eigen::Vector2d point_diff = q - v1;

    double t = segment_diff.dot(point_diff) / segment_diff.dot(segment_diff);

    t = std::clamp(t, 0.0, 1.0);

    return segment_diff * t + v1;
}

