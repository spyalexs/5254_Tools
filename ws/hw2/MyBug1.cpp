#include "MyBug1.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug1::plan(const amp::Problem2D& problem) {

    std::vector<Segment> segments;

    //format everything into segments
    int index = 0;
    for(int i = 0; i < problem.obstacles.size(); i++){
        for(int j = 0; j < problem.obstacles.at(i).verticesCW().size(); j++){
            Segment s;
            s.v1 = problem.obstacles.at(i).verticesCW().at(j);
            if(j+1 < problem.obstacles.at(i).verticesCW().size()){
                s.v2 = problem.obstacles.at(i).verticesCW().at(j + 1);
            }else{
                s.v2 = problem.obstacles.at(i).verticesCW().at(0);
            }

            s.obstacle_index = i;
            s.intra_index = j;
            s.index = index;

            segments.push_back(s);

            index++;
        }
    }

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);


    Eigen::Vector2d prev_point = problem.q_init;
    //loop will not reached goal
    while(path.waypoints.size() < 20000000){
        Segment inter_seg;
        Eigen::Vector2d inter_point;

        //find next collision obstalce or move to goal
        printf("previous point x: %f y: %f \n",prev_point.x(), prev_point.y() );
        if(!determineMoveToGoalIntersection(segments, prev_point, problem.q_goal, &inter_seg, &inter_point)){
            path.waypoints.push_back(problem.q_goal);

            return path;
        }

        //index of the path where in the intersection occured
        int intersection_path_index = path.waypoints.size();

        //record signature of initial segment
        Eigen::Vector2d target_point = inter_point;
        Segment current_segment = inter_seg;
        double closest_distance = -1;
        double dist_since_intersection = 0;
        double dist_to_closest = 0;

        //don't start on the obstalce
        apply_tolerance(&prev_point, &inter_point);
        path.waypoints.push_back(inter_point);

        // the starting point to follow the obstalce
        Eigen::Vector2d current_point = inter_point;

        //point and step number thats closest to the goal
        Eigen::Vector2d closest_point = inter_point;
        int closest_step = 0; 

        //waypoint start
        int following_start_waypoint = path.waypoints.size();

        int step_count = 0; // number of steps for which the bug has followed the obstacle
        double previous_obstalce_clearance = BUMP_DIST; // how far the bug was from the obstalce previously
        double d_obstalce_clearance = 0;
        double heading = atan2(inter_seg.v2.y() - inter_seg.v1.y(), inter_seg.v2.x() - inter_seg.v1.x());

        while(euclideanDistance(inter_point, current_point) > BUMP_DIST || step_count < BUMP_DIST / INCREMENT * 2){

            double delta_heading = -(previous_obstalce_clearance - FOLLOWING_DIST) * fabs((previous_obstalce_clearance - FOLLOWING_DIST)) * 1000000 - d_obstalce_clearance * 1000;
            heading += delta_heading;

            Eigen::Vector2d step;
            step.y() = INCREMENT * sin(heading);
            step.x() = INCREMENT * cos(heading);

            current_point = current_point + step;
            path.waypoints.push_back(current_point);

            //printf("Current Point x: %f y: %f\n", current_point.x(), current_point.y());

            //calculate the distance from each obstalce
            double obstalce_clearance = -1;
            Eigen::Vector2d temp;
            for(int i = 0; i < segments.size(); i++){
                double segment_distance = findClosestPointDist(segments.at(i).v2, segments.at(i).v1, current_point, &temp);
                if(segment_distance < obstalce_clearance || obstalce_clearance < 0){
                    obstalce_clearance = segment_distance;
                }
            }
            
            //rate of clearance change
            d_obstalce_clearance = obstalce_clearance - previous_obstalce_clearance;
            previous_obstalce_clearance = obstalce_clearance;

            step_count++;

            //keep track of the closest point to the goal
            if((closest_distance < 0 || closest_distance > euclideanDistance(problem.q_goal, current_point))){
                closest_distance = euclideanDistance(problem.q_goal, current_point);
                closest_point = current_point;
                closest_step = step_count;
                
            }

            //totally never need this
            if(step_count > 3000000){
                return path;
            }
            
        }

        printf("Closest point x: %f y: %f step: %d\n", closest_point.x(), closest_point.y(), closest_step);
            
        //determine the clostest direction of travel around the obstalce
        if(closest_step > step_count / 2){
            //go in reverse
            
            int end_size = path.waypoints.size();
            for(int i = end_size - 1; i >= end_size - (step_count - closest_step); i--){
                path.waypoints.push_back(path.waypoints.at(i));
            }

        } else {

            for(int i = following_start_waypoint; i < following_start_waypoint + (closest_step); i++){
                path.waypoints.push_back(path.waypoints.at(i));
            }
        }

        prev_point = path.waypoints.at(path.waypoints.size() - 1);
    }



    return path;
}

bool MyBug1::determineMoveToGoalIntersection(std::vector<Segment> segments, Eigen::Vector2d starting, Eigen::Vector2d ending, Segment *inter_seg, Eigen::Vector2d *inter_point){


    //determine the list of segments that intersect the line between the startpoint and intersection
    std::vector<Segment> int_segments;
    std::vector<Eigen::Vector2d> intersections;
    for(int i = 0; i < segments.size(); i++){
        Eigen::Vector2d inter;

        if(findSegmentIntersection(segments.at(i).v1, segments.at(i).v2, starting, ending, &inter)){
            intersections.push_back(inter);
            int_segments.push_back(segments.at(i));

        }
    }

    if(int_segments.size() == 0){
        return false;
    }

    //determine the closest
    double closest_dist = 0;
    int closest_index = -1;
    for(int i = 0; i < int_segments.size(); i++){
        Eigen::Vector2d diff = intersections.at(i) - starting;

        if(diff.norm() < closest_dist || closest_index < 0){
            closest_index = i;
            closest_dist = diff.norm();
        }
    }

    *inter_seg = int_segments.at(closest_index);
    *inter_point = intersections.at(closest_index);

    return true;

}

Eigen::Vector2d MyBug1::apply_tolerance(Eigen::Vector2d *previous_point, Eigen::Vector2d *path_point){

    //get the direction of motion
    Eigen::Vector2d dir = *path_point - *previous_point;

    dir.normalize();

    //move back by the set dist
    *path_point = *path_point - dir * BUMP_DIST; 

    return dir;
}


//I had chatgpt write the base for this function! 
//Prompt: write a function that determines the point of intersection of two lines in cpp. The vertexes of each thing will be passed as eigen::vector2d
// Function to find the intersection of two lines represented by two points each
bool MyBug1::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter) {
    // Representing the two lines as follows:
    // Line 1: p1 + t * (p2 - p1)
    // Line 2: q1 + u * (q2 - q1)
    // We need to find the values of t and u where the lines intersect.
    
    Eigen::Vector2d dir1 = p2 - p1;  // Direction vector of line 1
    Eigen::Vector2d dir2 = q2 - q1;  // Direction vector of line 2
    
    // We solve for t and u using the determinant method.
    double denom = dir1.x() * dir2.y() - dir1.y() * dir2.x();
    
    // If denom is 0, the lines are parallel or coincident, so there's no single intersection point
    if (denom == 0) {
        return false;
    }

    // Calculate the numerators for t and u using the cross-product method
    double t_num = (q1.x() - p1.x()) * dir2.y() - (q1.y() - p1.y()) * dir2.x();
    double u_num = (q1.x() - p1.x()) * dir1.y() - (q1.y() - p1.y()) * dir1.x();
    
    // Calculate the parameter t (or u), and then use it to find the intersection point
    double t = t_num / denom;
    
    // Intersection point is p1 + t * (p2 - p1)
    *inter = p1 + t * dir1;

    bool within_x = std::min(p1.x(), p2.x()) <= inter->x() && std::max(p1.x(), p2.x()) >= inter->x() && std::min(q1.x(), q2.x()) <= inter->x() && std::max(q1.x(), q2.x()) >= inter->x();
    bool within_y = std::min(p1.y(), p2.y()) <= inter->y() && std::max(p1.y(), p2.y()) >= inter->y() && std::min(q1.y(), q2.y()) <= inter->y() && std::max(q1.y(), q2.y()) >= inter->y();
    
    return within_x && within_y;
}


//I had chatgpt inspire me on the best way to go about this... I wrote myself
double MyBug1::findClosestPointDist(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d goal, Eigen::Vector2d *closest){

    //find the point on the line in which the vector from the segment to the goal is perpendicular to the segment itself
    Eigen::Vector2d segment_diff = v2 - v1;
    Eigen::Vector2d point_diff = goal - v1;

    double t = segment_diff.dot(point_diff) / segment_diff.dot(segment_diff);

    t = std::clamp(t, 0.0, 1.0);

    *closest =  segment_diff * t + v1;

    Eigen::Vector2d goal_diff = *closest - goal;

    return goal_diff.norm();
}

double MyBug1::euclideanDistance(Eigen::Vector2d p1, Eigen::Vector2d p2){

    Eigen::Vector2d point_diff = p1 - p2;

    return point_diff.norm();

}


