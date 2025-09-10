#include "MyBug2.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug2::plan(const amp::Problem2D& problem) {

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

    //declare the m line
    std::vector<Segment> m_line;
    Segment s;
    s.v1 = problem.q_init;
    s.v2 = problem.q_goal;
    s.obstacle_index = 0;
    s.intra_index = 0;
    s.index = 0;
    m_line.push_back(s);


    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    double t = 0;

    double inter_dist = 100000000;


    Eigen::Vector2d prev_point = problem.q_init;
    //loop will not reached goal
    while(path.waypoints.size() < 100000){
        Segment inter_seg;
        Eigen::Vector2d inter_point;

        //find next collision obstalce or move to goal
        //printf("Starting from: %f %f\n", prev_point.x(), prev_point.y());
        if(!determineMoveToGoalIntersection(segments, prev_point, problem.q_goal, &inter_seg, &inter_point)){
            path.waypoints.push_back(problem.q_goal);

            return path;
        }

        //index of the path where in the intersection occured
        int intersection_path_index = path.waypoints.size();

        //record signature of initial segment
        Eigen::Vector2d target_point = inter_point;
        Segment current_segment = inter_seg;
        bool moved = false;
        bool completing_loop = false;

        apply_tolerance(&prev_point, &inter_point);


        //inter distance - distance from where the bug intersected
        Eigen::Vector2d inter_goal_diff = problem.q_goal - inter_point;
        inter_dist = std::min(inter_goal_diff.norm(), inter_dist);

        while(path.waypoints.size() < 100000){

            //printf("Eval Next top: %d\n", current_segment.index);

            //with the autograder, it appears that a direct move across the edge of an obstacle is a collision so a tolerance is added
            Eigen::Vector2d tol_dir = apply_tolerance(&prev_point, &target_point);
            path.waypoints.push_back(target_point);
    
            //printf("Target Point b4: %f %f Starting Point %f %f \n", target_point.x(), target_point.y(), prev_point.x(), prev_point.y());

            prev_point = target_point;

            //determine the next waypoint - always going to be based on endpoint2 of segment (known from ordering) (unless another intersection)

            target_point = current_segment.v2;

            Eigen::Vector2d current_segment_direction = current_segment.v2 - current_segment.v1;
            current_segment_direction.normalize();
            target_point = target_point + current_segment_direction * BUMP_DIST * 2;

            //printf("Target Point: %f %f Starting Point %f %f \n", target_point.x(), target_point.y(), prev_point.x(), prev_point.y());

            if(!determineMoveToGoalIntersection(segments, prev_point, target_point, &current_segment, &target_point)){


                Eigen::Vector2d temp_point;
                Segment temp_segment;
                if(determineMoveToGoalIntersection(m_line, prev_point, target_point, &temp_segment, &temp_point)){
                    //printf("Got m-line intersection at %f %f \n", temp_point.x(), temp_point.y());

                    //determine if this is a closer point then where the bug started to move around the obstalce
                    Eigen::Vector2d exit_goal_diff = problem.q_goal - temp_point;
                    double exit_distance = exit_goal_diff.norm();

                    if(exit_distance < inter_dist && moved){
                        target_point = temp_point;

                        inter_dist = exit_distance;

                        break;
                    }

                }

                // if(completing_loop){
                //     path.waypoints.push_back(target_point);

                //     printf("I'm not supposed to be here!");
                //     return path;
                // }

                //determine the next segment to follow
                //printf("Eval: %d\n", current_segment.index);
                if(current_segment.index + 1 >= segments.size()){
                    current_segment = segments.at(current_segment.index - current_segment.intra_index);
                }else if(current_segment.obstacle_index != segments.at(current_segment.index + 1).obstacle_index){
                    current_segment = segments.at(current_segment.index - current_segment.intra_index);
                }else{
                    current_segment = segments.at(current_segment.index + 1);
                }



                //printf("Eval Next: %d V1: %f %f V2: %f %f \n", current_segment.index, current_segment.v1.x(), current_segment.v1.y(), current_segment.v2.x(), current_segment.v2.y());

            } else{

                Eigen::Vector2d temp_point;
                Segment temp_segment;
                if(determineMoveToGoalIntersection(m_line, prev_point, target_point, &temp_segment, &temp_point)){
                    //printf("Got m-line intersection at %f %f \n", temp_point.x(), temp_point.y());

                    //determine if this is a closer point then where the bug started to move around the obstalce
                    Eigen::Vector2d exit_goal_diff = problem.q_goal - temp_point;
                    double exit_distance = exit_goal_diff.norm();

                    if(path.waypoints.size() >= 18 && path.waypoints.size() <= 21){
                            t = inter_dist - exit_distance;
                    }

                    if(exit_distance < inter_dist && moved){

                        inter_dist = exit_distance;

                        target_point = temp_point;
                        break;
                    }

                }

            }

            //determine if the closest point on the previous segment is the closest point to the goal

            moved = true;

        }

        Eigen::Vector2d current_segment_direction = current_segment.v2 - current_segment.v1;
        current_segment_direction.normalize();
        target_point = target_point + current_segment_direction * BUMP_DIST * 2;
        path.waypoints.push_back(target_point);
        prev_point = target_point;

    }


    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 1.0));
    // path.waypoints.push_back(Eigen::Vector2d(4.0, 1.0));
    // path.waypoints.push_back(Eigen::Vector2d(5.0, 3.0));
    // path.waypoints.push_back(Eigen::Vector2d(5.0, 7.0));
    // path.waypoints.push_back(problem.q_goal);

    return path;
}

bool MyBug2::determineMoveToGoalIntersection(std::vector<Segment> segments, Eigen::Vector2d starting, Eigen::Vector2d ending, Segment *inter_seg, Eigen::Vector2d *inter_point){


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

Eigen::Vector2d MyBug2::apply_tolerance(Eigen::Vector2d *previous_point, Eigen::Vector2d *path_point){

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
bool MyBug2::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter) {
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
double MyBug2::findClosestPointToGoal(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d goal, Eigen::Vector2d *closest){

    //find the point on the line in which the vector from the segment to the goal is perpendicular to the segment itself
    Eigen::Vector2d segment_diff = v2 - v1;
    Eigen::Vector2d point_diff = goal - v1;

    double t = segment_diff.dot(point_diff) / segment_diff.dot(segment_diff);

    t = std::clamp(t, 0.0, 1.0);

    *closest =  segment_diff * t + v1;

    Eigen::Vector2d goal_diff = *closest - goal;

    return goal_diff.norm();
}


