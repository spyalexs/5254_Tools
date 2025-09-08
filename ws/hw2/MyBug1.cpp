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
    while(path.waypoints.size() < 10000){
        Segment inter_seg;
        Eigen::Vector2d inter_point;

        //find next collision obstalce or move to goal
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
        Eigen::Vector2d closest_point = inter_point;
        double closest_distance = -1;
        double dist_since_intersection = 0;
        double dist_to_closest = 0;
        int closest_path_index;

        apply_tolerance(&prev_point, &inter_point);

        while(path.waypoints.size() < 10000){

            //printf("Eval Next top: %d\n", current_segment.index);

            //with the autograder, it appears that a direct move across the edge of an obstacle is a collision so a tolerance is added
            Eigen::Vector2d tol_dir = apply_tolerance(&prev_point, &target_point);
            path.waypoints.push_back(target_point);
            
            //record distance travelled around
            Eigen::Vector2d diff = target_point - prev_point;
            dist_since_intersection += diff.norm();

            //printf("Target Point b4: %f %f Starting Point %f %f \n", target_point.x(), target_point.y(), prev_point.x(), prev_point.y());

            prev_point = target_point;

            //determine the next waypoint - always going to be based on endpoint2 of segment (known from ordering) (unless another intersection)

            target_point = current_segment.v2;

            Eigen::Vector2d current_segment_direction = current_segment.v2 - current_segment.v1;
            current_segment_direction.normalize();
            target_point = target_point + current_segment_direction * BUMP_DIST * 2;

            //if completing loop around - target initial intersection point
            completing_loop = false;
            if(current_segment.index == inter_seg.index && moved){
                printf("Here: %d, %d\n", current_segment.index, current_segment.index == inter_seg.index);
                target_point = inter_point;
                completing_loop = true;
            }

            //printf("Target Point: %f %f Starting Point %f %f \n", target_point.x(), target_point.y(), prev_point.x(), prev_point.y());

            if(!determineMoveToGoalIntersection(segments, prev_point, target_point, &current_segment, &target_point)){

                if(completing_loop){
                    path.waypoints.push_back(target_point);

                    //printf("Closest point to goal is: %f %f", closest_point.x(), closest_point.y());

                    break;

                    //check to ensure that the exit point is closer than the entry point
                    Eigen::Vector2d inter_goal_diff = problem.q_goal - inter_point;
                    if(closest_distance >= inter_goal_diff.norm()){
                        //didn't move closer, must be trapped
                        throw std::runtime_error("No possible paths detected!");

                    }
                }

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
                //printf("I got an intersection at: %f %f \n", target_point.x(), target_point.y());

            }

            //determine if the closest point on the previous segment is the closest point to the goal
        
            Eigen::Vector2d segment_closest;
            double distance_from_goal = findClosestPointToGoal(prev_point, target_point, problem.q_goal, &segment_closest);
            if(distance_from_goal < closest_distance || closest_distance < 0){
                closest_distance = distance_from_goal;
                closest_point = segment_closest;
                dist_to_closest = dist_since_intersection;
                closest_path_index = path.waypoints.size();
            }

            moved = true;
        }

        //move to the closest point to the goal
        if(dist_to_closest * 2 > dist_since_intersection){
            //shorter to go backward
            for(int i = path.waypoints.size() - 2; i >= closest_path_index; i--){
                path.waypoints.push_back(path.waypoints.at(i));
            }

            path.waypoints.push_back(closest_point);
        } else {
            for(int i = intersection_path_index; i < closest_path_index; i++){
                path.waypoints.push_back(path.waypoints.at(i));
            }

            path.waypoints.push_back(closest_point);
        }

        prev_point = closest_point;

    }

    // for(int i = 0; i < 200; i++){
    //     printf("Waypoint %d: X: %f, %f\n", i, path.waypoints.at(i).x(), path.waypoints.at(i).y());
    // }

    // printf("x_vals = [");
    // for(int i = 0; i < 200; i++){
    //     printf("%f, ", path.waypoints.at(i).x());
    // }
    // printf("];\n");

    // printf("y_vals = [");
    // for(int i = 0; i < 200; i++){
    //     printf("%f, ", path.waypoints.at(i).y());
    // }
    // printf("];\n");

    // printf("\n\nhold on\n");


    // for(int i = 0; i < problem.obstacles.size(); i++){
    //     printf("ob_%d = [", i);
    //     for(int j = 0; j < problem.obstacles.at(i).verticesCW().size(); j++){
    //         printf("[%f, %f];", problem.obstacles.at(i).verticesCW().at(j).x(), problem.obstacles.at(i).verticesCW().at(j).y());
    //     }
    //     printf("[%f, %f];", problem.obstacles.at(i).verticesCW().at(0).x(), problem.obstacles.at(i).verticesCW().at(0).y());
    //     printf("];\n");

    //     printf("plot(ob_%d(:, 1), ob_%d(:, 2))\n", i, i);
    // }

    // printf("hold off\n\n");

    // printf("goal %f %f", problem.q_goal.x(), problem.q_goal.y());
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 1.0));
    // path.waypoints.push_back(Eigen::Vector2d(4.0, 1.0));
    // path.waypoints.push_back(Eigen::Vector2d(5.0, 3.0));
    // path.waypoints.push_back(Eigen::Vector2d(5.0, 7.0));
    // path.waypoints.push_back(problem.q_goal);

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
double MyBug1::findClosestPointToGoal(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d goal, Eigen::Vector2d *closest){

    //find the point on the line in which the vector from the segment to the goal is perpendicular to the segment itself
    Eigen::Vector2d segment_diff = v2 - v1;
    Eigen::Vector2d point_diff = goal - v1;

    double t = segment_diff.dot(point_diff) / segment_diff.dot(segment_diff);

    t = std::clamp(t, 0.0, 1.0);

    *closest =  segment_diff * t + v1;

    Eigen::Vector2d goal_diff = *closest - goal;

    return goal_diff.norm();
}


