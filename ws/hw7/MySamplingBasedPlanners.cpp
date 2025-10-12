# include "MySamplingBasedPlanners.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {

    Eigen::VectorXd lower(2);
    lower << problem.x_min, problem.y_min;
    Eigen::VectorXd upper(2);
    upper << problem.x_max, problem.y_max;

    PointCollisionChecker pcc(problem.obstacles);

    return planXD(problem.q_init, problem.q_goal, pcc, lower, upper);
}

double MyPRM::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2){
    if(v1.size() != v2.size()){
        printf("Invalid distance calculation, vectors do not have same dimension\n");

        return 0.0;
    }

    double sum = 0;
    for(int i = 0; i < v1.size(); i++){
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}



amp::Path2D MyPRM::planXD(Eigen::VectorXd init_state, Eigen::VectorXd goal_state, PointCollisionChecker collision_checker, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound){

    amp::Path2D path;


    //seed the random numbers
    srand(time(0));

    int dim = upper_bound.size();

    if(upper_bound.size() != lower_bound.size()){
        printf("Invalid bound vectors, vectors do not have same dimension\n");
    }

    Eigen::VectorXd d_bound = upper_bound - lower_bound;

    bool valid_sample = false;
    Eigen::VectorXd sample(dim);

    while(valid_sample == false){
        //determine if this is a goal sample
        if(double(rand()) / RAND_MAX < GOAL_BIAS){
            sample = goal_state;
            break;
        }

        //generate a random sample
        for(int i = 0; i < dim; i++){
            sample[i] = (upper_bound[i] * rand()) / RAND_MAX;
        }

        //add lower bound
        sample = sample + lower_bound;

        if(collision_checker.check(sample) == false){
            valid_sample = true;
        }
    }

    if(dim != 2){
        printf("Not ready yet!\n");
        exit(0);
    }

    Eigen::Vector2d p;
    p = sample;
    path.waypoints.push_back(p);

    return path;

}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}

double MyRRT::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2){
    if(v1.size() != v2.size()){
        printf("Invalid distance calculation, vectors do not have same dimension\n");

        return 0.0;
    }

    double sum = 0;
    for(int i = 0; i < v1.size(); i++){
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}

PointCollisionChecker::PointCollisionChecker(std::vector<amp::Obstacle2D> obstacles){
    int index = 0;
    for(int i = 0; i < obstacles.size(); i++){

        std::vector<Eigen::Vector2d> obs;
        for(int j = 0; j < obstacles.at(i).verticesCW().size(); j++){
            Segment s;
            s.v1 = obstacles.at(i).verticesCW().at(j);
            if(j+1 < obstacles.at(i).verticesCW().size()){
                s.v2 = obstacles.at(i).verticesCW().at(j + 1);
            }else{
                s.v2 = obstacles.at(i).verticesCW().at(0);
            }

            s.obstacle_index = i;
            s.intra_index = j;
            s.index = index;

            segments.push_back(s);

            index++;

            obs.push_back(obstacles.at(i).verticesCW().at(j));

        }
        vertices_cw.push_back(obs);
    }
}

bool PointCollisionChecker::check(Eigen::Vector2d point){

    for(int i = 0; i < vertices_cw.size(); i++){
        
        double previous_angle = atan2(vertices_cw.at(i).at(0)[1]- point[1], vertices_cw.at(i).at(0)[0]- point[0]);
        for(int j = 1; j < vertices_cw.at(i).size(); j++){
            double current_angle = atan2(vertices_cw.at(i).at(j)[1]- point[1], vertices_cw.at(i).at(j)[0]- point[0]);

            if(fmod((previous_angle - current_angle), 6.28) > 3.14){
                return true;
            }
        }

    }

    return false;
}

bool PointCollisionChecker::check_segment(Eigen::Vector2d p1, Eigen::Vector2d p2){
    Eigen::Vector2d dummy;

    bool in_collision = false;
    for(int i = 0; i < segments.size(); i++){
        if(findSegmentIntersection(p1, p2, segments.at(i).v1, segments.at(i).v2, &dummy)){
            in_collision = true;
            break;
        }
    }
    
    return in_collision;
}


//used from previous homework 
//this function was ChatGPT inspired but I've heavily edited the code
//Prompt: write a function that determines the point of intersection of two lines in cpp. The vertexes of each thing will be passed as eigen::vector2d
bool PointCollisionChecker::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter) {
    // Representing the two lines as follows:
    // Line 1: p1 + t * (p2 - p1)
    // Line 2: q1 + u * (q2 - q1)
    // We need to find the values of t and u where the lines intersect.
    
    Eigen::Vector2d dir1 = p2 - p1;  // Direction vector of line 1
    Eigen::Vector2d dir2 = q2 - q1;  // Direction vector of line 2
    
    // We solve for t and u using the determinant method.
    double denom = dir1.x() * dir2.y() - dir1.y() * dir2.x();
    
    //gpt moment - tsk tsk
    if(fabs(dir2.y()) < FLOAT_TOL && fabs(dir1.x()) < FLOAT_TOL){

        inter->x() = p1.x();
        inter->y() = q1.y();

        bool within_y = std::min(p1.y(), p2.y()) <= inter->y() && std::max(p1.y(), p2.y()) >= inter->y();
        bool within_x = std::min(q1.x(), q2.x()) <= inter->x() && std::max(q1.x(), q2.x()) >= inter->x();
            
        return within_x && within_y;

    }else if(fabs(dir2.x()) < FLOAT_TOL && fabs(dir1.y()) < FLOAT_TOL){
        inter->x() = q1.x();
        inter->y() = p1.y();

        bool within_y = std::min(q1.y(), q2.y()) <= inter->y() && std::max(q1.y(), q2.y()) >= inter->y();
        bool within_x = std::min(p1.x(), p2.x()) <= inter->x() && std::max(p1.x(), p2.x()) >= inter->x();
            
        return within_x && within_y;
    }

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
