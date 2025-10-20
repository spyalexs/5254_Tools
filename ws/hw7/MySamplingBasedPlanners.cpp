#include "MySamplingBasedPlanners.h"
#include "MyAStar.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {

    Eigen::VectorXd lower(2);
    lower << problem.x_min, problem.y_min;
    Eigen::VectorXd upper(2);
    upper << problem.x_max, problem.y_max;

    PointCollisionChecker pcc(problem.obstacles);

    return planXD(problem.q_init, problem.q_goal, pcc, lower, upper);
}

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {

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
    auto start = std::chrono::high_resolution_clock::now();


    nodes.clear();
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();


    amp::Path2D path;

    //seed the random numbers
    //srand(time(0));

    int dim = upper_bound.size();

    if(upper_bound.size() != lower_bound.size()){
        printf("Invalid bound vectors, vectors do not have same dimension\n");
    }

    Eigen::VectorXd d_bound = upper_bound - lower_bound;

    //printf("Upper %f Lower %f\n", upper_bound[0], lower_bound[0]);

    nodes[0] = init_state;
    nodes[1] = goal_state;
    
    int node_counter = 2;
    while(nodes.size() < number_of_nodes){
        bool valid_sample = false;
        Eigen::VectorXd sample(dim);

        while(valid_sample == false){
            //determine if this is a goal sample
            if(double(rand()) / RAND_MAX < 0){
                sample = goal_state;
                break;
            }

            //generate a random sample
            for(int i = 0; i < dim; i++){
                sample[i] = (d_bound[i] * rand()) / RAND_MAX;
            }

            //add lower bound
            sample = sample + lower_bound;

            //printf("Sampling %f %f \n", sample[0], sample[1]);

            if(collision_checker.check(sample)){
                valid_sample = true;
            }
        }

        //add the node to the graph
        nodes[node_counter] = sample;

        //attempt to connect the nodes
        for(int i = 0; i < nodes.size() - 1; i++){
            if(get_distance(nodes[i], nodes[node_counter]) < max_connect){
                //within range to connect

                if(!collision_checker.check_segment(nodes[i], nodes[node_counter])){
                    //okay to connect - connect in graph

                    //connect both ways
                    graphPtr->connect(i, node_counter, get_distance(nodes[i], nodes[node_counter]));
                    graphPtr->connect(node_counter, i, get_distance(nodes[i], nodes[node_counter]));
                }

            }
        }

        node_counter++;
    }

    //printf("Took %d samples\n", nodes.size());

    //printf("Third %f\n", nodes.at(3)[0]);

    //setup astar
    Heuristic heuristic(1, nodes);
    MyAStarAlgo astar;
    amp::ShortestPathProblem graph_problem;
    graph_problem.goal_node = 1;
    graph_problem.init_node = 0;
    graph_problem.graph = graphPtr;    

    //calculate the best route
    MyAStarAlgo::GraphSearchResult astar_out = astar.search(graph_problem, heuristic);
    cost = astar_out.path_cost;
    valid = astar_out.success;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);


    elapsed_time = duration.count();
    
    //connect the node numbers to the node locations
    for(auto it = astar_out.node_path.begin(); it != astar_out.node_path.end(); ++it){
        path.waypoints.push_back(nodes[*it]);
    }

    
    if(path.waypoints.size() == 0){
        return path;
    }
    
    //path smoothing
    amp::Path2D temp = path;
    double smooth_cost = 0;
    if(true){
        temp.waypoints.push_back(path.waypoints.at(0));


        for(int i = 0; i < path.waypoints.size(); i++){
            for(int j = path.waypoints.size() - 1; j > i; j--){
                if(!collision_checker.check_segment(path.waypoints.at(i), path.waypoints.at(j))){

                    smooth_cost += (temp.waypoints.back() - path.waypoints.at(j)).norm();


                    temp.waypoints.push_back(path.waypoints.at(j));
                    i = j;

                    break;
                }
                
            }
        }       
    }

    cost = smooth_cost;


    temp.waypoints.push_back(path.waypoints.back());


    path.waypoints = temp.waypoints;

    if(dim != 2){
        printf("Not ready yet!\n");
        exit(0);
    }

    return path;

}


// Implement your RRT algorithm here
amp::Path2D MyRRT::planXD(Eigen::VectorXd init_state, Eigen::VectorXd goal_state, PointCollisionChecker collision_checker, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound){
    auto start = std::chrono::high_resolution_clock::now();

    nodes.clear();
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

    amp::Path2D path;

    //seed the random numbers

    int dim = upper_bound.size();

    if(upper_bound.size() != lower_bound.size()){
        printf("Invalid bound vectors, vectors do not have same dimension\n");
    }

    Eigen::VectorXd d_bound = upper_bound - lower_bound;

    nodes[0] = init_state;
    
    int node_counter = 1;
    while(nodes.size() < 5000){
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
                sample[i] = (d_bound[i] * rand()) / RAND_MAX;
            }

            //add lower bound
            sample = sample + lower_bound;

            //printf("Sampling %f %f \n", sample[0], sample[1]);

            if(collision_checker.check(sample)){
                valid_sample = true;
            }
        }

        //attempt to connect the nodes
        amp::Node nearest_node = 0;
        double lowest_distance = -1;
        for(int i = 0; i < nodes.size() - 1; i++){
            if(get_distance(nodes[i], sample) < lowest_distance || lowest_distance < 0){
                //within range to connect

                lowest_distance = get_distance(nodes[i], sample);
                nearest_node = i;
            }
        }

        //propagate as far as possible
        Eigen::VectorXd direction = sample - nodes[nearest_node];
        direction.normalize();
        direction = direction * CONNECT_STEP;
        Eigen::VectorXd branch_end = nodes[nearest_node];
        bool iterated = false;
        for(int i = 1; i * CONNECT_STEP < MAX_CONNECT; i++){
            if(collision_checker.check_segment(nodes[nearest_node], nodes[nearest_node] + direction * i)){
                break;
            }

            iterated = true;
            branch_end = nodes[nearest_node] + direction * i;
        }
  
        if(iterated){
            //add the node to the graph
            nodes[node_counter] = branch_end;
            
            graphPtr->connect(nearest_node, node_counter, get_distance(nodes[nearest_node], nodes[node_counter]));
            graphPtr->connect(node_counter, nearest_node, get_distance(nodes[nearest_node], nodes[node_counter]));
        

            //determine if it is time to end
            //printf("Distance: %f\n", get_distance(nodes[node_counter], goal_state));
            if(get_distance(nodes[node_counter], goal_state) < GOAL_CONNECT){
                nodes[node_counter + 1] = goal_state;
                
                graphPtr->connect(node_counter, node_counter + 1, get_distance(nodes[node_counter], nodes[node_counter + 1]));
                graphPtr->connect(node_counter + 1, node_counter, get_distance(nodes[node_counter], nodes[node_counter + 1]));

                break;
            }

            node_counter++;
        }

        //printf("Nodes size: %d\n", nodes.size());
    }

    //setup astar
    Heuristic heuristic(node_counter + 1, nodes);
    MyAStarAlgo astar;
    amp::ShortestPathProblem graph_problem;
    graph_problem.goal_node = node_counter + 1;
    graph_problem.init_node = 0;
    graph_problem.graph = graphPtr;
    

    //calculate the best route
    MyAStarAlgo::GraphSearchResult astar_out = astar.search(graph_problem, heuristic);
    
    cost = astar_out.path_cost;
    valid = astar_out.success;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    elapsed_time = duration.count();


    //connect the node numbers to the node locations
    for(auto it = astar_out.node_path.begin(); it != astar_out.node_path.end(); ++it){
        path.waypoints.push_back(nodes[*it]);
    }



    if(dim != 2){
        printf("Not ready yet!\n");
        exit(0);
    }

    return path;

}

double MyRRT::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2){
    if(v1.size() != v2.size()){
        printf("Invalid distance calculation, vectors do not have same dimension %ld %ld \n", v1.size(), v2.size());

        return 0.0;
    }

    double sum = 0;
    for(int i = 0; i < v1.size(); i++){
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}

double Heuristic::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2){
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
        
        bool inside_obstacle = true;
        double previous_angle = atan2(vertices_cw.at(i).back()[1]- point[1], vertices_cw.at(i).back()[0]- point[0]);
        for(int j = 0; j < vertices_cw.at(i).size(); j++){
            double current_angle = atan2(vertices_cw.at(i).at(j)[1]- point[1], vertices_cw.at(i).at(j)[0]- point[0]);

            if(fmod((previous_angle - current_angle) + 6.28, 6.28) > 3.14){
                inside_obstacle = false;
                break;
            }

            previous_angle = current_angle;
        }

        if(inside_obstacle){
            return false;
        }
    }

    return true;
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

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    //std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    MyNode init_node(0, problem.init_node);

    std::list<MyNode> open_nodes;
    std::list<MyNode> closed_nodes;

    open_nodes.push_back(init_node);

    //loop till the goal node is found
    bool goal_found = false;
    int iter_count = 0;
    while(goal_found == false){
        
        //determine the lowest cost open node
        double lowest_cost = -1;
        MyNode *lowest_node;
        int lowest_count = 0;

        int counter = 0;

        if(open_nodes.size() == 0){
            break;
        }

        for(auto it = open_nodes.begin(); it != open_nodes.end(); ++it){
            if(lowest_cost < 0 || lowest_cost > it->calculateCost()){
                lowest_cost = it->calculateCost();
                lowest_node = &(*it);
                lowest_count = counter;
            }

            counter++;
        }

        if(lowest_node->isSame(problem.goal_node)){
            result.node_path = lowest_node->getPath();
            result.success = true;
            result.path_cost = lowest_node->calculateCost();
            //printf("Path Cost: %f\n", result.path_cost);
            result.print();

            //printf("Iterations: %d", iter_count);

            return result;
        }

        //move the node from open to closed4
        auto open_it = open_nodes.begin();
        std::advance(open_it, lowest_count);
        closed_nodes.push_back(*lowest_node);
        open_nodes.erase(open_it);

        lowest_node = &(closed_nodes.back());

        //open nodes one by one
        counter = 0;
        for(auto it = problem.graph->children(lowest_node->location).begin(); it != problem.graph->children(lowest_node->location).end(); ++it){
            
            //create children
            MyNode newNode(problem.graph->outgoingEdges(lowest_node->location).at(counter), *it);
            newNode.parent = lowest_node;

            //check to make sure the node hasn't been opened yet
            bool added = false;
            for(auto it2 = open_nodes.begin(); it2 != open_nodes.end(); it2++){
                if(it2->isSame(*it)){
                    if(it2->calculateCost() > lowest_node->calculateCost() + problem.graph->outgoingEdges(lowest_node->location).at(counter)){
                        it2->cost_from_parent = problem.graph->outgoingEdges(lowest_node->location).at(counter);
                        it2->parent = lowest_node;
                    }

                    added = true;
                    break;
                }
            }

            for(auto it2 = closed_nodes.begin(); it2 != closed_nodes.end(); it2++){
                if(it2->isSame(*it)){
                    // it2->cost_from_parent = problem.graph->outgoingEdges(lowest_node->location).at(counter);
                    // it2->parent = lowest_node;
                    added = true;
                    break;
                }
            }

            if(!added){
                open_nodes.push_back(newNode);
            }

            counter++;
        }

        iter_count++;

    }

    //back calculate the path
    result.success = false;

    //result.print();
    return result;
}
