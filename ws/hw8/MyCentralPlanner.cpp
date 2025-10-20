#include "MyMultiAgentPlanners.h"

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D &problem)
{


    // printf("Nodes size: %d\n", nodes.size());
    printf("Path Agents: %ld\n", problem.numAgents());

    std::vector<double> agent_sizes;
    for (int i = 0; i < problem.numAgents(); i++)
    {
        agent_sizes.push_back(problem.agent_properties.at(i).radius);
    }

    PointCollisionChecker collision_checker(problem.obstacles, agent_sizes);

    amp::MultiAgentPath2D path;

    auto start = std::chrono::high_resolution_clock::now();

    nodes.clear();
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

    // create the initial, goal, upper and lower states
    Eigen::VectorXd init_state(problem.numAgents() * 2);
    Eigen::VectorXd goal_state(problem.numAgents() * 2);
    Eigen::VectorXd upper_bound(problem.numAgents() * 2);
    Eigen::VectorXd lower_bound(problem.numAgents() * 2);
    for (int i = 0; i < problem.numAgents(); i++)
    {
        init_state[i * 2] = problem.agent_properties.at(i).q_init[0];
        init_state[i * 2 + 1] = problem.agent_properties.at(i).q_init[1];
        goal_state[i * 2] = problem.agent_properties.at(i).q_goal[0];
        goal_state[i * 2 + 1] = problem.agent_properties.at(i).q_goal[1];

        lower_bound[i * 2] = problem.x_min;
        lower_bound[i * 2 + 1] = problem.y_min;
        upper_bound[i * 2] = problem.x_max;
        upper_bound[i * 2 + 1] = problem.y_max;
    }

    printf("Going to %f %f\n", goal_state[0], goal_state[1]);
    printf("Going from %f %f\n", init_state[0], init_state[1]);

    // seed the random numbers
    int dim = upper_bound.size();

    if (upper_bound.size() != lower_bound.size())
    {
        printf("Invalid bound vectors, vectors do not have same dimension\n");
    }

    Eigen::VectorXd d_bound = upper_bound - lower_bound;


    int node_counter = 1;
    bool path_found = false;
    while(!path_found){
        nodes.clear();
        nodes[0] = init_state;
        node_counter = 1;

        graphPtr = std::make_shared<amp::Graph<double>>();

        printf("Resetting\n");

        while (nodes.size() < 10000)
        {
            bool valid_sample = false;
            Eigen::VectorXd sample(dim);

            while (valid_sample == false)
            {
                // determine if this is a goal sample
                if (double(rand()) / RAND_MAX < GOAL_BIAS)
                {
                    sample = goal_state;
                    break;
                }

                // generate a random sample
                for (int i = 0; i < dim; i++)
                {
                    sample[i] = (d_bound[i] * rand()) / RAND_MAX;
                }

                // add lower bound
                sample = sample + lower_bound;

                // printf("Sampling %f %f \n", sample[0], sample[1]);

                if (!collision_checker.checkXdPointCollision(sample))
                {
                    valid_sample = true;
                }
            }

            // attempt to connect the nodes
            amp::Node nearest_node = 0;
            double lowest_distance = -1;
            for (int i = 0; i < nodes.size() - 1; i++)
            {
                if (get_distance(nodes[i], sample) < lowest_distance || lowest_distance < 0)
                {
                    // within range to connect

                    lowest_distance = get_distance(nodes[i], sample);
                    nearest_node = i;
                }
            }

            // propagate as far as possible
            Eigen::VectorXd direction = sample - nodes[nearest_node];
            direction.normalize();
            direction = direction * CONNECT_STEP;
            Eigen::VectorXd branch_end = nodes[nearest_node];
            int iterated = 0;
            for (int i = 1; i * CONNECT_STEP < MAX_CONNECT; i++)
            {
                if (collision_checker.checkXdSegmentCollision(nodes[nearest_node], nodes[nearest_node] + direction * i))
                {
                    break;
                }

                iterated++;
                branch_end = nodes[nearest_node] + direction * i;
            }

            if (iterated > 0)
            {
                // add the node to the graph
                nodes[node_counter] = branch_end;

                graphPtr->connect(nearest_node, node_counter, get_distance(nodes[nearest_node], nodes[node_counter]));
                graphPtr->connect(node_counter, nearest_node, get_distance(nodes[nearest_node], nodes[node_counter]));

                // determine if it is time to end
                // printf("Distance: %f\n", get_distance(nodes[node_counter], goal_state));
                // printf("Adding %f %f\n", nodes[node_counter][0], nodes[node_counter][1]);
                if (get_distance(nodes[node_counter], goal_state) < GOAL_CONNECT)
                {
                    nodes[node_counter + 1] = goal_state;

                    graphPtr->connect(node_counter, node_counter + 1, get_distance(nodes[node_counter], nodes[node_counter + 1]));
                    graphPtr->connect(node_counter + 1, node_counter, get_distance(nodes[node_counter], nodes[node_counter + 1]));

                    path_found = true;

                    break;
                }

                node_counter++;
            }

            // printf("Nodes size: %d\n", nodes.size());
        }
    }

    // setup astar
    Heuristic heuristic(node_counter + 1, nodes);
    MyAStarAlgo astar;
    amp::ShortestPathProblem graph_problem;
    graph_problem.goal_node = node_counter + 1;
    graph_problem.init_node = 0;
    graph_problem.graph = graphPtr;

    // calculate the best route
    MyAStarAlgo::GraphSearchResult astar_out = astar.search(graph_problem, heuristic);

    cost = astar_out.path_cost;
    valid = astar_out.success;
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    elapsed_time = duration.count();

    // add paths
    for (int i = 0; i < problem.numAgents(); i++)
    {
        amp::Path2D path_unit;
        path.agent_paths.push_back(path_unit);
    }

    // connect the node numbers to the node locations
    for (auto it = astar_out.node_path.begin(); it != astar_out.node_path.end(); ++it)
    {
        Eigen::VectorXd pos = nodes[*it];

        for (int j = 0; j < path.numAgents(); j++)
        {
            Eigen::Vector2d agent_pos(pos[2 * j], pos[2 * j + 1]);

            path.agent_paths.at(j).waypoints.push_back(agent_pos);

            // printf("(%f, %f)   ", pos[2 * j], pos[2 * j + 1]);
        }

        // printf("\n");
    }

    // printf("Path Agents: %ld\n", path.numAgents());

    path.valid = true;
    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D &problem)
{


    // printf("Nodes size: %d\n", nodes.size());
    printf("Path Agents: %ld\n", problem.numAgents());

    std::vector<double> agent_sizes;
    std::vector<int> step;


    amp::MultiAgentPath2D path;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < problem.numAgents(); i++)
    {
        amp::Path2D path_unit;
        path.agent_paths.push_back(path_unit);
    }

    std::vector<int> order_vector(problem.numAgents());
    std::iota(order_vector.begin(), order_vector.end(), 0);

    bool valid_path = false;
    while(!valid_path){


        path.agent_paths.clear();
        agent_sizes.clear();
        for (int i = 0; i < problem.numAgents(); i++)
        {
            amp::Path2D path_unit;
            path.agent_paths.push_back(path_unit);
        }

        //next three lines are courtesy of goolge gemni
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine rng(seed);
        std::shuffle(order_vector.begin(), order_vector.end(), rng);

        amp::MultiAgentProblem2D problem_temp = problem;

        for(int i = 0; i < problem.numAgents(); i++){
            printf("%d\n", order_vector.at(i));

            problem_temp.agent_properties.at(i) = problem.agent_properties.at(order_vector.at(i));
        }

        for (int i = 0; i < problem.numAgents(); i++){
            agent_sizes.push_back(problem_temp.agent_properties.at(i).radius);
            step.push_back(0);
        }

        PointCollisionChecker collision_checker(problem.obstacles, agent_sizes);


        for (int k = 0; k < problem.numAgents(); k++){

            nodes.clear();
            std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();

            // create the initial, goal, upper and lower states
            Eigen::VectorXd init_state = problem_temp.agent_properties.at(k).q_init;
            Eigen::VectorXd goal_state = problem_temp.agent_properties.at(k).q_goal;
            Eigen::Vector2d upper_bound(problem_temp.x_max, problem_temp.y_max);
            Eigen::Vector2d lower_bound(problem_temp.x_min, problem_temp.y_min);

            printf("Going to %f %f\n", goal_state[0], goal_state[1]);
            printf("Going from %f %f\n", init_state[0], init_state[1]);

            // seed the random numbers
            int dim = upper_bound.size();

            if (upper_bound.size() != lower_bound.size())
            {
                printf("Invalid bound vectors, vectors do not have same dimension\n");
            }

            Eigen::VectorXd d_bound = upper_bound - lower_bound;

            nodes[0] = init_state;

            int node_counter = 1;
            while (nodes.size() < 10000)
            {
                bool valid_sample = false;
                Eigen::VectorXd sample(dim);

                while (valid_sample == false)
                {
                    // determine if this is a goal sample
                    if (double(rand()) / RAND_MAX < GOAL_BIAS)
                    {
                        sample = goal_state;
                        break;
                    }

                    // generate a random sample
                    for (int i = 0; i < dim; i++)
                    {
                        sample[i] = (d_bound[i] * rand()) / RAND_MAX;
                    }

                    // add lower bound
                    sample = sample + lower_bound;

                    // printf("Sampling %f %f \n", sample[0], sample[1]);

                    if (collision_checker.check(sample, k))
                    {
                        valid_sample = true;
                    }
                }

                // attempt to connect the nodes
                amp::Node nearest_node = 0;
                double lowest_distance = -1;
                for (int i = 0; i < nodes.size() - 1; i++)
                {
                    if (get_distance(nodes[i], sample) < lowest_distance || lowest_distance < 0)
                    {
                        // within range to connect

                        lowest_distance = get_distance(nodes[i], sample);
                        nearest_node = i;
                    }
                }

                // propagate as far as possible
                Eigen::VectorXd direction = sample - nodes[nearest_node];
                direction.normalize();
                direction = direction * CONNECT_STEP;
                Eigen::VectorXd branch_end = nodes[nearest_node];
                bool iterated = false;
                for (int i = 1; i * CONNECT_STEP < MAX_CONNECT; i++)
                {
                    if (collision_checker.check_segment(nodes[nearest_node], nodes[nearest_node] + direction * i, k))
                    {
                        break;
                    }

                    iterated = true;
                    branch_end = nodes[nearest_node] + direction * i;
                }

                if (iterated)
                {
                    // add the node to the graph
                    nodes[node_counter] = branch_end;

                    graphPtr->connect(nearest_node, node_counter, get_distance(nodes[nearest_node], nodes[node_counter]));
                    graphPtr->connect(node_counter, nearest_node, get_distance(nodes[nearest_node], nodes[node_counter]));

                    // determine if it is time to end
                    // printf("Distance: %f\n", get_distance(nodes[node_counter], goal_state));
                    // printf("Adding %f %f\n", nodes[node_counter][0], nodes[node_counter][1]);
                    if (get_distance(nodes[node_counter], goal_state) < GOAL_CONNECT)
                    {
                        nodes[node_counter + 1] = goal_state;

                        graphPtr->connect(node_counter, node_counter + 1, get_distance(nodes[node_counter], nodes[node_counter + 1]));
                        graphPtr->connect(node_counter + 1, node_counter, get_distance(nodes[node_counter], nodes[node_counter + 1]));

                        break;
                    }

                    node_counter++;
                }

                // printf("Nodes size: %d\n", nodes.size());
            }

            // setup astar
            Heuristic heuristic(node_counter + 1, nodes);
            MyAStarAlgo astar;
            amp::ShortestPathProblem graph_problem;
            graph_problem.goal_node = node_counter + 1;
            graph_problem.init_node = 0;
            graph_problem.graph = graphPtr;

            // calculate the best route
            MyAStarAlgo::GraphSearchResult astar_out = astar.search(graph_problem, heuristic);

            cost = astar_out.path_cost;
            valid = astar_out.success;
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            elapsed_time = duration.count();

            // add paths


            // connect the node numbers to the node locations
            for (auto it = astar_out.node_path.begin(); it != astar_out.node_path.end(); ++it)
            {
                Eigen::VectorXd pos = nodes[*it];

                Eigen::Vector2d agent_pos(pos[0], pos[1]);

                path.agent_paths.at(k).waypoints.push_back(agent_pos);

                for(int j = k + 1; j < path.numAgents(); j++){
                    path.agent_paths.at(j).waypoints.push_back(problem_temp.agent_properties.at(j).q_init);
                }

                // printf("(%f, %f)   ", pos[2 * j], pos[2 * j + 1]);

                // printf("\n");
            }
        }

        // bool collision = true;
        // while(collision){

        //     collision = false;

        //     bool all_goal = false;
        //     while(!all_goal){
        //         std::vector<Eigen::Vector2d> points_1;
        //         std::vector<Eigen::Vector2d> points_2;
        //         for(int i = 0; i < path.numAgents(); i++){
        //             points_1.push_back(path.agent_paths.at(i).waypoints.at(step.at(i)));
        //             points_2.push_back(path.agent_paths.at(i).waypoints.at(step.at(i) + 1));
        //             printf(" %d ", step.at(i));
        //         }

        //         printf("\n");

        //         int agent_delay = collision_checker.check_agents2(points_1, points_2);
        //         //printf("%d\n", agent_delay);

        //         if(agent_delay >= 0){
        //             std::reverse(path.agent_paths.at(agent_delay).waypoints.begin(), path.agent_paths.at(agent_delay).waypoints.end());
        //             path.agent_paths.at(agent_delay).waypoints.push_back(path.agent_paths.at(agent_delay).waypoints.at(path.agent_paths.at(agent_delay).waypoints.size() - 1));
        //             std::reverse(path.agent_paths.at(agent_delay).waypoints.begin(), path.agent_paths.at(agent_delay).waypoints.end());

        //             collision = true;
        //             break;
        //         }

        //         all_goal = true;
        //         for(int j = 0; j < problem.numAgents(); j++){
        //             if(step.at(j) + 2 != path.agent_paths.at(j).waypoints.size()){
        //                 step.at(j)++;
        //                 all_goal = false;

        //             }
        //         }
        //     }

        //     for(int j = 0; j < problem.numAgents(); j++){
        //             step.at(j) = 0;
        //     }

        // }

        //remove path delays to optimize
        for(int i = 1; i < problem_temp.numAgents(); i++)
        {
            bool collision = false;
            amp::Path2D temp = path.agent_paths.at(i);
            amp::Path2D temp2 = path.agent_paths.at(i);
            while(!collision){
                temp2 = temp;
                temp = path.agent_paths.at(i);

                //remove the first delay
                if(path.agent_paths.at(i).waypoints.at(1) != problem_temp.agent_properties.at(i).q_init){
                    break;
                }
                
                printf("Here\n");
                std::reverse(path.agent_paths.at(i).waypoints.begin(), path.agent_paths.at(i).waypoints.end());
                path.agent_paths.at(i).waypoints.pop_back();
                std::reverse(path.agent_paths.at(i).waypoints.begin(), path.agent_paths.at(i).waypoints.end());

                bool all_goal = false;
                while(!all_goal){
                    std::vector<Eigen::Vector2d> points_1;
                    std::vector<Eigen::Vector2d> points_2;
                    for(int i = 0; i < path.numAgents(); i++){
                        points_1.push_back(path.agent_paths.at(i).waypoints.at(step.at(i)));
                        points_2.push_back(path.agent_paths.at(i).waypoints.at(step.at(i) + 1));
                        printf(" %d ", step.at(i));
                    }

                    printf("\n");

                    int agent_delay = collision_checker.check_agents(points_1, points_2);
                    printf("%d\n", agent_delay);

                    if(agent_delay >= 0){
                        printf("Restoring path\n");

                        path.agent_paths.at(i) = temp2;
                        collision = true;
                        break;
                    }

                    all_goal = true;
                    for(int j = 0; j < problem_temp.numAgents(); j++){
                        if(step.at(j) + 2 != path.agent_paths.at(j).waypoints.size()){
                            step.at(j)++;
                            all_goal = false;

                        }
                    }

                    printf("All goal %d\n", all_goal);
                    if(all_goal){
                        valid_path = true;
                    }
                }

                for(int j = 0; j < problem_temp.numAgents(); j++){
                    step.at(j) = 0;
                }

            }
            
        }

        if(problem.numAgents() == 1){
            valid_path = true;
        }


    }

    amp::MultiAgentPath2D path_temp = path;
    for(int i = 0; i < problem.numAgents(); i++){
        path.agent_paths.at(order_vector.at(i)) = path_temp.agent_paths.at(i);
    }

    //printf("Path Agents: %ld\n", path.numAgents());

    path.valid = true;
    return path;
}

double MyCentralPlanner::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    if (v1.size() != v2.size())
    {
        printf("Invalid distance calculation, vectors do not have same dimension %ld %ld \n", v1.size(), v2.size());

        return 0.0;
    }

    double sum = 0;
    for (int i = 0; i < v1.size(); i++)
    {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}

double MyDecentralPlanner::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    if (v1.size() != v2.size())
    {
        printf("Invalid distance calculation, vectors do not have same dimension %ld %ld \n", v1.size(), v2.size());

        return 0.0;
    }

    double sum = 0;
    for (int i = 0; i < v1.size(); i++)
    {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}

bool PointCollisionChecker::checkXdPointCollision(Eigen::VectorXd test_v)
{
    if (test_v.rows() % 2 != 0)
    {
        printf("Test Vector should always be even! Got dim of: \n", test_v.rows());
    }

    bool collision;
    for (int i = 0; i < test_v.rows(); i += 2)
    {
        Eigen::Vector2d point1(test_v[i], test_v[i + 1]);
        if (!check(point1, i / 2))
        {
            // a collision is deteceted
            // printf("Here\n");
            return true;
        }

        // test for robot to robot collisions
        for (int j = i + 2; j < test_v.rows(); j += 2)
        {
            Eigen::Vector2d point2(test_v[j], test_v[j + 1]);

            if ((point1 - point2).norm() < disk_radius.at(i / 2) + disk_radius.at(j / 2))
            {
                // printf("Here2\n");

                return true;
            }
        }
    }

    return false;
}

int PointCollisionChecker::check_agents(std::vector<Eigen::Vector2d> p1, std::vector<Eigen::Vector2d> p2)
{
    for (int i = 0; i < p1.size(); i++)
    {
        Eigen::Vector2d point1 = p1.at(i);
        Eigen::Vector2d point2 = p2.at(i);

        // check for robot to robot collision
        for (int j = i + 1; j < p2.size(); j++)
        {
            Eigen::Vector2d point12 = p1.at(j);
            Eigen::Vector2d point22 = p2.at(j);

            if ((point12 - point1).norm() < 1.01*(disk_radius.at(i) + disk_radius.at(j)))
            {
                // printf("Here7\n");

                return j;
            }

            if ((point22 - point2).norm() < 1.01*(disk_radius.at(i) + disk_radius.at(j)))
            {
                // printf("Here8\n");

                return j;
            }

            // ensure these movemovent never collide
            Eigen::Vector2d dummy;
            if (findSegmentIntersection(point1, point2, point12, point22, &dummy))
            {
                // printf("Here9\n");

                return j;
            }
        }
    }

    return -1;
}

int PointCollisionChecker::check_agents2(std::vector<Eigen::Vector2d> p1, std::vector<Eigen::Vector2d> p2)
{
    for (int i = 0; i < p1.size(); i++)
    {
        Eigen::Vector2d point1 = p1.at(i);
        Eigen::Vector2d point2 = p2.at(i);

        // check for robot to robot collision
        for (int j = i + 1; j < p2.size(); j++)
        {
            Eigen::Vector2d point12 = p1.at(j);
            Eigen::Vector2d point22 = p2.at(j);

            if ((point12 - point1).norm() < disk_radius.at(i / 2) + disk_radius.at(j / 2))
            {
                // printf("Here7\n");

                return i;
            }

            if ((point22 - point2).norm() < disk_radius.at(i / 2) + disk_radius.at(j / 2))
            {
                // printf("Here8\n");

                return i;
            }

            // ensure these movemovent never collide
            Eigen::Vector2d dummy;
            if (findSegmentIntersection(point1, point2, point12, point22, &dummy))
            {
                // printf("Here9\n");

                return i;
            }
        }
    }

    return -1;
}

bool PointCollisionChecker::checkXdSegmentCollision(Eigen::VectorXd test_v1, Eigen::VectorXd test_v2)
{
    if (test_v1.rows() % 2 != 0)
    {
        printf("Test Vector should always be even! Got dim of: %d \n", test_v1.rows());
    }

    bool collision;
    for (int i = 0; i < test_v1.rows(); i += 2)
    {
        Eigen::Vector2d point1(test_v1[i], test_v1[i + 1]);
        Eigen::Vector2d point2(test_v2[i], test_v2[i + 1]);

        if (check_segment(point1, point2, i / 2))
        {
            // a collision is deteceted
            // printf("Here6\n");
            return true;
        }

        // check for robot to robot collision
        for (int j = i + 2; j < test_v1.rows(); j += 2)
        {
            Eigen::Vector2d point12(test_v1[j], test_v1[j + 1]);
            Eigen::Vector2d point22(test_v2[j], test_v2[j + 1]);

            if ((point12 - point1).norm() < disk_radius.at(i / 2) + disk_radius.at(j / 2))
            {
                // printf("Here7\n");

                return true;
            }

            if ((point22 - point2).norm() < disk_radius.at(i / 2) + disk_radius.at(j / 2))
            {
                // printf("Here8\n");

                return true;
            }

            // ensure these movemovent never collide
            Eigen::Vector2d dummy;
            if (findSegmentIntersection(point1, point2, point12, point22, &dummy))
            {
                // printf("Here9\n");

                return true;
            }
        }
    }

    return false;
}

PointCollisionChecker::PointCollisionChecker(std::vector<amp::Obstacle2D> obstacles, std::vector<double> r)
{
    int index = 0;

    disk_radius = r;

    // define a rotation matrix that rotates by positive 90 degrees
    Eigen::Matrix<double, 2, 2> rot_mat;
    rot_mat << 0.0, -1.0, 1.0, 0.0;

    for (int k = 0; k < r.size(); k++)
    {
        std::vector<Segment> disk_segments;
        std::vector<std::vector<Eigen::Vector2d>> disk_vertices;
        std::vector<Eigen::Vector2d> disk_base_vertices;

        for (int i = 0; i < obstacles.size(); i++)
        {

            // going to expand the obstalces by r to handle the rigid disk of the robot

            std::vector<Eigen::Vector2d> expanded_vertices;
            for (int j = 0; j < obstacles.at(i).verticesCW().size(); j++)
            {
                Segment s;
                s.v1 = obstacles.at(i).verticesCW().at(j);
                if (j + 1 < obstacles.at(i).verticesCW().size())
                {
                    s.v2 = obstacles.at(i).verticesCW().at(j + 1);
                }
                else
                {
                    s.v2 = obstacles.at(i).verticesCW().at(0);
                }

                // rotate the segment by positive 90 degrees
                Eigen::Vector2d diff_segment = s.v2 - s.v1;
                diff_segment.normalize();
                diff_segment = diff_segment * r.at(k);
                Eigen::Vector2d diff_segment_rotate = rot_mat * diff_segment;

                // apply the diff segment to each vertex
                expanded_vertices.push_back(s.v1 + diff_segment_rotate);
                expanded_vertices.push_back(s.v2 + diff_segment_rotate);

                disk_base_vertices.push_back(obstacles.at(i).verticesCW().at(j));
            }

            // now calculate the obstalces
            std::vector<Eigen::Vector2d> obs;
            for (int j = 0; j < expanded_vertices.size(); j++)
            {
                Segment s;
                s.v1 = expanded_vertices.at(j);
                if (j + 1 < expanded_vertices.size())
                {
                    s.v2 = expanded_vertices.at(j + 1);
                }
                else
                {
                    s.v2 = expanded_vertices.at(0);
                }

                s.obstacle_index = i;
                s.intra_index = j;
                s.index = index;

                disk_segments.push_back(s);

                index++;

                obs.push_back(expanded_vertices.at(j));
            }
            disk_vertices.push_back(obs);
        }

        base_vertices_cw.push_back(disk_base_vertices);
        vertices_cw.push_back(disk_vertices);
        segments.push_back(disk_segments);
    }
}

bool PointCollisionChecker::check(Eigen::Vector2d point, int disk)
{

    for (int i = 0; i < vertices_cw.at(disk).size(); i++)
    {

        bool inside_obstacle = true;
        double previous_angle = atan2(vertices_cw.at(disk).at(i).back()[1] - point[1], vertices_cw.at(disk).at(i).back()[0] - point[0]);
        for (int j = 0; j < vertices_cw.at(disk).at(i).size(); j++)
        {
            double current_angle = atan2(vertices_cw.at(disk).at(i).at(j)[1] - point[1], vertices_cw.at(disk).at(i).at(j)[0] - point[0]);

            if (fmod((previous_angle - current_angle) + 6.28, 6.28) > 3.14)
            {
                inside_obstacle = false;
                break;
            }

            previous_angle = current_angle;
        }

        // also need to check to see if the disk is within r of any of the unmodified vertices
        for (int j = 0; j < base_vertices_cw.at(disk).size(); j++)
        {

            // disk intersects with a vertex
            if ((base_vertices_cw.at(disk).at(j) - point).norm() < disk_radius.at(disk))
            {
                return false;
            }
        }

        if (inside_obstacle)
        {
            return false;
        }
    }

    return true;
}

bool PointCollisionChecker::check_segment(Eigen::Vector2d p1, Eigen::Vector2d p2, int disk)
{
    Eigen::Vector2d dummy;

    Eigen::Matrix<double, 2, 2> rot_mat;
    rot_mat << 0.0, -1.0, 1.0, 0.0;

    // check to determine if the movement segment intersects with the disk - respects expanded vertices
    bool in_collision = false;
    for (int i = 0; i < segments.at(disk).size(); i++)
    {
        if (findSegmentIntersection(p1, p2, segments.at(disk).at(i).v1, segments.at(disk).at(i).v2, &dummy))
        {
            in_collision = true;
            break;
        }
    }

    // must also check original vertices
    for (int j = 0; j < base_vertices_cw.at(disk).size(); j++)
    {

        // closest point on a circle to a segment is the point directly perpendicular to the segment
        Eigen::Vector2d diff_segment = p1 - p2;
        diff_segment.normalize();
        diff_segment = diff_segment * disk_radius.at(disk);
        Eigen::Vector2d diff_segment_rotate = rot_mat * diff_segment;

        // line intersects edge of the circle
        Eigen::Vector2d v1 = base_vertices_cw.at(disk).at(j) + diff_segment_rotate;
        Eigen::Vector2d v2 = base_vertices_cw.at(disk).at(j) - diff_segment_rotate;
        if (findSegmentIntersection(p1, p2, v1, v2, &dummy))
        {
            in_collision = true;
            break;
        }

        // also need to consider the case in which the segment terminates inside of vertex
        if ((base_vertices_cw.at(disk).at(j) - p1).norm() < disk_radius.at(disk))
        {
            in_collision = true;
            break;
        }

        if ((base_vertices_cw.at(disk).at(j) - p1).norm() < disk_radius.at(disk))
        {
            in_collision = true;
            break;
        }
    }

    return in_collision;
}

// used from previous homework
// this function was ChatGPT inspired but I've heavily edited the code
// Prompt: write a function that determines the point of intersection of two lines in cpp. The vertexes of each thing will be passed as eigen::vector2d
bool PointCollisionChecker::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter)
{
    // Representing the two lines as follows:
    // Line 1: p1 + t * (p2 - p1)
    // Line 2: q1 + u * (q2 - q1)
    // We need to find the values of t and u where the lines intersect.

    Eigen::Vector2d dir1 = p2 - p1; // Direction vector of line 1
    Eigen::Vector2d dir2 = q2 - q1; // Direction vector of line 2

    // We solve for t and u using the determinant method.
    double denom = dir1.x() * dir2.y() - dir1.y() * dir2.x();

    // gpt moment - tsk tsk
    if (fabs(dir2.y()) < FLOAT_TOL && fabs(dir1.x()) < FLOAT_TOL)
    {

        inter->x() = p1.x();
        inter->y() = q1.y();

        bool within_y = std::min(p1.y(), p2.y()) <= inter->y() && std::max(p1.y(), p2.y()) >= inter->y();
        bool within_x = std::min(q1.x(), q2.x()) <= inter->x() && std::max(q1.x(), q2.x()) >= inter->x();

        return within_x && within_y;
    }
    else if (fabs(dir2.x()) < FLOAT_TOL && fabs(dir1.y()) < FLOAT_TOL)
    {
        inter->x() = q1.x();
        inter->y() = p1.y();

        bool within_y = std::min(q1.y(), q2.y()) <= inter->y() && std::max(q1.y(), q2.y()) >= inter->y();
        bool within_x = std::min(p1.x(), p2.x()) <= inter->x() && std::max(p1.x(), p2.x()) >= inter->x();

        return within_x && within_y;
    }

    // If denom is 0, the lines are parallel or coincident, so there's no single intersection point
    if (denom == 0)
    {
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
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem &problem, const amp::SearchHeuristic &heuristic)
{
    // std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // initialize the results object

    MyNode init_node(0, problem.init_node);

    std::list<MyNode> open_nodes;
    std::list<MyNode> closed_nodes;

    open_nodes.push_back(init_node);

    // loop till the goal node is found
    bool goal_found = false;
    int iter_count = 0;
    while (goal_found == false)
    {

        // determine the lowest cost open node
        double lowest_cost = -1;
        MyNode *lowest_node;
        int lowest_count = 0;

        int counter = 0;

        if (open_nodes.size() == 0)
        {
            break;
        }

        for (auto it = open_nodes.begin(); it != open_nodes.end(); ++it)
        {
            if (lowest_cost < 0 || lowest_cost > it->calculateCost())
            {
                lowest_cost = it->calculateCost();
                lowest_node = &(*it);
                lowest_count = counter;
            }

            counter++;
        }

        if (lowest_node->isSame(problem.goal_node))
        {
            result.node_path = lowest_node->getPath();
            result.success = true;
            result.path_cost = lowest_node->calculateCost();
            // printf("Path Cost: %f\n", result.path_cost);
            result.print();

            // printf("Iterations: %d", iter_count);

            return result;
        }

        // move the node from open to closed4
        auto open_it = open_nodes.begin();
        std::advance(open_it, lowest_count);
        closed_nodes.push_back(*lowest_node);
        open_nodes.erase(open_it);

        lowest_node = &(closed_nodes.back());

        // open nodes one by one
        counter = 0;
        for (auto it = problem.graph->children(lowest_node->location).begin(); it != problem.graph->children(lowest_node->location).end(); ++it)
        {

            // create children
            MyNode newNode(problem.graph->outgoingEdges(lowest_node->location).at(counter), *it);
            newNode.parent = lowest_node;

            // check to make sure the node hasn't been opened yet
            bool added = false;
            for (auto it2 = open_nodes.begin(); it2 != open_nodes.end(); it2++)
            {
                if (it2->isSame(*it))
                {
                    if (it2->calculateCost() > lowest_node->calculateCost() + problem.graph->outgoingEdges(lowest_node->location).at(counter))
                    {
                        it2->cost_from_parent = problem.graph->outgoingEdges(lowest_node->location).at(counter);
                        it2->parent = lowest_node;
                    }

                    added = true;
                    break;
                }
            }

            for (auto it2 = closed_nodes.begin(); it2 != closed_nodes.end(); it2++)
            {
                if (it2->isSame(*it))
                {
                    // it2->cost_from_parent = problem.graph->outgoingEdges(lowest_node->location).at(counter);
                    // it2->parent = lowest_node;
                    added = true;
                    break;
                }
            }

            if (!added)
            {
                open_nodes.push_back(newNode);
            }

            counter++;
        }

        iter_count++;
    }

    // back calculate the path
    result.success = false;

    // result.print();
    return result;
}

double Heuristic::get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2)
{
    if (v1.size() != v2.size())
    {
        printf("Invalid distance calculation, vectors do not have same dimension\n");

        return 0.0;
    }

    double sum = 0;
    for (int i = 0; i < v1.size(); i++)
    {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return sqrt(sum);
}
