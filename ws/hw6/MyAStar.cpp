#include "MyAStar.h"

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
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
            result.print();

            printf("Iterations: %d", iter_count);

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

    result.print();
    return result;
}
