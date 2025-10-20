#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW6.h"

class MyNode{

    public:
        MyNode(double Cost, amp::Node Location){
            cost_from_parent = Cost;
            location = Location;
        }

        bool isSame(amp::Node compare){
            if(compare == location){
                return true;
            }
            return false;
        }

        double calculateCost(){
            if(parent != NULL){
                return parent->calculateCost() + cost_from_parent;
            }

            return cost_from_parent;
        }

        std::list<amp::Node> getPath(){
            if(parent != NULL){
                //printf("Adding %d at a cost of %f \n", location, cost_from_parent);
                std::list<amp::Node> path = parent->getPath();
                path.push_back(location);
                return path;
            }

            std::list<amp::Node> path;
            path.push_back(location);
            //printf("Adding %d at a cost of %f \n", location, cost_from_parent);
            return path;
        }

        double cost_from_parent;
        MyNode* parent = NULL;
        amp::Node location;
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};

class Heuristic : public amp::SearchHeuristic{
    public:
        Heuristic(amp::Node goal_node, std::map<amp::Node, Eigen::VectorXd> node_map){
            goal = goal_node;
            map = node_map;
        }

        double operator()(amp::Node node) {
            Eigen::VectorXd v1 = map.find(node)->second;
            Eigen::VectorXd v2 = map.find(goal)->second;

            get_distance(v1, v2);

            return 0.0;
        }

        double get_distance(Eigen::VectorXd v1, Eigen::VectorXd v2);

        amp::Node goal;
        std::map<amp::Node, Eigen::VectorXd> map;


};
