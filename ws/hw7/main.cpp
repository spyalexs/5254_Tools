// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MyAStar.h"
#include "MySamplingBasedPlanners.h"
#include "algorithm"

using namespace amp;

int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;
    
    // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    //graphPtr->print();

    // // Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace1();
    MyPRM prm;
    Path2D path;

    path = prm.plan(problem);

        printf("Here ---------------------------------- \n");

    //HW7::generateAndCheck(prm, path, problem);
    Visualizer::makeFigure(problem, prm.plan(problem), *(prm.graphPtr), nodes);

    printf("Here ---------------------------------- \n");


    srand(time(NULL));


    std::vector<double> connects;
    std::vector<int> node_counts;
    connects.push_back(1.0);
    connects.push_back(2.0);
        connects.push_back(1.0);
    connects.push_back(2.0);  
        connects.push_back(1.0);
    connects.push_back(2.0);      

    node_counts.push_back(200);
    node_counts.push_back(200);
    node_counts.push_back(500);
    node_counts.push_back(500);
    node_counts.push_back(1000);
    node_counts.push_back(1000);


    std::vector<double> avg_cost;
    std::vector<double> avg_time;
    std::vector<double> avg_valid;

    std::list<std::vector<double>> costs_2;
    std::list<std::vector<double>> time_2;
    std::list<std::vector<double>> valid_2;

    std::vector<std::string> labels;



    printf("Here ---------------------------------- \n");


    int benchmarks = 100;
    for(int i = 0; i < connects.size(); i++){

        avg_cost.push_back(0);
        avg_time.push_back(0);
        avg_valid.push_back(0);

        std::vector<double> costs;
        std::vector<double> times;
        std::vector<double> valids;

        // if(i == 0){
        //     problem = HW2::getWorkspace1();
        // } else if(i == 1){
        //     problem = HW2::getWorkspace2();
        // } else {
        //     problem = HW5::getWorkspace1();
        // }

        for(int j = 0; j < benchmarks; j++){
            prm = MyPRM();

            prm.max_connect = connects.at(i);
            prm.number_of_nodes = node_counts.at(i);

            Visualizer::saveFigures();

            prm.plan(problem);
            
            if(prm.valid){
                printf("Here\n");
            }

            if(prm.cost > 0){
                avg_cost.at(i) += prm.cost;
                avg_time.at(i) += double(prm.elapsed_time);
                avg_valid.at(i) += 1;

                costs.push_back(prm.cost);
                times.push_back(double(prm.elapsed_time));

                valids.push_back(1);
            } else{
                valids.push_back(0);
            }


            
        }

        avg_cost.at(i) = avg_cost.at(i) / avg_valid.at(i);
        avg_time.at(i) = avg_time.at(i) / avg_valid.at(i);

        std::sort(costs.begin(), costs.end());
        std::sort(times.begin(), times.end());

        costs_2.push_back(costs);
        time_2.push_back(times);
        valid_2.push_back(valids);

        labels.push_back("n: " + std::to_string(node_counts.at(i)) + " r: " + std::to_string(connects.at(i)));

        printf("\n\n---------------------------NODES: %d CONNECT %f---------------------------------\n", node_counts.at(i), connects.at(i));
        printf("Avg Cost: %f Avg Time %f Avg Valid %f\n", avg_cost.at(i), avg_time.at(i), avg_valid.at(i));
        //printf("Median %f Q1 %f, Q3 %f, min %f max %f\n", costs.at(50), costs.at(25), costs.at(75), costs.at(0), costs.at(99));
        //printf("Median %f Q1 %f, Q3 %f, min %f max %f\n", times.at(50), times.at(25), times.at(75), times.at(0), times.at(99));
        printf("---------------------------------------------------------------------------------------\n\n\n");

    }

    printf("Here\n");

    // labels.push_back("HW2: WS1");
    // labels.push_back("HW2: WS2");
    // labels.push_back("HW5: WS1");

    std::string title = "Costs HW2 WS1";
    std::string xlabel = "Set";
    std::string ylabel = "Cost";
    
    Visualizer::makeBoxPlot(costs_2, labels, title, xlabel, ylabel);
    
    title = "Times HW2 WS1";
    ylabel = "Time (us)";
    Visualizer::makeBoxPlot(time_2, labels, title, xlabel, ylabel);

        
    title = "Sucessfull Runs HW2 WS1";
    ylabel = "Is Valid";
    Visualizer::makeBoxPlot(valid_2, labels, title, xlabel, ylabel);


    // // Generate a random problem and test RRT
    // printf("\n\n\nRRT----------------------------------------------------------------------------\n\n\n\n");
    // MyRRT rrt;
    // //HW7::generateAndCheck(rrt, path, problem);
    // problem = HW5::getWorkspace1();
    // path = rrt.plan(problem);
    // Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    Visualizer::saveFigures();

    // Grade method
    //HW7::grade<MyPRM, MyRRT>("alex.schuler@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}