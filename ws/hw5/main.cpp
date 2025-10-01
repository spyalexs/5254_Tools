// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"


// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    Path2D path;
    Problem2D prob = amp::HW2::getWorkspace1();
    //bool success = HW5::generateAndCheck(algo, path, prob);
    path = algo.plan(prob);

    Visualizer::makeFigure(prob, path);

    // Visualize your potential function
    Visualizer::makeFigure(*algo.func, prob, 30);
    Visualizer::saveFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("alex.schuler@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}