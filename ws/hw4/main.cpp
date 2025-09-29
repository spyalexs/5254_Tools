// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;

    amp::ManipulatorState test_state = manipulator.getConfigurationFromIK(Eigen::Vector2d(2.0, 0));

    // You can visualize your manipulator given an angle state like so:
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    test_state << 3.1415 / 6.0, 3.1415 / 3.0, 3.1415 * 7.0 / 4.0;
    Visualizer::makeFigure(manipulator, test_state); 

    //Create the collision space constructor
    std::size_t n_cells = 500;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);
    Visualizer::makeFigure(HW4::getEx3Workspace1());

        // Create the collision space using a given manipulator and environment
    cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace2());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);
    Visualizer::makeFigure(HW4::getEx3Workspace2());


        // Create the collision space using a given manipulator and environment
    cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);
    Visualizer::makeFigure(HW4::getEx3Workspace3());

    Visualizer::saveFigures();

    // // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "alex.schuler@colorado.edu", argc, argv);
    return 0;
}