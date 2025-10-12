#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"

#include "segment.h"

#include "ManipulatorSkeleton.h"

#define FLOAT_TOL .000001

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// Derive the amp::GridCSpace2D class and override the missing method

class Cell{
    public:
        Cell(Eigen::Vector2i Location){
            location = Location;
        }

        std::vector<Eigen::Vector2i> get_neighbors(bool looped, int cells){
            std::vector<Eigen::Vector2i> neighbors;

            if(!looped){
                neighbors.push_back(location + Eigen::Vector2i(1, 0));
                neighbors.push_back(location + Eigen::Vector2i(0, 1));
                neighbors.push_back(location + Eigen::Vector2i(-1, 0));
                neighbors.push_back(location + Eigen::Vector2i(0, -1));
            } else {
                //allow the neighboring cells to be looped across the edge of the cspace
                neighbors.push_back(location + Eigen::Vector2i(1, 0));
                neighbors.back()[0] = neighbors.back()[0] % cells;
                neighbors.push_back(location + Eigen::Vector2i(0, 1));
                neighbors.back()[1] = neighbors.back()[1] % cells;
                neighbors.push_back(location + Eigen::Vector2i(-1, 0));
                neighbors.back()[0] = neighbors.back()[0] % cells;
                neighbors.push_back(location + Eigen::Vector2i(0, -1));
                neighbors.back()[1] = neighbors.back()[1] % cells;
            }


            return neighbors;
        }

    Eigen::Vector2i location;

    Cell* parent;
};

class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {
            cells_per_dim = x0_cells;
        }

        std::size_t cells_per_dim;

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
        virtual Eigen::Vector2d getCellCenter(int x0, int x1) const override;

};


// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

        bool findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter);

    private:
        std::size_t m_cells_per_dim;
};

//////////////////////////////////////////////////////////////

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;
        
        bool findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter);


    private:
        std::size_t m_cells_per_dim;
};

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {

    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;

        bool allow_looping = false;



};

