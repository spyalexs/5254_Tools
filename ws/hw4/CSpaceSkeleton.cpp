#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = std::size_t(round((x0) / (M_PI *2) * cells_per_dim)) % cells_per_dim; // x index of cell
    std::size_t cell_y = std::size_t(round((x1) / (M_PI *2) * cells_per_dim)) % cells_per_dim; // x index of cell

    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    //format everything into segments - taken from HW2
    std::vector<Segment> segments;
    int index = 0;
    for(int i = 0; i < env.obstacles.size(); i++){
        for(int j = 0; j < env.obstacles.at(i).verticesCW().size(); j++){
            Segment s;
            s.v1 = env.obstacles.at(i).verticesCW().at(j);
            if(j+1 < env.obstacles.at(i).verticesCW().size()){
                s.v2 = env.obstacles.at(i).verticesCW().at(j + 1);
            }else{
                s.v2 = env.obstacles.at(i).verticesCW().at(0);
            }

            s.obstacle_index = i;
            s.intra_index = j;
            s.index = index;

            segments.push_back(s);

            index++;
        }
    }

    int test = m_cells_per_dim;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
   for(int row = 0; row < m_cells_per_dim; row++){
        for(int col = 0; col < m_cells_per_dim; col++){
            //determine the center of the cell
            Eigen::Vector2d cell_center((double(col) / m_cells_per_dim) * (M_PI*2), (double(row) / m_cells_per_dim) * (M_PI*2));

            //printf("Testing %f %f \n", cell_center(0), cell_center(1));

            //determine where each point is
            amp::ManipulatorState test_state(2);
            test_state << cell_center(0), cell_center(1);

            Eigen::Vector2d p2 = manipulator.getJointLocation(test_state, 0);
            Eigen::Vector2d p0 = manipulator.getJointLocation(test_state, 2);
            Eigen::Vector2d p1 = manipulator.getJointLocation(test_state, 1);

            //initially no collisions
            cspace(col, row) = false;

            //check for collision
            for(int i = 0; i < segments.size(); i++){
                //test against each obstalce
                Eigen::Vector2d dummy(0.0, 0.0);
                if(findSegmentIntersection(p2, p1, segments.at(i).v1, segments.at(i).v2, &dummy)){
                    cspace(col, row) = true;
                }

                if(findSegmentIntersection(p0, p1, segments.at(i).v1, segments.at(i).v2, &dummy)){
                    //printf("Link 1 collision at: %f, %f T1: %f T2: %f\n", dummy(0), dummy(1), cell_center(0), cell_center(1));
                    cspace(col, row) = true;
                }

                if(cspace(col, row) == false){
                    //printf("p0 x: %f y%f p1 x: %f y: %f\n", p0(0), p0(1), p1(0), p1(1));
                }
            }
        }
     }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


//used from previous homework 
//this function was ChatGPT inspired but I've heavily edited the code
//Prompt: write a function that determines the point of intersection of two lines in cpp. The vertexes of each thing will be passed as eigen::vector2d
bool MyManipulatorCSConstructor::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter) {
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
