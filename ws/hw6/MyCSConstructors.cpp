#include "MyCSConstructors.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = std::size_t((x0 - x0Bounds().first) / (x0Bounds().second - x0Bounds().first) * cells_per_dim) % cells_per_dim; // x index of cell
    std::size_t cell_y = std::size_t((x1 - x1Bounds().first) / (x1Bounds().second - x1Bounds().first) * cells_per_dim) % cells_per_dim; // x index of cell

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
            Eigen::Vector2d cell_center = cspace.getCellCenter(col, row);

            //printf("Testing %f %f \n", cell_center(0), cell_center(1));

            //determine where each point is
            amp::ManipulatorState test_state(2);
            test_state << cell_center(0), cell_center(1);

            Eigen::Vector2d p2 = manipulator.getJointLocation(test_state, 0);
            Eigen::Vector2d p0 = manipulator.getJointLocation(test_state, 2);
            Eigen::Vector2d p1 = manipulator.getJointLocation(test_state, 1);
            //printf("ts: %f %f, p2: %f %f\n", test_state[0], test_state[1], p1[0], p1[1]);


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

    cspace.m_link_lengths = manipulator.getLinks();

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

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

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

    std::vector<Eigen::Vector2d> relative_neighbors;
    relative_neighbors.push_back(Eigen::Vector2d(1,0));
    relative_neighbors.push_back(Eigen::Vector2d(0,1));
    relative_neighbors.push_back(Eigen::Vector2d(-1,0));
    relative_neighbors.push_back(Eigen::Vector2d(0,-1));

    Eigen::Vector2d dummy(0,0);

    for(int i = 0; i < m_cells_per_dim; i++){
        for(int j = 0; j < m_cells_per_dim; j++){

            for(int k = 0; k < relative_neighbors.size(); k++){
                
                for(int l = 0; l < segments.size(); l++){
                    

                    //check for intersection points
                    if(findSegmentIntersection(cspace.getCellCenter(i,j), cspace.getCellCenter(i + relative_neighbors.at(k)[0], j + relative_neighbors.at(k)[1]), segments.at(l).v1, segments.at(l).v2, &dummy)){
                        cspace(i,j) = true;
                    }

                }

            }

        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

bool MyPointAgentCSConstructor::findSegmentIntersection(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2, Eigen::Vector2d *inter) {
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

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    if(isManipulator){
        allow_looping = true;
    } else {
        allow_looping = false;
    }
   
    bool goal_found = false;

    amp::Path2D path;

    //open cells and open child cells
    std::list<Cell> open_cells;
    std::queue<Cell> child_cells;


    //create a matrix of the cells that have been checked
    std::pair<std::size_t, std::size_t> lower = grid_cspace.getCellFromPoint(grid_cspace.x0Bounds().first, grid_cspace.x1Bounds().first);
    std::pair<std::size_t, std::size_t> upper = grid_cspace.getCellFromPoint(grid_cspace.x0Bounds().second, grid_cspace.x1Bounds().second);
    Eigen::MatrixXi checked_mat(grid_cspace.num_cells, grid_cspace.num_cells);
    checked_mat.setZero();

    printf("Beginning Checks, qinit: %f %f q_goal: %f %f grid_size: %ld %ld \n", q_init[0], q_init[1], q_goal[0], q_goal[1], grid_cspace.num_cells, grid_cspace.num_cells);



    std::pair<std::size_t, std::size_t> q_init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    if(grid_cspace.inCollision(q_init_cell.first, q_init_cell.second, true)){
        printf("Flipping it\n");
        
        MyManipulator2D mani;
        mani.setLinks(grid_cspace.m_link_lengths);

        Eigen::Vector2d mani_state(q_init[0], q_init[1]);
        Eigen::Vector2d pos = mani.getJointLocation(mani_state, 2);
        amp::ManipulatorState second_state = mani.getConfigurationFromIK(pos, false);

        q_init_cell = grid_cspace.getCellFromPoint(second_state[0], second_state[1]);
    }

    std::pair<std::size_t, std::size_t> q_goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    if(grid_cspace.inCollision(q_goal_cell.first, q_goal_cell.second, true)){

        MyManipulator2D mani;
        mani.setLinks(grid_cspace.m_link_lengths);

        Eigen::Vector2d mani_state(q_goal[0], q_goal[1]);
        Eigen::Vector2d pos = mani.getJointLocation(mani_state, 2);
        amp::ManipulatorState second_state = mani.getConfigurationFromIK(pos, false);

        q_goal_cell = grid_cspace.getCellFromPoint(second_state[0], second_state[1]);
    }

    //open q_init
    printf("x: %d y: %d\n", q_init_cell.first, q_init_cell.second);
    child_cells.push(Cell(Eigen::Vector2i(q_init_cell.first, q_init_cell.second)));
    checked_mat(q_init_cell.first, q_init_cell.second) = true;

    printf("Beginning Checks, qinit cell: %d %d q_goal_cell: %d %d \n", q_init_cell.first, q_init_cell.second, q_goal_cell.first, q_goal_cell.second);


    int neighbors_evaluted = 0;
    while(!goal_found){
        Cell test = child_cells.front();

        //printf("Checking: %d %d \n", test.location[0], test.location[1]);


        if(test.location == Eigen::Vector2i(q_goal_cell.first, q_goal_cell.second)){
            //found goal
            printf("Found Goal\n\n\n\n\n\n\n");
            break;
        }

        open_cells.push_back(test);


        //check if neighbors are open
        for(int i = 0; i < test.get_neighbors(allow_looping, grid_cspace.num_cells).size(); i++){

            if((checked_mat(std::max(test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0], 0), std::max(test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1], 0)) == false) && (test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0] >= 0) && (test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1] >= 0) && (test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0] < grid_cspace.num_cells) && (test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1] < grid_cspace.num_cells)){

                //printf("Here %d %d \n", test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0], test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1]);


                //if collision free
                if(grid_cspace.inCollision(test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0], test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1], true) == false){

                    neighbors_evaluted++;

                    //create a new child node
                    child_cells.push(Cell(test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)));
                    //set parent
                    child_cells.back().parent = &open_cells.back();

                    //printf("Parent x: %d\n",child_cells.back().parent->location[0]);

                    //mark the space as checked
                    checked_mat(test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[0], test.get_neighbors(allow_looping, grid_cspace.num_cells).at(i)[1]) = true;

                    //printf("Add: %d %d \n", child_cells.back().location[0], child_cells.back().location[1]);

                }

            }

        }

        child_cells.pop();

        if(child_cells.size() == 0){
            printf("Failed to find a valid path. Searched %ld cells and %d neighbors\n", open_cells.size(), neighbors_evaluted);

            if(open_cells.size() == 1){
                printf("q_init is %d\n", grid_cspace.inCollision(open_cells.front().location[0], open_cells.front().location[1]), true);
            }

            return path;
        }
    }

    path.waypoints.push_back(q_goal);
    Cell test = child_cells.front();

    while(open_cells.size() > 0){

        path.waypoints.push_back(grid_cspace.getCellCenter(test.location[0], test.location[1]));
        test = *(test.parent);

        if(test.location == Eigen::Vector2i(q_init_cell.first, q_init_cell.second)){
            path.waypoints.push_back(grid_cspace.getCellCenter(test.location[0], test.location[1]));

            break;
        }
    }

    path.waypoints.push_back(q_init);

    std::reverse(path.waypoints.begin(), path.waypoints.end());

    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}

Eigen::Vector2d MyGridCSpace2D::getCellCenter(int x0, int x1) const
{
    double cell_x = (x0 + 0.5) * (x0Bounds().second - x0Bounds().first) / cells_per_dim + x0Bounds().first; // x index of cell
    double cell_y = (x1 + 0.5) * (x1Bounds().second - x1Bounds().first) / cells_per_dim + x1Bounds().first; // x index of cell

    return Eigen::Vector2d(cell_x, cell_y);
}
