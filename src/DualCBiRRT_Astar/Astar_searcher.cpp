#include "baxter_moveit_application/DualCBiRRT_Astar/Astar_searcher.h"

using namespace std;
using namespace Eigen;

//初始化栅格地图，分配内存，得到每一个index 对应的 3D坐标点， data保存的是障碍物的信息，GridNodeMap保存的是每一个节点对应的信息
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }



    neighbor_dir<<
            resolution,0,0,1,
            0,resolution,0,1,
            0,0,resolution,1,
            -resolution,0,0,1,
            0,-resolution,0,1,
            0,0,-resolution,1,
            resolution,resolution,0,1.414,
            -resolution,resolution,0,1.414,
            resolution,-resolution,0,1.414,
           -resolution,-resolution,0,1.414,
            0,resolution,resolution,1.414,
            0,resolution,-resolution,1.414,
            0,-resolution,resolution,1.414,
            0,-resolution,-resolution,1.414,
            resolution,0,resolution,1.414,
            -resolution,0,resolution,1.414,
            resolution,0,-resolution,1.414,
            -resolution,0,-resolution,1.414,
            resolution,resolution,resolution,1.732,
            resolution,resolution,-resolution,1.732,
            resolution,-resolution,resolution,1.732,
            resolution,-resolution,-resolution,1.732,
            -resolution,resolution,resolution,1.732,
            -resolution,resolution,-resolution,1.732,
            -resolution,-resolution,resolution,1.732,
            -resolution,-resolution,-resolution,1.732;
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
ptr->id = 0;
ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z) //把这些位置设置为障碍物
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;  //because grid is stored in a one dimension dataset
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() //得到所有遍历过的点
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index)  //计算出数组index对应的3D坐标
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) //计算出3D坐标对应的数组index
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord) //应该是把一个格子内的坐标点，转换成这个格子所保存的一个坐标点
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
    //扩展26个方向
    Eigen::Vector3d tmp_cord;
    Eigen::Vector3i tmp_index;
    for(int i=0; i<26; i++){

        tmp_cord[0] = currentPtr->coord[0] + neighbor_dir(i, 0);
        tmp_cord[1] = currentPtr->coord[1] + neighbor_dir(i, 1);
        tmp_cord[2] = currentPtr->coord[2] + neighbor_dir(i, 2);
        tmp_index = coord2gridIndex(tmp_cord);
        if(isFree(tmp_index)){
            neighborPtrSets.push_back(GridNodeMap[tmp_index[0]][tmp_index[1]][tmp_index[2]]);
            edgeCostSets.push_back(neighbor_dir(i,3));
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    //Euclidean

    return node1->gScore + (node2->coord - node1->coord).norm();
}

double AstarPathFinder::getHeuMultiTarget(GridNodePtr node1, const std::vector< Eigen::Vector3d > & end_pts)
{
    /*
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    //Euclidean
    double mininum = numeric_limits<double >::max();
    double euclidean_dis;
    for(size_t i=0; i<end_pts.size();i++){
        euclidean_dis = (end_pts[i] - node1->coord).norm();
        if(euclidean_dis < mininum){
            mininum = euclidean_dis;
        }
    }
    return node1->gScore + mininum;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    int openset_size_count;
    // this is the main loop
    while ( !openSet.empty() ){
        openset_size_count  =openSet.size();
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        auto it = openSet.begin();
        currentPtr = it->second;
        currentPtr->id = -1;
        openSet.erase(it);


        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            std::cout<<"terminal index  "<<terminatePtr->index.transpose()<<std::endl;

            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore);
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = 1 : unexpanded
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr->id = 1;
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = getHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr;
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                if(neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i]){

                    auto it = openSet.find(neighborPtr->fScore);
                    while (it->second->index !=  neighborPtr->index){
                        it++;
                    }
                    openSet.erase(it);
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = getHeu(neighborPtr,endPtr);
                    neighborPtr->cameFrom = currentPtr;
                    openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );

                }
                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
        }      
    }
    //if search fails
    ROS_WARN("Astar fails");

    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

bool AstarPathFinder::AstarGraphSearchMultiTarget(Eigen::Vector3d start_pt, const std::vector< Eigen::Vector3d > & end_pts)
{
    ros::Time time_1 = ros::Time::now();

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    std::vector< Eigen::Vector3i >end_idxes;
    Vector3i end_idx;
    for(size_t i=0; i<end_pts.size();i++){
        end_idx = coord2gridIndex(end_pts[i]);
        end_idxes.push_back(end_idx);
    }

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeuMultiTarget(startPtr,end_pts);
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1;
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    int openset_size_count;
    // this is the main loop
    while ( !openSet.empty() ){
        openset_size_count  =openSet.size();
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below

        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        auto it = openSet.begin();
        currentPtr = it->second;
        currentPtr->id = -1;
        openSet.erase(it);


        // if the current node is the goal
        auto ret = std::find(end_idxes.begin(), end_idxes.end(), currentPtr->index);
        if(!(ret == end_idxes.end()))
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            std::cout<<"terminal index  "<<terminatePtr->index.transpose()<<std::endl;

            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore);
            return true;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself

        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *
        */

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below

            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = 1 : unexpanded
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            *
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                neighborPtr->id = 1;
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = getHeuMultiTarget(startPtr,end_pts);
                neighborPtr->cameFrom = currentPtr;
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                if(neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i]){

                    auto it = openSet.find(neighborPtr->fScore);
                    while (it->second->index !=  neighborPtr->index){
                        it++;
                    }
                    openSet.erase(it);
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = getHeuMultiTarget(startPtr,end_pts);
                    neighborPtr->cameFrom = currentPtr;
                    openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );

                }
                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *
                */
                continue;
            }
        }
    }
    //if search fails

    ROS_WARN("Astar fails");

    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );

    return false;
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    GridNodePtr currentPtr = terminatePtr;
    ROS_INFO("OKOK");
    int count = 0;
    if(currentPtr){
        ROS_INFO("???");
    }
    else{
        ROS_INFO("aaa");
    }
    std::cout<<"goal index  "<<goalIdx.transpose()<<std::endl;
    std::cout<<"terminal index  "<<terminatePtr->index.transpose()<<std::endl;


    while (currentPtr->cameFrom)
    {
        gridPath.push_back(currentPtr);
        count++;
        ROS_INFO("count:  %d", count);
        GridNodePtr tmp = currentPtr->cameFrom;
        currentPtr = tmp;
    }
    gridPath.push_back(currentPtr);
    ROS_INFO("hereOKOK");
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}
