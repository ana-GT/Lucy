/**
 * @file Planner1.cpp
 * @brief This is my paper.
 */

#include <iostream>
#include <Tools/Robot.h>
#include "Planner1.h"
#include "JT_RRT.h"
#include <time.h>

using namespace std;
using namespace Eigen;

/**
 * @function Planner1
 * @brief Constructor
*/
Planner1::Planner1()
{ 

}

/**
 * @function Planner1
 * @brief Destructor
*/
Planner1::~Planner1()
{ delete this->world; }

/**
 * @function initialize
 * @brief Initialize data
*/
void Planner1::initialize( World &_world,
                             const int &_robot_ID, 
			     const std::vector<int> &_links_ID,
                             const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase, 
			     const Eigen::Transform<double, 3, Eigen::Affine> &_Tee )
{
  this->num_iter = 10;
  this->world = new World( _world );
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->TWBase = _TWBase;
  this->Tee = _Tee;


  this->num_bodies_left_ = 16;   
  string bodies_left_names[16] = { "LEFTARM0", "LEFTARM1", "LEFTARM2", "LEFTARM3", "LEFTARM4", "LEFTARM5", "LEFTARM6",
	                           "LHAND_THUMB0", "LHAND_THUMB1", "LHAND_THUMB2", "LHAND_INDEX0", "LHAND_INDEX1", "LHAND_INDEX2", 
	                           "LHAND_MIDDLE0", "LHAND_MIDDLE1", "LHAND_MIDDLE2" };
 
  this->bodies_left_ID_.resize( num_bodies_left_ );
  for( int i = 0; i < num_bodies_left_; i++ )
  { bodies_left_ID_[i] = this->world->robots[robot_ID]->findLink( bodies_left_names[i] ); }

}

/**
 * @function findPath
 * @brief Find a path (!)
 */
bool Planner1::findPath( const Eigen::VectorXd &_startConfig, 
				const Eigen::VectorXd &_goal_position, 
				std::vector<Eigen::VectorXd> &_path )
{
  time_t ts; time_t tf; double dt;
  std::cout << "---Find Path---" << std::endl;

  _path.clear();
  this->startConfig = _startConfig;
  this->goalPosition = _goal_position;
  goalPose.setIdentity();
  goalPose.translation() = goalPosition;
  startPosition.resize(3);

  //-- 1. Build the Voxel and get information
  CheckProcess cp( 1.5, 1.5, 1.2, -0.5, -0.5, -0.1, 0.01 );
  cp.getLinksData( world->robots[robot_ID], bodies_left_ID_ );
  cp.getObjectsData( world->objects );
  cp.reportObjects();
  cp.fillVoxel( world->objects );
  cp.getLinksVoxels(  world->robots[robot_ID] );

  //-- 2. Locating our start and target positions in the voxel 
  Eigen::Transform<double, 3, Eigen::Affine> Tw;
  robinaLeftArm_fk( startConfig, TWBase, Tee, Tw ); 
  startPosition = Tw.translation();

  cp.voxel->world2VoxelCell( startPosition(0), startPosition(1), startPosition(2), startVoxelCell(0), startVoxelCell(1), startVoxelCell(2) );
  cp.voxel->world2VoxelCell( goalPosition(0), goalPosition(1), goalPosition(2), goalVoxelCell(0), goalVoxelCell(1), goalVoxelCell(2) );
  
  //-- 3. Finding a Euclidean route to them
  std::cout << "-- Start voxel: " << startVoxelCell.transpose() << " -- State: " << cp.voxel->getVoxelCell( startVoxelCell(0), startVoxelCell(1), startVoxelCell(2) ) <<  std::endl;
  std::cout << "-- End voxel: " << goalVoxelCell.transpose() << " -- State: " << cp.voxel->getVoxelCell( goalVoxelCell(0), goalVoxelCell(1), goalVoxelCell(2) ) <<  std::endl;

  //-- 4. Plan
  a.initialize( cp.voxel, world, robot_ID, links_ID, TWBase, Tee ); 
  directPoint.initialize( world, robot_ID, links_ID, TWBase, Tee );
  std::vector< Eigen::VectorXi > a_path;

  //-- 5. Write it in a file
  char* path3D_name = "paths3D.txt";
  pPath3D = fopen ( path3D_name,"w" );

  /** A* replanning */
  blackList.resize(0);
  bool isValidPath;

  std::vector<int> num_collisions;
  std::vector< std::vector< Eigen::VectorXd > > jointPaths;
  std::vector< Eigen::VectorXd >collision_free;

  /** Generate a few possible paths */
  for( int i = 0; i < num_iter; i++ )
  { 
    int num_col;
    a_path.resize(0);

    ts= clock();
    a.plan( startVoxelCell, goalVoxelCell, startConfig, a_path );
    tf = clock();
    dt = (double) (tf - ts)/CLOCKS_PER_SEC; 
    printf("-- [%d] A* plan time: %.3f  -- path size: %d \n", i, dt, a_path.size() );

    isValidPath = checkPath( a_path, cp, num_col, collision_free );
    jointPaths.push_back(  collision_free );
    num_collisions.push_back( num_col );

    if( isValidPath == true )
    { break; }

  }

  fclose( pPath3D );

  //-- Save the path with least collisions
  int min_collisions = 1000; int ind_collisions = -1;

  for( int i = 0; i < num_collisions.size(); i++ )
  {
    if( num_collisions[i] < min_collisions )
    { min_collisions = num_collisions[i];
      ind_collisions = i;
    } 
  }

  printf(" -- The shortest path is the [%d]-one , size: %d  collisions: %d \n", ind_collisions, jointPaths[ind_collisions].size(), num_collisions[ind_collisions] );
/*
  if( isValidPath == true || isValidPath == false )
  {
*/
    
    std::vector< Eigen::VectorXd > b_path = jointPaths[ind_collisions];   
/*
    for( int i = 0; i < b_path.size(); i++ )
    {
      _path.push_back( b_path[i] );
    }

  }
*/
  //-- Now do a RRT
  JG_RRT jg;
  jg.initialize( world, robot_ID, links_ID, TWBase, Tee );
  bool  result = jg.plan( startConfig, goalPose, b_path, _path ) ;


  if( result == true )
  {  printf( " Yay! Gotcha! \n");
     return true;
  }
  if( result == false )
  {  printf( " Snap! I did not get it sweetie, sorry \n");
     return true;
  }  
}



/**
 * @function checkPath
 * @brief Check the path given in voxels, converting it to jointspace
 */
bool Planner1::checkPath( std::vector< Eigen::VectorXi > path, CheckProcess &cp, int &num_col, std::vector< Eigen::VectorXd > &collision_free )
{
  Eigen::VectorXd q;
  Eigen::VectorXd q_next;    
  Eigen::Transform<double, 3, Eigen::Affine> nextPose;
  bool inBlackList;

  int count = 0;
  collision_free.resize(0);
  q = startConfig;

  collision_free.push_back( q );

  for( int i = 0; i < path.size() - 1; i++ )
  {
     nextPose.setIdentity();
     nextPose.translation() = cp.voxel->getCenter( path[i+1](0), path[i+1](1), path[i+1](2) );  
     q_next = directPoint.getWSConfig( q, nextPose.translation(), 0.0025 );
     inBlackList = false;

     fprintf( pPath3D, " %.3f %.3f %.3f \n", nextPose.translation()(0), nextPose.translation()(1), nextPose.translation()(2) );
 
     //-- Check collisions in this location
     world->robots[ robot_ID ]->setConf( links_ID, q_next );
     if( world->checkCollisions() ) 
     {
        //-- If collisions, check if in blackList already
        for( int j = 0; j < blackList.size(); j++ )
        { if( blackList[j] == path[i+1] )
           { inBlackList = true; break; }          
        }

        //-- If not in blackList  
        if( inBlackList == false )
        {  blackList.push_back( path[i+1] ); 
           //-- Set A* grid as RISKY
          (*a.voxel_).setVoxelCell( path[i+1](0), path[i+1](1), path[i+1](2), BinaryVoxel::RISKY );
        }
      
        count++;
     } /** End if */   

     else
     { collision_free.push_back( q ); }    


     q = q_next;

  } /** End for */

  num_col = count;

  if( count == 0 ) 
   { return true; }

  else
   { 
     printf("** Number of collisions: %d \n", count );
     for( int i = 1; i < path.size() - 1; i++ )
     { (*a.voxel_).setVoxelCell( path[i](0), path[i](1), path[i](2), BinaryVoxel::RISKY );
     }
     return false; }

}	



