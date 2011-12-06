/**
 * @file MyTesterPlanner.cpp
 * @brief This is my paper.
 */

#include <iostream>
#include <Tools/Robot.h>
#include "MyTesterPlanner.h"
#include "JT_RRT.h"
#include <time.h>

using namespace std;
using namespace Eigen;

/**
 * @function MyTesterPlanner
 * @brief Constructor
*/
MyTesterPlanner::MyTesterPlanner()
{ 

}

/**
 * @function MyTesterPlanner
 * @brief Destructor
*/
MyTesterPlanner::~MyTesterPlanner()
{ delete this->world; }

/**
 * @function initialize
 * @brief Initialize data
*/
void MyTesterPlanner::initialize( World &_world,
                             const int &_robot_ID, 
			     const std::vector<int> &_links_ID,
                             const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase, 
			     const Eigen::Transform<double, 3, Eigen::Affine> &_Tee )
{
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
bool MyTesterPlanner::findPath( const Eigen::VectorXd &_startConfig, 
				const Eigen::VectorXd &_goal_position, 
				std::vector<Eigen::VectorXd> &_path )
{
  time_t ts; time_t tf; double dt;
  std::cout << "---Find Path---" << std::endl;

  _path.clear();
  this->startConfig = _startConfig;
  this->goalPosition = _goal_position;
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

  for( int i = 0; i < 1; i++ )
  { 
    a_path.resize(0);

    ts= clock();
    a.plan( startVoxelCell, goalVoxelCell, startConfig, a_path );
    tf = clock();
    dt = (double) (tf - ts)/CLOCKS_PER_SEC; 
    printf("-- [%d] A* plan time: %.3f  -- path size: %d \n", i, dt, a_path.size() );

    isValidPath = checkPath( a_path, cp );
    if( isValidPath == true )
    { break; }
  }

  fclose( pPath3D );

  //-- If valid, save this path
  Eigen::Transform<double, 3, Eigen::Affine> nextPose;
  Eigen::VectorXd q;

  FILE* pBlack;
  pBlack = fopen("blackList.txt", "w");
  Eigen::Transform<double, 3, Eigen::Affine> black;
  black.setIdentity(); 
  for( int i = 0; i <blackList.size(); i++ )  
  { 
    black.setIdentity(); 
    black.translation() =  cp.voxel->getCenter( blackList[i](0), blackList[i](1), blackList[i](2) );    
    fprintf(pBlack, "%.3f %.3f %.3f \n", black.translation()(0), black.translation()(1), black.translation()(2) );
  }
 fclose(pBlack);

  if( isValidPath == true || isValidPath == false )
  {
    a_path.resize(0);
    a.plan( startVoxelCell, goalVoxelCell, startConfig, a_path ); 
    q = startConfig;   
    for( int i = 1; i < a_path.size(); i++ )
    {
      nextPose.setIdentity();
      nextPose.translation() =  cp.voxel->getCenter( a_path[i](0), a_path[i](1), a_path[i](2) );   
      directPoint.straightPath( q, nextPose, _path );  
      q = _path[ _path.size() - 1 ]; 
      _path.pop_back(); 
    }

    return true;
  }
}



/**
 * @function checkPath
 * @brief Check the path given in voxels, converting it to jointspace
 */
bool MyTesterPlanner::checkPath( std::vector< Eigen::VectorXi > path, CheckProcess &cp )
{
  Eigen::VectorXd q;
  Eigen::VectorXd q_next;    
  Eigen::Transform<double, 3, Eigen::Affine> nextPose;
  bool inBlackList;

  int count = 0;
  q = startConfig;

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
        //printf(" COLLISION! -- node: %d %d %d ! \n", path[i+1](0), path[i+1](1), path[i+1](2) ); 
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
        //-- If in blackList
        else
        { //printf(" Already in blackList -- node: %d %d %d ! \n", path[i+1](0), path[i+1](1), path[i+1](2) ); 
        }
      
        count++;
     } /** End if */   
    
     q = q_next;
  } /** End for */


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



