/**
 * @file HopePlanner.cpp
 * @brief Aww
 */

#include <iostream>
#include <Tools/Robot.h>
#include <time.h>
#include "HopePlanner.h"


/**
 * @function HopePlanner
 * @brief Constructor
*/
HopePlanner::HopePlanner()
{ 
   this->mResolution = 0.01;
   this->mDefaultVoxelObject = 0;
}

/**
 * @function HopePlanner
 * @brief Destructor
*/
HopePlanner::~HopePlanner()
{ delete this->mWorld; }

/**
 * @function initialize
 * @brief Initialize data
*/
void HopePlanner::initialize( World &_world,
                              const int &_robot_ID, 
			      const std::vector<int> &_links_ID,
                              const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase, 
			      const Eigen::Transform<double, 3, Eigen::Affine> &_Tee )
{
  this->mWorld = new World( _world );
  this->mRobotID = _robot_ID;
  this->mLinksID = _links_ID;
  this->mTWBase = _TWBase;
  this->mTee = _Tee;


  this->mOrigX = -0.20;   this->mOrigY = -0.3;   this->mOrigZ = -0.05;
  this->mSizeX = 1.0;   this->mSizeY = 1.5;   this->mSizeZ = 1.05;


  this->mNumBodiesLeft = 16;   
  string bodies_left_names_[16] = { "LEFTARM0", "LEFTARM1", "LEFTARM2", "LEFTARM3", "LEFTARM4", "LEFTARM5", "LEFTARM6",
	                            "LHAND_THUMB0", "LHAND_THUMB1", "LHAND_THUMB2", "LHAND_INDEX0", "LHAND_INDEX1", "LHAND_INDEX2", 
	                            "LHAND_MIDDLE0", "LHAND_MIDDLE1", "LHAND_MIDDLE2" };
 
  this->mBodiesLeftID.resize( mNumBodiesLeft );
  for( int i = 0; i < mNumBodiesLeft; i++ )
  { mBodiesLeftID[i] = this->mWorld->robots[ mRobotID ]->findLink( bodies_left_names_[i] ); }
}

/**
 * @function findPath
 * @brief Find a path (!)
 */
bool HopePlanner::findPath( const Eigen::VectorXd &startConfig, 
			    const Eigen::Transform<double, 3, Eigen::Affine> &goalPose, 
			    std::vector<Eigen::VectorXd> &_path )
{
   printf(" |--- findPath ---| \n");

   //-- Voxelization and Distance Field calculation
   mVoxel = new VoxelGrid<int>( mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ, mDefaultVoxelObject );
   mPf = new PFDistanceField( mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ );
   mPf->reset(); 
   calculateInfo3D( *mVoxel, *mPf  ); 

   //-- Setting start and goal cells
   fromJoints2Position( startConfig, mStartPosition );
   mGoalPose = goalPose; 
   mGoalPosition = mGoalPose.translation(); 
  
   if( ! mVoxel->worldToGrid( mStartPosition(0), mStartPosition(1), mStartPosition(2), mStartCell(0), mStartCell(1), mStartCell(2) ) )
   {  printf("--(!) Error -- No Start Cell in Voxel \n"); } 
   if( ! mVoxel->worldToGrid( mGoalPosition(0), mGoalPosition(1), mGoalPosition(2), mGoalCell(0), mGoalCell(1), mGoalCell(2) ) )
   {  printf("--(!) Error -- No Start Cell in Voxel \n"); } 


 
   //-- Build Dijkstra
   mDijkstraEE.setGoal( mGoalCell(0), mGoalCell(1), mGoalCell(2) );
   printf("-- Goal Loc: %d, %d, %d --- State: %d \n", mGoalCell(0), mGoalCell(1), mGoalCell(2), mVoxel->getCell( mGoalCell(0), mGoalCell(1), mGoalCell(2) ) );
   mDijkstraEE.setGrid3D( mVoxel );
   mDijkstraEE.setDF( mPf );

   printf("--** Start Dijkstra construction process \n");
   ts= clock();
   mDijkstraEE.search();
   tf = clock(); 
   printf("--** End Dijkstra construction process: Time elapsed: %.3f  \n", (double) (tf - ts)/CLOCKS_PER_SEC );

   float dist = mDijkstraEE.getDistance( mStartCell(0), mStartCell(1), mStartCell(2) );
   printf("Distance from the start according to Dijkstra: %f \n", dist );
   std::vector<Eigen::Vector3i> path = mDijkstraEE.getPath( mStartCell(0), mStartCell(1), mStartCell(2) );

   FILE* pdij;
   pdij = fopen("Dijkstra.txt", "w");
   double x, y, z;
   for( int i = 0; i < path.size(); i++ )
   {
      mVoxel->gridToWorld( path[i](0), path[i](1), path[i](2), x, y, z );
      fprintf( pdij, "%f %f %f \n", x,y,z );
   }

   fclose(pdij);

   //-- search
   printf("--** End by now \n");
   Search search;
   //search.initialize( mTWBase, mTee );
   //search.plan( _start_config, _goal_pose, _path );
   
}


/**
 * @function calculateInfo3D
 */
void HopePlanner::calculateInfo3D( VoxelGrid<int> &voxel, PFDistanceField &df )
{
  //-- Build voxel
  CheckProcess cp(mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ );
  cp.getObjectsData( mWorld->objects );
  cp.reportObjects();   
  cp.build_voxel( mWorld->objects, voxel );
  //-- Build distance field
  cp.build_df( df );
}

/**
 * @function FromJoints2Position
 */
void HopePlanner::fromJoints2Position( Eigen::VectorXd q, Eigen::Vector3d &p )
{
   Eigen::Transform<double, 3, Eigen::Affine> T;
   robinaLeftArm_fk( q, mTWBase, mTee, T ); 
   p = T.translation();   
}
