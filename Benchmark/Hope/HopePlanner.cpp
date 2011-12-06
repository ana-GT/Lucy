/**
 * @file HopePlanner.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date December 4th, 2011
 */

#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "HopePlanner.h"

#include "myFunctions/CheckProcess.h"

/**
 * @function HopePlanner
 * @brief Constructor
*/
HopePlanner::HopePlanner()
{ 

    mCopyWorld = false;
    mWorld = NULL;

   mResolution = 0.01;
   mDefaultVoxelObject = 0;

   mOrigX = -0.20;   mOrigY = -0.3;  mOrigZ = -0.05;
   mSizeX = 1.0;     mSizeY = 1.5;   mSizeZ = 1.05;

   mStep_XYZ = 0.02;
   mThresh_XYZ = 0.02;

   mMaxIter = 1000;

   mThresh_RPY = 0.1;
   mStep_RPY = 0.02; // 2 deg +/-

}

/**
 * @function HopePlanner
 * @brief Constructor
 */
HopePlanner::HopePlanner( planning::World &_world, 
                          Collision *_collision,
                          bool _copyWorld, 
                          double _stepSize ) {

    mCopyWorld = _copyWorld;

    if( mCopyWorld ) {
       printf( "Not implemented yet. Sorry -- achq \n" );
    } else {
        mWorld = &_world;
    }

    mCollision = _collision;
    mStepSize = _stepSize;


   mResolution = 0.01;
   mDefaultVoxelObject = 0;

   mOrigX = -0.20;   mOrigY = -0.3;  mOrigZ = -0.05;
   mSizeX = 1.0;     mSizeY = 1.5;   mSizeZ = 1.05;

   mStep_XYZ = 0.02;
   mThresh_XYZ = 0.02;

   mMaxIter = 1000;

   mThresh_RPY = 0.1;
   mStep_RPY = 0.02; // 2 deg +/-
}


/**
 * @function ~HopePlanner
 * @brief Destructor
*/
HopePlanner::~HopePlanner() {

  if( mCopyWorld ) {
    delete mWorld;
  } 
}

/**
 * @function planPath
 * @brief Main function
 */
bool HopePlanner::planPath( int _robotId, 
                            const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start,  // Config (nx1)
                            const Eigen::VectorXd &_target, // Pose (6x1)
                            const string &_EEName, 
                            bool _smooth, 
                            unsigned int _maxNodes ) {

    if( _maxNodes <= 0 ) {
        printf("--(x) Set maxNodes to some value, you moron! (x)--\n"); return false;
    } 

    /// Store information
    mRobotId = _robotId;
    mEENode = mWorld->mRobots[mRobotId]->getNode( _EEName.c_str() );
    mEEId = mEENode->getSkelIndex();
    mLinks = _links;
 
    /// Check start position is not in collision
    mWorld->mRobots[mRobotId]->setDofs( _start, mLinks );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return false; 
    }

   //-- Voxelization and Distance Field calculation
   mVoxel = new VoxelGrid<int>( mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ, mDefaultVoxelObject );
   mPf = new PFDistanceField( mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ );
   mPf->reset(); 
   calculateInfo3D( *mVoxel, *mPf  ); 

   //-- Setting start and goal cells
   mStartConfig = _start;
   mStartXYZ = GetEE_XYZ( mStartConfig );

   mTargetPose = _target; 
   mTargetXYZ(0) = _target(0); mTargetXYZ(1) = _target(1); mTargetXYZ(2) = _target(2); 
  
   if( ! mVoxel->worldToGrid( mStartXYZ(0), mStartXYZ(1), mStartXYZ(2), mStartCell(0), mStartCell(1), mStartCell(2) ) )
   {  printf("--(!) Error -- No Start Cell in Voxel \n"); } 
   if( ! mVoxel->worldToGrid( mTargetXYZ(0), mTargetXYZ(1), mTargetXYZ(2), mTargetCell(0), mTargetCell(1), mTargetCell(2) ) )
   {  printf("--(!) Error -- No Start Cell in Voxel \n"); }    

   //-- Build Dijkstra
   mDijkstraEE.SetGoal( mTargetCell(0), mTargetCell(1), mTargetCell(2) );
   printf("-- Target Loc: %d, %d, %d --- State: %d \n", mTargetCell(0), mTargetCell(1), mTargetCell(2), mVoxel->getCell( mTargetCell(0), mTargetCell(1), mTargetCell(2) ) );
   mDijkstraEE.SetGrid3D( mVoxel );
   mDijkstraEE.SetDF( mPf );


   //-- Fill
   printf("--** Start Dijkstra construction process \n");
   ts= clock();
   mDijkstraEE.Search();
   tf = clock(); 
   printf("--** End Dijkstra construction process: Time elapsed: %.3f  \n", (double) (tf - ts)/CLOCKS_PER_SEC );

   float dist = mDijkstraEE.GetDistance( mStartCell(0), mStartCell(1), mStartCell(2) );
   printf("Distance from the start according to Dijkstra: %f \n", dist );
   std::vector<Eigen::Vector3i> path = mDijkstraEE.GetPath( mStartCell(0), mStartCell(1), mStartCell(2) );

   //-- Print path 3D
   FILE* pdij;
   pdij = fopen("Dijkstra.txt", "w");
   double x, y, z;
   for( int i = 0; i < path.size(); i++ )
   {
      mVoxel->gridToWorld( path[i](0), path[i](1), path[i](2), x, y, z );
      fprintf( pdij, "%f %f %f \n", x,y,z );
   }

   fclose(pdij);


    if( _maxNodes <= 0 ) {
        printf("--(x) Set maxNodes to some value, you moron! (x)--\n"); return false;
    } 


    /// Get a direct path 
   GetDirectPath( _start, _target, mPath );
   //GetOrientedPath( _start, _target, mPath );
   //GetTestPath( _start, _target, mPath );


    bool result = (mPath.size() != 0);
    return result;
}


  

/**
 * @function calculateInfo3D
 */
void HopePlanner::calculateInfo3D( VoxelGrid<int> &_voxel, PFDistanceField &_df )
{
  //-- Build voxel
  CheckProcess cp(mSizeX, mSizeY, mSizeZ, mResolution, mOrigX, mOrigY, mOrigZ );
  cp.getObjectsData( mWorld->mObjects );
  cp.reportObjects();   
  cp.build_voxel( mWorld->mObjects, _voxel );
  //-- Build distance field
  cp.build_df( _df );
}


