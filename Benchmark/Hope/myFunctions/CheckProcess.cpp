/**
 * @function CheckProcess.cpp
 */
#include "CheckProcess.h"

/**
 * @function CheckProcess
 * @brief Constructor
 */
CheckProcess::CheckProcess( double _sizeX, double _sizeY, double _sizeZ,
	    		    		double _originX, double _originY, double _originZ, 
			    			double _resolution )
{

  //-- Save data
  mSizeX = _sizeX;
  mSizeY = _sizeY;
  mSizeZ = _sizeZ; 
  mOriginX = _originX;
  mOriginY = _originY;
  mOriginZ = _originZ;
  mResolution = _resolution;

  //-- Create our slideBox
  build_slideBox();
}

/**
 * @function initialize
 * @brief Initialize the slideBox, basically
 */
void CheckProcess::build_slideBox()
{ 

  //-- Initialize the slideBox 
  mSlideBox = new RAPID_model;

  static double p0[3] = { 0, 0, 0 };
  static double p1[3] = { 0, mResolution, 0 };
  static double p2[3] = { mResolution, mResolution, 0 };
  static double p3[3] = { mResolution, 0, 0 };
  static double p4[3] = { 0, 0, mResolution };
  static double p5[3] = { 0, mResolution, mResolution };
  static double p6[3] = { mResolution, mResolution, mResolution };
  static double p7[3] = { mResolution, 0, mResolution };

  mSlideBox->BeginModel();
  mSlideBox->AddTri( p0, p2, p1, 0 );
  mSlideBox->AddTri( p0, p2, p3, 1 );
  mSlideBox->AddTri( p4, p6, p5, 2 );
  mSlideBox->AddTri( p4, p6, p7, 3 );
  mSlideBox->AddTri( p0, p5, p4, 4 );
  mSlideBox->AddTri( p0, p5, p1, 5 );
  mSlideBox->AddTri( p3, p6, p7, 6 );
  mSlideBox->AddTri( p3, p6, p2, 7 );
  mSlideBox->AddTri( p0, p7, p4, 8 );
  mSlideBox->AddTri( p0, p7, p3, 9 );
  mSlideBox->AddTri( p1, p6, p5, 10 );
  mSlideBox->AddTri( p1, p6, p2, 11 );
  //-- Should we add more inner triangles here?
  mSlideBox->EndModel();

}

/**
 * @function getObjectsData
 * @brief Copy the information of the world objects in our class
 */
void CheckProcess::getObjectsData( std::vector<planning::Object*> _objects )
{
  mNumObjs = _objects.size();
  mObjs.resize( mNumObjs );

  //-- NOTICE! Each object has only one model (my ASSUMPTION)
  for( int i = 0; i < mNumObjs; i++ )
  { mObjs[i].getModelData( _objects[i]->mModels[0] ); }
}  

/**
 * @function getLinksData
 * @brief Get data from the robot links
 */
void CheckProcess::getLinksData( planning::Robot* _robot, Eigen::VectorXi _linksID )
{
  mNumLinks = _linksID.size();
  mLinksID = _linksID;
  mLinks.resize( mNumLinks );


  Eigen::VectorXi ind(mNumLinks);

  // Only use the models indicated
  for( int i = 0; i < mNumLinks; i++ ) {
     for( int j = 0; j < _robot->mModels.size(); j++ ) {
	   if( _robot->mModelIndices[j] == _linksID[i] ) {
	     ind[i] = j;
       }
     }

  }

  
  for( int i = 0; i < mNumLinks; i++ ) 
  { 
    mLinks[i].getModelData( _robot->mModels[ind[i]] );
    mLinks[i].updateObjectData( _robot->getNode( mLinksID[i] ) );
  }
}


/**
 * @function build_voxel
 * @brief Collision quick checking
 */
void CheckProcess::build_voxel( std::vector<planning::Object*> _objects, VoxelGrid<int> &_voxel )
{
   time_t ts; time_t tf; double dt;
   mObjsVoxels.clear();

   int a = 0;
   _voxel.reset(a);

   ts = clock();
   //-- For each object
   for( int k = 0; k < mNumObjs; k++ )
   {
      // Each object has only one node, so get node 0 do it
      mObjs[k].updateObjectData( _objects[k]->getNode(0) );

    //-- Convert to voxel grid
    _voxel.worldToGrid( mObjs[k].min_x, mObjs[k].min_y, mObjs[k].min_z, mObjs[k].minCell_x, mObjs[k].minCell_y, mObjs[k].minCell_z );
    _voxel.worldToGrid( mObjs[k].max_x, mObjs[k].max_y, mObjs[k].max_z, mObjs[k].maxCell_x, mObjs[k].maxCell_y, mObjs[k].maxCell_z );

    //-- 
    double idt[3][3];
    idt[0][0] = 1; idt[0][1] = 0; idt[0][2] = 0;
    idt[1][0] = 0; idt[1][1] = 1; idt[1][2] = 0;
    idt[2][0] = 0; idt[2][1] = 0; idt[2][2] = 1;
  
    double tt[3]; 

    //-- Building the voxel grid, sliding the basic box
    int occupied = 1;  

    for( int i = mObjs[k].minCell_x; i <= mObjs[k].maxCell_x; i++ )
     { for( int j = mObjs[k].minCell_y; j <= mObjs[k].maxCell_y; j++ )
       { for( int q = mObjs[k].minCell_z; q <= mObjs[k].maxCell_z; q++ )
         { 
           _voxel.gridToWorld( i, j, q, tt[0], tt[1], tt[2] ); 
           RAPID_Collide( mObjs[k].R, mObjs[k].T, mObjs[k].rapidObject, idt, tt, mSlideBox, RAPID_FIRST_CONTACT );
           if( RAPID_num_contacts != 0)
            { _voxel.setCell( i, j, q, occupied );
              mObjsVoxels.push_back( Eigen::Vector3i( i, j, q ) );
            }
         } 
       }
     }

  } //-- End of objects

  tf = clock();
  dt = (double) ( tf - ts )/CLOCKS_PER_SEC; 
  printf("--** Voxel construction time: %.3f \n", dt );

  //-- Write data in a file
  FILE * pFile;
  pFile = fopen( "PointCloudData.txt", "w" );

   for( int i = 0; i < _voxel.getNumCells( VoxelGrid<int>::DIM_X ); i++ )
    { for( int j = 0; j < _voxel.getNumCells( VoxelGrid<int>::DIM_Y ); j++ )
      { for( int q = 0; q < _voxel.getNumCells( VoxelGrid<int>::DIM_Z ); q++ )
        { 
          if ( _voxel.getCell( i, j, q ) == 1 )        
           { 
             double x, y, z;
             _voxel.gridToWorld( i, j, q, x, y, z );
             fprintf( pFile, "  %.4f  %.4f  %.4f  \n", x, y, z ); 
           }
         }
      }
    }  

  fclose( pFile ); 

   printf("Writen point cloud data in txt file! Yay! \n");

}

/**
 * @function build_df
 */
void CheckProcess::build_df( PFDistanceField &_df )
{
  time_t ts; time_t tf; double dt;
  ts = clock();

  printf("Size obj voxels: %d \n", mObjsVoxels.size() );

  /** Add points and calculate the DT -- this does everything folks */
  _df.addPointsToField( mObjsVoxels ); 

  tf = clock();
  dt = (double) ( tf - ts )/CLOCKS_PER_SEC; 
  printf("--** DT calculation time: %.3f \n", dt );

}


/**
 * @function reportObjects
 * @brief print Stuff
 */
void CheckProcess::reportObjects()
{
  for( int i = 0; i < mNumObjs; i++ )
  { printf( " Object [%d]: center: ( %.3f,%.3f, %.3f ) -- radius: ( %.3f, %.3f, %.3f ) \n", 
	      i, mObjs[i].center[0], mObjs[i].center[1], mObjs[i].center[2], mObjs[i].radius[0], mObjs[i].radius[1], mObjs[i].radius[2] ); }
}


