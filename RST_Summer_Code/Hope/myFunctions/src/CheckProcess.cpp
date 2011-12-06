/**
 * @function CheckProcess.cpp
 */
#include "CheckProcess.h"

/**
 * @function CheckProcess
 * @brief Constructor
 */
CheckProcess::CheckProcess( double size_x, double size_y, double size_z,
	    		    double origin_x, double origin_y, double origin_z, 
			    double resolution )
{

  //-- Save data
  size_x_ = size_x;
  size_y_ = size_y;
  size_z_ = size_z; 
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  origin_z_ = origin_z;
  resolution_ = resolution;

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
  slideBox = new RAPID_model;

  static double p0[3] = { 0, 0, 0 };
  static double p1[3] = { 0, resolution_, 0 };
  static double p2[3] = { resolution_, resolution_, 0 };
  static double p3[3] = { resolution_, 0, 0 };
  static double p4[3] = { 0, 0, resolution_ };
  static double p5[3] = { 0, resolution_, resolution_ };
  static double p6[3] = { resolution_, resolution_, resolution_ };
  static double p7[3] = { resolution_, 0, resolution_ };

  slideBox->BeginModel();
  slideBox->AddTri( p0, p2, p1, 0 );
  slideBox->AddTri( p0, p2, p3, 1 );
  slideBox->AddTri( p4, p6, p5, 2 );
  slideBox->AddTri( p4, p6, p7, 3 );
  slideBox->AddTri( p0, p5, p4, 4 );
  slideBox->AddTri( p0, p5, p1, 5 );
  slideBox->AddTri( p3, p6, p7, 6 );
  slideBox->AddTri( p3, p6, p2, 7 );
  slideBox->AddTri( p0, p7, p4, 8 );
  slideBox->AddTri( p0, p7, p3, 9 );
  slideBox->AddTri( p1, p6, p5, 10 );
  slideBox->AddTri( p1, p6, p2, 11 );
  //-- Should we add more inner triangles here?
  slideBox->EndModel();

}

/**
 * @function getObjectsData
 * @brief Copy the information of the world objects in our class
 */
void CheckProcess::getObjectsData( std::vector<Object*> objects )
{
  num_objs_ = objects.size();
  objs.resize( num_objs_ );

  for( int i = 0; i < num_objs_; i++ )
  { objs[i].getModelData( objects[i] ); }
}  

/**
 * @function getLinksData
 * @brief Get data from the robot links
 */
void CheckProcess::getLinksData( Robot* robot, std::vector<int> bodies_ID )
{
  num_links_ = bodies_ID.size();
  bodies_ID_ = bodies_ID;
  links.resize( num_links_ );

  for( int i = 0; i < num_links_; i++ ) 
  { 
    links[i].getModelData( robot->links[ bodies_ID[i] ] );
    links[i].updateObjectData( robot->links[ bodies_ID[i] ] );
  }
}


/**
 * @function build_voxel
 * @brief Collision quick checking
 */
void CheckProcess::build_voxel( std::vector<Object*> objects, VoxelGrid<int> &voxel )
{
   time_t ts; time_t tf; double dt;
   objs_voxels.clear();

   int a = 0;
   voxel.reset(a);

   ts = clock();
  //-- For each object
  for( int k = 0; k < num_objs_; k++ )
  {
    objs[k].updateObjectData( objects[k] );

    //-- Convert to voxel grid
    voxel.worldToGrid( objs[k].min_x, objs[k].min_y, objs[k].min_z, objs[k].minCell_x, objs[k].minCell_y, objs[k].minCell_z );
    voxel.worldToGrid( objs[k].max_x, objs[k].max_y, objs[k].max_z, objs[k].maxCell_x, objs[k].maxCell_y, objs[k].maxCell_z );
    //-- 
    double idt[3][3];
    idt[0][0] = 1; idt[0][1] = 0; idt[0][2] = 0;
    idt[1][0] = 0; idt[1][1] = 1; idt[1][2] = 0;
    idt[2][0] = 0; idt[2][1] = 0; idt[2][2] = 1;
  
    double tt[3]; 

    //-- Building the voxel grid, sliding the basic box
    int occupied = 1;  
    for( int i = objs[k].minCell_x; i <= objs[k].maxCell_x; i++ )
     { for( int j = objs[k].minCell_y; j <= objs[k].maxCell_y; j++ )
       { for( int q = objs[k].minCell_z; q <= objs[k].maxCell_z; q++ )
         { 
           voxel.gridToWorld( i, j, q, tt[0], tt[1], tt[2] ); 
           RAPID_Collide( objs[k].R, objs[k].T, objs[k].rapidObject, idt, tt, slideBox, RAPID_FIRST_CONTACT );
           if( RAPID_num_contacts != 0)
            { voxel.setCell( i, j, q, occupied );
              objs_voxels.push_back( Eigen::Vector3i( i, j, q ) );
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

   for( int i = 0; i < voxel.getNumCells(VoxelGrid<int>::DIM_X); i++ )
    { for( int j = 0; j < voxel.getNumCells(VoxelGrid<int>::DIM_Y); j++ )
      { for( int q = 0; q < voxel.getNumCells(VoxelGrid<int>::DIM_Z); q++ )
        { 
          if ( voxel.getCell( i, j, q ) == 1 )        
           { 
             double x, y, z;
             voxel.gridToWorld( i, j, q, x, y, z );
             fprintf( pFile, "  %.4f  %.4f  %.4f  \n", x, y, z ); 
           }
         }
      }
    }  

  fclose( pFile ); 

}

/**
 * @function build_df
 */
void CheckProcess::build_df( PFDistanceField &df )
{
  time_t ts; time_t tf; double dt;
  ts = clock();

  printf("Size obj voxels: %d \n", objs_voxels.size() );

  /** Add points and calculate the DT -- this does everything folks */
  df.addPointsToField( objs_voxels ); 

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
  for( int i = 0; i < num_objs_; i++ )
  { printf( " Object [%d]: center: ( %.3f,%.3f, %.3f ) -- radius: ( %.3f, %.3f, %.3f ) \n", 
	      i, objs[i].center[0], objs[i].center[1], objs[i].center[2], objs[i].radius[0], objs[i].radius[1], objs[i].radius[2] ); }
}


