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

  //-- Start our Binary Voxel
  voxel = new BinaryVoxel( size_x_, size_y_, size_z_, 
			   origin_x_, origin_y_, origin_z_, 
			   resolution_, BinaryVoxel::FREE );  

  //-- Create our slideBox
  initialize();
}

/**
 * @function initialize
 * @brief Initialize the slideBox, basically
 */
void CheckProcess::initialize()
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
    links[i].updateObjectData( robot->links[ bodies_ID[i] ], voxel );
  }
}


/**
 * @function fillVoxel
 * @brief Collision quick checking
 */
void CheckProcess::fillVoxel( std::vector<Object*> objects )
{
   time_t a; time_t b; double d;

  a = clock();
  //-- For each object
  for( int k = 0; k < num_objs_; k++ )
  {
    objs[k].updateObjectData( objects[k], voxel );

    //-- Convert to voxel grid
    voxel->world2VoxelCell( objs[k].min_x, objs[k].min_y, objs[k].min_z, objs[k].minCell_x, objs[k].minCell_y, objs[k].minCell_z );
    voxel->world2VoxelCell( objs[k].max_x, objs[k].max_y, objs[k].max_z, objs[k].maxCell_x, objs[k].maxCell_y, objs[k].maxCell_z );

    //-- 
    double idt[3][3];
    idt[0][0] = 1; idt[0][1] = 0; idt[0][2] = 0;
    idt[1][0] = 0; idt[1][1] = 1; idt[1][2] = 0;
    idt[2][0] = 0; idt[2][1] = 0; idt[2][2] = 1;
  
    double tt[3]; 

    //-- Building the voxel grid, sliding the basic box
    int touchvoxel = 0; 
    for( int i = objs[k].minCell_x; i <= objs[k].maxCell_x; i++ )
     { for( int j = objs[k].minCell_y; j <= objs[k].maxCell_y; j++ )
       { for( int q = objs[k].minCell_z; q <= objs[k].maxCell_z; q++ )
         { 
           voxel->voxelCell2World( i, j, q, tt[0], tt[1], tt[2] ); 
           RAPID_Collide( objs[k].R, objs[k].T, objs[k].rapidObject, idt, tt, slideBox, RAPID_FIRST_CONTACT );
           if( RAPID_num_contacts != 0)
            { voxel->setPadVoxelCell( i, j, q, 0, BinaryVoxel::OCCUPIED );
              touchvoxel++;
            }
         } 
       }
     }

  } //-- End of objects
    b = clock();
    d = (double) (b - a)/CLOCKS_PER_SEC; 
    printf("-- Voxel construction time: %.3f \n", d);

  //-- Write data in a file
  FILE * pFile;
  pFile = fopen( "PointCloudData.txt", "w" );

   for( int i = 0; i < voxel->getNumCells( BinaryVoxel::DIM_X ); i++ )
    { for( int j = 0; j < voxel->getNumCells( BinaryVoxel::DIM_Y ); j++ )
      { for( int q = 0; q < voxel->getNumCells( BinaryVoxel::DIM_Z ); q++ )
        { 
          double x, y, z;
          voxel->voxelCell2World( i, j, q, x, y, z );
          if ( voxel->getVoxelCell( i, j, q ) == BinaryVoxel::OCCUPIED )        
           { fprintf( pFile, "  %.4f  %.4f  %.4f  \n", x, y, z ); }
         }
      }
    }  

  fclose( pFile ); 

  //-- Calculate the EDT of our environment
  a = clock();
  voxel->calculateISDT();
  b = clock();
  d = (double) (b - a)/CLOCKS_PER_SEC;
  printf("-- ISDT calculation time : %f \n", d);
}


/**
 * @function reportObjects
 * @brief print Stuff
 */
void CheckProcess::reportObjects()
{
  for( int i = 0; i < num_objs_; i++ )
  { printf( " Object [%d]: center: ( %.3f,%.3f, %.3f ) -- radius: ( %.3f, %.3f, %.3f )", i, objs[i].center[0], objs[i].center[1], objs[i].center[2], objs[i].radius[0], objs[i].radius[1], objs[i].radius[2] ); }
}

/**
 * @function getLinkVoxels
 * @brief Get voxels belonging to the damned links
 */
void CheckProcess::getLinksVoxels( Robot* robot )
{
   time_t a; time_t b; double d;

   int touchvoxel = 0; 

   std::vector< Eigen::VectorXi > links_voxels_;
   links_voxels_.resize(0);
    a = clock();
  //-- For each object
  for( int k = 0; k < num_links_; k++ )
  {
    links[k].updateObjectData( robot->links[ bodies_ID_[k] ], voxel );

    //-- Convert to voxel grid
    voxel->world2VoxelCell( links[k].min_x, links[k].min_y, links[k].min_z, links[k].minCell_x, links[k].minCell_y, links[k].minCell_z );
    voxel->world2VoxelCell( links[k].max_x, links[k].max_y, links[k].max_z, links[k].maxCell_x, links[k].maxCell_y, links[k].maxCell_z );

    //-- 
    double idt[3][3];
    idt[0][0] = 1; idt[0][1] = 0; idt[0][2] = 0;
    idt[1][0] = 0; idt[1][1] = 1; idt[1][2] = 0;
    idt[2][0] = 0; idt[2][1] = 0; idt[2][2] = 1;
  
    double tt[3]; 

    //-- Building the voxel grid, sliding the basic box

    for( int i = links[k].minCell_x; i <= links[k].maxCell_x; i++ )
     { for( int j = links[k].minCell_y; j <= links[k].maxCell_y; j++ )
       { for( int q = links[k].minCell_z; q <= links[k].maxCell_z; q++ )
         { 
           voxel->voxelCell2World( i, j, q, tt[0], tt[1], tt[2] ); 
           RAPID_Collide( links[k].R, links[k].T, links[k].rapidObject, idt, tt, slideBox, RAPID_FIRST_CONTACT );
           if( RAPID_num_contacts != 0)
            {
              Eigen::VectorXi new_voxel(3);
              new_voxel << i, j, q ;
              links_voxels_.push_back( new_voxel );  
              touchvoxel++;
            }
         } 
       }
     }

  } //-- End of objects

  double mindist = DBL_MAX;

  for( int i = 0; i < links_voxels_.size(); i++ )
   {
     double dist = voxel->getISDTCell(links_voxels_[i](0), links_voxels_[i](1), links_voxels_[i](2));
     if( dist < mindist )
      { mindist = dist; }
   }

  b = clock();
  d = (double) (b - a)/CLOCKS_PER_SEC; 
  printf(" -- Robot link Voxels -- time : %.3f -- Voxels: %d - min value ISDT: %.3f \n", d, touchvoxel, mindist );

  //-- Write data in a file
  FILE * pFile;
  pFile = fopen( "Links.txt", "w" );

  for( int q = 0; q < links_voxels_.size(); q++ )
    { 
       double x, y, z;
       voxel->voxelCell2World( links_voxels_[q](0), links_voxels_[q](1), links_voxels_[q](2), x, y, z );
       fprintf( pFile, "  %.4f  %.4f  %.4f  \n", x, y, z ); 
     }
  fclose(pFile);
}
