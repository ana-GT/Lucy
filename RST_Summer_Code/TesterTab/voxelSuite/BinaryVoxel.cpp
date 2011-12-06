/**
 * @file binaryVoxel.cpp
 * @brief Definition of class binaryVoxel
 */
#include "BinaryVoxel.h"
#include "EDT.h"
#include <stdio.h>

double const BinaryVoxel::DT_INF;

/**
 * @function BinaryVoxel
 * @brief Constructor
 */
BinaryVoxel::BinaryVoxel( const double &size_x, const double &size_y, const double &size_z, 
			  const double &origin_x, const double &origin_y, const double &origin_z,
			  const double &resolution, const CellState &error_object )
{
  size_[DIM_X] = size_x;
  size_[DIM_Y] = size_y;
  size_[DIM_Z] = size_z;

  origin_[DIM_X] = origin_x;
  origin_[DIM_Y] = origin_y;
  origin_[DIM_Z] = origin_z;

  num_cells_total_ = 1;

  for ( int i = DIM_X; i <= DIM_Z; i++ )
  {
    
    resolution_[i] = resolution;
    num_cells_[i] = (int) round( size_[i]/resolution_[i] );

    num_cells_total_ *= num_cells_[i];
  }

  error_object_ = error_object;

  stride1_ = num_cells_[DIM_Y]*num_cells_[DIM_Z];
  stride2_ = num_cells_[DIM_Z];

  //-- initialize the data:
  data_ = new CellState[num_cells_total_];

  reset( BinaryVoxel::FREE );
}

/**
 * @function ~BinaryVoxel
 * @brief Destructor
 */
BinaryVoxel::~BinaryVoxel()
{
  delete[] data_;
}

/**
 * @function padding
 * @brief What else? Pad!!!
 */
void BinaryVoxel::setPadVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const int& pad, const BinaryVoxel::CellState& obj )
{
  for( int i = -pad; i <= pad; i++ )
  { for( int j = -pad; j <= pad; j++ )
    { for( int k = -pad; k <= pad; k++ )
      { 
        int x = cell_x + i; 
        int y = cell_y + j; 
        int z = cell_z + k;
        if( isVoxelCellValid( x, y, z ) )
        { setVoxelCell( x, y, z, obj ); }
      } 
    }    
  }   
}

/**
 * @function padding
 * @brief What else? Pad!!!
 */
void BinaryVoxel::setPadVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const int& pad, const BinaryVoxel::CellState& obj, const BinaryVoxel::CellState& obj_pad )
{
  for( int i = -pad; i <= pad; i++ )
  { for( int j = -pad; j <= pad; j++ )
    { for( int k = -pad; k <= pad; k++ )
      { 
        int x = cell_x + i; 
        int y = cell_y + j; 
        int z = cell_z + k;
        if( isVoxelCellValid( x, y, z ) && getVoxelCell( x,y,z) != BinaryVoxel::OCCUPIED )
        { setVoxelCell( x, y, z, obj_pad ); }
      } 
    }    
  }   

  if( isVoxelCellValid( cell_x, cell_y, cell_z ) )
  { setVoxelCell( cell_x, cell_y, cell_z, obj ); }

}


/**
 * @function voxel_complement
 * @brief Get the complement of the voxel
 */
BinaryVoxel::CellState* BinaryVoxel::voxel_complement() const
{
  CellState* voxel_complement = new CellState[num_cells_total_];

  for( int i = 0; i < num_cells_total_; i++ )  
  {
    if( data_[i] == BinaryVoxel::FREE )
     { voxel_complement[i] = BinaryVoxel::OCCUPIED; }

    else if( data_[i] == BinaryVoxel::OCCUPIED )
     { voxel_complement[i] = BinaryVoxel::FREE; }
    else
     { printf("--(!) Something was not set here. UNKNOWN cell? \n"); break; }
  } 

  return voxel_complement;
}

/**
 * @function calculateISDT
 */
void BinaryVoxel::calculateISDT( )
{
  ISDT_ = new double[num_cells_total_];

  //-- Calculate the complement of our voxel
  BinaryVoxel::CellState* comp;
  comp = voxel_complement();

  //-- Calculate the EDT of voxel
  EDT* edt_voxel = new EDT();
  double* edt1 = (*edt_voxel).calculateEDT( data_, num_cells_[0], num_cells_[1], num_cells_[2] );


  //-- Calculate the EDT of complement
  EDT* edt_comp = new EDT();
  double* edt2 = (*edt_comp).calculateEDT( comp, num_cells_[0], num_cells_[1], num_cells_[2] );


  double min = 1000;
  double max = -1000;

  //-- Calculate ISDT by substracting edt2 - edt1
  for( int i = 0; i < num_cells_total_; i++ )
  { 
    ISDT_[i] = edt1[i];
    if ( ISDT_[i] < min)
     { min = ISDT_[i];}
    if( ISDT_[i] > max )
     { max = ISDT_[i]; }  
  }


  for( int i = 0; i < num_cells_total_; i++ )
  {
    ISDT_[i]= ISDT_[i] - min ;
  }

 
  min_ISDT_entry_ = min;
  max_ISDT_entry_ = max;
  printf(" Minimum isdt: %.3f \n", min );
  printf(" Maximum isdt: %.3f \n", max );



}
