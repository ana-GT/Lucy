/**
 * @file EDT.cpp
 * @brief Define the routines for EDT in the BinaryVoxel class
 */

#include "EDT.h"


/**
 * @function initializeEDT
 * @brief Fill the EDT with default values
 */

/**
 * @function initializer
 */
EDT::EDT()
{
}

void EDT::initializeEDT( BinaryVoxel::CellState* voxel_data, int dim_x, int dim_y, int dim_z )
{
  //-- EDT( Free ) = DT_INF 
  //-- EDT( Occupied ) = 0

  dim_x_ = dim_x;
  dim_y_ = dim_y;
  dim_z_ = dim_z;

  stride1_ = dim_y_*dim_z_;
  stride2_ = dim_z_;

  voxel_data_ = voxel_data;
  EDT_ = new double[dim_x_*dim_y_*dim_z_];

  for( int i = 0; i < dim_x_; i++ )
  { for( int j = 0; j < dim_y_; j++ )
    { for( int k = 0; k < dim_z_; k++ )
      {
        if( getVoxelData( i, j, k ) == BinaryVoxel::FREE )
         { this->setEDTCell( i, j, k, BinaryVoxel::DT_INF ); }
        if( getVoxelData( i, j, k ) == BinaryVoxel::OCCUPIED )
         { this->setEDTCell( i, j, k, 0 ); }
      }
    }
  }
  
}

/**
 * @function calculateEDT
 * @brief Euclidean Distance Transform function
 */
double* EDT::calculateEDT( BinaryVoxel::CellState* voxel_data, int dim_x, int dim_y, int dim_z )
{
  initializeEDT( voxel_data, dim_x, dim_y, dim_z );

  int nx = dim_x_;
  int ny = dim_y_;
  int nz = dim_z_;

  std::vector<double> fcol;
  std::vector<double> dtcol;
  std::vector<BinaryVoxel::CellState> gcol;

  //-- Calculate 1D-EDT for x coordinates
  fcol.resize( nx ); dtcol.resize( nx ); gcol.resize( nx );
  for( int j = 0; j < ny; j++ )
  { for( int k = 0; k < nz; k++ )
    {  for( int i = 0; i < nx ; i++ )
       { gcol[i] = getVoxelData( i, j, k );
         fcol[i] = getEDTCell( i, j, k );
       }
       dtcol = EDT_1D( gcol, fcol ); 
      
       for( int i = 0; i < nx; i++ )
       { setEDTCell( i, j, k, dtcol[i] ); }
    }
  }    

  //-- Calculate 1D-EDT for y coordinates
  fcol.resize( ny ); dtcol.resize( ny ); gcol.resize( ny );
  for( int i = 0; i < nx ; i++ )
  { for( int k = 0; k < nz ; k++ )
    {  for( int j = 0; j < ny ; j++ )
       { gcol[j] = getVoxelData( i, j, k );
         fcol[j] = getEDTCell( i, j, k );
       }
       dtcol = EDT_1D( gcol, fcol ); 
      
       for( int j = 0; j < ny; j++ )
       { setEDTCell( i, j, k, dtcol[j] ); }
    }
  }   

  //-- Calculate 1D-EDT for z coordinates
  fcol.resize( nz ); dtcol.resize( nz ); gcol.resize( nz );
  for( int i = 0; i < nx ; i++ )
  { for( int j = 0; j < ny ; j++ )
    {  for( int k = 0; k < nz ; k++ )
       { gcol[k] = getVoxelData( i, j, k );
         fcol[k] = getEDTCell( i, j, k );
       }
       dtcol = EDT_1D( gcol, fcol ); 
      
       for( int k = 0; k < nz; k++ )
       { setEDTCell( i, j, k, dtcol[k] ); }
    }
  }  

  return EDT_;
}

/**
 * @function EDT_1D
 * @brief Self-explanatory
 */
std::vector<double> EDT::EDT_1D( const std::vector<BinaryVoxel::CellState> &grid, const std::vector<double> &f )
{  

  int k;
  std::vector<int> v;
  std::vector<double> z; 
  std::vector<double> edt;
  bool flag;

  k = 0;
  v.resize(1);
  z.resize(2);
  edt.resize( grid.size() );

  /**< For default, initialize to the first parabola */
  v[ k ] = 0; 
  /**< For default the limits too */
  z[ k ] = -BinaryVoxel::DT_INF;
  z[ k + 1 ] = BinaryVoxel::DT_INF; 

  for( int q = 1; q < grid.size(); q++ )
  {
    double s;
    if( f[q] == BinaryVoxel::DT_INF )
     { continue; }
 
    do
    {
      flag = false;
      s = intersectPoint( q, v[k], f );
      if( s <= z[k] )
       {
         v.pop_back();
         z.pop_back();
         k = v.size() - 1;
         flag = true;

         if ( k < 0 )
         { flag = false; 
           k = 0; }
       } 
    }while( flag == true );

    v.push_back( q );
    z[k + 1] = s;
    z.push_back( BinaryVoxel::DT_INF );   
    k = v.size() - 1;
  }

  int k_ind = 0;

  for( int q = 0; q < grid.size(); q++ )
  {
    while( z[k_ind + 1] < q )
     { 
       if( k_ind == k )
        { break; }
 
       k_ind++; 
     }

    if( f[ v[k_ind] ] == BinaryVoxel::DT_INF )
     { edt[q] = BinaryVoxel::DT_INF; }
    else
     { //edt[q] =  sqrt( (q - v_[k_ind])*(q-v_[k_ind]) ) + f[ v_[k_ind] ];
       edt[q] =  sqrt( (q - v[k_ind])*(q - v[k_ind])  + f[ v[k_ind] ]*f[ v[k_ind] ] );
     }
  }

  return edt;
}

/**
 * @function intersectPoint
 */
double EDT::intersectPoint( const int &next, const int &old, const std::vector<double> &f )
{
  if( f[old] == BinaryVoxel::DT_INF )
  { return -BinaryVoxel::DT_INF; }
  else
  { return  ( ( f[next] + next*next - f[old] - old*old )/ ( 2.0*next - 2.0*old ) ); }
}
