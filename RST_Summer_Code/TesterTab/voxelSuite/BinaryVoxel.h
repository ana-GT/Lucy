/**
 * @file BinaryVoxel
 * @brief Voxel + EDT + Signed EDT
 * @author A. Huaman
 */
#ifndef BINARY_VOXEL_H_
#define BINARY_VOXEL_H_
#include <cfloat>
#include <algorithm>
#include <math.h>
#include <Eigen/Core>

/**
 * @function BinaryVoxel
 */
class BinaryVoxel
{
  public: 
   double static const DT_INF = DBL_MAX;

   enum Dimension{ DIM_X = 0, DIM_Y = 1, DIM_Z = 2 };
   enum CellState{ FREE = 0, OCCUPIED = 1, RISKY = 2, UNDETERMINED = 3 };    
 
   /**< Constructor */
   BinaryVoxel( const double &size_x, const double &size_y, const double &size_z, 
		const double &origin_x, const double &origin_y, const double &origin_z,
		const double &resolution, const CellState &error_object );

   /**< Destructor */
   virtual ~BinaryVoxel();

   /**< Gets the value at the given 3D position */
   const CellState& operator() ( double x, double y, double z ) const;

   /**< Gets the value at a given voxel cell position */
   CellState& getVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z ) const;   

   /**< Set the value at a given voxel cell position */
   void setVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const BinaryVoxel::CellState &obj );

   /**< Padding guy */
   void setPadVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const int& pad, const BinaryVoxel::CellState& obj );

   /**< Padding guy - 2 */
   void setPadVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const int& pad, const BinaryVoxel::CellState& obj, const BinaryVoxel::CellState& obj_pad );

   /**< Reset the entire grid to a given initial state */
   void reset( const CellState &init_value );

   /**< Get the complement of the grid */
   CellState* voxel_complement() const;

   /**< Gets the size of the dimension given */
   double getSize( const Dimension &dim ) const;

   /**< Gets the resolution of the given dimension */
   double getResolution( const Dimension &dim ) const;

   /**< Gets the origin of the given dimension */
   double getOrigin( const Dimension &dim ) const;

   /**< Gets the number of cells of the given dimension */
   int getNumCells( const Dimension &dim ) const;

   /**< Converts voxel cell coordinates to world coordinates */
   bool voxelCell2World( const int &cell_x, const int &cell_y, const int &cell_z, 
 			 double &x, double &y, double &z ) const;

   /**< Convert world coordinate to voxel */
   bool world2VoxelCell( const double &x, const double &y, const double &z,
 			 int &cell_x, int &cell_y, int &cell_z ) const;

   /**< Convert world coordinate to voxel */
   bool world2VoxelCellMax( const double &x, const double &y, const double &z,
 			    int &cell_x, int &cell_y, int &cell_z ) const;

   /**< Get center in workspace..meaning world coordinate */
   Eigen::VectorXd getCenter( const int &cell_x, const int &cell_y, const int &cell_z ) const;

   //------------- ISDT
   /**< Calculate the inverse Signed Distance Transform */
   void calculateISDT();

   double getISDTCell( const int &cell_x, const int &cell_y, const int &cell_z ) const;

  /**<  Protected Data */

   double* SDT_;
   double* ISDT_;
 
   double min_ISDT_entry_;
   double max_ISDT_entry_;
   CellState error_object_;
   CellState** data_pointers_;
   double size_[3];  
   double resolution_[3];
   double origin_[3];
   int num_cells_[3];
   int num_cells_total_;
   int stride1_;
   int stride2_;

   /**< Gets the reference in the data array for the given cell location */
   int ref( const int &cell_x, const int &cell_y, const int &cell_z ) const;

   /**< Gets the cell location in data_ from its world position */
   int getCellPosFromWorldPos( const Dimension &dim, const double &world_dim ) const;

   /**< Variant that use ceil instead of floor */
   int getCellPosFromWorldPosMax( const Dimension &dim, const double &world_dim ) const;

   /**< Gets the world position from cell location in data */
   double getWorldPosFromCellPos( const Dimension &dim, const int &cell ) const;

   /**< Checks if the given cell is within the binary voxel */
   bool isVoxelCellValid( const int &cell_x, const int &cell_y, const int &cell_z ) const;

   /**< Checks validity of the given cell for a particular dimension */
   bool isVoxelCellValid( const Dimension &dim, const int &cell ) const;

  protected:

   CellState* data_;

};

/*** ---------------------------------------------- */

/**
 * @function operator
 * @brief Not a function really
 */
inline const BinaryVoxel::CellState& BinaryVoxel::operator()(double x, double y, double z) const
{
  int cell_x = getCellPosFromWorldPos( DIM_X, x );
  int cell_y = getCellPosFromWorldPos( DIM_Y, y );
  int cell_z = getCellPosFromWorldPos( DIM_Z, z );

  if( !isVoxelCellValid( cell_x, cell_y, cell_z ) )
    { return error_object_; }

  return getVoxelCell( cell_x, cell_y, cell_z );
}

/**
 * @function isVoxelCellValid
 * @brief Check if the cell is valid in our voxel
 */
inline bool BinaryVoxel::isVoxelCellValid( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return (
           cell_x >= 0 && cell_x < num_cells_[DIM_X] &&
           cell_y >= 0 && cell_y < num_cells_[DIM_Y] &&
           cell_z >= 0 && cell_z < num_cells_[DIM_Z] );
}

/**
 * @function isVoxelCellValid
 * @brief Check if the cell is valid in our voxel
 */
inline bool BinaryVoxel::isVoxelCellValid( const Dimension &dim, const int &cell_dim ) const
{
  return ( cell_dim >= 0 && cell_dim < num_cells_[dim] );
}

/**
 * @function getVoxelCell
 * @brief Get the value of the voxel cell
 */
inline BinaryVoxel::CellState& BinaryVoxel::getVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return data_[ ref( cell_x, cell_y, cell_z ) ];
}

/**
 * @function setVoxelCell
 */
inline void BinaryVoxel::setVoxelCell( const int &cell_x, const int &cell_y, const int &cell_z, const BinaryVoxel::CellState &obj )
{
  data_[ ref( cell_x, cell_y, cell_z) ] = obj;
}


/**
 * @function reset
 * @brief Reset the voxel to a value
 */
inline void BinaryVoxel::reset( const CellState &init_value )
{
  std::fill( data_, data_ + num_cells_total_, init_value );
}

/**
 * @function getSize
 * @brief Get size of the dimension X, Y or Z
 */
inline double BinaryVoxel::getSize( const Dimension &dim ) const
{
  return size_[dim];
}

/**
 * @function getResolution
 * @brief Get resolution of the dimension. It is the same for 3 dimensions. Still
 */
inline double BinaryVoxel::getResolution( const Dimension &dim ) const
{
  return resolution_[dim];
}

/**
 * @function getOrigin
 * @brief Get origin in the dimension
 */
inline double BinaryVoxel::getOrigin( const Dimension &dim ) const
{
  return origin_[dim];
}

/**
 * @function getNumCells
 * @brief Get number of cells
 */
inline int BinaryVoxel::getNumCells( const Dimension &dim ) const
{
  return num_cells_[dim];
}

/**
 * @function voxelCell2World
 * @brief Converts voxel cell coordinates to world coordinates
 */
inline bool BinaryVoxel::voxelCell2World( const int &cell_x, const int &cell_y, const int &cell_z, 
 			 		  double &x, double &y, double &z ) const
{
  x = getWorldPosFromCellPos( DIM_X, cell_x );
  y = getWorldPosFromCellPos( DIM_Y, cell_y );
  z = getWorldPosFromCellPos( DIM_Z, cell_z );
  return isVoxelCellValid( cell_x, cell_y, cell_z );
}

/**
 * @function world2VoxelCell
 * @brief Convert world coordinate to voxel 
 */
inline bool BinaryVoxel::world2VoxelCell( const double &x, const double &y, const double &z,
					  int &cell_x, int &cell_y, int &cell_z ) const
{
  cell_x = getCellPosFromWorldPos( DIM_X, x );
  cell_y = getCellPosFromWorldPos( DIM_Y, y );
  cell_z = getCellPosFromWorldPos( DIM_Z, z );
  return isVoxelCellValid( cell_x, cell_y, cell_z );
}

/**< Convert world coordinate to voxel */
inline bool BinaryVoxel::world2VoxelCellMax( const double &x, const double &y, const double &z,
 			    int &cell_x, int &cell_y, int &cell_z ) const
{
  cell_x = getCellPosFromWorldPosMax( DIM_X, x );
  cell_y = getCellPosFromWorldPosMax( DIM_Y, y );
  cell_z = getCellPosFromWorldPosMax( DIM_Z, z );
  return isVoxelCellValid( cell_x, cell_y, cell_z );
}

/**
 * @function getCellPosFromWorldPos
 * @brief Gets the cell location in data_ from its world position
 */
inline int BinaryVoxel::getCellPosFromWorldPos( const Dimension &dim, const double &world_dim ) const
{
  return int( floor(( world_dim - origin_[dim] )/resolution_[dim]) );
}

/**
 * @function getCellPosFromWorldPosMax
 * @brief Gets the cell location in data_ from its world position
 */
inline int BinaryVoxel::getCellPosFromWorldPosMax( const Dimension &dim, const double &world_dim ) const
{
  return int( ceil(( world_dim - origin_[dim] )/resolution_[dim]) );
}

/**
 * @function getWorldPosFromCellPos
 * @brief Gets the world position from cell location in data 
 */
inline double BinaryVoxel::getWorldPosFromCellPos( const Dimension &dim, const int &cell ) const
{
  return origin_[dim] + resolution_[dim]*(double(cell));
}

/**
 * @function getCenter
 * @brief Get center of voxel
 */
inline Eigen::VectorXd BinaryVoxel::getCenter( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  double x = origin_[DIM_X] + resolution_[DIM_X]*( double(cell_x) + 0.5 );
  double y = origin_[DIM_Y] + resolution_[DIM_Y]*( double(cell_y) + 0.5 );
  double z = origin_[DIM_Z] + resolution_[DIM_Z]*( double(cell_z) + 0.5 );
  Eigen::VectorXd center(3); center<< x, y, z;

  return center;
}

/**
 * @function ref
 * @brief Gives back the position in data_ of the voxel cell
 */
inline int BinaryVoxel::ref( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return ( cell_x*stride1_ + cell_y*stride2_ + cell_z );
}


//--- ISDT
inline double BinaryVoxel::getISDTCell( const int &cell_x, const int &cell_y, const int &cell_z ) const 
{
  return ISDT_[ ref( cell_x, cell_y, cell_z ) ];
}


#endif /* BINARY_VOXEL_H_ */
