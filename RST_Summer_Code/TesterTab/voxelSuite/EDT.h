/**
 * @file EDT.h
 * @brief Define the routines for EDT in the BinaryVoxel class
 */
#ifndef EDT_H_
#define EDT_H_

//-- EDT
#include <vector>
#include <stdio.h>

#include "BinaryVoxel.h"

/**
 * @class EDT
 * @brief Calculates the EDT in voxel units
 */
class EDT
{
  /**< EDT- functions   */

  public:

  double* EDT_;
  int dim_x_;
  int dim_y_;
  int dim_z_;
  int stride1_;
  int stride2_;
  BinaryVoxel::CellState* voxel_data_;

  /**< Constructor */
  EDT();

  /**< Initialize the EDT with INF or 0 */
  void initializeEDT( BinaryVoxel::CellState* voxel_data, int dim_x, int dim_y, int dim_z );

  /**< calculate EDT */
  double* calculateEDT( BinaryVoxel::CellState* voxel_data, int dim_x, int dim_y, int dim_z );

  /**< Standard 1D method from Pedro */
  std::vector<double> EDT_1D( const std::vector<BinaryVoxel::CellState> &grid, const std::vector<double> &f );

  /**< intersection point between 2 parabolas */
  double intersectPoint( const int &next, const int &old, const std::vector<double> &f );

  /**< Gets the value at a given voxel cell position */
  double& getEDTCell( const int &cell_x, const int &cell_y, const int &cell_z ) const;   

  /**< Set the value at a given voxel cell position */
  void setEDTCell( const int &cell_x, const int &cell_y, const int &cell_z, const double &obj );

  /** Similar to the function in BinaryVoxel */
  BinaryVoxel::CellState& getVoxelData( const int &cell_x, const int &cell_y, const int &cell_z ) const;

  /**< Gives back the location in the data array */
  int ref( const int& x, const int &y, const int &z ) const;

};

//------------------------------------------
//-- EDT functions
//------------------------------------------

/**
 * @function getEDTCell
 * @brief Get the value of the EDT cell
 */
inline double& EDT::getEDTCell( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return EDT_[ ref( cell_x, cell_y, cell_z) ];
}  

/**
 * @function setEDTCell
 */
inline void EDT::setEDTCell( const int &cell_x, const int &cell_y, const int &cell_z, const double &obj )
{
  EDT_[ ref( cell_x, cell_y, cell_z) ] = obj;
}

/**
 * @function getVoxelData
 * @brief Get the value of the voxel data cell
 */
inline BinaryVoxel::CellState& EDT::getVoxelData( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return voxel_data_[ ref( cell_x, cell_y, cell_z) ];
}  


/**
 * @function ref
 * @brief Gives back the position in data
 */
inline int EDT::ref( const int &cell_x, const int &cell_y, const int &cell_z ) const
{
  return ( cell_x*stride1_ + cell_y*stride2_ + cell_z );
}

#endif /* EDT_H_ */
