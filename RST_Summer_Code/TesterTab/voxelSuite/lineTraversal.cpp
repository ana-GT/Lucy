/**
 * @file lineTraversal.cpp
 */

#include "lineTraversal.h"

/**
 * @function getLineVoxels
 */
std::vector< Eigen::VectorXi > getLineVoxels( const int &startCell_x, const int &startCell_y, const int &startCell_z,
					      const int &endCell_x, const int &endCell_y, const int &endCell_z )
{
 
  //-- 1. Initialization
  //-- 0 <= delta < 1
  double delta = 0.5;

  //-- a. Get where the line begins (integer locations)
  int X = startCell_x;
  int Y = startCell_y;
  int Z = startCell_z;

  //-- Line Equation L: u + t*v

  double uX = ( (double)startCell_x + delta );
  double uY = ( (double)startCell_y + delta );
  double uZ = ( (double)startCell_z + delta );

  double vX = ( (double)endCell_x + delta ) - uX; 
  double vY = ( (double)endCell_y + delta ) - uY; 
  double vZ = ( (double)endCell_z + delta ) - uZ; 

  //-- b. Get the steps
  int stepX = getStep( vX );
  int stepY = getStep( vY );
  int stepZ = getStep( vZ );

  //-- c. Get the values of t for crossing to a new x, y or z
  double tMaxX; double tMaxY; double tMaxZ;

  if( vX != 0 ){ tMaxX = fabs( ( 1.0 - delta ) / vX ); } else { tMaxX = DBL_MAX; }; 
  if( vY != 0 ){ tMaxY = fabs( ( 1.0 - delta ) / vY ); } else { tMaxY = DBL_MAX; };
  if( vZ != 0 ){ tMaxZ = fabs( ( 1.0 - delta ) / vZ ); } else { tMaxZ = DBL_MAX; };

  //-- d. Get the values of t for crossing to a new x, y or z
  double tDeltaX; double tDeltaY; double tDeltaZ;

  if( vX != 0 ){ tDeltaX = stepX / vX; } else { tMaxX = DBL_MAX; }; 
  if( vY != 0 ){ tDeltaY = stepY / vY; } else { tMaxY = DBL_MAX; }; 
  if( vZ != 0 ){ tDeltaZ = stepZ / vZ; } else { tMaxZ = DBL_MAX; }; 

  //-- 2. Incremental Phase
  Eigen::VectorXi newVoxel(3);
  std::vector< Eigen::VectorXi > lineVoxels;
  lineVoxels.resize(0);

  do
  {
    newVoxel(0) = X; newVoxel(1) = Y; newVoxel(2) = Z;
    lineVoxels.push_back( newVoxel );

    if( tMaxX < tMaxY )
     { if( tMaxX < tMaxZ )
        { X = X + stepX;
          tMaxX = tMaxX + tDeltaX; 
        }
       else
        { Z = Z + stepZ;
          tMaxZ = tMaxZ + tDeltaZ; 
        }
     } 
    else
     {
       if( tMaxY < tMaxZ )
        { Y = Y + stepY;
          tMaxY = tMaxY + tDeltaY;
        }
       else
       { Z = Z + stepZ;
         tMaxZ = tMaxZ + tDeltaZ;
       }
     }

  }while( X != endCell_x || Y != endCell_y || Z != endCell_z );

  newVoxel(0) = X; newVoxel(1) = Y; newVoxel(2) = Z;
  lineVoxels.push_back( newVoxel );

  return lineVoxels;
}

/**
 * @function getStep
 */
int getStep( const int &diff )
{
  int step;

  if( diff < 0 )
   { step = -1; }
  else if( diff > 0 )
   { step = 1; }
  else
   { step = 0; }

  return step;
}
