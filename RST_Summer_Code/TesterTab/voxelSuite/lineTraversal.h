#ifndef LINE_TRAVERSAL_H
#define LINE_TRAVERSAL_H

/**
 * @file lineTraversal.h
 * @author A. Huaman
 */
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <Eigen/Core>
#include <vector>

/** Function declarations */
std::vector< Eigen::VectorXi > getLineVoxels( const int &startCell_x, const int &startCell_y, const int &startCell_z,
					      const int &endCell_x, const int &endCell_y, const int &endCell_z );

int getStep( const int &diff );

#endif /**< LINE_TRAVERSAL_H */
