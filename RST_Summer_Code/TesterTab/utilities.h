#ifndef INVERSE_H 
#define INVERSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <amino/include/amino.h>

/**
 * @file utilities.h
 * @brief Functions for pseudo guys. Using amino (not my code, by the way) 
 */

void pseudoInv( int n, int m, Eigen::MatrixXd J, Eigen::MatrixXd &Jinv );
void getRPY( const Eigen::Transform<double, 3, Eigen::Affine> &T, double &roll, double &pitch, double &yaw );

#endif /** INVERSE_H */
