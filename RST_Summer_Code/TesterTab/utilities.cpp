/**
 * @file utilities.cpp
 * @brief Some functions for pseudo inverse and that stuff (sort of a wrapper for the amino column-major functions)
 * @author achq
 */

#include "utilities.h"

/**
 * @function pseudoInv
 * @brief J (mxn) --> Jinv(nxm)
 */
void pseudoInv( int m, int n, Eigen::MatrixXd J, Eigen::MatrixXd &Jinv )
{
 //-- J: m rows, n columns
 //-- Jinv: n rows, m columns

 double J_array[ m*n ];
 double Jinv_array[ m*n ];
 for( int i = 0; i < n; i++ )
 { for( int j = 0; j < m; j++ )
   { J_array[ j + m*i ] = J( j, i ); }
 }

 //-- Amino routine (yes I get it, the WORLD should be column-major!
 aa_la_dpinv( m, n, 0, J_array, Jinv_array ) ;

 for( int i = 0; i < m; i++ )
 { for( int j = 0; j < n; j++ )
   { Jinv( j, i ) = Jinv_array[ j + n*i ] ; }
 }

}

/***********ORIENTATION FUNCTIONS *******************/
/**
 * @function getRPY
 * @brief Get roll, pitch and yaw from a Transform in Eigen
 */
void getRPY( const Eigen::Transform<double, 3, Eigen::Affine> &T, double &roll, double &pitch, double &yaw )
{
  roll = atan2( T(2,1), T(2,2) );
  pitch = atan2( -T(2,0), sqrt( T(2,1)*T(2,1) + T(2,2)*T(2,2) )  );
  yaw = atan2( T(1,0), T(0,0) ); 
}
