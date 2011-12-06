#include "lineTraversal.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/LU> 

int main( int argc, char* argv[] )
{
  int N = 10;
  Eigen::MatrixXd A( N, N );

  if( N%2 == 0 )
  {
    A( 0, 0 ) = 1;
    A( 1, 0 ) = -2; A( 1, 1 ) = 1;

    for( int i = 2; i < N-2; i++ )
    { 
      A( i, i ) = 1;
    
      if( i < N/2 )
      { A( i, i-1 ) = -2; 
        A( i, i-2 ) = 1;
      }
 
      else
      { A( i, i+1 ) = -2;
        A( i, i+2 ) = -1;
      }
    }  

    A( N-2, N-2 ) = 1; A( N-2, N-1 ) = -2;
    A( N-1, N-1 ) = 1; 
  } 

  std::cout << A << std::endl; 

  Eigen::MatrixXd R( N, N );
  R = A.transpose()*A;

//  Eigen::MatrixXd Ainv = A.inverse();
  std::cout << R << std::endl;
  std::cout << "  -- " << std::endl;
  std::cout << R.inverse() << std::endl;
  return 0;
}
