// Benjamin Cohen
// Test file for BFS3D class

#include <iostream>
#include <bfs_3d/bfs_3d.h>
#include "dijkstra.h"

using namespace std;

int main(int argc, char *argv[])
{
  Node3D(3,4,5);
  VoxelGrid<int> vg( 2.0, 2.0, 2.0, 1.0, 0, 0, 0, 0 );
  int val = 1;
  vg.setCell(1, 0, 0, val);

  Dijkstra3D ns;
  ns.setGoal(0,0,0);
  ns.setGrid3D( &vg );
  ns.search();

  for( int i = 0; i < 2; i++ )
  {   for( int j = 0; j < 2; j++ )
      {  for( int k = 0; k < 2; k++ )
         {  
            printf("Cell[%d,%d,%d] = %f \n", i,j,k, ns.getDistance(i,j,k) );
         }
      }
  }
  return 1;
}
