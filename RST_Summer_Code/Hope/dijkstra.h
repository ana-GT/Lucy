/**
 * @file dijkstra.h
 * @brief Dijkstra algorithm for 3D
 */

#include <float.h>
#include <distance_field/distance_field.h>
#include <distance_field/pf_distance_field.h>
#include <stdio.h>
#include <time.h>

#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#define INF_VAL DBL_MAX

using namespace distance_field;

/**
 * @class State3D
 */
class Node3D
{
   public:
   short int x; short int y; short int z;
   float dist;
   int parent;
   int color; // 2: WHITE(Nowhere) 1: GRAY(OpenList) 0: BLACK (Closed) 3: Obstacle
   Node3D();
   ~Node3D();
   void init( int _x, int _y, int _z );

};

/**
 * @class Dijkstra3D
 */
class Dijkstra3D
{
   public:
   Dijkstra3D();
   ~Dijkstra3D();
   void setGrid3D( VoxelGrid<int>* _grid );
   void setDF( PFDistanceField* _df );
   void setGoal( int _goalX, int _goalY, int _goalZ );
   void search();
   float getDistance( int x, int y, int z );
   std::vector<Eigen::Vector3i> getPath( int ID );
   std::vector<Eigen::Vector3i> getPath( int x, int y, int z );

   static const int mNX[26];
   static const int mNY[26];
   static const int mNZ[26];

   static const float smSqr2;
   static const float smSqr3;
   

   private:
   int goalX; int goalY; int goalZ;
   int mDimX; int mDimY; int mDimZ;
   float mMinObstDist;
   int mStride1; int mStride2;
   int mTotalCells;
   float mBaseCost;

   VoxelGrid<int>* grid;
   Node3D* V;
   std::vector< int >Q;
   int* HT;
   PFDistanceField* distField; 

   time_t ts; time_t tf; double dt;

   void cleanup();
   std::vector<int> neighbors( int ID );
   void insertOpenSet( int ID );
   int popOpenSet( );
   void lowerKeyOpenSet( int ID );
   float edgeCost( int ID1, int ID2 );
   int ref( int _x, int _y, int _z );
};

/*-----------------------------------------------------------*/

/**
 * @function ref
 */
inline int Dijkstra3D::ref( int _x, int _y, int _z )
{
   return ( _x*mStride1 + _y*mStride2 + _z );
}

/**
 */
inline float Dijkstra3D::getDistance( int x, int y, int z )
{
   if( ref(x,y,z) < 0 )
   { return -1; }
   else{ return V[ ref(x,y,z) ].dist; }
}

#endif /** _DIJKSTRA_H_ */
