/**
 * @file Dijkstra.h
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
 * @class Node3D
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
   void Init( int x, int y, int z );

};

/**
 * @class Dijkstra3D
 * @brief Implements a Dijkstra search in a voxelized 3D environment 
 */
class Dijkstra3D
{
   public:
   Dijkstra3D();
   ~Dijkstra3D();
   void SetGrid3D( VoxelGrid<int>* grid );
   void SetDF( PFDistanceField* df );
   void SetGoal( int goalX, int goalY, int goalZ );
   void Search();
   float GetDistance( int x, int y, int z );
   std::vector<Eigen::Vector3i> GetPath( int ID );
   std::vector<Eigen::Vector3i> GetPath( int x, int y, int z );

   static const int mNX[26];
   static const int mNY[26];
   static const int mNZ[26];

   static const float smSqr2;
   static const float smSqr3;
   

   private:
   int mGoalX; int mGoalY; int mGoalZ;
   int mDimX; int mDimY; int mDimZ;
   float mMinObstDist;
   int mStride1; int mStride2;
   int mTotalCells;
   float mBaseCost;

   VoxelGrid<int>* mGrid;
   Node3D* mV;
   std::vector< int > mQ;
   int* mHT;
   PFDistanceField* mDistField; 

   time_t ts; time_t tf; double dt;

   void Cleanup();
   std::vector<int> GetNeighbors( int id );
   void InsertOpenSet( int id );
   int PopOpenSet( );
   void LowerKeyOpenSet( int id );
   float GetEdgeCost( int id1, int id2 );
   int GetRef( int x, int y, int z );
};

/*-----------------------------------------------------------*/

/**
 * @function GetRef
 * @brief Get the real location of a 3D cell (x,y,z) in the data array (1D)
 */
inline int Dijkstra3D::GetRef( int x, int y, int z )
{
   return ( x*mStride1 + y*mStride2 + z );
}

/**
 * @function GetDistance
 * @brief After performing the Dijkstra's search, it gives the distance of the shortest path from the cell to goal
 */
inline float Dijkstra3D::GetDistance( int x, int y, int z )
{
  int ind = GetRef(x,y,z);
   if( ind < 0 )
   { return -1; }
   else{ return mV[ ind ].dist; }
}

#endif /** _DIJKSTRA_H_ */
