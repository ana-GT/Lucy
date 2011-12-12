/**
 * @file Vertice.cpp
 * @brief 2D Manifold Vertice class
 */
#include "Vertice.h"

/**
 * @function Vertice
 * @brief Constructor
 */
Vertice::Vertice( ) {
}

/**
 * @function Vertice
 * @brief Constructor
 */
Vertice::Vertice( int _x, int _y, int _dist ) {

	mPosX = _x;
    mPosY = _y;
    mDist = _dist;
}

/**
 * @function Vertice
 * @brief Constructor
 */
Vertice::Vertice( Eigen::Vector2i _p, int _dist ) {
    mPosX = _p(0);
    mPosY = _p(1);
	mDist = _dist;
}

/**
 * @function ~Vertice
 * @brief Destructor
 */
Vertice::~Vertice() {

}

/**
 * @function Neighbors
 * @brief Returns the 4 adjacent grid positions to the vertex
 */
std::vector<Eigen::Vector2i> Vertice::Neighbors() {

	std::vector<Eigen::Vector2i> neighbors;
	Eigen::Vector2i n;
    n << mPosX - 1, mPosY; neighbors.push_back( n );
	n << mPosX + 1, mPosY; neighbors.push_back( n );
	n << mPosX, mPosY - 1; neighbors.push_back( n );
	n << mPosX, mPosY + 1; neighbors.push_back( n );
    
	return neighbors;      
}




