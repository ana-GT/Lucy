/**
 * @file Vertice.h
 * @date 2011/12/12
 */

#ifndef _HP2D_VERTICE_H_
#define _HP2D_VERTICE_H_

#include <vector>
#include <eigen3/Eigen/Core>

/**
 * @class Vertice
 * @brief Define a Manifold vertice
 */
class Vertice {

private:
  int mPosX;
  int mPosY;
  int mDist;
  int mIndex;
  std::vector<Vertice*> mAdjacent;

public:
  Vertice();
  Vertice( int _x, int _y, int _dist = 0 );
  Vertice( Eigen::Vector2i _p, int _dist = 0 );
  ~Vertice();
  int GetPosX() const;
  int GetPosY() const;
  Eigen::Vector2i GetPos() const;
  int GetDist() const; 
  int GetIndex() const;
  int SetIndex( int _i );

  std::vector<Eigen::Vector2i> Neighbors();
  std::vector<Vertice*> Adjacent() const;	
};

//////////////////// Inline Functions /////////////////////
/**
 * @function GetPosX
 */
inline int Vertice::GetPosX() const{
    return mPosX;
}

/**
 * @function GetPosY
 */
inline int Vertice::GetPosY() const{
    return mPosY;
}

/**
 * @function 
 */
inline Eigen::Vector2i Vertice::GetPos() const {
	Eigen::Vector2i pos; 
	pos << mPosX, mPosY;
    return pos;
}

/**
 * @function GetDist
 */
inline int Vertice::GetDist() const {
    return mDist;
}

/**
 * @function GetIndex
 */
inline int Vertice::GetIndex() const {
    return mIndex;
}

/**
 * @function SetIndex
 */
inline int Vertice::SetIndex( int _i ) {
    mIndex = _i;
}

/**
 * @function Adjacent
 */
inline std::vector<Vertice*> Vertice::Adjacent() const {
    return mAdjacent;
}	

#endif /**_HP2D_EDGE_H_ */

