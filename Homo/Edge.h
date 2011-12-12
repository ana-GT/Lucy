/**
 * @file Edge.h
 */
#ifndef _HP2D_EDGE_H_
#define _HP2D_EDGE_H_

#include "Vertice.h"

/**
 * @class Edge
 * @brief Define a Manifold Edge
 */
class Edge {

private:
  int mV1_index;
  int mV2_index;
  Vertice* mV1;
  Vertice* mV2;

public:
  Edge( Vertice *_v1, Vertice *_v2 );
  ~Edge();
};

#endif /** _HP2D_EDGE_H_ */

