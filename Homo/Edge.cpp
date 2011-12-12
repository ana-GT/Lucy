/**
 * @file Edge.cpp
 * @brief Define a Manifold Edge
 */

#include "Edge.h"

/**
 * @function Edge
 * @brief Constructor
 */
Edge::Edge( Vertice *_v1, Vertice *_v2 ) {

 mV1 = _v1;
 mV2 = _v2;
}

/**
 * @function Edge
 * @brief Destructor
 */
Edge::~Edge() {
}



