/**
 * @file HP2D.h
 * @brief Homotopy Paths in 2D
 */
#include <vector>
#include "Vertice.h"
#include "Edge.h"

/**
 * @class HP2D
 */
class HP2D {

private:
	std::vector<Vertice> mV;
	std::vector<Edge> mE;
	std::vector<int> mQ; /** Queue with vertices indices */

	int mDIST_MAX;

public:

    //-- Manifold direct functions
	HP2D();	
	~HP2D();
    void BuildManifold( Vertice _v0 );
    std::vector<Vertice> Successors( Vertice *_va );
	int InsertVertice( Vertice &_vt );
	int InsertEdge( Vertice *_v1, Vertice *_v2 );

    //-- Queue related functions
    int GetSizeQueue() const;
    int InsertQueue( Vertice _v );
    Vertice* PopQueue();

    //-- Grid 2D related functions
	bool IsFreeGrid( Eigen::Vector2i _p ) const;
	bool IsFreeGrid( int _x, int _y ) const;

	//-- Auxiliar functions
	std::vector<Eigen::Vector2i> Vert2Pos( std::vector<Vertice*> _vertices ) const;
	bool InSet( Eigen::Vector2i _el, std::vector<Eigen::Vector2i> _set ) const; 
	std::vector<Vertice*> GetAdjacent( std::vector<Vertice*> _vert );

};


////////////// Inline Functions //////////////////////

/**
 * @function GetSizeQueue
 */
inline int HP2D::GetSizeQueue() const {
    return mQ.size();
} 
