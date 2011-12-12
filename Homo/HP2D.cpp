/**
 * @file HP2D.cpp
 * @brief Homotopy Paths in 2D
 */
#include "HP2D.h"

/**
 * @function HP2D
 * @ brief Constructor
 */
HP2D::HP2D() {
	mDIST_MAX = 4;
}

/**
 * @function ~HP2D
 * @brief Destructor
 */
HP2D::~HP2D() {

}

/**
 * @function BuildManifold
 */
void HP2D::BuildManifold( Vertice _v0 ) {

    Vertice* va;
    Vertice vt;
	Vertice* vb;
    std::vector<Vertice> S; 
    std::vector<Vertice*> B; 

	InsertQueue( _v0 );

    do{
    	va = PopQueue();        
        S = Successors( va );
        
        for( int i = 0; i < S.size(); i++ ) {
            vt = S[i];
            InsertVertice( vt );
            InsertEdge( &vt, va );
            if( vt.GetDist() < mDIST_MAX ) {
                InsertQueue( vt );
            }
            B = GetAdjacent( va->Adjacent() );
            
            for( unsigned j = 0; j < B.size(); j++ ) {
                vb = B[j];
                if( InSet( vb->GetPos(), vt.Neighbors() ) ) {
                    InsertEdge( &vt, vb );
                }          
            }
        } 
    } while( GetSizeQueue() > 0 ); 
}

/**
 * @function Successors
 */
std::vector<Vertice> HP2D::Successors( Vertice *_va ) {

    std::vector<Vertice> S;
    std::vector<Eigen::Vector2i> N;
    Eigen::Vector2i pi;

    N = _va->Neighbors();

    for( unsigned int i = 0; i < N.size(); i++ ) {
        pi = N[i];

        if( IsFreeGrid( pi ) == true && InSet( pi, Vert2Pos( _va->Adjacent() ) ) == false  ) {
            Vertice vi( pi, _va->GetDist() + 1 );
            S.push_back( vi );
        }
    }    
    return S;
}

/**
 * @function InsertVertice
 */
int HP2D::InsertVertice( Vertice &_vt ) {
    int n = mV.size();
    _vt.SetIndex( n );
	mV.push_back( _vt );
	return n;
}

/**
 * @function InsertEdge
 */
int HP2D::InsertEdge( Vertice *_v1, Vertice *_v2 ) {
	int n = mE.size();
	Edge edge( _v1, _v2 );
	mE.push_back(edge);
	return n;
}

////////////////// Queue Functions /////////////////////////

/**
 * @function InsertQueue
 */
int HP2D::InsertQueue( Vertice _v ) {

    int index = _v.GetIndex();

    mQ.push_back( index );

    return index;
}

/**
 * @function PopQueue
 */
Vertice* HP2D::PopQueue() {
    
	Vertice* v0;

    // Pop first guy without any reordering
    int index_0 = mQ[0];
    mQ.erase( mQ.begin() );    

    v0 = &mV[index_0];

	return v0;
}


//////////////////// 2D GRID FUNCTIONS //////////////////////////

/**
 * @function IsFreeGrid
 */
bool HP2D::IsFreeGrid( Eigen::Vector2i _p ) const {

    return true;
}

/**
 * @function IsFreeGrid
 */
bool HP2D::IsFreeGrid( int _x, int _y ) const {

    return true;
}

/////////////////// AUXILIAR FUNCTIONS ////////////////////////

/**
 * @function GetAdjacent
 */
std::vector<Vertice*> HP2D::GetAdjacent( std::vector<Vertice*> _vert ) {

	std::vector<Vertice*> A;

	for( unsigned int i = 0; i < _vert.size(); i++ ) {
		std::vector<Vertice*> adj = _vert[i]->Adjacent();
		for( unsigned int j = 0; j < adj.size(); j++ ) {
			A.push_back( adj[j] );
		}		
	}

	return A;
}

/**
 * @function Vert2Pos
 */
std::vector<Eigen::Vector2i> HP2D::Vert2Pos( std::vector<Vertice*> _vertices ) const {

    std::vector<Eigen::Vector2i> _pos;
	
	for( int i = 0; i < _vertices.size(); i++ ) {
		_pos.push_back( _vertices[i]->GetPos() );
	}
    return _pos;	
}

/**
 * @function InSet
 * @brief Check if _el is in the _set given
 */
bool HP2D::InSet( Eigen::Vector2i _el, std::vector<Eigen::Vector2i> _set ) const {
	
	for( unsigned int i = 0; i < _set.size(); i++ ) {
		if( _el == _set[i] ) {
			return true;
		}
    }
	return false;
}
