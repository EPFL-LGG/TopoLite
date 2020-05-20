///////////////////////////////////////////////////////////////
//
// StrucCreator.h
//
//   Create Topological Interlocking Structure
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _STRUC_CREATOR_H
#define _STRUC_CREATOR_H


#include <vector>
#include "ConvexBlock.h"
#include "Mesh/CrossMesh.h"

using namespace std;

template<typename Scalar>
class StrucCreator : public TopoObject
{
public:
    typedef shared_ptr<CrossMesh<Scalar>> pCrossMesh;
    typedef shared_ptr<ConvexBlock<Scalar>> pConvexBlock;
    typedef shared_ptr<Cross<Scalar>> pCross;
    typedef Matrix<Scalar, 2, 1> Vector2;


public:
    vector<pConvexBlock> blocks;

public:

    StrucCreator(shared_ptr<InputVarList> var);
    ~StrucCreator();

public:
    // Create Structure
    bool compute(pCrossMesh crossMesh);
};

#endif