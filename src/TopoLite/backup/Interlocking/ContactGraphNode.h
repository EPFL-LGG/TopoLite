//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHNODE_H
#define TOPOLOCKCREATOR_CONTACTGRAPHNODE_H

#include "ContactGraphEdge.h"
#include <map>
#include <memory>

using std::shared_ptr;
using std::weak_ptr;
using std::vector;
using std::pair;

/*!
 * \brief Graph node records one part's geometry and contacts with its neighbors.
 */

template<typename Scalar>
class ContactGraphNode
{
public:

    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;

    typedef shared_ptr<ContactGraphEdge<Scalar>> pContactGraphEdge;

    typedef weak_ptr<ContactGraphEdge<Scalar>> wpContactGraphEdge;

    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;
    
    using ContactNeighbor = pair<wpContactGraphNode, wpContactGraphEdge> ;

    typedef Matrix<Scalar, 3, 1> Vector3;
    
public:

    bool isBoundary; // True if the part is fixed

    Vector3 centroid; // the arithmetic mean position of all the points

    Vector3 centerofmass; // the gravity center

    float mass;

public:

    ContactGraphNode(bool _isBoundary, Vector3 _centroid, Vector3 _centerofmass, float _mass)
    {
        isBoundary = _isBoundary;
        centroid = _centroid;
        centerofmass = _centerofmass;
        mass = _mass;
    }

    float height(){
        return centerofmass[1];
    }


public: //Automatic Generated

    vector<ContactNeighbor> neighbors; // neighbor and the corresponding contact

    int staticID; // Static part ID (include the fixed parts).

    int dynamicID; // Dynamic part ID (exclude the fixed parts)
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPHNODE_H
