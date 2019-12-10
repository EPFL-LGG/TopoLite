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

class ContactGraphNode;

using pContactGraphNode = shared_ptr<ContactGraphNode>;
using pContactGraphEdge = shared_ptr<ContactGraphEdge>;
using wpContactGraphEdge = weak_ptr<ContactGraphEdge>;
using wpContactGraphNode = weak_ptr<ContactGraphNode>;

using ContactNeighbor = pair<wpContactGraphNode, wpContactGraphEdge> ;

/*!
 * \brief Graph node records one part's geometry and contacts with its neighbors.
 */
class ContactGraphNode
{

public:

    bool isBoundary; //!< True if the part is fixed

    EigenPoint centroid; //!< the arithmetic mean position of all the points

    EigenPoint centerofmass; //!< the gravity center

    float mass;

public:

    ContactGraphNode(bool _isBoundary, EigenPoint _centroid, EigenPoint _centerofmass, float _mass)
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

    vector<ContactNeighbor> neighbors; //!< neighbor and the corresponding contact

    int staticID; //!< Static part ID (include the fixed parts).

    int dynamicID; //!< Dynamic part ID (exclude the fixed parts)
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPHNODE_H
