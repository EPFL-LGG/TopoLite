///////////////////////////////////////////////////////////////
//
// Cross.h
//
//   Cross polygon with two sets of tilt normals for constructing a non-convex polyhedron
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef _CROSS_H
#define _CROSS_H

#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/HelpStruct.h"
#include "TopoLite/Mesh/Polygon.h"
#include "TopoLite/Utility/TopoObject.h"

#include <vector>
#include <memory>
#include <Eigen/Dense>

using namespace std;
using std::weak_ptr;
using std::shared_ptr;
using std::make_shared;

class Cross;
using pCross = shared_ptr<Cross>;

class Cross: public _Polygon, public TopoObject
{
public:
    //storage

	int crossID;                               			//!< Cross ID in the list

	bool atBoundary;									//!< At Boundary or not

	vector<weak_ptr<Cross>> neighbors;                  //!< Neighbors of the cross

	vector<shared_ptr<OrientPoint>> oriPoints;            //!< A set of oriented points for constructing upper polyhedron (saved in the same order as neighbors)

public:
    //temporary variables

	bool isVisited;                           			//!< For correcting tilt angles

public:

    Cross(const Cross &Cross);
    Cross(std::shared_ptr<InputVarList> var);
	~Cross();
	void Print();

public:
    //edge tilt normal

	/*!
	 * \brief Compute Initial Tilt Normal, the rotation angle is 0.
	 */
	void InitTiltNormals();

	/*!
	 * \brief: Initially update all oriPoints in this cross \n
	 * 1) alternatively update the sign by following the order as oriPoints \n
	 * 2) the sign of first orient point is positive \n
	 */
	void UpdateTiltNormal_Root(float tiltAngle);

	/*!
	 * \brief: Update oriPoints in this cross \n
	 * 1) Apply breadth first search \n
	 * 2) The sign of some orient point could be changed in order to resolve conflict \n
	 * 3) In odd polygon, neighboring orient point could have same sign
	 */
	void UpdateTiltNormal(float tiltAngle);

	/*!
	 * \brief: Compute vector rotation
	 * \param[in] normal based vector
	 * \param[in] rotAxis rotation axis
	 * \param[in] rotAngle rotation angle
	 * \return vector after rotation
	 */
    Vector3f RotateNormal(Vector3f normal, Vector3f rotAxis, float rotAngle);

public:
    //neighbor

    /*!
     * \brief: found the EdgeID (order in oriPoints) which correspond crossID is currCrossID
     */
	int GetNeighborEdgeID(int currCrossID);

	int GetVertexEdgeID(int vertexID);

	int GetSharedCross(weak_ptr<Cross> ncross);

	int GetShareEdge(weak_ptr<Cross> ncross);

	int GetPrevEdgeID(int edgeID);
};

#endif
