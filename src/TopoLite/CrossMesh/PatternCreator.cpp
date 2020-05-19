///////////////////////////////////////////////////////////////
//
// MeshCreator.cpp
//
//   Create Polygonal Meshes
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#include "Utility/HelpDefine.h"

#include "CrossMesh/BaseMeshCreator.h"
#include "CrossMesh/PatternCreator.h"

#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"

#include <queue>

using std::queue;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//
#include "PatternCreator.h"

template<typename Scalar>
PatternCreator<Scalar>::PatternCreator()
{

}

template<typename Scalar>
PatternCreator<Scalar>::~PatternCreator()
{

}

//**************************************************************************************//
//                           Create Mesh (2D Regular Pattern)
//**************************************************************************************//

template<typename Scalar>
bool PatternCreator<Scalar>::checkPolygonExistance(PatternCreator::pPolygon poly, const PatternCreator::setVertex &vertices_set) {
    for(size_t j = 0; j < poly->size(); j++)
    {
        Vector3 edge_sta = poly->pos(j);
        Vector3 edge_end = poly->pos(j + 1);
        Vector3 edge_mid = (edge_sta + edge_end) / 2;
        if(vertices_set.find(edge_mid) == vertices_set.end()){
            return false;
        }
    }
    return true;
}

template<typename Scalar>
void PatternCreator<Scalar>::addToVerticesSet(pPolygon poly, setVertex &vertices_set)
{
    // add the vertices into the vertices_set
    for(size_t j = 0; j < poly->size(); j++){
        Vector3 edge_sta = poly->pos(j);
        Vector3 edge_end = poly->pos(j + 1);
        Vector3 edge_mid = (edge_sta + edge_end) / 2;
        if(vertices_set.find(edge_mid) == vertices_set.end()){
            vertices_set.insert(edge_mid);
        }
    }
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PatternCreator<Scalar>::rotateVector(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle, Vector3 tagtPt)
{
    // Get the rotation matrix using OpenGL modelView matrix
    Matrix<Scalar, 4, 4> trans1 = Matrix<Scalar, 4, 4>::Identity();
    trans1(0, 3) = rotCenter.x(); trans1(1, 3) = rotCenter.y(); trans1(2, 3) = rotCenter.z();

    Matrix<Scalar, 4, 4> rotate = Matrix<Scalar, 4, 4>::Identity();
    Scalar x, y, z, c, s;
    x = rotAxis.x(); y = rotAxis.y(); z = rotAxis.z();
    c = std::cos(rotAngle / 180 * M_PI);
    s = std::sin(rotAngle / 180 * M_PI);
    rotate << c + x * x *(1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s, 0,
            y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s, 0,
            z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c), 0,
            0, 0, 0, 1;

    Matrix<Scalar, 4, 4> trans2 = Matrix<Scalar, 4, 4>::Identity();
    trans2(0, 3) = -rotCenter.x(); trans2(1, 3) = -rotCenter.y(); trans2(2, 3) = -rotCenter.z();

    Matrix<Scalar, 4, 1> pos;
    pos << tagtPt.x(), tagtPt.y(), tagtPt.z(), 1;

    Matrix<Scalar, 4, 1> result = trans1 * rotate * trans2 * pos;
    return result.head(3);
}

template<typename Scalar>
void PatternCreator<Scalar>::create2DPattern(PatternType patternID,
                                             int patternRadius,
                                             pCrossMesh &crossMesh)
{
    // The algorithm is a BFS searching.
    // Each time, we pop a polygon from the queue
    //, expand the polygon by finding all its neighbouring polygon
    // and add the neighbouring polygons into the queue
    // until the depth of the BFS tree reaches the user specific number.


    //BFS(Breadth-First Search) Node Data Structure
    //
    struct BFSNode{
        pPolygon poly;              // the polygon
        int depth;                  // the depth of the BFS tree

        BFSNode(pPolygon _poly, int _depth){
            depth = _depth;
            poly = _poly;
        }
    };

    //the mesh class of the pattern
    pPolyMesh patternMesh = make_shared<PolyMesh<Scalar>>(getVarList());

    //a set to record all vertices of the pattern mesh
    //a very helpful tool to verify whether a polygon already exist in patternMesh
    setVertex vertices_set;

    //BFS node
    pPolygon root_poly;
    createPolygonRoot(patternID, CROSS_L, root_poly);
    addToVerticesSet(root_poly, vertices_set);
    BFSNode node_root(root_poly, 0);

    // create a BFS queue and add the root to the queue
    queue<BFSNode> bfsQueue;
    bfsQueue.push(node_root);

    // start Breadth-First Search Traversal
    while(!bfsQueue.empty())
    {
        BFSNode curr_node = bfsQueue.front();
        bfsQueue.pop();

        if (curr_node.depth >= patternRadius)
        {
            // if deepth is larger than a user specific number, exit
            break;
        }
        else{
            // add the polygon into the mesh
            patternMesh->polyList.push_back(curr_node.poly);
        }

        // find a one ring neighbor of the polygon.
        vector<pPolygon> neighbor_polys;
        computeNeighbors(patternID, curr_node.poly, neighbor_polys);

        for (size_t i = 0; i < neighbor_polys.size(); i++)
        {
            if(!checkPolygonExistance(neighbor_polys[i], vertices_set)){

                // if the polygon is not existed in the queue
                // add the polygon into the queue
                BFSNode next_node(neighbor_polys[i], curr_node.depth + 1);
                bfsQueue.push(next_node);

                // update the vertices_set to avoid duplicate polygon inside the queue
                addToVerticesSet(neighbor_polys[i], vertices_set);
            }
        }
    }


    patternMesh->update();
    patternMesh->normalize();

    crossMesh = make_shared<CrossMesh<Scalar>>(*patternMesh);
    crossMesh->baseMesh2D = patternMesh;

    return;
}


//**************************************************************************************//
//                           Create Mesh (2D Regular Pattern)
//**************************************************************************************//

template<typename Scalar>
void PatternCreator<Scalar>::computeNeighbors(PatternType patternID, pPolygon poly, vector<pPolygon> &neighbors)
{
	for (size_t i = 0; i < poly->vers.size(); i++)
	{
		Vector3 staPt = poly->vers[i]->pos;
		Vector3 endPt = poly->vers[(i + 1) % poly->vers.size()]->pos;

		pPolygon neighbor;

		if      ( patternID == CROSS_SQUARE )                     ComputeNeighbor_Square(poly, neighbor);
		else if ( patternID == CROSS_HEXAGON )                    ComputeNeighbor_Hexagon(poly, neighbor);
		else if ( patternID == CROSS_OCTAGON_SQUARE )             ComputeNeighbor_Octagon_Square(poly, i, neighbor);
		else if ( patternID == CROSS_DODECAGON )                  ComputeNeighbor_Dodecagon(poly, i, neighbor);
		else if ( patternID == CROSS_HEXAGON_RHOMBUS )            ComputeNeighbor_Hexagon_Rhombus(poly, i, neighbor);
		else if ( patternID == CROSS_DODECAGON_HEXAGON_QUAD )     ComputeNeighbor_Dodecagon_Hexagon_Square(poly, i, neighbor);
		else if  (patternID == CROSS_SQUARE_RHOMBUS )             ComputeNeighbor_Square_Rhombus(poly, i, neighbor);
		else if  (patternID == CROSS_PENTAGON_CROSS )             ComputeNeighbor_Pentagon_Cross(poly, i, neighbor);
		else if  (patternID == CROSS_PENTAGON_SNOW )              ComputeNeighbor_Pentagon_Snow(poly, i, neighbor);
		else if  (patternID == CROSS_PENTAGON_MIRROR )            ComputeNeighbor_Pentagon_Mirror(poly, i, neighbor);
		else if  (patternID == CROSS_RHOMBUS )                    ComputeNeighbor_Rhombus(poly, i, neighbor);
		else if  (patternID == CROSS_OCTAGON_SQUARE_COLINEAR )    ComputeNeighbor_Octagon_Square_Colinear(poly, i, neighbor);

		if( neighbor != NULL )
		{
			// Translate the neighborCross
			Vector3 transVec;

			if( patternID == CROSS_OCTAGON_SQUARE_COLINEAR )
			{
				transVec = ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(poly, i);
			}
			else
			{
				transVec = ComputeTileTranslation(neighbor, staPt, endPt);
			}

			neighbor->translatePolygon(transVec);

			neighbors.push_back(neighbor);
		}
	}
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PatternCreator<Scalar>::ComputeTileTranslation(pPolygon neighbor, Vector3 tagtStaPt, Vector3 tagtEndPt)
{
	Vector3 tagtVec = (tagtEndPt - tagtStaPt).normalized();
	Vector3 tagtMidPt = (tagtEndPt + tagtStaPt) / 2.0f;

	// Identify the edge shared by tagtCross and neighborCross
	Vector3 transVec = Vector3(0, 0, 0);
	bool reverseTheta;
	for (size_t i = 0; i < neighbor->vers.size(); i++)
	{
		Vector3 staPt = neighbor->vers[i]->pos;
		Vector3 endPt = neighbor->vers[(i + 1) % neighbor->vers.size()]->pos;

		Vector3 edgeVec = (endPt - staPt).normalized();
		Vector3 edgeMidPt = (endPt + staPt) / 2.0f;

		Scalar dotp = tagtVec.dot(edgeVec);
		if (fabs(dotp + 1) < FLOAT_ERROR_LARGE)
		{
			transVec = tagtMidPt - edgeMidPt;
			break;
		}
	}

	//printf("transVec: [%.6f %.6f %.6f] \n", transVec.x, transVec.y, transVec.z);

	return transVec;
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PatternCreator<Scalar>::ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(pPolygon poly, int edgeID)
{
	Vector3 transVec;

	if( poly->getPolyType() == POLY_OCTAGON_COLINEAR)
	{
		Scalar A = (poly->vers[1]->pos - poly->vers[0]->pos).norm();

		if(edgeID == 0)         transVec = Vector3( 1.5*A,   0.5*A, 0);
		else if(edgeID == 1)    transVec = Vector3(     A,   2.0*A, 0);
		else if(edgeID == 2)    transVec = Vector3(-0.5*A,   1.5*A, 0);
		else if(edgeID == 3)    transVec = Vector3(-2.0*A,       A, 0);
		else if(edgeID == 4)    transVec = Vector3(-1.5*A,  -0.5*A, 0);
		else if(edgeID == 5)    transVec = Vector3(    -A,  -2.0*A, 0);
		else if(edgeID == 6)    transVec = Vector3( 0.5*A,  -1.5*A, 0);
		else if(edgeID == 7)    transVec = Vector3( 2.0*A,      -A, 0);

		transVec += poly->center();

		//printf("transVec [%.3f %.3f %.3f] \n", transVec.x, transVec.y, transVec.z);
	}

	else if( poly->getPolyType() == POLY_SQUARE_THETA_45 )
	{
		Scalar B = (poly->vers[1]->pos - poly->vers[0]->pos).norm();

		if(edgeID == 0)         transVec = Vector3( -0.5*B,   1.5*B, 0);
		else if(edgeID == 1)    transVec = Vector3( -1.5*B,  -0.5*B, 0);
		else if(edgeID == 2)    transVec = Vector3(  0.5*B,  -1.5*B, 0);
		else if(edgeID == 3)    transVec = Vector3(  1.5*B,   0.5*B, 0);

		transVec += poly->center();
	}

	return transVec;
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Square(pPolygon poly, pPolygon &neighbor)
{
	CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Hexagon(pPolygon polym, pPolygon &neighbor)
{
	CreatePolygon_Hexagon(neighbor, CROSS_L);
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Octagon_Square(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if(poly->getPolyType() == POLY_SQUARE_THETA_45)
	{
		CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_REGULAR);
	}
	else if (poly->getPolyType() == POLY_OCTAGON_REGULAR)
	{
		if( edgeID %2 == 0 )    CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_REGULAR);
		else                    CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Hexagon_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if     (poly->getPolyType() == POLY_RHOMBUS_THETA_90)
	{
		CreatePolygon_Hexagon(neighbor, CROSS_L);
	}
	else if ( poly->getPolyType() == POLY_HEXAGON_TYPE_0 )
	{
		if( edgeID == 1 || edgeID == 4 )
			CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
		else
			CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Dodecagon(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( edgeID % 2 == 1 ) {
		neighbor = nullptr;
		return;
	}

	CreatePolygon_Dodecagon(neighbor, CROSS_L);
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Square_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	if( poly->getPolyType() == POLY_SQUARE_THETA_15 )
	{
		if( edgeID == 0 )             CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 1 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 2 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 3 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );

	}
	else if( poly->getPolyType() == POLY_SQUARE_THETA_75 )
	{
		if( edgeID == 0 )             CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 1 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 2 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 3 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
	}
	else if( poly->getPolyType() == POLY_RHOMBUS_THETA_0 )
	{
		if( edgeID == 0 )             CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 1 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
		else if( edgeID == 2 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 3 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
	}
	else if( poly->getPolyType() == POLY_RHOMBUS_THETA_90 )
	{
		if( edgeID == 0 )             CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 1 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
		else if( edgeID == 2 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 3 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Dodecagon_Hexagon_Square(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	// Three types of Squares
	if     (poly->vers.size() == 4)
	{
		if( poly->getPolyType() == POLY_SQUARE_THETA_45 )
		{
			if( edgeID == 0 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1 );
			else if (edgeID == 2)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0 );
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->getPolyType() == POLY_SQUARE_THETA_75 )
		{
			if( edgeID == 1 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1);
			else if (edgeID == 3)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->getPolyType() == POLY_SQUARE_THETA_15 )
		{
			if( edgeID == 1 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
			else if (edgeID == 3)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1);
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
	}

	// Two types of Hexagons
	else if ( poly->vers.size() == 6 )
	{
		if( poly->getPolyType() == POLY_HEXAGON_TYPE_1 )
		{
			if( edgeID == 0 )       CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_75);
			else if (edgeID == 2)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_15);
			else if (edgeID == 4)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
			else                    CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->getPolyType() == POLY_HEXAGON_TYPE_0 )
		{
			if( edgeID == 1 )       CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
			else if (edgeID == 3)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_75);
			else if (edgeID == 5)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_15);
			else                    CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
	}

	// One type of Dodecagons
	else if (poly->vers.size() == 12 )
	{
		if( edgeID == 1 || edgeID == 7 )                         CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_15);
		else if( edgeID == 3 || edgeID == 9 )                    CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_75);
		else if( edgeID == 5 || edgeID == 11 )                   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
		else if ( edgeID == 0 || edgeID == 4 || edgeID == 8 )    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1);
		else                                                     CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Pentagon_Cross(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->getPolyType() == POLY_PENTAGON_CROSS_TYPE_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
	}

	else if( poly->getPolyType() == POLY_PENTAGON_CROSS_TYPE_1 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
	}

	else if( poly->getPolyType() == POLY_PENTAGON_CROSS_TYPE_2 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
	}

	else if( poly->getPolyType() == POLY_PENTAGON_CROSS_TYPE_3 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Pentagon_Snow(pPolygon poly, int edgeID, pPolygon &neighbor)
{
    if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_0 )
    {
        if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
        else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
        else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
        else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
        else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
    }

    else if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_1 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
    }

    else if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_2 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
    }

    else if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_3 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
    }

	else if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_4 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
	}

	else if( poly->getPolyType() == POLY_PENTAGON_SNOW_TYPE_5 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Pentagon_Mirror(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->getPolyType() == POLY_PENTAGON_MIRROR_TYPE_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
	}

	else if( poly->getPolyType() == POLY_PENTAGON_MIRROR_TYPE_1 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	if( poly->getPolyType() == POLY_RHOMBUS_THETA_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
	}

	else if( poly->getPolyType() == POLY_RHOMBUS_THETA_120 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
	}

	else if( poly->getPolyType() == POLY_RHOMBUS_THETA_240 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::ComputeNeighbor_Octagon_Square_Colinear(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->getPolyType() == POLY_OCTAGON_COLINEAR )
	{
		if( edgeID == 0 )        CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
		else if ( edgeID == 1 )  CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if ( edgeID == 2 )  CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
		else if ( edgeID == 3 )  CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if ( edgeID == 4 )  CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
		else if ( edgeID == 5 )  CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if ( edgeID == 6 )  CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
		else if ( edgeID == 7 )  CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
	}

	else if( poly->getPolyType() == POLY_SQUARE_THETA_45 )
	{
		if( edgeID == 0 )       CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if (edgeID == 1)   CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if (edgeID == 2)   CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
		else if (edgeID == 3)   CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_COLINEAR);
	}
}

//**************************************************************************************//
//                                   Create Polygons 
//**************************************************************************************//

template<typename Scalar>
void PatternCreator<Scalar>::createPolygonRoot(int patternID, Scalar edgeLen, pPolygon &poly)
{
	if      (patternID == CROSS_SQUARE)                      CreatePolygon_Square(poly, edgeLen, POLY_SQUARE_THETA_45);
	else if (patternID == CROSS_HEXAGON)                     CreatePolygon_Hexagon(poly, edgeLen, POLY_HEXAGON_TYPE_0);
	else if (patternID == CROSS_OCTAGON_SQUARE)              CreatePolygon_Octagon(poly, edgeLen, POLY_OCTAGON_REGULAR);
	else if (patternID == CROSS_DODECAGON)                   CreatePolygon_Dodecagon(poly, edgeLen);
	else if (patternID == CROSS_HEXAGON_RHOMBUS)             CreatePolygon_Hexagon(poly, edgeLen, POLY_HEXAGON_TYPE_0);
	else if (patternID == CROSS_DODECAGON_HEXAGON_QUAD)      CreatePolygon_Dodecagon(poly, edgeLen);
	else if (patternID == CROSS_SQUARE_RHOMBUS)              CreatePolygon_Square(poly, edgeLen, POLY_SQUARE_THETA_15);
	else if (patternID == CROSS_PENTAGON_CROSS)              CreatePolygon_Pentagon_Cross(poly, edgeLen, POLY_PENTAGON_CROSS_TYPE_0);
    else if (patternID == CROSS_PENTAGON_SNOW)               CreatePolygon_Pentagon_Snow(poly, edgeLen, POLY_PENTAGON_SNOW_TYPE_0);
	else if (patternID == CROSS_PENTAGON_MIRROR)             CreatePolygon_Pentagon_Mirror(poly, edgeLen, POLY_PENTAGON_MIRROR_TYPE_0);
	else if (patternID == CROSS_RHOMBUS)                     CreatePolygon_Rhombus(poly, edgeLen, POLY_RHOMBUS_THETA_120);
	//else if (patternID == CROSS_OCTAGON_SQUARE_COLINEAR)     poly = CreatePolygon_Square(edgeLen, POLY_SQUARE_THETA_45);
	else if (patternID == CROSS_OCTAGON_SQUARE_COLINEAR)     CreatePolygon_Octagon(poly, edgeLen, POLY_OCTAGON_COLINEAR);
}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Square(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar initTheta;
	if     ( polyType == POLY_SQUARE_THETA_45 )      initTheta = 45;
	else if( polyType == POLY_SQUARE_THETA_15 )      initTheta = 15;
	else if( polyType == POLY_SQUARE_THETA_75 )      initTheta = 75;

	Scalar r = 0.5f * edgeLen / cos(45* M_PI / 180.0);

	//Scalar initTheta = 45;
	Scalar stepTheta = 90;

	for (int i = 0; i < 4; ++i)
	{
		Scalar currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3 point = Vector3(r*cos(currTheta), r*sin(currTheta), 0);

		poly->push_back(point);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Pentagon_Cross(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar rotTheta;
	if     ( polyType == POLY_PENTAGON_CROSS_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_1 )      rotTheta = 90;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_2 )      rotTheta = 180;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_3 )      rotTheta = 270;

	Scalar A = 0.5f * edgeLen;
	Scalar B = 1.5f * edgeLen;

	poly->push_back( Vector3( 0,  0, 0) );
	poly->push_back( Vector3( A, -A, 0) );
	poly->push_back( Vector3( B, -A, 0) );
	poly->push_back( Vector3( B,  A, 0) );
	poly->push_back( Vector3( A,  A, 0) );

	// Rotate the whole polygon
	for (size_t i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i]->pos = rotateVector(Vector3(0,0,0), Vector3(0,0,1), rotTheta, poly->vers[i]->pos);
	}
}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Pentagon_Snow(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar rotTheta;
	if     ( polyType == POLY_PENTAGON_SNOW_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_1 )      rotTheta = 60;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_2 )      rotTheta = 120;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_3 )      rotTheta = 180;
    else if( polyType == POLY_PENTAGON_SNOW_TYPE_4 )      rotTheta = 240;
    else if( polyType == POLY_PENTAGON_SNOW_TYPE_5 )      rotTheta = 300;

    Scalar A = edgeLen;
	Scalar B = 0.5f * edgeLen;
    Scalar C = 1.25f * edgeLen;
    Scalar D = (sqrtf(3.0f) / 4.0) * edgeLen;
    Scalar E = (sqrtf(3.0f) / 2.0) * edgeLen;

	poly->push_back(Vector3( 0,  0, 0 ));
	poly->push_back(Vector3( A,  0, 0 ));
	poly->push_back(Vector3( C,  D, 0 ));
	poly->push_back(Vector3( A,  E, 0 ));
	poly->push_back(Vector3( B,  E, 0 ));

    // Rotate the whole polygon
	for (size_t i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i]->pos = rotateVector(Vector3(0,0,0), Vector3(0,0,1), rotTheta, poly->vers[i]->pos);
	}

}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Pentagon_Mirror(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar rotTheta;
	if     ( polyType == POLY_PENTAGON_MIRROR_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_MIRROR_TYPE_1 )      rotTheta = 180;

	Scalar A = edgeLen;
	Scalar B = 0.5f * edgeLen;
	Scalar C = (sqrtf(3.0f) / 2.0) * edgeLen;
	Scalar D = ((sqrtf(3.0f) / 4.0) + 0.5f) * edgeLen;

	poly->push_back(Vector3( 0, -C, 0));
	poly->push_back(Vector3( B, -C, 0));
	poly->push_back(Vector3( D,  0, 0));
	poly->push_back(Vector3( B,  C, 0));
	poly->push_back(Vector3( 0,  C, 0));

	// Rotate the whole polygon
	for (size_t i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i]->pos = rotateVector(Vector3(0,0,0), Vector3(0,0,1), rotTheta, poly->vers[i]->pos);
	}

}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Hexagon(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar r = 0.5f * edgeLen / cos(60* M_PI / 180.0);

	Scalar initTheta = 0;
	Scalar stepTheta = 60;

	for (int i = 0; i < 6; ++i)
	{
		Scalar currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3 point = Vector3(r*cos(currTheta), r*sin(currTheta), 0);

		poly->push_back(point);
	}


}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Octagon(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	if( polyType == POLY_OCTAGON_REGULAR )
	{
		Scalar r = 0.5f * edgeLen / cos(67.5* M_PI / 180.0);

		Scalar initTheta = 22.5;
		Scalar stepTheta = 45;

		for (int i = 0; i < 8; ++i)
		{
			Scalar currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

			Vector3 point = Vector3(r*cos(currTheta), r*sin(currTheta), 0);

			poly->push_back(point);
		}
	}

	else if(polyType == POLY_OCTAGON_COLINEAR )
	{
		Scalar A = edgeLen;

		poly->push_back( Vector3( A,  0, 0) );
		poly->push_back( Vector3( A,  A, 0) );
		poly->push_back( Vector3( 0,  A, 0) );
		poly->push_back( Vector3(-A,  A, 0) );
		poly->push_back( Vector3(-A,  0, 0) );
		poly->push_back( Vector3(-A, -A, 0) );
		poly->push_back( Vector3( 0, -A, 0) );
		poly->push_back( Vector3( A, -A, 0) );
	}


}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Dodecagon(pPolygon &poly, Scalar edgeLen)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( POLY_DODECAGON );

	Scalar r = 0.5f * edgeLen / cos(75* M_PI / 180.0);

	Scalar initTheta = 15;
	Scalar stepTheta = 30;

	for (int i = 0; i < 12; ++i)
	{
		Scalar currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3 point = Vector3(r*cos(currTheta), r*sin(currTheta), 0);

		poly->push_back(point);
	}


}

template<typename Scalar>
void PatternCreator<Scalar>::CreatePolygon_Rhombus(pPolygon &poly, Scalar edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon<Scalar>>();

	poly->setPolyType( polyType );

	Scalar rotTheta;
	if     ( polyType == POLY_RHOMBUS_THETA_0 )        rotTheta = 0;
	else if( polyType == POLY_RHOMBUS_THETA_90 )       rotTheta = 90;
	else if( polyType == POLY_RHOMBUS_THETA_120 )      rotTheta = 120;
	else if( polyType == POLY_RHOMBUS_THETA_240 )      rotTheta = 240;


	Scalar A = (sqrtf(3.0f) / 2.0) * edgeLen;
	Scalar B = 0.5f * edgeLen;

	poly->push_back(Vector3( A,  0, 0));
	poly->push_back(Vector3( 0,  B, 0));
	poly->push_back(Vector3(-A,  0, 0));
	poly->push_back(Vector3( 0, -B, 0));

	// Rotate the whole polygon
	for (size_t i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i]->pos = rotateVector(Vector3(0,0,0), Vector3(0,0,1), rotTheta, poly->vers[i]->pos);
	}


}


//**************************************************************************************//
//                           Python Interface
//**************************************************************************************//
template<typename Scalar>
vector<vector<double>> PatternCreator<Scalar>::PyCreateMesh_2DPattern(int patternID, int patternRadius) {
    pCrossMesh crossMesh;
    create2DPattern(PatternType(patternID), patternRadius, crossMesh);

    vector<vector<double>> polys;
    for(size_t id = 0; id < crossMesh->baseMesh2D->polyList.size(); id++){
        pPolygon poly = crossMesh->baseMesh2D->polyList[id];
        vector<double> coords;
        for(size_t jd = 0; jd < poly->vers.size(); jd++){
            coords.push_back(poly->vers[jd]->pos[0]);
            coords.push_back(poly->vers[jd]->pos[1]);
        }
        polys.push_back(coords);
    }
    return polys;
}

template class PatternCreator<double>;
