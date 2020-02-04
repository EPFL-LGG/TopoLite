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
#include "Utility/HelpFunc.h"

#include "CrossMesh/BaseMeshCreator.h"
#include "CrossMesh/PatternCreator.h"

#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/MeshConverter.h"
#include "Mesh/HEdgeMesh.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PatternCreator::PatternCreator()
{

}

PatternCreator::~PatternCreator()
{

}

//**************************************************************************************//
//                           Create Mesh from Multiple Meshes
//**************************************************************************************//

// TODO: remove duplicated vertices and faces
void PatternCreator::CreateMesh_Merge(vector<pPolyMesh> polyMeshes, pPolyMesh &polyMesh)
{
	polyMesh.reset();
	polyMesh = make_shared<PolyMesh>(getVarList());

//	for (int i = 0; i < polyMeshes.size(); i++)
//	{
//		Mesh *tempMesh = polyMeshes[i];
//
//		for (int j = 0; j < tempMesh->vertexList.size(); j++)
//		{
//			polyMesh->vertexList.push_back( tempMesh->vertexList[j] );
//		}
//	}

	for (int i = 0; i < polyMeshes.size(); i++)
	{
		pPolyMesh tempMesh = polyMeshes[i];

		for (int j = 0; j < tempMesh->polyList.size(); j++)
		{
			tempMesh->polyList[j]->ComputeNormal();

			polyMesh->polyList.push_back( tempMesh->polyList[j] );
		}
	}

    polyMesh->removeDuplicatedVertices();

	//MeshConverter myMeshConverter;
	//myMeshConverter.Convert2PolyMesh( polyMesh, true );
}

//**************************************************************************************//
//                           Create Mesh (2D Regular Pattern)
//**************************************************************************************//

void PatternCreator::CreateMesh_2DPattern( int patternID,
                                        int patternRadius,
                                        shared_ptr<CrossMesh> &crossMesh)
{
    if(patternID == 13)
    {
        CreateMesh_2DPattern_HexagonPattern2(patternRadius, crossMesh);
    }
    else if(patternID == 14)
    {
        CreateMesh_2DPattern_HexagonPattern3(patternRadius, crossMesh);
    }
    else if(patternID == 15)
    {
        CreateMesh_2DPattern_PentagonPattern3(patternRadius, crossMesh);
    }
    else{

        shared_ptr<PolyMesh> polyMesh= make_shared<PolyMesh>(getVarList());

        pPolygon poly;
        CreatePolygon_Root(patternID, CROSS_L, poly);
        polyMesh->polyList.push_back(poly);

        vector<pPolygon> bfsQueue;

        // Find a root for Breadth-First Search Traversal
        pPolygon root = polyMesh->polyList[0];
        root->SetDistance(0);
        bfsQueue.push_back(root);

        //return polyMesh;

        // Start Breadth-First Search Traversal
        //for (int j = 0; j <2 ; ++j)
        while (true)
        {
            pPolygon currVisit = bfsQueue[0];
            bfsQueue.erase(bfsQueue.begin());

            if (currVisit->GetDistance() >= patternRadius)
            {
                break;
            }

            vector<pPolygon> neighbors;
            ComputeNeighbors(patternID, currVisit, neighbors);

            for (int i = 0; i < neighbors.size(); i++)
            {
                pPolygon nextVisit = neighbors[i];

                nextVisit->SetDistance(currVisit->GetDistance() + 1);

                int index = GetPolygonIndexInList(nextVisit, polyMesh->polyList);
                if (index == ELEMENT_OUT_LIST)
                {
                    polyMesh->polyList.push_back(nextVisit);
                    bfsQueue.push_back(nextVisit);
                }
                else
                {
                    nextVisit.reset();
                }
            }
        }
        polyMesh->removeDuplicatedVertices();
        Vector3f trans;
        float scale;
        polyMesh->NormalizeMesh(trans, scale);

        BaseMeshCreator meshCreator(getVarList());
        crossMesh = make_shared<CrossMesh>(getVarList());
        meshCreator.InitCrossMesh(polyMesh, crossMesh);
        pHEdgeMesh hedgeMesh = make_shared<HEdgeMesh>();
        hedgeMesh->InitHEdgeMesh(polyMesh);
        hedgeMesh->BuildHalfEdgeMesh();
        meshCreator.ComputeCrossNeighbors(hedgeMesh, crossMesh);
        crossMesh->baseMesh2D = polyMesh;
    }

    if(crossMesh){
        crossMesh->setVarList(getVarList());
    }

    return;
}

void PatternCreator::CreateMesh_2DPattern_HexagonPattern2(int patternRadius, shared_ptr<CrossMesh> &crossMesh)
{
    Vector3f DX, DY;
    vector<_Polygon> root_polys;
    vector<pPolygon> hex;
    hex.resize(4);
    CreatePolygon_Hexagon(hex[0], 1);

    int cornerCentroid[3] = {1, 3, 5};

    for(int id = 0; id < 3; id++){
        CreatePolygon_Hexagon(hex[id + 1], 0.3);
        int verID = cornerCentroid[id];
        hex[id + 1]->Translate(Vector3f(hex[0]->vers[verID].pos[0], hex[0]->vers[verID].pos[1], 0));
    }

    int polyID[4][6] = {
            {2, 2, 2, 2, 2, 2},
            {1, 1, 0, 2, 2, 0},
            {3, 3, 0, 2, 2, 0},
            {1, 1, 0, 3, 3, 0},
    };
    int verID[4][6] = {
            {0, 1, 2, 3, 4, 5},
            {4, 3, 2, 1, 0, -1},
            {3, 2, -1, 0, 5, 4},
            {5, 4, -1, 2, 1, 0}
    };

    for(int id = 0; id < 4; id++){
        _Polygon poly;
        for(int jd = 0; jd < 6; jd++)
        {
            int pID = polyID[id][jd], vID = verID[id][jd];
            if(vID >= 0)
            {
                poly.vers.push_back(_Vertex(hex[pID]->vers[vID].pos));
            }
            else{
                poly.vers.push_back(_Vertex(Vector3f(0, 0, 0)));
            }
        }
        poly.ComputeCenter();
        poly.ComputeNormal();
        root_polys.push_back(poly);
    }

    DY = hex[1]->vers[0].pos - hex[2]->vers[0].pos;
    DX = 3.0f * hex[0]->vers[0].pos;


    CreateMesh_2DPattern(root_polys, DX, DY, patternRadius,2 * patternRadius, crossMesh);
}

void PatternCreator::CreateMesh_2DPattern_HexagonPattern3(int patternRadius, shared_ptr<CrossMesh> &crossMesh)
{
    Vector3f DX, DY;
    vector<_Polygon> root_polys;
    vector<pPolygon> hex;
    hex.resize(4);
    double dH = 0.8;
    CreatePolygon_Hexagon(hex[0], 1);
    CreatePolygon_Hexagon(hex[1], 1 + dH);
    CreatePolygon_Hexagon(hex[2], 1 + dH / 2);

    hex[3] = make_shared<_Polygon>();
    for(int id = 0; id < 6; id++){
        int jd = (id + 1) % 6;
        Vector3f dx = hex[0]->vers[jd].pos - hex[0]->vers[id].pos; dx /= len(dx);
        Vector3f dv = hex[1]->vers[jd].pos - hex[0]->vers[id].pos;
        Vector3f dh = dv - dx * (dv DOT  dx);
        hex[3]->vers.push_back(_Vertex(hex[0]->vers[id].pos + dh));
        hex[3]->vers.push_back(_Vertex(hex[0]->vers[jd].pos + dh));
    }

    int polyID[4][6] = {
            {0, 0, 0, 0, 0, 0},
            {0, 2, 3, 3, 2, 0},
            {0, 2, 3, 3, 2, 0},
            {0, 2, 3, 3, 2, 0},
    };
    int verID[4][6] = {
            {0, 1, 2, 3, 4, 5},
            {1, 1, 2, 3, 2, 2},
            {2, 2, 4, 5, 3, 3},
            {3, 3, 6, 7, 4, 4}
    };

    for(int id = 0; id < 4; id++){
        _Polygon poly;
        for(int jd = 0; jd < 6; jd++)
        {
            int pID = polyID[id][jd], vID = verID[id][jd];
            if(vID >= 0)
            {
                poly.vers.push_back(_Vertex(hex[pID]->vers[vID].pos));
            }
            else{
                poly.vers.push_back(_Vertex(Vector3f(0, 0, 0)));
            }
        }
        poly.ComputeCenter();
        poly.ComputeNormal();
        root_polys.push_back(poly);
    }


    DY = root_polys[3].vers[2].pos - root_polys[0].vers[1].pos;
    DX = 2.0f * (root_polys[2].vers[5].pos - root_polys[2].vers[4].pos)
            + (root_polys[0].vers[0].pos - root_polys[0].vers[3].pos)
            + (hex[2]->vers[1].pos - hex[2]->vers[2].pos);
    CreateMesh_2DPattern(root_polys, DX, DY, patternRadius,2 * patternRadius, crossMesh);
}

void PatternCreator::CreateMesh_2DPattern_PentagonPattern3(int patternRadius, shared_ptr<CrossMesh> &crossMesh)
{
    Vector3f DX, DY;
    vector<_Polygon> root_polys;
    vector<pPolygon> hex;
    hex.resize(2);
    double scaleX = 0.8;
    CreatePolygon_Hexagon(hex[0], 1);

    for(int id = 0; id < hex[0]->vers.size(); id++){
        hex[0]->vers[id].pos = Vector3f(hex[0]->vers[id].pos[0] * scaleX,  hex[0]->vers[id].pos[1], 0);
    }

    Vector3f a = hex[0]->vers[1].pos - hex[0]->vers[2].pos;
    Vector3f b = hex[0]->vers[3].pos - hex[0]->vers[2].pos;
    double angle = acos(a DOT b / len(a) / len(b)) / 2;
    hex[1] = make_shared<_Polygon>();

    float y0 = hex[0]->vers[1].pos[1]  - len(a) / 2 * tan(angle);
    float y1 = hex[0]->vers[5].pos[1]  + len(a) / 2 * tan(angle);

    hex[1]->vers.push_back(_Vertex(Vector3f(0, y0, 0)));
    hex[1]->vers.push_back(_Vertex(Vector3f(0, y1, 0)));
    hex[1]->vers.push_back(_Vertex((hex[0]->vers[0].pos + hex[0]->vers[1].pos) * 0.5f));
    hex[1]->vers.push_back(_Vertex((hex[0]->vers[2].pos + hex[0]->vers[3].pos) * 0.5f));
    hex[1]->vers.push_back(_Vertex((hex[0]->vers[3].pos + hex[0]->vers[4].pos) * 0.5f));
    hex[1]->vers.push_back(_Vertex((hex[0]->vers[0].pos + hex[0]->vers[5].pos) * 0.5f));

    int polyID[5][5] = {
            {0, 0, 0, 0, 0},
            {1, 0, 0, 1, 1},
            {1, 1, 0, 1, 1},
            {1, 1, 1, 1, 0},
            {1, 1, 1, 0, 0}
    };
    int verID[5][5] = {
            {0, 1, 2, 3, 4},
            {2, 1 ,2 ,3 ,0},
            {0, 3, 3, 4, 1},
            {2, 0, 1, 5, 0},
            {5, 1, 4, 4, 5}
    };

    for(int id = 1; id < 5; id++){
        _Polygon poly;
        for(int jd = 0; jd < 5; jd++)
        {
            int pID = polyID[id][jd], vID = verID[id][jd];
            if(vID >= 0)
            {
                poly.vers.push_back(_Vertex(hex[pID]->vers[vID].pos));
            }
            else{
                poly.vers.push_back(_Vertex(Vector3f(0, 0, 0)));
            }
        }
        poly.ComputeCenter();
        poly.ComputeNormal();
        root_polys.push_back(poly);
    }


    DY = hex[1]->vers[2].pos - hex[1]->vers[4].pos;
    DX = hex[0]->vers[1].pos - hex[0]->vers[2].pos + hex[0]->vers[0].pos - hex[0]->vers[3].pos;

    CreateMesh_2DPattern(root_polys, DX, DY, patternRadius,2 * patternRadius, crossMesh);
}

void PatternCreator::CreateMesh_2DPattern(vector<_Polygon> &root_polys, Vector3f DX, Vector3f DY, int Nx, int Ny,
                                       shared_ptr<CrossMesh> &crossMesh) {
    shared_ptr<PolyMesh> polyMesh= make_shared<PolyMesh>(getVarList());
    for(int id = 0; id < Ny; id++)
    {
        //row
        for(int jd = 0; jd < Nx; jd++)
        {
            //col
            for(int kd = 0; kd < root_polys.size(); kd++){
                shared_ptr<_Polygon> poly = make_shared<_Polygon>(root_polys[kd]);
                float dx = ((id % 2 == 0) ? 0 : 1) * DY[0] + DX[0] * jd;
                float dy = DY[1] * id;
                poly->Translate(Vector3f(dx, dy, 0));
                polyMesh->polyList.push_back(poly);
            }
        }
    }

    polyMesh->removeDuplicatedVertices();
    Vector3f trans;
    float scale;
    polyMesh->NormalizeMesh(trans, scale);

    BaseMeshCreator meshCreator(getVarList());
    crossMesh = make_shared<CrossMesh>(getVarList());
    meshCreator.InitCrossMesh(polyMesh, crossMesh);
    pHEdgeMesh hedgeMesh = make_shared<HEdgeMesh>();
    hedgeMesh->InitHEdgeMesh(polyMesh);
    hedgeMesh->BuildHalfEdgeMesh();
    meshCreator.ComputeCrossNeighbors(hedgeMesh, crossMesh);
    crossMesh->baseMesh2D = polyMesh;
    return;
}

int PatternCreator::GetPolygonIndexInList(pPolygon tagtPoly, vector<pPolygon> polyList)
{
	for (int i = 0; i < polyList.size(); i++)
	{
		if (polyList[i]->IsEqual(tagtPoly.get()) == true)
		{
			return i;
		}
	}

	return ELEMENT_OUT_LIST;
}

//**************************************************************************************//
//                           Create Mesh (2D Regular Pattern)
//**************************************************************************************//

void PatternCreator::ComputeNeighbors(int patternID, pPolygon poly, vector<pPolygon> &neighbors)
{
	for (int i = 0; i < poly->vers.size(); i++)
	{
		Vector3f staPt = poly->vers[i].pos;
		Vector3f endPt = poly->vers[(i + 1) % poly->vers.size()].pos;

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
			Vector3f transVec;

			if( patternID == CROSS_OCTAGON_SQUARE_COLINEAR )
			{
				transVec = ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(poly, i);
			}
			else
			{
				transVec = ComputeTileTranslation(neighbor, staPt, endPt);
			}

			neighbor->Translate(transVec);

			neighbors.push_back(neighbor);
		}
	}
}

Vector3f PatternCreator::ComputeTileTranslation(pPolygon neighbor, Vector3f tagtStaPt, Vector3f tagtEndPt)
{
	Vector3f tagtVec = (tagtEndPt - tagtStaPt) / len(tagtEndPt - tagtStaPt);
	Vector3f tagtMidPt = (tagtEndPt + tagtStaPt) / 2.0f;

	// Identify the edge shared by tagtCross and neighborCross
	Vector3f transVec = Vector3f(0, 0, 0);
	bool reverseTheta;
	for (int i = 0; i < neighbor->vers.size(); i++)
	{
		Vector3f staPt = neighbor->vers[i].pos;
		Vector3f endPt = neighbor->vers[(i + 1) % neighbor->vers.size()].pos;

		Vector3f edgeVec = (endPt - staPt) / len(endPt - staPt);
		Vector3f edgeMidPt = (endPt + staPt) / 2.0f;

		float dotp = tagtVec DOT edgeVec;
		if (fabs(dotp + 1) < FLOAT_ERROR_LARGE)
		{
			transVec = tagtMidPt - edgeMidPt;
			break;
		}
	}

	//printf("transVec: [%.6f %.6f %.6f] \n", transVec.x, transVec.y, transVec.z);

	return transVec;
}

Vector3f PatternCreator::ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(pPolygon poly, int edgeID)
{
	Vector3f transVec;

	if( poly->GetPolyType() == POLY_OCTAGON_COLINEAR)
	{
		float A = len(poly->vers[1].pos - poly->vers[0].pos);

		if(edgeID == 0)         transVec = Vector3f( 1.5*A,   0.5*A);
		else if(edgeID == 1)    transVec = Vector3f(     A,   2.0*A);
		else if(edgeID == 2)    transVec = Vector3f(-0.5*A,   1.5*A);
		else if(edgeID == 3)    transVec = Vector3f(-2.0*A,       A);
		else if(edgeID == 4)    transVec = Vector3f(-1.5*A,  -0.5*A);
		else if(edgeID == 5)    transVec = Vector3f(    -A,  -2.0*A);
		else if(edgeID == 6)    transVec = Vector3f( 0.5*A,  -1.5*A);
		else if(edgeID == 7)    transVec = Vector3f( 2.0*A,      -A);

		transVec += poly->center;

		//printf("transVec [%.3f %.3f %.3f] \n", transVec.x, transVec.y, transVec.z);
	}

	else if( poly->GetPolyType() == POLY_SQUARE_THETA_45 )
	{
		float B = len(poly->vers[1].pos - poly->vers[0].pos);

		if(edgeID == 0)         transVec = Vector3f( -0.5*B,   1.5*B);
		else if(edgeID == 1)    transVec = Vector3f( -1.5*B,  -0.5*B);
		else if(edgeID == 2)    transVec = Vector3f(  0.5*B,  -1.5*B);
		else if(edgeID == 3)    transVec = Vector3f(  1.5*B,   0.5*B);

		transVec += poly->center;
	}

	return transVec;
}

void PatternCreator::ComputeNeighbor_Square(pPolygon poly, pPolygon &neighbor)
{
	CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
}

void PatternCreator::ComputeNeighbor_Hexagon(pPolygon polym, pPolygon &neighbor)
{
	CreatePolygon_Hexagon(neighbor, CROSS_L);
}

void PatternCreator::ComputeNeighbor_Octagon_Square(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if(poly->GetPolyType() == POLY_SQUARE_THETA_45)
	{
		CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_REGULAR);
	}
	else if (poly->GetPolyType() == POLY_OCTAGON_REGULAR)
	{
		if( edgeID %2 == 0 )    CreatePolygon_Octagon(neighbor, CROSS_L, POLY_OCTAGON_REGULAR);
		else                    CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
	}
}

void PatternCreator::ComputeNeighbor_Hexagon_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if     (poly->GetPolyType() == POLY_RHOMBUS_THETA_90)
	{
		CreatePolygon_Hexagon(neighbor, CROSS_L);
	}
	else if ( poly->GetPolyType() == POLY_HEXAGON_TYPE_0 )
	{
		if( edgeID == 1 || edgeID == 4 )
			CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
		else
			CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90);
	}
}

void PatternCreator::ComputeNeighbor_Dodecagon(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( edgeID % 2 == 1 ) {
		neighbor = nullptr;
		return;
	}

	CreatePolygon_Dodecagon(neighbor, CROSS_L);
}

void PatternCreator::ComputeNeighbor_Square_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	if( poly->GetPolyType() == POLY_SQUARE_THETA_15 )
	{
		if( edgeID == 0 )             CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 1 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 2 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 3 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );

	}
	else if( poly->GetPolyType() == POLY_SQUARE_THETA_75 )
	{
		if( edgeID == 0 )             CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 1 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
		else if( edgeID == 2 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_90  );
		else if( edgeID == 3 )        CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0 );
	}
	else if( poly->GetPolyType() == POLY_RHOMBUS_THETA_0 )
	{
		if( edgeID == 0 )             CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 1 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
		else if( edgeID == 2 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 3 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
	}
	else if( poly->GetPolyType() == POLY_RHOMBUS_THETA_90 )
	{
		if( edgeID == 0 )             CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 1 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
		else if( edgeID == 2 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_15 );
		else if( edgeID == 3 )        CreatePolygon_Square(neighbor, CROSS_L,  POLY_SQUARE_THETA_75 );
	}
}

void PatternCreator::ComputeNeighbor_Dodecagon_Hexagon_Square(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	// Three types of Squares
	if     (poly->vers.size() == 4)
	{
		if( poly->GetPolyType() == POLY_SQUARE_THETA_45 )
		{
			if( edgeID == 0 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1 );
			else if (edgeID == 2)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0 );
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->GetPolyType() == POLY_SQUARE_THETA_75 )
		{
			if( edgeID == 1 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1);
			else if (edgeID == 3)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->GetPolyType() == POLY_SQUARE_THETA_15 )
		{
			if( edgeID == 1 )        CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_0);
			else if (edgeID == 3)    CreatePolygon_Hexagon(neighbor, CROSS_L, POLY_HEXAGON_TYPE_1);
			else                     CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
	}

	// Two types of Hexagons
	else if ( poly->vers.size() == 6 )
	{
		if( poly->GetPolyType() == POLY_HEXAGON_TYPE_1 )
		{
			if( edgeID == 0 )       CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_75);
			else if (edgeID == 2)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_15);
			else if (edgeID == 4)   CreatePolygon_Square(neighbor, CROSS_L, POLY_SQUARE_THETA_45);
			else                    CreatePolygon_Dodecagon(neighbor, CROSS_L);
		}
		else if( poly->GetPolyType() == POLY_HEXAGON_TYPE_0 )
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

void PatternCreator::ComputeNeighbor_Pentagon_Cross(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->GetPolyType() == POLY_PENTAGON_CROSS_TYPE_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
	}

	else if( poly->GetPolyType() == POLY_PENTAGON_CROSS_TYPE_1 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
	}

	else if( poly->GetPolyType() == POLY_PENTAGON_CROSS_TYPE_2 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_3);
	}

	else if( poly->GetPolyType() == POLY_PENTAGON_CROSS_TYPE_3 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_2);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Cross(neighbor, CROSS_L, POLY_PENTAGON_CROSS_TYPE_0);
	}
}

void PatternCreator::ComputeNeighbor_Pentagon_Snow(pPolygon poly, int edgeID, pPolygon &neighbor)
{
    if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_0 )
    {
        if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
        else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
        else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
        else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
        else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
    }

    else if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_1 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
    }

    else if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_2 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
    }

    else if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_3 )
    {
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
    }

	else if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_4 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_5);
	}

	else if( poly->GetPolyType() == POLY_PENTAGON_SNOW_TYPE_5 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_4);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_2);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_3);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Snow(neighbor, CROSS_L, POLY_PENTAGON_SNOW_TYPE_0);
	}
}

void PatternCreator::ComputeNeighbor_Pentagon_Mirror(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->GetPolyType() == POLY_PENTAGON_MIRROR_TYPE_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
	}

	else if( poly->GetPolyType() == POLY_PENTAGON_MIRROR_TYPE_1 )
	{
		if( edgeID == 0 )       CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 1)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 2)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
		else if (edgeID == 3)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_1);
		else if (edgeID == 4)   CreatePolygon_Pentagon_Mirror(neighbor, CROSS_L, POLY_PENTAGON_MIRROR_TYPE_0);
	}
}

void PatternCreator::ComputeNeighbor_Rhombus(pPolygon poly, int edgeID, pPolygon &neighbor)
{

	if( poly->GetPolyType() == POLY_RHOMBUS_THETA_0 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
	}

	else if( poly->GetPolyType() == POLY_RHOMBUS_THETA_120 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_240);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
	}

	else if( poly->GetPolyType() == POLY_RHOMBUS_THETA_240 )
	{
		if( edgeID == 0 )       CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 1)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
		else if (edgeID == 2)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_0);
		else if (edgeID == 3)   CreatePolygon_Rhombus(neighbor, CROSS_L, POLY_RHOMBUS_THETA_120);
	}
}

void PatternCreator::ComputeNeighbor_Octagon_Square_Colinear(pPolygon poly, int edgeID, pPolygon &neighbor)
{
	if( poly->GetPolyType() == POLY_OCTAGON_COLINEAR )
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

	else if( poly->GetPolyType() == POLY_SQUARE_THETA_45 )
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

void PatternCreator::CreatePolygon_Root(int patternID, float edgeLen, pPolygon &poly)
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

void PatternCreator::CreatePolygon_Square(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float initTheta;
	if     ( polyType == POLY_SQUARE_THETA_45 )      initTheta = 45;
	else if( polyType == POLY_SQUARE_THETA_15 )      initTheta = 15;
	else if( polyType == POLY_SQUARE_THETA_75 )      initTheta = 75;

	float r = 0.5f * edgeLen / cos(45* M_PI / 180.0);

	//float initTheta = 45;
	float stepTheta = 90;

	for (int i = 0; i < 4; ++i)
	{
		float currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3f point = Vector3f(r*cos(currTheta), r*sin(currTheta), 0);

		poly->vers.push_back( _Vertex(point) );
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Pentagon_Cross(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float rotTheta;
	if     ( polyType == POLY_PENTAGON_CROSS_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_1 )      rotTheta = 90;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_2 )      rotTheta = 180;
	else if( polyType == POLY_PENTAGON_CROSS_TYPE_3 )      rotTheta = 270;

	float A = 0.5f * edgeLen;
	float B = 1.5f * edgeLen;

	poly->vers.push_back( _Vertex(Vector3f( 0,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( A, -A, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( B, -A, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( B,  A, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( A,  A, 0)) );

	// Rotate the whole polygon
	for (int i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i].pos = RotateVector(Vector3f(0,0,0), Vector3f(0,0,1), rotTheta, poly->vers[i].pos);
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Pentagon_Snow(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float rotTheta;
	if     ( polyType == POLY_PENTAGON_SNOW_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_1 )      rotTheta = 60;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_2 )      rotTheta = 120;
	else if( polyType == POLY_PENTAGON_SNOW_TYPE_3 )      rotTheta = 180;
    else if( polyType == POLY_PENTAGON_SNOW_TYPE_4 )      rotTheta = 240;
    else if( polyType == POLY_PENTAGON_SNOW_TYPE_5 )      rotTheta = 300;

    float A = edgeLen;
	float B = 0.5f * edgeLen;
    float C = 1.25f * edgeLen;
    float D = (sqrtf(3.0f) / 4.0) * edgeLen;
    float E = (sqrtf(3.0f) / 2.0) * edgeLen;

	poly->vers.push_back( _Vertex(Vector3f( 0,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( A,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( C,  D, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( A,  E, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( B,  E, 0)) );

    // Rotate the whole polygon
	for (int i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i].pos = RotateVector(Vector3f(0,0,0), Vector3f(0,0,1), rotTheta, poly->vers[i].pos);
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Pentagon_Mirror(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float rotTheta;
	if     ( polyType == POLY_PENTAGON_MIRROR_TYPE_0 )      rotTheta = 0;
	else if( polyType == POLY_PENTAGON_MIRROR_TYPE_1 )      rotTheta = 180;

	float A = edgeLen;
	float B = 0.5f * edgeLen;
	float C = (sqrtf(3.0f) / 2.0) * edgeLen;
	float D = ((sqrtf(3.0f) / 4.0) + 0.5f) * edgeLen;

	poly->vers.push_back( _Vertex(Vector3f( 0, -C, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( B, -C, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( D,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( B,  C, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( 0,  C, 0)) );

	// Rotate the whole polygon
	for (int i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i].pos = RotateVector(Vector3f(0,0,0), Vector3f(0,0,1), rotTheta, poly->vers[i].pos);
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}


void PatternCreator::CreatePolygon_Hexagon(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float r = 0.5f * edgeLen / cos(60* M_PI / 180.0);

	float initTheta = 0;
	float stepTheta = 60;

	for (int i = 0; i < 6; ++i)
	{
		float currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3f point = Vector3f(r*cos(currTheta), r*sin(currTheta), 0);

		poly->vers.push_back( _Vertex(point) );
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Octagon(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	if( polyType == POLY_OCTAGON_REGULAR )
	{
		float r = 0.5f * edgeLen / cos(67.5* M_PI / 180.0);

		float initTheta = 22.5;
		float stepTheta = 45;

		for (int i = 0; i < 8; ++i)
		{
			float currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

			Vector3f point = Vector3f(r*cos(currTheta), r*sin(currTheta), 0);

			poly->vers.push_back( _Vertex(point) );
		}
	}

	else if(polyType == POLY_OCTAGON_COLINEAR )
	{
		float A = edgeLen;

		poly->vers.push_back( _Vertex(Vector3f( A,  0, 0)) );
		poly->vers.push_back( _Vertex(Vector3f( A,  A, 0)) );
		poly->vers.push_back( _Vertex(Vector3f( 0,  A, 0)) );
		poly->vers.push_back( _Vertex(Vector3f(-A,  A, 0)) );
		poly->vers.push_back( _Vertex(Vector3f(-A,  0, 0)) );
		poly->vers.push_back( _Vertex(Vector3f(-A, -A, 0)) );
		poly->vers.push_back( _Vertex(Vector3f( 0, -A, 0)) );
		poly->vers.push_back( _Vertex(Vector3f( A, -A, 0)) );
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Dodecagon(pPolygon &poly, float edgeLen)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( POLY_DODECAGON );

	float r = 0.5f * edgeLen / cos(75* M_PI / 180.0);

	float initTheta = 15;
	float stepTheta = 30;

	for (int i = 0; i < 12; ++i)
	{
		float currTheta = (initTheta + i * stepTheta) * M_PI / 180.0;

		Vector3f point = Vector3f(r*cos(currTheta), r*sin(currTheta), 0);

		poly->vers.push_back( _Vertex(point) );
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

void PatternCreator::CreatePolygon_Rhombus(pPolygon &poly, float edgeLen, int polyType)
{
	poly.reset();
	poly = make_shared<_Polygon>();

	poly->SetPolyType( polyType );

	float rotTheta;
	if     ( polyType == POLY_RHOMBUS_THETA_0 )        rotTheta = 0;
	else if( polyType == POLY_RHOMBUS_THETA_90 )       rotTheta = 90;
	else if( polyType == POLY_RHOMBUS_THETA_120 )      rotTheta = 120;
	else if( polyType == POLY_RHOMBUS_THETA_240 )      rotTheta = 240;


	float A = (sqrtf(3.0f) / 2.0) * edgeLen;
	float B = 0.5f * edgeLen;

	poly->vers.push_back( _Vertex(Vector3f( A,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( 0,  B, 0)) );
	poly->vers.push_back( _Vertex(Vector3f(-A,  0, 0)) );
	poly->vers.push_back( _Vertex(Vector3f( 0, -B, 0)) );

	// Rotate the whole polygon
	for (int i = 0; i < poly->vers.size(); ++i)
	{
		poly->vers[i].pos = RotateVector(Vector3f(0,0,0), Vector3f(0,0,1), rotTheta, poly->vers[i].pos);
	}

	// Compute properties
	poly->ComputeCenter();
	poly->ComputeNormal();
}

//**************************************************************************************//
//                           Create Mesh (a Single Fan)
//**************************************************************************************//

void PatternCreator::CreateMesh_Fan(Vector3f verA, Vector3f verB, Vector3f verM, pPolyMesh &polyMesh)
{
	// Construct a fan-shaped polygon
	pPolygon poly;
	CreatePolygon_Fan(verA, verB, verM, poly);

	// Construct the mesh
	polyMesh.reset();
	polyMesh = make_shared<PolyMesh>(getVarList());

	for (int i = 0; i < poly->vers.size(); i++)
	{
		polyMesh->vertexList.push_back(poly->vers[i].pos);
	}

	polyMesh->polyList.push_back(poly);
}


void PatternCreator::CreatePolygon_Fan(Vector3f verA, Vector3f verB, Vector3f verM, pPolygon &poly)
{
	Vector3f edgeDirA = verA / len(verA);
	Vector3f edgeDirB = verB / len(verB);
	Vector3f edgeDirM = verM / len(verM);

	Vector3f rotAxis = edgeDirA CROSS edgeDirM;

	// Compute fan angle
	float dotp = edgeDirA DOT edgeDirB;
	float angle;
	if (fabs(dotp + 1) < FLOAT_ERROR_LARGE)
	{
		angle = 180;

	}
	else
	{
		angle = acos(dotp) * 180 / M_PI;
	}


	poly.reset();
	poly = make_shared<_Polygon>();

	// Compute fan vertices
	poly->vers.push_back(_Vertex(Vector3f(0, 0, 0)));
	poly->vers.push_back(_Vertex(verA));

	const float angleStep = 10; // Unit: degree
	float sampleAngle = 0;
	do
	{
		Vector3f ver = RotateVector(Vector3f(0, 0, 0), rotAxis, sampleAngle, verA);

		poly->vers.push_back(_Vertex(ver));

		sampleAngle += angleStep;
	} while (sampleAngle < angle);

	poly->vers.push_back(_Vertex(verB));

	poly->ComputeCenter();
	poly->ComputeNormal();
}

//**************************************************************************************//
//                           Python Interface
//**************************************************************************************//

vector<vector<double>> PatternCreator::PyCreateMesh_2DPattern(int patternID, int patternRadius) {
    shared_ptr<CrossMesh> crossMesh;
    CreateMesh_2DPattern(patternID, patternRadius, crossMesh);

    vector<vector<double>> polys;
    for(int id = 0; id < crossMesh->baseMesh2D->polyList.size(); id++){
        pPolygon poly = crossMesh->baseMesh2D->polyList[id];
        vector<double> coords;
        for(int jd = 0; jd < poly->vers.size(); jd++){
            coords.push_back(poly->vers[jd].pos[0]);
            coords.push_back(poly->vers[jd].pos[1]);
        }
        polys.push_back(coords);
    }
    return polys;
}

