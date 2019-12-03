///////////////////////////////////////////////////////////////
//
// HEdgeMesh.cpp
//
//   Half-Edge Mesh Structure Class
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 10/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#include "../Utility/HelpFunc.h"
#include "Polygon.h"
#include "PolyMesh.h"
#include "HEdgeMesh.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

HEdgeMesh::HEdgeMesh()
{

}

HEdgeMesh::~HEdgeMesh()
{
	ClearHEdgeMesh();
}

void HEdgeMesh::ClearHEdgeMesh()
{
	vertexList.clear();
	edgeList.clear();
	faceList.clear();
}

void HEdgeMesh::InitHEdgeMesh(vector<Vector3f> _vertices, vector<Vector3i> _triangles)
{
	ClearHEdgeMesh();

	for (int i = 0; i < _vertices.size(); i++)
	{
		pHVertex vertex = make_shared<HVertex>();

		vertex->id = i;
		vertex->point = _vertices[i];

		vertexList.push_back(vertex);
	}

	for (int i = 0; i < _triangles.size(); i++)
	{
		pHFace face = make_shared<HFace>();

		face->id = i;
		for (int j = 0; j < 3; j++)
		{
			int verID = _triangles[i][j];
			face->vertices.push_back(vertexList[verID]);
		}

		faceList.push_back(face);
	}
}


void HEdgeMesh::InitHEdgeMesh(pPolyMesh polyMesh)
{
	ClearHEdgeMesh();

	for (int i = 0; i < polyMesh->vertexList.size(); i++)
	{
		pHVertex vertex = make_shared<HVertex>();
		vertex->id = vertexList.size();
		vertex->point = polyMesh->vertexList[i];

		vertexList.push_back(vertex);
	}

	for (int i = 0; i < polyMesh->polyList.size(); i++)
	{
		pHFace face = make_shared<HFace>();

		pPolygon poly = polyMesh->polyList[i];

		for (int j = 0; j < poly->verIDs.size(); j++)
		{
			int verID = poly->verIDs[j];

			pHVertex ver = vertexList[verID];
			face->vertices.push_back(ver);
		}

		face->id = faceList.size();
		face->normal = poly->normal;

		faceList.push_back(face);
	}
}




//**************************************************************************************//
//                             Build Half Edge Structure
//**************************************************************************************//

void HEdgeMesh::BuildHalfEdgeMesh()
{
	GenerateHalfEdges();

	UpdateVertices();

	ComputeTwinHEdges();
}


void HEdgeMesh::GenerateHalfEdges()
{
	for (int i = 0; i < faceList.size(); i++)
	{
		pHFace face = faceList[i];

		//////////////////////////////////////////////////
		// 1. Create half edges for a face

		vector<pHEdge> faceEdges;
		int faceVerNum = face->vertices.size();
		for (int j = 0; j < faceVerNum; j++)
		{
			// 1. Construct a new half-edge
			pHEdge edge = make_shared<HEdge>();

			edge->staVer = face->vertices[j];
			if (j < faceVerNum - 1)
				edge->endVer = face->vertices[j + 1];
			else
				edge->endVer = face->vertices[0];

			edge->face = face;
			edge->id = edgeList.size();

			// 2. Update edge's start vertex
			edge->staVer.lock()->outEdges.push_back(edge);

			// 3. Update face (if it is the 1st edge)
			if (j == 0)
				face->startEdge = edge;

			// 4. For get previous and next edge in a face
			faceEdges.push_back(edge);

			// 5. Save the edge into the list
			edgeList.push_back(edge);
		}


		//////////////////////////////////////////////////
		// 2. Update previous and next half edges

		int faceEdgeNum = faceEdges.size();
		for (int i = 0; i < faceEdgeNum; i++)
		{
			if (i == 0)   faceEdges[i]->prev = faceEdges[faceEdgeNum - 1];
			else            faceEdges[i]->prev = faceEdges[i - 1];

			if (i == faceEdgeNum - 1)
				faceEdges[i]->next = faceEdges[0];
			else
				faceEdges[i]->next = faceEdges[i + 1];
		}


		//////////////////////////////////////////////////
		// 3. Update face half edges

		face->edges.clear();
		for(auto p : faceEdges)
			face->edges.push_back(p);
	}
}

void HEdgeMesh::ComputeTwinHEdges()
{
	for (int i=0; i<edgeList.size(); i++)
	{
		if ( edgeList[i]->twin.lock() != nullptr )
		{
			continue;
		}

		pHVertex sta_ver = edgeList[i]->staVer.lock();
		pHVertex end_ver = edgeList[i]->endVer.lock();
		for(wpHEdge _twin_edge: end_ver->outEdges){
			pHEdge twin_edge = _twin_edge.lock();
			if(twin_edge && twin_edge->endVer.lock()->id == sta_ver->id){
				edgeList[i]->twin = twin_edge;
				twin_edge->twin = edgeList[i];
			}
		}
	}

	////////////////////////////////////////////////////////
	// 2. Warning for empty twin edges, probably due to the input model (e.g., non watertight)

	//for (int i = 0; i < edgeList.size(); i++)
	//{
	//	if ( edgeList[i]->twin == NULL )
	//	{
	//		printf("Warning: some twin edges are empty. \n");
	//	}
	//}
}

void HEdgeMesh::UpdateVertices()
{
	for (int i = 0; i < edgeList.size(); i++)
	{
		edgeList[i]->staVer.lock()->outEdges.push_back(edgeList[i]);
	}

	for (int i = 0; i < vertexList.size(); i++)
	{
		ComputeVertexNormal(vertexList[i]);
	}
}




//**************************************************************************************//
//                                 Compute Neighbors
//**************************************************************************************//

void HEdgeMesh::GetNeighborVertices(pHVertex vertex, vector<pHVertex> &neiborVertices)
{
	for (int i=0; i<vertex->outEdges.size(); i++)
	{
		pHVertex neiborVer = vertex->outEdges[i].lock()->endVer.lock();
		neiborVertices.push_back( neiborVer );
	}
}

void HEdgeMesh::GetNeighborFaces(pHVertex vertex, vector<pHFace> &neiborFaces)
{
	for (int i=0; i<vertex->outEdges.size(); i++)
	{
		pHFace neiborFace = vertex->outEdges[i].lock()->face.lock();
		neiborFaces.push_back( neiborFace );
	}
}

void HEdgeMesh::GetNeighborFaces(pHFace face, vector<pHFace> &neiborFaces)
{
	vector<int> neiborFaceIDs = GetNeighborFaces(face->id);

	for (int i = 0; i < neiborFaceIDs.size(); i++)
	{
		int neiborFaceID = neiborFaceIDs[i];
		neiborFaces.push_back(faceList[neiborFaceID]);
	}
}

vector<int> HEdgeMesh::GetNeighborFaces(int faceID)
{
	vector<int> neiborFaceIDs;

	pHFace face = faceList[faceID];
	pHEdge startEdge = face->startEdge.lock();
	pHEdge currEdge = startEdge;

	while ( true )
	{
		// Push back a neighboring face
		pHFace neiborFace = currEdge->twin.lock()->face.lock();  // Note: a bug here

		if(neiborFace != nullptr)
			neiborFaceIDs.push_back( neiborFace->id );

		// Move to the next edge
		currEdge = currEdge->next.lock();

		// Stop if the next edge is the same as the start edge
		if ( currEdge->IsEqual( startEdge ) )
		{
			break;
		}
	}

	return neiborFaceIDs;
}




//**************************************************************************************//
//                               Compute Boundary Edges
//**************************************************************************************//

void HEdgeMesh::GetBoundaryEdges(vector<int> faceIDs, vector<pHEdge> &outputEdges)
{
	for (int i = 0; i < faceIDs.size(); i++)
	{
		int faceID = faceIDs[i];
		pHFace face = faceList[faceID];

		vector<pHFace> neiborFaces;
		GetNeighborFaces(face, neiborFaces);

		for (int j = 0; j < neiborFaces.size(); j++)
		{
			pHFace neiborFace = neiborFaces[j];

			if (GetElementIndexInList(neiborFace->id, faceIDs) == ELEMENT_OUT_LIST)
			{
				pHEdge sharedEdge = GetSharedEdge(face, neiborFace);

				if ( sharedEdge != NULL )
				{
					outputEdges.push_back(sharedEdge);
				}
				//outputEdges.push_back(face->startEdge);
			}
		}
	}
}

pHEdge HEdgeMesh::GetSharedEdge(pHFace faceA, pHFace faceB)
{
	for (int i = 0; i < faceA->edges.size(); i++)
	{
		pHEdge currEdge = faceA->edges[i].lock();
		pHEdge twinEdge = currEdge->twin.lock();

		if (twinEdge == nullptr)
			continue;

		pHFace neiborFace = twinEdge->face.lock();

		if (neiborFace->id == faceB->id)
		{
			return currEdge;
		}
	}
	return nullptr;
}




//**************************************************************************************//
//                                Utility Functions
//**************************************************************************************//

bool HEdgeMesh::IsTwinEdges(pHEdge edgeA, pHEdge edgeB)
{
	if (edgeA->staVer.lock()->id == edgeB->endVer.lock()->id &&
		edgeA->endVer.lock()->id == edgeB->staVer.lock()->id)
	{
		return true;
	}

	return false;
}

void HEdgeMesh::ComputeVertexNormal(pHVertex vertex)
{
	vector<pHFace> neiborFaces;
	GetNeighborFaces(vertex, neiborFaces);

	Vector3f normalSum = Vector3f(0, 0, 0);
	for (int i = 0; i < neiborFaces.size(); i++)
	{
		normalSum += neiborFaces[i]->normal;
	}

	vertex->normal = normalSum / len(normalSum);
}

vector<pHFace> HEdgeMesh::GetFaceList()
{
	return faceList;
}



#if USE_OPENGL_DRAW
//**************************************************************************************//
//                                   Draw Mesh
//**************************************************************************************//

void HEdgeMesh::DrawModel_Poly()
{
	for (int i = 0; i < faceList.size(); i++)
	{
		pHFace face = faceList[i];
		Vector3f normal = face->normal;

		glBegin(GL_POLYGON);
		glNormal3f(normal.x, normal.y, normal.z);
		for (int j = 0; j < face->vertices.size(); j++)
		{
			Vector3f ver = face->vertices[j].lock()->point;
			glVertex3f(ver.x, ver.y, ver.z);
		}
		glEnd();
	}
}


void HEdgeMesh::DrawModel_Wire()
{
	float epsilon = 5e-4;
	//float zero_epsilon = 0;

	float projMat[16];
	glGetFloatv(GL_PROJECTION_MATRIX, projMat);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -epsilon);
	glMultMatrixf(projMat);

	glDisable(GL_LIGHTING);
	glLineWidth(1.5);
	glPointSize(12.0);
	glColor3f(0.2, 0.2, 0.2);

	// Draw the wire frame of each face
	for (int i = 0; i < faceList.size(); i++)
	{
		pHFace face = faceList[i];

		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < face->vertices.size(); j++)
		{
			//int index = face.vertices[j];
			Vector3f ver = face->vertices[j].lock()->point;
			glVertex3f(ver.x, ver.y, ver.z);
		}
		glEnd();
	}

	glEnable(GL_LIGHTING);
	glLineWidth(1.0);
	glPointSize(1.0);

	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}
#endif