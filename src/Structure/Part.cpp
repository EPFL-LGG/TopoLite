///////////////////////////////////////////////////////////////
//
// Part.cpp
//
//   Part Model Class
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 15/July/2018
//
///////////////////////////////////////////////////////////////


#ifdef WIN32
#include <GL/glut.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

#include "Utility/HelpDefine.h"
#include "Utility/HelpStruct.h"
#include "Utility/HelpFunc.h"

#include "CrossMesh/PatternCreator.h"
#include "Mesh/Cross.h"

#include "Mesh/MeshConverter.h"
#include "Mesh/PolyMesh.h"

#include "PartGeom.h"
#include "Part.h"
#include "IO/InputVar.h"

//**************************************************************************************//
//                                   Basic Operations
//**************************************************************************************//

Part::Part(pCross _cross, shared_ptr<InputVarList> var):TopoObject(var)
{
	cross = _cross;

	touchGround = false;
	atBoundary = false;

	isRemove = false;

	polyMesh = NULL;


	groupID = NONE_GROUP;

	mtlDiffuse = Vector3f(1.0, 1.0, 1.0);
	mtlSpecular = Vector3f(1.0, 1.0, 1.0);
}

Part::~Part()
{
	Clear();
}

Part::Part(const Part &part): TopoObject(part)
{
    if(part.polyMesh) polyMesh = make_shared<PolyMesh>(*part.polyMesh);
    if(part.partGeom) partGeom = make_shared<PartGeom>(*part.partGeom);

    partID = part.partID;
    assemblyID = part.assemblyID;
    groupID = part.groupID;
    initNeighbors = part.initNeighbors;
    atBoundary = part.atBoundary;
    mtlDiffuse = part.mtlDiffuse;
    mtlSpecular = part.mtlSpecular;
    text3DPos = part.text3DPos;
    textLinkPt = part.textLinkPt;
    wireColor = part.wireColor;
    isRemove = part.isRemove;
    touchGround = part.touchGround;
}

void Part::Clear()
{
	polyMesh.reset();
	partGeom.reset();
	initNeighbors.clear();
}

void Part::PrintPart()
{
	printf("partID: %2d   ", partID);

	printf("%d neighbors: ", (int)initNeighbors.size());
	for (int i = 0; i < initNeighbors.size(); i++)
	{
		if (initNeighbors[i].lock() == nullptr)
		{
			printf(" %3d ", -1);
		}
		else
		{
			printf(" %3d ", initNeighbors[i].lock()->partID);
		}
	}

	printf("\n");
}

//**************************************************************************************//
//                                Compute Part Geometry 
//**************************************************************************************//

bool Part::CheckLegalGeometry(bool use_orient_opt)
{
    vector<Vector3f> polyVers = cross.lock()->GetVertices();
    partGeom.reset();
    partGeom = make_shared<PartGeom>(cross.lock(), getVarList());
	bool is_legal = partGeom->IsLegalGeometry();
	return is_legal;
}

void Part::ComputePartGeometry(bool convexPart, Vector2f cutPlaneHeight, bool previewMode)
{
	polyMesh.reset();
	vector<pPolyMesh> tempPolyMeshes;

	if (partGeom == nullptr)
	{
		cout<<"Warning: check whether the geometry is valid first!"<<endl;
		return;
	}

	if( convexPart )
	{
		clock_t sta = clock();
		partGeom->ComputePartGeometry(cutPlaneHeight, polyMesh);
		//timer[5] += (float)(clock() - sta) /(CLOCKS_PER_SEC);
	}

	if(!previewMode)
	{
		polyMesh->ComputeBBox();
		polyMesh->ComputeCentroid();
		polyMesh->ComputeVolume();
		polyMesh->ComputeLowestPt();
	}
}

void Part::Compute3DTextPosition()
{
	if (polyMesh == NULL)
		return;

	const float linkLen = 0.3f;

	textLinkPt = polyMesh->ComputeExtremeVertex(cross.lock()->normal);
	text3DPos = textLinkPt + linkLen * cross.lock()->normal;
}

//**************************************************************************************//
//                                      Part Operations
//**************************************************************************************//

int Part::GetNeighborIndex(pPart neiborPart)
{
	for (int i = 0; i < initNeighbors.size(); i++)
	{
		if ( initNeighbors[i].lock() == nullptr )
			continue;

		if ( initNeighbors[i].lock()->partID == neiborPart->partID)
		{
			return i;
		}
	}

	return NONE_PART;
}

int Part::GetFaceIndex(pPolygon neiborPartFace)
{
	Vector3f neiborFaceNormal = neiborPartFace->ComputeNormal();
	Vector3f partFaceNormal = -1.0f * neiborFaceNormal;

	printf("partNorm: [%.3f %.3f %.3f] \n", partFaceNormal.x, partFaceNormal.y, partFaceNormal.z);

	for (int i = 0; i < polyMesh->polyList.size(); i++)
	{
		Vector3f tempFaceNormal = polyMesh->polyList[i]->ComputeNormal();

		printf("tempNorm: [%.3f %.3f %.3f] \n", tempFaceNormal.x, tempFaceNormal.y, tempFaceNormal.z);

		float dotp = tempFaceNormal DOT partFaceNormal;

		printf("i=%d   dotp: %.3f \n", i, dotp);

		if( fabs(dotp-1.0f) < FLOAT_ERROR_SMALL )
		{
			printf("faceID = %d \n", i);
			return i;
		}

	}

	return NONE_FACE;
}

void Part::GetContactFaces(pPart neiborPart, vector<pair<int, int>> &contaFaceIDPairs)
{
	for (int i = 0; i < this->polyMesh->polyList.size(); i++)
	{
		pPolygon currFace = this->polyMesh->polyList[i];
		Vector3f currNormal = currFace->ComputeNormal();
		Vector3f currCenter = currFace->ComputeCenter();

		for (int j = 0; j < neiborPart->polyMesh->polyList.size(); j++)
		{
			pPolygon neiborFace = neiborPart->polyMesh->polyList[j];
			Vector3f neiborNormal = neiborFace->ComputeNormal();
			Vector3f neiborCenter = neiborFace->ComputeCenter();

			float dotp = currNormal DOT neiborNormal;
			float dist = (currCenter-neiborCenter) DOT currNormal;

			// Determine two contacting faces from two different parts
			if (fabs(dotp+1) < FLOAT_ERROR_LARGE && fabs(dist) < FLOAT_ERROR_LARGE)
			{
				//currContaFaceID = i;
				//neiborContaFaceID = j;
				//return true;

				contaFaceIDPairs.push_back( make_pair(i, j) );
			}
		}
	}
}

Vector2f Part::computeYExtrem()
{
	Vector2f extrem(std::numeric_limits<float>::max(), std::numeric_limits<float>::min());
	for(int id = 0; id < polyMesh->vertexList.size(); id++)
	{
		Vector3f pt = polyMesh->vertexList[id];
		if(pt[1] > extrem[1]) extrem[1] = pt[1];
		if(pt[1] < extrem[0]) extrem[0] = pt[1];
	}
	return extrem;
}

//**************************************************************************************//
//                                      I/O
//**************************************************************************************//

void Part::WriteOBJModel(char *objFileName)
{
    if (polyMesh == NULL || isRemove)
        return;
    shared_ptr<PolyMesh> mesh = make_shared<PolyMesh>(*polyMesh);
    mesh->TranslateMesh(Vector3f(0, -getVarList()->get<float>("ground_height"), 0));
    mesh->WriteOBJModel(objFileName, true);
}

void Part::WriteOBJModel(char *objFileName, Vector3f movement) {
    if (polyMesh == NULL || isRemove)
        return;
    shared_ptr<PolyMesh> mesh = make_shared<PolyMesh>(*polyMesh);
    mesh->TranslateMesh(Vector3f(0, -getVarList()->get<float>("ground_height"), 0));
    mesh->TranslateMesh(movement * getVarList()->get<float>("mult_move_scale"));
    mesh->WriteOBJModel(objFileName, true);
}

void Part::WriteOBJWireFrameModel(const char *objFileName)
{
    if(polyMesh && !isRemove)
    {
        shared_ptr<HEdgeMesh> hedgeMesh = make_shared<HEdgeMesh>();
        shared_ptr<PolyMesh> mesh = make_shared<PolyMesh>(*polyMesh);
        mesh->TranslateMesh(Vector3f(0, -getVarList()->get<float>("ground_height"), 0));
        hedgeMesh->InitHEdgeMesh(mesh);
        hedgeMesh->BuildHalfEdgeMesh();

        shared_ptr<PolyMesh> wireframe;
        vector<Vector3f> ver;
        vector<Vector3i> tri;

        float thickness = getVarList()->get<float>("wireframe_thickness");
        int circle_vertex_num = getVarList()->get<int>("wireframe_circle_vertex_num");
        for(int id = 0; id < hedgeMesh->edgeList.size(); id++)
        {
            shared_ptr<HEdge> edge = hedgeMesh->edgeList[id];
            shared_ptr<HVertex> sta = edge->staVer.lock();
            shared_ptr<HVertex> end = edge->endVer.lock();
            if(sta->id < end->id || edge->twin.lock() == nullptr)
            {
                Vector3f z_axis = (end->point - sta->point); z_axis /= len(z_axis);
                Vector3f x_axis = Vector3f(0, 1, 0) CROSS z_axis;
                if(len(x_axis) < FLOAT_ERROR_LARGE) x_axis = Vector3f(1, 0, 0) CROSS z_axis; x_axis /= len(x_axis);
                Vector3f y_axis = z_axis CROSS x_axis; y_axis /= len(y_axis);

                for(int id = 0; id < circle_vertex_num; id++)
                {
                    float angle0 = 2 * M_PI/circle_vertex_num * id;
                    float angle1 = 2 * M_PI/circle_vertex_num * (id + 1);
                    Vector3f v_sta0 = sta->point + x_axis * thickness * cos(angle0) + y_axis * thickness * sin(angle0);
                    Vector3f v_sta1 = sta->point + x_axis * thickness * cos(angle1) + y_axis * thickness * sin(angle1);
                    Vector3f v_end0 = v_sta0 + (end->point - sta->point);
                    Vector3f v_end1 = v_sta1 + (end->point - sta->point);
                    int f = ver.size();
                    ver.push_back(v_sta0);ver.push_back(v_sta1);ver.push_back(v_end0);ver.push_back(v_end1);
                    tri.push_back(Vector3i(f, f + 1, f + 2));
                    tri.push_back(Vector3i(f + 1, f + 3, f + 2));
                }
            }
        }

        MeshConverter converter;
        converter.InitPolyMesh(ver, tri, wireframe);
        wireframe->WriteOBJModel(objFileName);
    }
    return;
}
