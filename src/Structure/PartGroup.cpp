///////////////////////////////////////////////////////////////
//
// PartGroup.cpp
//
//   A group of parts
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Jan/2018
//
///////////////////////////////////////////////////////////////


#include "Utility/Controls.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpStruct.h"
#include "Utility/HelpFunc.h"

#include "Mesh/MeshConverter.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/Polygon.h"

#include "Mesh/Cross.h"

#include "BodyMobili.h"
#include "Part.h"
#include "PartGroup.h"
#include "IO/gluiVar.h"

#include "igl/writeOBJ.h"
extern gluiVarList varList;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PartGroup::PartGroup(const vector<wpPart>& _groupParts) {
	groupMobili = NULL;

	for (int i = 0; i < _groupParts.size(); i++)
	{
		pPart part = _groupParts[i].lock();
		if(part)
			groupParts.push_back(part);
	}
}


PartGroup::PartGroup(vector<pPart> partList, vector<int> groupPartIDs)
{
	groupMobili = NULL;

	for (int i = 0; i < groupPartIDs.size(); i++)
	{
		int partID = groupPartIDs[i];

		if ( partID < 0 || partID >= partList.size() )
		{
			printf("Warning: the group partID should be within the part list. \n");
			continue;
		}

		pPart part = partList[partID];

		groupParts.push_back(part);
	}
}

PartGroup::PartGroup(vector<wpPart> partList, vector<int> groupPartIDs)
{
	groupMobili = NULL;

	for (int i = 0; i < groupPartIDs.size(); i++)
	{
		int partID = groupPartIDs[i];

		if ( partID < 0 || partID >= partList.size() )
		{
			printf("Warning: the group partID should be within the part list. \n");
			continue;
		}

		wpPart part = partList[partID];

		groupParts.push_back(part);
	}
}


PartGroup::~PartGroup()
{
	groupMobili.reset();
}




//**************************************************************************************//
//                                 Identify Group Boundary
//**************************************************************************************//

void PartGroup::IdentifyGroupBoundary()
{
	oriPoints.clear();

	for (int i = 0; i < groupParts.size(); i++)
	{
		pPart part = groupParts[i].lock();

		for (int j = 0; j < part->initNeighbors.size(); j++)
		{
			pPart neibor = part->initNeighbors[j].lock();

			int partIndex = GetPartIndexInList(neibor, groupParts);

			if  (partIndex == NONE_PART)
			{
				oriPoints.push_back(part->cross.lock()->oriPoints[j]);
			}
		}
	}

	printf("oriPoint num: %d \n", (int)oriPoints.size());

    //SelectTiltNormals();
}

int PartGroup::GetPartIndexInList(pPart tagtPart, vector<wpPart> partList)
{
	for (int i = 0; i < partList.size(); i++)
	{
		if (partList[i].lock()->partID == tagtPart->partID)
		{
			return i;
		}
	}

	return NONE_PART;
}




//**************************************************************************************//
//                              Evaluate Group Mobility
//**************************************************************************************//

bool PartGroup::EvaluateGroupMobility()
{
	groupMobili.reset();
	// Compute plane normals
	vector<Vector3f> planeNormals = ComputePlaneNormals();
	if (planeNormals.size() < 2) {
		groupMobili = make_shared<BodyMobili>(planeNormals, 0, Vector3f(0, 0, 0));
		groupMobili->mobiliScore = (4 - 2 * planeNormals.size()) * M_PI;
		return true;
	} else {
		// Compute body scale
		Box groupBBox = ComputeBBox();
		float bodyScale = 0.2f * len(groupBBox.maxPt - groupBBox.minPt);
		// Compute body translation
		Vector3f bodyTrans = ComputeCentroid();
		// Evaluate group mobility
		groupMobili = make_shared<BodyMobili>(planeNormals, bodyScale, bodyTrans);
		bool isMobile = groupMobili->EvaluateBodyMobility();
		return isMobile;
	}
}

vector<Vector3f> PartGroup::ComputePlaneNormals()
{
	vector<int> groupPartIDs;
	for (int i = 0; i < groupParts.size(); i++)
	{
		groupPartIDs.push_back(groupParts[i].lock()->partID);
	}

	vector<Vector3f> planeNormals;

	for (int i = 0; i < groupParts.size(); i++)
	{
		pPart part = groupParts[i].lock();

		for (int j = 0; j < part->initNeighbors.size(); j++)
		{
			pPart neiborPart = part->initNeighbors[j].lock();

			if (neiborPart == NULL)
				continue;
			if (GetElementIndexInList(neiborPart->partID, groupPartIDs) != ELEMENT_OUT_LIST)
				continue;
			if (GetPointIndexInList(part->cross.lock()->oriPoints[j]->normal, planeNormals) == ELEMENT_OUT_LIST)
				planeNormals.push_back(part->cross.lock()->oriPoints[j]->normal);
//			if (GetPointIndexInList(part->cross->oriPoints_Lower[j]->normal, planeNormals) == ELEMENT_OUT_LIST)
//				planeNormals.push_back(part->cross->oriPoints_Lower[j]->normal);
		}
	}
	return planeNormals;
}

Box PartGroup::ComputeBBox()
{
	float minX, minY, minZ, maxX, maxY, maxZ;
	minX = minY = minZ = MAX_FLOAT;
	maxX = maxY = maxZ = MIN_FLOAT;

	for (int i = 0; i < groupParts.size(); i++)
	{
		Box partBBox = groupParts[i].lock()->polyMesh->bbox;

		if (maxX < partBBox.maxPt.x)  maxX = partBBox.maxPt.x;
		if (minX > partBBox.minPt.x)  minX = partBBox.minPt.x;

		if (maxY < partBBox.maxPt.x)  maxY = partBBox.maxPt.y;
		if (minY > partBBox.minPt.y)  minY = partBBox.minPt.y;

		if (maxZ < partBBox.maxPt.z)  maxZ = partBBox.maxPt.z;
		if (minZ > partBBox.minPt.z)  minZ = partBBox.minPt.z;
	}

	Box groupBBox;
	groupBBox.minPt = Vector3f(minX, minY, minZ);
	groupBBox.maxPt = Vector3f(maxX, maxY, maxZ);
	groupBBox.cenPt = 0.5f*(groupBBox.minPt + groupBBox.maxPt);

	return groupBBox;
}

Vector3f PartGroup::ComputeCentroid()
{
	Vector3f centroid = Vector3f(0, 0, 0);

	for (int i = 0; i < groupParts.size(); i++)
	{
		centroid += groupParts[i].lock()->polyMesh->centroid;
	}

	centroid /= (float)groupParts.size();

	return centroid;
}




//**************************************************************************************//
//                                   Write OBJ Model
//**************************************************************************************//

void PartGroup::WriteGroupOBJModel(const char *objFileName, bool triangulate)
{
	FILE *fp;
	if ((fp = fopen(objFileName, "w+")) == NULL)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		vector<Vector3f> allVertices = GetAllVertices();
		vector<vector<int>> allPolys = GetAllPolygons(allVertices);

		////////////////////////////////////////////////////////
		// Write the vertex info of piece OBJ Part

		//fprintf(fp, "# Wavefront OBJ generated by Peng SONG \n\n");

		fprintf(fp, "# %d vertices \n", (int)allVertices.size());

		for (int i = 0; i < allVertices.size(); i++)
		{
			fprintf(fp, "v %f %f %f \n", allVertices[i].x, allVertices[i].y, allVertices[i].z);
		}
		fprintf(fp, "\n");


		////////////////////////////////////////////////////////
		// Write the quad info of piece OBJ Part

		fprintf(fp, "# %d faces \n", (int)allPolys.size());

		for (int i = 0; i < allPolys.size(); i++)
		{
			vector<int> poly = allPolys[i];

			if(!triangulate){
				fprintf(fp, "f ");
				for (int j = 0; j < poly.size(); j++)
				{
					//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
					fprintf(fp, " %d", poly[j] + 1);
				}
				fprintf(fp, "\n");
			}
			else{
				for(int j = 0; j < poly.size() - 2; j++)
				{
					fprintf(fp, "f ");
					int indices[3] = {0, j + 1, j + 2};
					for(int k = 0; k < 3; k++)
					{
						int verID = poly[indices[k]];
						fprintf(fp, " %d", verID + 1);
					}
					fprintf(fp, "\n");
				}
			}

		}
		fprintf(fp, "\n");

		fclose(fp);

		//printf("done \n");
	}

}

void PartGroup::WriteGroupOBJWireModel(char *objFileName)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
	vector<Vector3f> ver;
	vector<Vector3i> tri;
	shared_ptr<PolyMesh> wireframe;

	float thickness = varList.get<float>("wireframe_thickness");
	int circle_vertex_num = varList.get<int>("wireframe_circle_vertex_num");

	for(wpPart part : groupParts)
	{
		shared_ptr<HEdgeMesh> hedgeMesh = make_shared<HEdgeMesh>();
        shared_ptr<PolyMesh> mesh = make_shared<PolyMesh>(*part.lock()->polyMesh);

		hedgeMesh->InitHEdgeMesh(mesh);
		hedgeMesh->BuildHalfEdgeMesh();

		for(int id = 0; id < hedgeMesh->edgeList.size(); id++)
		{
			shared_ptr<HVertex> sta = hedgeMesh->edgeList[id]->staVer.lock();
			shared_ptr<HVertex> end = hedgeMesh->edgeList[id]->endVer.lock();
			if(sta->id < end->id)
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
	}

	MeshConverter converter;
	converter.Convert2EigenMesh(ver, tri, V, F);
	igl::writeOBJ(objFileName, V, F);
}

vector<Vector3f> PartGroup::GetAllVertices()
{
	vector<Vector3f> allVertices;

	for (int i = 0; i < groupParts.size(); i++)
	{
		pPolyMesh polyMesh = make_shared<PolyMesh>(*groupParts[i].lock()->polyMesh);
        polyMesh->TranslateMesh(Vector3f(0, -varList.get<float>("ground_height"), 0));

		//allVertices.insert(allVertices.end(), polyMesh->verList.begin(), polyMesh->verList.end());

		for (int j = 0; j < polyMesh->vertexList.size(); j++)
		{
			allVertices.push_back(polyMesh->vertexList[j]);
		}
	}

	return allVertices;
}

vector<vector<int>> PartGroup::GetAllPolygons(vector<Vector3f> allVertices)
{
	vector<vector<int>> allPolys;

	for (int i = 0; i < groupParts.size(); i++)
	{
		pPolyMesh polyMesh = make_shared<PolyMesh>(*groupParts[i].lock()->polyMesh);
        polyMesh->TranslateMesh(Vector3f(0, -varList.get<float>("ground_height"), 0));
		
		for (int j = 0; j < polyMesh->polyList.size(); j++)
		{
			pPolygon poly = polyMesh->polyList[j];

			// Push back one face of a part
			vector<int> polyIndices;
			for (int k = 0; k < poly->vers.size(); k++)
			{
				int index = GetPointIndexInList(poly->vers[k].pos, allVertices);

				if (index == ELEMENT_OUT_LIST)
				{
					printf("Warning: the polygon's vertex should be in the list. \n");
					continue;
				}
			
				polyIndices.push_back(index);
			}

			allPolys.push_back(polyIndices);
		}	
	}

	return allPolys;
}



#if USE_OPENGL_DRAW
//**************************************************************************************//
//                                 Draw Group Mobility
//**************************************************************************************//

void PartGroup::DrawMobiliFaces()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityFaces();
}

void PartGroup::DrawMobiliEdges()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityEdges();
}

void PartGroup::DrawMobiliVertices()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityVertices();
}

void PartGroup::DrawMobiliRays()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityRay();
}

void PartGroup::DrawMobiliVector()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityVector(3.0, Vector3f(0.3, 0.9, 0.3));
}

void PartGroup::DrawMobiliMesh()
{
	if (groupMobili == NULL)
		return;

	groupMobili->DrawMobilityMesh();
}




//**************************************************************************************//
//                                    Draw Debug
//**************************************************************************************//

void PartGroup::DrawOriPoints()
{
	if (oriPoints.size() == 0)
		return;

	//Vector3f color = Vector3f(0.9, 0.2, 0.9);
	//if (id % 2 == 0)   color = Vector3f(0.9, 0.2, 0.9);
	//else              color = Vector3f(0.9, 0.5, 0.2);

	//glColor3f(color.x, color.y, color.z);

	//glColor3f(0.9, 0.3, 0.9);

	glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	//glPointSize(16.0);
	glPointSize(8.0);

	float norLen = 1.2* len(oriPoints[1].lock()->point - oriPoints[0].lock()->point);

	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3f point = oriPoints[i].lock()->point;
		Vector3f normal = oriPoints[i].lock()->normal;

		Vector3f endPt = point + normal * norLen;

        //if( oriPoints[i]->isSelect)    glColor3f(0.3, 0.9, 0.3);
        //else                          glColor3f(0.9, 0.3, 0.9);

		//glColor3f(0.9, 0.2, 0.9);
		glBegin(GL_POINTS);
		glVertex3f(point.x, point.y, point.z);
		glEnd();

		//glColor3f(0.3, 0.9, 0.3);
		glBegin(GL_LINES);
		glVertex3f(point.x, point.y, point.z);
		glVertex3f(endPt.x, endPt.y, endPt.z);
		glEnd();
	}

	glEnable(GL_LIGHTING);
	glLineWidth(1.0);
	glPointSize(1.0);
}


#endif
