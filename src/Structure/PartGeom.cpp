///////////////////////////////////////////////////////////////
//
// PartGeom.h
//
//   Construct Part Geometry
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Dec/2017
//
//
// Input:  a set of oriented points (oriPoints)
// Output: geometry of a convex polyhedron (polyMesh)
//
///////////////////////////////////////////////////////////////

#include "Utility/Controls.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"
#include "Utility/ConvexHull3D.cpp"
#include "Mesh/HEdgeMesh.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/MeshConverter.h"
#include "Mesh/Cross.h"
#include "PartGeom.h"
#include <list>
#include "IO/gluiVar.h"
extern Vector3f colorTable[18];
extern gluiVarList varList;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PartGeom::PartGeom(shared_ptr<Cross> _cross)
{
    UpdateCross(_cross);
}

void PartGeom::UpdateCross(shared_ptr<Cross> _cross)
{
    cross = _cross;
    polygon.clear();
    oriPoints.clear();

    for(auto pt : _cross->vers)
    {
        polygon.push_back(pt.pos);
    }

    for(auto pt : _cross->oriPoints) {
        oriPoints.push_back(pt);
    }

    for (int i = 0; i < oriPoints.size(); i++)
    {
        oriPoints[i].lock()->normal = oriPoints[i].lock()->normal / len(oriPoints[i].lock()->normal);
    }
}

PartGeom::PartGeom(const PartGeom &_geom)
{
    UpdateCross(_geom.cross.lock());
    for(int id = 0; id < _geom.hypList.size(); id++){
    	if(_geom.hypList[id]){
			pHypPlane hyp = make_shared<HypPlane>(*_geom.hypList[id]);
			hypList.push_back(hyp);
    	}
    }
    for(int id = 0; id < _geom.edgeList.size(); id++)
    {
    	if(_geom.edgeList[id])
    	{
			pHypEdge edge = make_shared<HypEdge>(*_geom.edgeList[id]);
			edgeList.push_back(edge);
    	}

    }
    for(int id = 0; id < _geom.verList.size(); id++)
    {
    	if(_geom.verList[id]){
			pHypVertex ver = make_shared<HypVertex>(*_geom.verList[id]);
			verList.push_back(ver);
    	}
    }
    for(int id = 0; id < _geom.faceList.size(); id++)
    {
    	if(_geom.faceList[id]){
			pPolygon poly = make_shared<_Polygon>(*_geom.faceList[id]);
			faceList.push_back(poly);
    	}
    }
}

PartGeom::~PartGeom()
{
	Clear();
}

void PartGeom::Clear()
{
	hypList.clear();
	edgeList.clear();
	verList.clear();
}

//**************************************************************************************//
//                                 Compute Part Geometry 
//**************************************************************************************//

bool PartGeom::ValidateTiltNormal()
{
	for (int i = 0; i < oriPoints.size(); i++)
	{
		for (int j = 0; j < oriPoints.size(); j++)
		{
			if (j == i) continue;
			Vector3f to_oriPoint =  oriPoints[j].lock()->point - oriPoints[i].lock()->point;
			if ((to_oriPoint DOT oriPoints[i].lock() -> normal) > 0)
			{
				return false;
			}
		}
	}
	return true;
}

Vector3f PartGeom::GetPseudoOrigin()
{
	Vector3f pseudo_origin = Vector3f(0, 0, 0);
	for (int i = 0; i < oriPoints.size(); i++) {
		pseudo_origin += oriPoints[i].lock()->point;
	}
	pseudo_origin /= oriPoints.size();
	return pseudo_origin;
}

Vector3f Get_dual_point_from_face(Vector3f normal, Vector3f point){
	// assume the normal of the face is facing out (doesn't include origin)
	float common_deno = normal DOT point;
	return normal / common_deno;
}

bool PartGeom::IsLegalGeometry()
{
	return true;
	//return ValidateTiltNormal();
}

void PartGeom::ComputePartGeometry(Vector2f cutPlaneHeight, pPolyMesh &polyMesh)
{
	if (oriPoints.size() == 0){
		polyMesh = nullptr;
		return;
	}

	///////////////////////////////////////////////////////////
	// 1. Compute all validate vertices
    //clock_t sta = clock();
	ComputeFaces(cutPlaneHeight);
	//timer[0] += (float)(clock() - sta) /(CLOCKS_PER_SEC);

	//sta = clock();
	ComputePartGeometry_Matrix();
	vector<Vector3f> pointList;
	ValidateVertices(pointList);
	//timer[2] += (float)(clock() - sta)/(CLOCKS_PER_SEC);

	///////////////////////////////////////////////////////////
	// 2. Compute convex hull of all vertices
    //sta = clock();
	double transMat[16];
	identityd(transMat);
	identityd(transMat);
	vector<Vector3f> ver;
	vector<Vector3i> tri;
	computeQuickHull(pointList, ver, tri);
	//timer[3] += (float)(clock() - sta)/(CLOCKS_PER_SEC);

	///////////////////////////////////////////////////////////
	// 3. Convert triangular mesh into polygonal mesh

	//sta = clock();
	MeshConverter converter;
	polyMesh.reset();
	Triangles2PolyMesh(ver, tri, polyMesh);
	//converter.Convert2PolyMesh(ver, tri, false, polyMesh);
    //timer[4] += (float)(clock() - sta)/(CLOCKS_PER_SEC);
}

void PartGeom::ComputePartGeometry_Matrix()
{
	// TODO:
	// 1. compute all possible vertices by solving 3x3 matrices
	// 2. check valid vertices by testing Nx3 inequalities
	// 3. compute convex hull of all valid vertices
	for(int l0 = 0; l0 < hypList.size(); l0++)
	{
		shared_ptr<HypPlane> P0 = hypList[l0];
		for(int l1 = l0 + 1; l1 < hypList.size(); l1++)
		{
			shared_ptr<HypPlane> P1 = hypList[l1];
			for(int l2 = l1 + 1; l2 < hypList.size(); l2++)
			{
				shared_ptr<HypPlane> P2 = hypList[l2];

				Eigen::Matrix3d mat;
				mat << P0->normal.x, P0->normal.y, P0->normal.z,
				       P1->normal.x, P1->normal.y, P1->normal.z,
					   P2->normal.x, P2->normal.y, P2->normal.z;
				double determinant = mat.determinant();
				if(std::fabs(determinant) < FLOAT_ERROR_LARGE)
				{
					continue;
				}

				Eigen::MatrixXd invmat = mat.inverse();
				Eigen::Vector3d D(P0->getD(), P1->getD(), P2->getD());
				Eigen::Vector3d invD = invmat * D;
				pHypVertex vertex = make_shared<HypVertex>();
				vertex->point = Vector3f(invD[0], invD[1], invD[2]);
				vertex->verID = verList.size();
				verList.push_back(vertex);
			}
		}
	}
}

//**************************************************************************************//
//                                Compute Faces and Edges
//**************************************************************************************//

void PartGeom::ComputeFaces(Vector2f cutPlaneHeight)
{
	if (oriPoints.size() < 3)
		return;

	float radius = _MAX(5.0 * len(oriPoints[1].lock()->point - oriPoints[0].lock()->point), 2.0);

	for (int i = 0; i < oriPoints.size(); i++)
	{
		pHypPlane face = make_shared<HypPlane>();

		face->point = oriPoints[i].lock()->point;
		face->normal = oriPoints[i].lock()->normal;

		face->radius = radius;
		face->planeID = i;

		hypList.push_back(face);
	}

	Vector3f normal = cross.lock()->normal;
	Vector3f center = cross.lock()->center;
	bool boundary = cross.lock()->atBoundary;

	if(boundary || !varList.get<bool>("only_cut_bdry"))
	{
		if(cutPlaneHeight[0] > 0.0)
		{
			//upper plane cut
			pHypPlane face = make_shared<HypPlane>();
			face->point = center + normal * cutPlaneHeight[0];
			face->normal = normal * (1.0f);
			face->radius = radius;
			face->planeID = hypList.size();
			hypList.push_back(face);
		}

		if(cutPlaneHeight[1] > 0.0)
		{
			// lower plane cut
			pHypPlane face = make_shared<HypPlane>();
			face->point = center - normal * cutPlaneHeight[1];
			face->normal = normal * (-1.0f);
			face->radius = radius;
			face->planeID = hypList.size();
			hypList.push_back(face);
		}
	}
}

void PartGeom::ComputeEdges(vector<pHypPlane> &_faceList)
{
	for (int i = 0; i < (int)(_faceList.size()) - 1; i++)
	{
		for (int j = i + 1; j < _faceList.size(); j++)
		{
			Vector3f normalA = _faceList[i]->normal;
			Vector3f normalB = _faceList[j]->normal;

			float dotp = normalA DOT normalB;

			//printf("dop: %.6f \n", dotp);

			if ( fabs(fabs(dotp) - 1.0) < FLOAT_ERROR_LARGE )
				continue;

			pHypEdge edge;
			PlanePlaneIntersect(_faceList[i], _faceList[j], edge);

			edge->planeIDs[0] = _faceList[i]->planeID;
			edge->planeIDs[1] = _faceList[j]->planeID;

			edge->edgeID = edgeList.size();

			edgeList.push_back(edge);
		}
	}

	//printf(" edgeNum: %d \n", edgeList.size());
}

void PartGeom::PlanePlaneIntersect(pHypPlane faceA, pHypPlane faceB, pHypEdge &edge)
{
	Vector3f edgeDir= faceA->normal CROSS faceB->normal;
	edgeDir = edgeDir / len(edgeDir);

	Vector3f tempEdgeDir = faceA->normal CROSS edgeDir;
	tempEdgeDir = tempEdgeDir / len(tempEdgeDir);

	Vector3f hitPoint;
	bool isHit = EdgeHitFace(faceA->point, tempEdgeDir, faceB, hitPoint);
	if ( isHit == false )
	{
		printf("Warning: The line is parallel with the plane without intersection point. \n");
	}

	edge = make_shared<HypEdge>();
	edge->point = hitPoint;
	edge->normal = edgeDir;

	//printf("faceA: [%.2f %.2f %.2f] \n", faceA->normal.x, faceA->normal.y, faceA->normal.z);
	//printf("faceB: [%.2f %.2f %.2f] \n", faceB->normal.x, faceB->normal.y, faceB->normal.z);

	//printf("normal: [%.2f %.2f %.2f] \n", edgeDir.x, edgeDir.y, edgeDir.z);
	//printf("point:  [%.2f %.2f %.2f] \n", hitPoint.x, hitPoint.y, hitPoint.z);

	//printf("\n");
}




//**************************************************************************************//
//             Compute Intersection Points between Edges and Tilt Planes
//**************************************************************************************//

void PartGeom::ComputeVertices(vector<pHypPlane> &_faceList, vector<pHypEdge> &_edgeList)
{
	for (int i = 0; i < _faceList.size(); i++)
	{
		for (int j = 0; j < _edgeList.size(); j++)
		{
			pHypPlane face = _faceList[i];
			pHypEdge edge = _edgeList[j];

			// Avoid intersection test between The edge and its two component faces
			if (face->planeID == edge->planeIDs[0] ||
				face->planeID == edge->planeIDs[1])
			{
				continue;
			}

			float dotp = face->normal DOT edge->normal;
			if ( fabs(dotp) < FLOAT_ERROR_SMALL )
			{
				continue;
			}

			Vector3f hitPt;
			bool isHit = EdgeHitFace(edge->point, edge->normal, face, hitPt);
			if ( isHit == false )
			{
				continue;
			}

			if (IsPointInList(hitPt, verList) == true )
			{
				continue;
			}

			pHypVertex vertex = make_shared<HypVertex>();

			vertex->point = hitPt;
			vertex->verID = verList.size();

			verList.push_back(vertex);
		}
	}

	//printf("VerNum: %d \n", verList.size());
}

bool PartGeom::EdgeHitFace(Vector3f edgePt, Vector3f edgeDir, pHypPlane face, Vector3f &hitPt)
{
	float dotp = face->normal DOT edgeDir;
	if (fabs(dotp) < FLOAT_ERROR_SMALL)
	{
		return false;
	}

	Vector3f temp = face->point - edgePt;
	float m = (face->normal DOT temp) / (face->normal DOT edgeDir);

	hitPt[0] = edgePt[0] + m * edgeDir[0];
	hitPt[1] = edgePt[1] + m * edgeDir[1];
	hitPt[2] = edgePt[2] + m * edgeDir[2];

	return true;
}

bool PartGeom::IsPointInList(Vector3f tagtPoint, vector<pHypVertex> &_verList)
{
	for (int i = 0; i < _verList.size(); i++)
	{
		float dist = len(tagtPoint - _verList[i]->point);

		// Note: this threshold depends on the scale of elements
		if (dist < FLOAT_ERROR_LARGE)
		{
			return true;
		}
	}

	return false;
}

void PartGeom::Triangles2PolyMesh(vector<Vector3f> &vers, vector<Vector3i> &tris, pPolyMesh &polyMesh)
{
	std::list<shared_ptr<_Polygon>> triangles;
	for(Vector3i tri : tris)
	{
		shared_ptr<_Polygon> poly = make_shared<_Polygon>();
		for(int id = 0; id < 3; id++)
		{
			_Vertex vertex;
			vertex.pos = vers[tri[id]];
			poly->vers.push_back(vertex);
			poly->verIDs.push_back(tri[id]);
		}
		poly->ComputeNormal();
		triangles.push_back(poly);
	}

	vector<vector<Vector3f>> facePolygons;
	facePolygons.resize(hypList.size());
	int k = 0;
	for(auto it = triangles.begin(); it != triangles.end();it ++)
	{
		Vector3f normal = (*it)->normal;
		double max_dot = 0;
		shared_ptr<HypPlane> max_plane;
		for(int kd = 0; kd < hypList.size(); kd++)
		{
			if(hypList[kd] == nullptr)
				continue;
			shared_ptr<HypPlane> plane = hypList[kd];
			Vector3f face_normal = plane->normal;
			face_normal /= len(face_normal);
			if((normal DOT face_normal) > max_dot)
			{
				max_plane = plane;
				max_dot = normal DOT face_normal;
			}
		}
		if(max_dot > 0.99 && max_plane != nullptr)
		{
			facePolygons[max_plane->planeID].push_back((*it)->vers[0].pos);
			facePolygons[max_plane->planeID].push_back((*it)->vers[1].pos);
			facePolygons[max_plane->planeID].push_back((*it)->vers[2].pos);
		}
	}

	//timer[1] += (float)(clock() - sta) /(CLOCKS_PER_SEC);
	polyMesh = make_shared<PolyMesh>();
	polyMesh->vertexList = vers;
	faceList.resize(hypList.size());
	for(int id = 0; id < facePolygons.size(); id++)
	{
		vector<Vector3f> poly = facePolygons[id];
		if(poly.size() <= 2) continue;

		shared_ptr<HypPlane> plane = hypList[id];
		if(plane == nullptr) continue;

		Vector3f origin = plane->point;
		Vector3f z_axis = -plane->normal;
		Vector3f x_axis = poly[1] - poly[0]; x_axis /= len(x_axis);
		Vector3f y_axis = z_axis CROSS x_axis; y_axis /= len(y_axis);

		vector<Vector3f> poly2D;
		for(Vector3f pt : poly){
			pt = pt - origin;
			poly2D.push_back(Vector3f(pt DOT x_axis, pt DOT y_axis, 0));
		}

		poly.clear();
		poly = convex_hull(poly2D);
		shared_ptr<_Polygon> face = make_shared<_Polygon>();
		for(Vector3f pt : poly){
			pt = x_axis * pt[0] + y_axis * pt[1] + origin;
			face->vers.push_back(pt);
		}
		face->ComputeNormal();
		faceList[id] = face;
		polyMesh->polyList.push_back(face);
	}
}



//**************************************************************************************//
//                                 Validate Vertices
//**************************************************************************************//

void PartGeom::ValidateVertices(vector<Vector3f> &pointList)
{
	//////////////////////////////////////////////////////////////////
	// 1. Save only valid (intersection) vertices
	pointList.clear();
	for (int i = 0; i < verList.size(); i++)
	{
		verList[i]->isValid = IsValidVertex(verList[i]->point);

		if (verList[i]->isValid == true)
		{
			pointList.push_back(verList[i]->point);
		}
	}

	for(auto it = verList.begin(); it != verList.end();){
	    if(!(*it)->isValid){
	        it = verList.erase(it);
	    }
	    else{
	        it++;
	    }
	}
}

bool PartGeom::IsValidVertex(Vector3f verPt)
{
	for (int i = 0; i < hypList.size(); i++)
	{
		Vector3f tempPt = verPt - hypList[i]->point;
		float dotp = tempPt DOT hypList[i]->normal;

		// A vertex is invalid if it is not within the hyperplane of any face
		if (dotp > 1e-6)
		{
			return false;
		}
	}

	return true;
}

#if USE_OPENGL_DRAW
//**************************************************************************************//
//                                    Draw Base Polygon 
//**************************************************************************************//

void PartGeom::DrawInnerPolygon()
{
	glColor3f(0.3, 0.9, 0.3);

	glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	glPointSize(10.0);

	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3f point = oriPoints[i].lock()->point;
		glVertex3f(point.x, point.y, point.z);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glLineWidth(1.0);
	glPointSize(1.0);
}

void PartGeom::DrawOriPoints()
{
	//Vector3f color = Vector3f(0.9, 0.2, 0.9);
	//if (id % 2 == 0)   color = Vector3f(0.9, 0.2, 0.9);
	//else              color = Vector3f(0.9, 0.5, 0.2);
	//glColor3f(color.x, color.y, color.z);

	glColor3f(0.9, 0.3, 0.9);

	glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	//glPointSize(16.0);
	glPointSize(8.0);

	float norLen = 0.8* len(oriPoints[1].lock()->point - oriPoints[0].lock()->point);

	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3f point = oriPoints[i].lock()->point;
		Vector3f normal = oriPoints[i].lock()->normal;

		Vector3f endPt = point + normal * norLen;

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

//**************************************************************************************//
//                                  Draw Part Construction
//**************************************************************************************//

void PartGeom::DrawFaces()
{
	for (int i = 0; i < hypList.size(); i++)
	{
		DrawFace(hypList[i]);
	}
}

void PartGeom::DrawFace(pHypPlane face)
{
	Vector3f point = face->point;
	Vector3f normal = face->normal;
	float radius = face->radius;
	Vector3f color = colorTable[face->planeID];

	DrawPlane(point, normal, radius, color);
}

void PartGeom::DrawEdges()
{
	float length = 8.0;

	for (int i = 0; i < edgeList.size(); i++)
	{
		DrawEdge(edgeList[i], length);
	}
}

void PartGeom::DrawEdge(pHypEdge edge, float length)
{
	glDisable(GL_LIGHTING);
	glPointSize(15.0);
	glLineWidth(4.0);

	//glColor3f(0.2, 0.2, 0.9);
	//glBegin(GL_POINTS);
	//glVertex3f(edge->point.x, edge->point.y, edge->point.z);
	//glEnd();

	Vector3f endPt1 = edge->point - 0.5f *length * edge->normal;
	Vector3f endPt2 = edge->point + 0.5f *length * edge->normal;

	glColor3f(0.9, 0.2, 0.2);
	glBegin(GL_LINES);
	glVertex3f(endPt1.x, endPt1.y, endPt1.z);
	glVertex3f(endPt2.x, endPt2.y, endPt2.z);
	glEnd();

	glEnable(GL_LIGHTING);
	glPointSize(1.0);
	glLineWidth(1.0);
}

void PartGeom::DrawVertices()
{
	for (int i = 0; i < verList.size(); i++)
	{
		if(verList[i]->isValid)
			DrawVertex(verList[i]);
	}
}

void PartGeom::DrawVertex(pHypVertex vertex)
{
	glDisable(GL_LIGHTING);
	glPointSize(18.0);

	Vector3f color;
	if (vertex->isValid)   color = Vector3f(0.2, 0.9, 0.9);
	else                   color = Vector3f(0.7, 0.7, 0.7);

	glColor3f(color.x, color.y, color.z);
	glBegin(GL_POINTS);
	glVertex3f(vertex->point.x, vertex->point.y, vertex->point.z);
	glEnd();

	glEnable(GL_LIGHTING);
	glPointSize(1.0);
}

void PartGeom::DrawVerDiffs()
{
    if(verDiffs.size() != verList.size()){
        return;
    }
    else{
        glColor3f(0.9f, 0.4f, 0.4f);
        glLineWidth(5);
        glDisable(GL_LIGHTING);
        for(int id = 0; id < verDiffs.size(); id++){
            glBegin(GL_LINES);
            glVertex3d(verList[id]->point.x, verList[id]->point.y, verList[id]->point.z);
            glVertex3d((verList[id]->point + verDiffs[id]).x, (verList[id]->point + verDiffs[id]).y, (verList[id]->point + verDiffs[id]).z);
            glEnd();
        }
        glEnable(GL_LIGHTING);
        glLineWidth(1);
    }
}


#endif