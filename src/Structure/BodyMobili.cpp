///////////////////////////////////////////////////////////////
//
// BodyMobili.h
//
//   Evaluate Part or Part Group Mobility
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Jan/2018
//
//
// Input:  a set of plane normals 
// Output: geometry of part's mobility space
//
///////////////////////////////////////////////////////////////
#include "Utility/Controls.h"
#include "../Utility/HelpDefine.h"
#include "../Utility/HelpStruct.h"
#include "../Utility/HelpFunc.h"
#include "../Utility/math3D.h"
#include "Utility/ConvexHull2D.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "CrossMesh/PatternCreator.h"
#include "BodyMobili.h"

extern Vector3f colorTable[18];

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//
BodyMobili::BodyMobili(vector<Vector3f> _planeNormals, float _bodyScale, Vector3f _bodyTrans)
{
	planeNormals = _planeNormals;
	for (int i = 0; i < planeNormals.size(); i++)
	{
		planeNormals[i] = planeNormals[i] / len(planeNormals[i]);
	}
	bodyScale = _bodyScale;
	bodyTrans = _bodyTrans;
	mobiliMesh = NULL;
	mobiliVec = Vector3f(0, 0, 0);
	mobiliScore = 0.0;
	stabiliScore = 0.0;
}

BodyMobili::~BodyMobili()
{
	Clear();
}

void BodyMobili::Clear()
{
	faceList.clear();
	edgeList.clear();
	verList.clear();
}
//**************************************************************************************//
//                              Evaluate Body Mobility 
//**************************************************************************************//
bool BodyMobili::EvaluateBodyMobility()
{
	///////////////////////////////////////////////////////////
	// 1. Compute all validate vertices
	ComputeFaces();
	ComputeEdges(faceList);
	ComputeVertices(edgeList);
	vector<Vector3f> pointList = ValidateVertices();
	// 2. Compute mobility
	// if there are no valid vertices, then the mobility score is zero
	if (pointList.size() < 2) return false;
	ComputeMobility(pointList);
	ComputeMobilityVector();
	ComputeStabilityScore();
	return true;
}
void BodyMobili::ComputeMobilityVector()
{
	mobiliVec = Vector3f(0, 0, 0);
	for (int i = 0; i < mobiliRays.size(); i++)
	{
		mobiliVec += mobiliRays[i];
	}
	mobiliVec /= len(mobiliVec);
}
void BodyMobili::ComputeStabilityScore()
{
	float minAngle = MAX_FLOAT;
	Vector3f gravityDir = Vector3f(0, -1, 0);
	for (int i = 0; i < mobiliRays.size(); i++)
	{
		Vector3f rayDir = mobiliRays[i] / len(mobiliRays[i]);
		float dotp = rayDir DOT gravityDir;
		float angle = acos(dotp) * 180.0 / M_PI;
		if (angle < minAngle)
			minAngle = angle;
	}
	stabiliScore = minAngle;
}
//**************************************************************************************//
//                                Compute Faces and Edges
//**************************************************************************************//
void BodyMobili::ComputeFaces()
{
	if (planeNormals.size() < 2)
		return;
	float radius = 4.0;
	for (int i = 0; i < planeNormals.size(); i++)
	{
		pHypPlane face = make_shared<HypPlane>();
		face->point = Vector3f(0,0,0);
		face->normal = -planeNormals[i];  // Reverse the plane normal
		face->radius = radius;
		face->planeID = i;
		faceList.push_back(face);
	}
}
void BodyMobili::ComputeEdges(vector<pHypPlane>& _faceList)
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
}
void BodyMobili::PlanePlaneIntersect(pHypPlane faceA, pHypPlane faceB, pHypEdge& edge)
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
}
bool BodyMobili::EdgeHitFace(Vector3f edgePt, Vector3f edgeDir, pHypPlane face, Vector3f &hitPt)
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
//**************************************************************************************//
//                                  Compute Vertices
//**************************************************************************************//
void BodyMobili::ComputeVertices(vector<pHypEdge>& _edgeList)
{
	for (int i = 0; i < _edgeList.size(); i++)
	{
		pHypEdge edge = _edgeList[i];
		Vector3f point1 =  edge->normal * MOBILITY_SPHERE_R;
		Vector3f point2 = -edge->normal * MOBILITY_SPHERE_R;
		if (IsPointInList(point1, verList) == false)
		{
			pHypVertex vertex1 = make_shared<HypVertex>();
			vertex1->point  = point1;
			vertex1->verID  = verList.size();
			vertex1->edgeID = edge->edgeID;
			verList.push_back(vertex1);
		}
		if (IsPointInList(point2, verList) == false)
		{
			pHypVertex vertex2 = make_shared<HypVertex>();
			vertex2->point  = point2;
			vertex2->verID  = verList.size();
			vertex2->edgeID = edge->edgeID;
			verList.push_back(vertex2);
		}
	}
}
bool BodyMobili::IsPointInList(Vector3f tagtPoint, vector<pHypVertex>& _verList)
{
	for (int i = 0; i < _verList.size(); i++)
	{
		float dist = len(tagtPoint - _verList[i]->point);
		// Note: this threshold depends on the scale of parts
		if (dist < FLOAT_ERROR_LARGE) return true;
	}
	return false;
}
//**************************************************************************************//
//                                 Validate Vertices
//**************************************************************************************//
vector<Vector3f> BodyMobili::ValidateVertices()
{
	//////////////////////////////////////////////////////////////////
	// 1. Save only valid (intersection) vertices
	vector<Vector3f> pointList;
	for (int i = 0; i < verList.size(); i++)
	{
		verList[i]->isValid = IsValidVertex(verList[i]->point);
		if (verList[i]->isValid == true)
		{
			pointList.push_back(verList[i]->point);
		}
	}
	return pointList;
}
bool BodyMobili::IsValidVertex(Vector3f verPt)
{
	for (int i = 0; i < faceList.size(); i++)
	{
		Vector3f tempPt = verPt - faceList[i]->point;
		tempPt = tempPt / len(tempPt);
		float dotp = tempPt DOT faceList[i]->normal;
		// A vertex is invalid if it is at bottom of the oriented plane
		if (dotp < -FLOAT_ERROR_LARGE) return false;
	}
	return true;
}

//**************************************************************************************//
//                            Compute Mobility
//**************************************************************************************//

void BodyMobili::ComputeMobility(vector<Vector3f> pointList)
{
	vector<Vector3f> sortedPoints = SortValidVertices(pointList);
	ComputeMobilityMesh(sortedPoints);
	if (pointList.size() > 2) CorrectMobilityMesh(mobiliVec);
	ComputeMobilityScore(sortedPoints);
}
Vector3f BodyMobili::GetPlaneVertex(Vector3f edgeVertex, Vector3f planeNormal)
{
	Vector3f planeVertex = edgeVertex CROSS planeNormal;
	if (IsValidVertex(planeVertex) == false)
	{
		planeVertex = - edgeVertex CROSS planeNormal;
	}
	return planeVertex;
}
vector<Vector3f> BodyMobili::SortValidVertices(vector<Vector3f> pointList)
{
	if (pointList.size() <=2) return pointList;
	///////////////////////////////////////////////////////
	// 1. Compute transform matrix for the vertex projection
	double transMat[16];
	Vector3f planeVec = pointList[1] - pointList[0];
	Vector3f planeNormal = (pointList[2] - pointList[0]) CROSS(pointList[1] - pointList[0]);
	planeNormal = planeNormal / len(planeNormal);
	GetPlaneTransformMatrix(planeNormal, planeVec, transMat);
	double inveTransMat[16];
	memcpy(inveTransMat, transMat, sizeof(double) * 16);
	if (invert4by4(inveTransMat) == 0)  printf("Inverse Matrix Error \n");
	///////////////////////////////////////////////////////
	// 2. Transform the vertices such that their z-coordinates are the same
	vector<Vector3f> projPtList;
	for (int i = 0; i < pointList.size(); i++)
	{
		Vector3f projPt;
		MultiplyPoint(pointList[i], transMat, projPt);
		projPtList.push_back(projPt);
	}
	///////////////////////////////////////////////////////
	// 3. Compute 2D convex hull of the projected vertices
	vector<Vector3f> hullPtList = convex_hull(projPtList);
	///////////////////////////////////////////////////////
	// 4. Project back the hull vertices (note: order is important)
	vector<Vector3f> sortedPoints;
	for (int i = 0; i < hullPtList.size(); i++)
	{
		Vector3f newVer;
		MultiplyPoint(hullPtList[i], inveTransMat, newVer);
		sortedPoints.push_back(newVer);
	}
	return sortedPoints;
}
void BodyMobili::ComputeMobilityMesh(vector<Vector3f> sortedPoints)
{
	if ( mobiliMesh != NULL )
	{
		mobiliMesh.reset();
	}
	mobiliMesh = make_shared<PolyMesh>();
	//////////////////////////////////////////////////////
	// 1. Vertices of mobility mesh
	Vector3f planeAVertex = GetPlaneVertex(sortedPoints[0], planeNormals[0]);
	Vector3f planeBVertex = GetPlaneVertex(sortedPoints[0], planeNormals[1]);
	mobiliMesh->vertexList = sortedPoints;
	mobiliMesh->vertexList.push_back(planeAVertex);
	mobiliMesh->vertexList.push_back(planeBVertex);
	mobiliMesh->vertexList.push_back(Vector3f(0, 0, 0));
	//////////////////////////////////////////////////////
	// 2. Polygons of mobility mesh
	int lastVerID[2];
	if (sortedPoints.size() == 2)
	{
		lastVerID[0] = mobiliMesh->vertexList.size()-3;
		lastVerID[1] = mobiliMesh->vertexList.size()-2;
	} else {
		lastVerID[0] = mobiliMesh->vertexList.size()-1;
		lastVerID[1] = mobiliMesh->vertexList.size()-1;
	}
	for (int i = 0; i < sortedPoints.size(); i++)
	{
		pPolygon poly = make_shared<_Polygon>();
		poly->vers.push_back(_Vertex(mobiliMesh->vertexList[i]));
		poly->vers.push_back(_Vertex(mobiliMesh->vertexList[(i + 1) % sortedPoints.size()]));
		poly->vers.push_back(_Vertex(mobiliMesh->vertexList[lastVerID[i % 2]]));
		mobiliMesh->polyList.push_back(poly);
	}
	//////////////////////////////////////////////////////
	// 3. Update mobility rays
	mobiliRays = sortedPoints;
	mobiliRays.push_back(planeAVertex);
	mobiliRays.push_back(planeBVertex);
}
void BodyMobili::CorrectMobilityMesh(Vector3f _mobiliVec)
{
	for (int i = 0; i < mobiliMesh->polyList.size(); i++)
	{
		pPolygon poly = mobiliMesh->polyList[i];
		float dotp = poly->normal DOT _mobiliVec;
		if ( dotp > 0 )
		{
			poly->ReverseVertices();
		}
	}
}
void BodyMobili::ComputeMobilityScore(vector<Vector3f> sortedPoints)
{
	mobiliScore = 0;
	int polyNum = mobiliMesh->polyList.size();
	for (int i = 0; i < mobiliMesh->polyList.size(); i++)
	{
		pPolygon polyA = mobiliMesh->polyList[i];
		pPolygon polyB = mobiliMesh->polyList[(i + 1) % polyNum];
		float dotp = polyA->normal DOT polyB->normal;
		float diheAngle = M_PI - acos(dotp);
		mobiliScore += diheAngle;
	}
	mobiliScore -= (polyNum - 2) * M_PI;
}


#if USE_OPENGL_DRAW
//**************************************************************************************//
//                                  Draw Construction
//**************************************************************************************//
void BodyMobili::DrawMobilityFaces()
{
	glPushMatrix();
	glTranslatef(bodyTrans.x, bodyTrans.y, bodyTrans.z);
	glScalef(bodyScale, bodyScale, bodyScale);
	for (int i = 0; i < faceList.size(); i++)
	{
		DrawMobilityFace(faceList[i]);
	}
	glPopMatrix();
}
void BodyMobili::DrawMobilityFace(pHypPlane face)
{
	Vector3f point = face->point;
	Vector3f normal = face->normal;
	float radius = face->radius;
	Vector3f color = colorTable[face->planeID];
	DrawPlane(point, normal, radius, color);
}
void BodyMobili::DrawMobilityEdges()
{
	float length = 8.0;
	glPushMatrix();
	glTranslatef(bodyTrans.x, bodyTrans.y, bodyTrans.z);
	glScalef(bodyScale, bodyScale, bodyScale);
	for (int i = 0; i < edgeList.size(); i++)
	{
		DrawMobilityEdge(edgeList[i], length);
	}
	glPopMatrix();
}
void BodyMobili::DrawMobilityEdge(pHypEdge edge, float length)
{
	glDisable(GL_LIGHTING);
	glPointSize(15.0);
	glLineWidth(4.0);
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
void BodyMobili::DrawMobilityVertices()
{
	glPushMatrix();
	glTranslatef(bodyTrans.x, bodyTrans.y, bodyTrans.z);
	glScalef(bodyScale, bodyScale, bodyScale);
	for (int i = 0; i < verList.size(); i++)
	{
		DrawMobilityVertex(verList[i]);
	}
	glPopMatrix();
}
void BodyMobili::DrawMobilityVertex(pHypVertex vertex)
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
//**************************************************************************************//
//                                  Draw Mobility
//**************************************************************************************//
void BodyMobili::DrawMobilityRay()
{
	float scale = 1.2f;
	glDisable(GL_LIGHTING);
	glLineWidth(4.0);
	glColor3f(0.9, 0.5, 0.2);
	glPushMatrix();
	glTranslatef(bodyTrans.x, bodyTrans.y, bodyTrans.z);
	glScalef(bodyScale, bodyScale, bodyScale);
	for (int i = 0; i < mobiliRays.size(); i++)
	{
		Vector3f endPt1 = Vector3f(0, 0, 0);
		Vector3f endPt2 = scale*mobiliRays[i];
		glBegin(GL_LINES);
		glVertex3f(endPt1.x, endPt1.y, endPt1.z);
		glVertex3f(endPt2.x, endPt2.y, endPt2.z);
		glEnd();
	}
	glPopMatrix();
	glEnable(GL_LIGHTING);
	glLineWidth(1.0);
}
void BodyMobili::DrawMobilityVector(float lineWidth, Vector3f color)
{
	glDisable(GL_LIGHTING);
	glLineWidth(lineWidth);
	glColor3f(color.x, color.y, color.z);
	Vector3f origin = bodyTrans;
	Vector3f point = bodyTrans + 3.5f * bodyScale * mobiliVec;
	glBegin(GL_LINES);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(point.x, point.y, point.z);
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}
void BodyMobili::DrawMobilityMesh()
{
	if (mobiliMesh == NULL){
		return;
	}
	//Vector3f color = Vector3f(0.9, 0.3, 0.3);
	Vector3f color = ColorMapping(0, MOBILITY_SCORE_MAX, mobiliScore);
	float mtlAmbient[4] = { color.x, color.y, color.z, 0.4 };
	float mtlDiffuse[4] = { color.x, color.y, color.z, 0.4 };
	float mtlSpecular[4] = { color.x, color.y, color.z, 0.4 };
	float mtlEmission[4] = { color.x, color.y, color.z, 0.4 };
	glPushAttrib(GL_LIGHTING_BIT);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mtlAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mtlDiffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mtlSpecular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mtlEmission);
	glPushMatrix();
	glTranslatef(bodyTrans.x, bodyTrans.y, bodyTrans.z);
	glScalef(bodyScale, bodyScale, bodyScale);
	mobiliMesh->DrawMesh();
	mobiliMesh->DrawMesh_Wire(2.0, Vector3f(0.3, 0.3, 0.3));
	glPopMatrix();
	glPopAttrib();
}
#endif