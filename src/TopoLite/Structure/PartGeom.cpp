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
#include "IO/InputVar.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PartGeom::PartGeom(shared_ptr<Cross> _cross, shared_ptr<InputVarList> var) :TopoObject(var)
{
    ParseCrossData(_cross);
}

void PartGeom::ParseCrossData(shared_ptr<Cross> _cross)
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

PartGeom::PartGeom(const PartGeom &_geom): TopoObject(_geom)
{
    ParseCrossData(_geom.cross.lock());
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
//                                 Validate Part Geometry
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

bool PartGeom::IsLegalGeometry()
{
	return ValidateTiltNormal();
}


//**************************************************************************************//
//                                Compute Part Geometry
//**************************************************************************************//

void PartGeom::ComputePartGeometry(Vector2f cutPlaneHeight, pPolyMesh &polyMesh)
{
    if (oriPoints.size() == 0){
        polyMesh = nullptr;
        return;
    }

    // 1) Compute all validate vertices
    ComputeFaces(cutPlaneHeight);

    // 2) Compute all vertices (valid or invalid)
    ComputeVertices();

    // 3) Remove all invalid vertices
    vector<Vector3f> pointList;
    ComputeValidVertices(pointList);

    // 4) Compute the convex hull
    vector<Vector3f> ver;
    vector<Vector3i> tri;
    computeQuickHull(pointList, ver, tri);

    // 5) Convert the triangle convex hull into a polyMesh
    Convert2PolyMesh(ver, tri, polyMesh);
}

void PartGeom::ComputeVertices()
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

	if(boundary || !getVarList()->get<bool>("only_cut_bdry"))
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

void PartGeom::Convert2PolyMesh(vector<Vector3f> &ver, vector<Vector3i> &tri, pPolyMesh &polyMesh)
{
    polyMesh.reset();

    vector<vector<Vector3f>> facePolygons;

    //get all triangle face
    std::list<shared_ptr<_Polygon>> triangles;
	for(Vector3i tri : tri)
	{
		shared_ptr<_Polygon> poly = make_shared<_Polygon>();
		for(int id = 0; id < 3; id++)
		{
			_Vertex vertex;
			vertex.pos = ver[tri[id]];
			poly->vers.push_back(vertex);
			poly->verIDs.push_back(tri[id]);
		}
		poly->ComputeNormal();
		triangles.push_back(poly);
	}

    //2) cluster triangle faces by checking its normal
    facePolygons.resize(hypList.size());
	int k = 0;
	for(auto it = triangles.begin(); it != triangles.end();it ++)
	{
		Vector3f normal = (*it)->normal;

		double max_dot = 0;
		shared_ptr<HypPlane> max_plane;
		for(int kd = 0; kd < hypList.size(); kd++)
		{
			if(hypList[kd] == nullptr) continue;
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

	//3) do convex hull to reconstruct the face
	polyMesh = make_shared<PolyMesh>(getVarList());
	polyMesh->vertexList = ver;
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
		for(Vector3f pt : poly)
		{
			pt = pt - origin;
			poly2D.push_back(Vector3f(pt DOT x_axis, pt DOT y_axis, 0));
		}

		poly.clear();
		poly = convex_hull(poly2D);
		shared_ptr<_Polygon> face = make_shared<_Polygon>();
		for(Vector3f pt : poly)
		{
			pt = x_axis * pt[0] + y_axis * pt[1] + origin;
			face->vers.push_back(pt);
		}
		face->ComputeNormal();
		faceList[id] = face;
		polyMesh->polyList.push_back(face);
	}
}

void PartGeom::ComputeValidVertices(vector<Vector3f> &pointList)
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