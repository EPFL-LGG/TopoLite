///////////////////////////////////////////////////////////////
//
// MeshConverter.cpp
//
//   Convert in between different mesh representations:
//   1) Triangular mesh
//   2) Polygonal mesh 
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 21/July/2018
//
//
///////////////////////////////////////////////////////////////

#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"
#include "Utility/ConvexHull2D.h"
#include "IO/gluiVar.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/MeshConverter.h"

#include <Eigen/Dense>
#include <igl/boundary_loop.h>
#include <igl/lscm.h>
#include <igl/triangle/triangulate.h>
#include <igl/remove_duplicate_vertices.h>


//**************************************************************************************//
//                      Generate Texture Mesh
//**************************************************************************************//

void MeshConverter::generateTexture(const PolyMesh *polyMesh, shared_ptr<PolyMesh> &out)
{
    Convert2TriMesh(polyMesh, out);
    out->UpdateVertices();
    Eigen::MatrixXd V(out->vertexList.size(), 3);
    Eigen::MatrixXi F(out->polyList.size(), 3);

    for(int id = 0; id < out->vertexList.size(); id ++ )
    {
        Vector3f pt = out->vertexList[id];
        V.row(id) = Eigen::RowVector3d(pt.x, pt.y, pt.z);
    }
    for(int id = 0; id < out->polyList.size(); id++)
    {
        F.row(id) << out->polyList[id]->verIDs[0], out->polyList[id]->verIDs[1], out->polyList[id]->verIDs[2];
    }

    // Fix two points on the boundary
    Eigen::VectorXi bnd,b(2,1);
    igl::boundary_loop(F,bnd);
    b(0) = bnd(0);
    b(1) = bnd(round(bnd.size()/2));
    Eigen::MatrixXd bc(2,2);
    bc<<0,0,1,0;

    Eigen::MatrixXd V_uv;
    // LSCM parametrization
    igl::lscm(V,F,b,bc,V_uv);

//	Eigen::Vector2d min, max;
//	min << V_uv.col(0).minCoeff(), V_uv.col(1).minCoeff();
//	max << V_uv.col(0).maxCoeff(), V_uv.col(1).maxCoeff();
//
//	double x_length = std::min(0.9, max[0]) - std::max(0.1, min[0]);
//	double y_length = std::min(0.9, max[1]) - std::max(0.1, min[1]);

    for(int id = 0; id < V_uv.rows(); id++)
    {
        Eigen::Vector2d tex = V_uv.row(id);

//		tex[0] = (tex[0] - min[0]) / (max[0] - min[0]) * x_length + std::max(0.1, min[0]);
//		tex[1] = (tex[1] - min[1]) / (max[1] - min[1]) * y_length + std::max(0.1, min[1]);
//		std::cout << tex.transpose() << std::endl;
        out->texCoordList.push_back(Vector2f(tex[0], tex[1]));
    }

    for(int id = 0; id < out->polyList.size(); id++)
    {
        out->polyList[id]->texIDs.resize(3);
        out->polyList[id]->texIDs[0] = F(id, 0);
        out->polyList[id]->texIDs[1] = F(id, 1);
        out->polyList[id]->texIDs[2] = F(id, 2);

        out->polyList[id]->vers[0].texCoord = out->texCoordList[F(id, 0)];
        out->polyList[id]->vers[1].texCoord = out->texCoordList[F(id, 1)];
        out->polyList[id]->vers[2].texCoord = out->texCoordList[F(id, 2)];
    }

    return;
}


//**************************************************************************************//
//                      Convert Polygonal Mesh to Triangular Mesh
//**************************************************************************************//

void MeshConverter::Convert2TriMesh(const PolyMesh* polyMesh, vector<pTriangle> &triList)
{
	for (int i = 0; i < polyMesh->polyList.size(); i++)
	{
		pPolygon poly = polyMesh->polyList[i];

		vector<pTriangle> triangles;
		poly->Convert2Triangles(triangles);

		for (int j = 0; j < triangles.size(); j++)
		{
			triList.push_back(triangles[j]);
		}
	}
}

void MeshConverter::Convert2TriMesh(const PolyMesh* polyMesh,  shared_ptr<PolyMesh> &out)
{
	vector<Vector3f> inVerList;
	vector<Vector3i> inTriList;
	for (int i = 0; i < polyMesh->polyList.size(); i++) {
		pPolygon poly = polyMesh->polyList[i];

		vector<pTriangle> triangles;
		poly->Convert2Triangles(triangles);

		for (int j = 0; j < triangles.size(); j++) {
			for(int k = 0; k < 3; k++){
				inVerList.push_back(triangles[j]->v[k]);
			}
			inTriList.push_back(Vector3i(inVerList.size() - 3, inVerList.size() - 2, inVerList.size() - 1));
		}
	}
	InitPolyMesh(inVerList, inTriList, out);
}

//**************************************************************************************//
//                      Convert to Mesh
//**************************************************************************************//

void MeshConverter::InitPolyMesh(   const vector<Vector3f> &inVerList,
                                    const vector<Vector3i> &inTriList,
                                    pPolyMesh &polyMesh)
{
    polyMesh.reset();
    polyMesh = make_shared<PolyMesh>();

    // Initialize vertices
    polyMesh->vertexList = inVerList;

    // Initialize faces
    for (int i = 0; i < inTriList.size(); i++)
    {
        pPolygon poly = make_shared<_Polygon>();

        for (int j = 0; j < 3; j++)
        {
            int verID = inTriList[i][j];
            Vector3f ver = inVerList[verID];
            poly->vers.push_back(ver);
        }

        poly->ComputeNormal();
        //poly->ComputeCenter();

        polyMesh->polyList.push_back(poly);
    }
}

void MeshConverter::InitPolyMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, pPolyMesh &out) {

    vector<Vector3f> inVerList;
    vector<Vector3i> inTriList;
    for(int id = 0; id < V.rows(); id++){
        inVerList.push_back(Vector3f(V(id, 0), V(id, 1), V(id, 2)));
    }
    for(int id = 0; id < F.rows(); id++){
        inTriList.push_back(Vector3i(F(id, 0), F(id, 1), F(id, 2)));
    }

    InitPolyMesh(inVerList, inTriList, out);

    return;
}

void MeshConverter::Convert2PolyMesh(const vector<Vector3f> &inVerList, const vector<Vector3i> &inTriList, pPolyMesh &polyMesh)
{
	// Initialize each face as a triangle
	InitPolyMesh(inVerList, inTriList, polyMesh);

	Convert2PolyMesh(polyMesh);
}

void MeshConverter::Convert2PolyMesh(pPolyMesh polyMesh)
{
	// Keep merging neighboring faces that are co-planar
	while (true)
	{
		bool isFinish = true;

		for (int i = 0; i < polyMesh->polyList.size(); i++)
		{
			for (int j = 0; j < polyMesh->polyList.size(); j++)
			{
				if (j == i)
					continue;

				pPolygon polyA = polyMesh->polyList[i];
				pPolygon polyB = polyMesh->polyList[j];

				if (IsCoplanar(polyA, polyB) == true)
				{
					isFinish = false;

					UpdatePolyList(polyA, i, polyB, j, polyMesh->vertexList, polyMesh->polyList);

					break;
				}
			}

			if (isFinish == false)
			{
				break;
			}
		}

		if (isFinish == true)
		{
			break;
		}
	}
}

void MeshConverter::Convert2PolyMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, pPolyMesh &out){

    InitPolyMesh(V, F, out);

    Convert2PolyMesh(out);
}

//**************************************************************************************//
//                      Convert to Eigen Mesh
//**************************************************************************************//

void MeshConverter::Convert2EigenMesh(const PolyMesh *polyMesh, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    vector<pTriangle> triangles;
    Convert2TriMesh(polyMesh, triangles);
    V = Eigen::MatrixXd(triangles.size() * 3, 3);
    F = Eigen::MatrixXi(triangles.size(), 3);
    for(int id = 0; id < triangles.size(); id++)
    {
        Triangle tri = *triangles[id];
        for(int kd = 0; kd < 3; kd++){
            F(id, kd) = 3 * id + kd;
            V.row(3 * id + kd) = Eigen::RowVector3d(tri.v[kd].x, tri.v[kd].y, tri.v[kd].z);
        }
    }

    Eigen::MatrixXd SV;
    Eigen::MatrixXi SVI, SVJ, SF;
    igl::remove_duplicate_vertices(V, F, 2e-5, SV, SVI, SVJ, SF);
    V = SV;
    F = SF;
    return;
}

void MeshConverter::Convert2EigenMesh(const vector<Vector3f> &inVerList,
                                      const vector<Vector3i> &inTriList,
                                      Eigen::MatrixXd &V,
                                      Eigen::MatrixXi &F){
    V = Eigen::MatrixXd(inVerList.size(), 3);
    F = Eigen::MatrixXi(inTriList.size(), 3);
    for(int id = 0; id < inVerList.size(); id++){
        V.row(id) << inVerList[id][0], inVerList[id][1], inVerList[id][2];
    }

    for(int id = 0; id < inTriList.size(); id++){
        F.row(id) << inTriList[id][0], inTriList[id][1], inTriList[id][2];
    }

}

//**************************************************************************************//
//                      ClipperPath <-> Mesh
//**************************************************************************************//

void MeshConverter::EigenMesh2ClipperPath(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, ClipperLib::Paths &contour)
{
    vector<vector<int>> L;
    igl::boundary_loop(F, L);

    double Scale = CLIPPER_INTERGER_SCALE;
    for(int id = 0; id < L.size(); id++)
    {
        ClipperLib::Path path;
        Eigen::RowVector3d preV(0, -1, 0);
        for(int jd = 0; jd < L[id].size(); jd++)
        {
            int vID = L[id][jd];
            ClipperLib::IntPoint pt;
            pt.X = V(vID, 0) * Scale;
            pt.Y = V(vID, 2) * Scale;
            if((preV - V.row(vID)).norm() > FLOAT_ERROR_LARGE){
                preV = V.row(vID);
                path.push_back(pt);
            }
        }
        if(path.size() >= 3)
        {
            contour.push_back(path);
        }
    }
    ClipperLib::CleanPolygons(contour);
    ClipperLib::SimplifyPolygons(contour, ClipperLib::pftPositive);
    return;
}

void MeshConverter::ClipperPathExtrusion(ClipperLib::Paths &contour, double height, shared_ptr<PolyMesh> &out)
{
    double Scale = CLIPPER_INTERGER_SCALE;
	int numV = 0, numE = 0, numH = 0;

	for(int id = 0; id < contour.size(); id++)
	{
        ClipperLib::Path path = contour[id];
        numV += path.size();
        numE += path.size();
        if(ClipperLib::Orientation(path) == false)
        {
            numH++;
        }
	}

    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    Eigen::MatrixXd H;
	V = Eigen::MatrixXd(numV, 2); numV = 0;
	E = Eigen::MatrixXi(numE, 2); numE = 0;
	H = Eigen::MatrixXd(numH, 2); numH = 0;
	for(int id = 0; id < contour.size(); id++)
	{
	    ClipperLib::Path path = contour[id];
	    for(int jd = 0; jd < path.size();jd++){
	        E(numE, 0) = numV + jd;
	        E(numE, 1) = numV + (jd + 1) % path.size();
            numE++;
	    }

        for(int jd = 0; jd < path.size(); jd++)
        {
            V(numV, 0) = path[jd].X / Scale;
            V(numV, 1) = path[jd].Y / Scale;
            numV++;
        }

        if(ClipperLib::Orientation(path) == false)
        {
            ClipperLib::ClipperOffset co;
            ClipperLib::Paths solution;
            co.AddPath(path, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            co.Execute(solution, -100);
            ClipperLib::IntPoint pt = solution.front().front();
            H.row(numH) << pt.X /Scale, pt.Y/Scale;
            numH++;
        }
	}
	Eigen::MatrixXd VA;
	Eigen::MatrixXi FA;

	out.reset();

	if(V.rows() >= 3)
	{
		igl::triangle::triangulate(V,E,H,"a0.01qQ",VA,FA);

		double thickness = getVarList()->get<float>("support_thickness");
		Eigen::MatrixXd VB = Eigen::MatrixXd(VA.rows() * 2 + V.rows() * 2, 3);
		Eigen::MatrixXi FB = Eigen::MatrixXi(FA.rows() * 2 + E.rows() * 2, 3);

		//bottom-up
		for(int id = 0; id < VA.rows(); id++)
		{
			VB.row(id) << VA(id, 0), height, VA(id, 1);
			VB.row(id + VA.rows()) << VA(id, 0), height + thickness, VA(id, 1);
		}
		//side
		for(int id = 0; id < V.rows(); id++){
			VB.row(id +            2 * VA.rows()) << V(id, 0), height,             V(id, 1);
			VB.row(id + V.rows() + 2 * VA.rows()) << V(id, 0), height + thickness, V(id, 1);
		}

		//bottom-up
		for(int id = 0; id < FA.rows(); id++)
		{
			FB.row(id) << FA(id, 0), FA(id, 1), FA(id, 2);
			FB.row(id + FA.rows()) << FA(id, 2) + VA.rows(), FA(id, 1) + VA.rows(), FA(id, 0) + VA.rows();
		}

		//side
		for(int id = 0; id < E.rows(); id++)
		{
			FB.row(2 * id     + 2 * FA.rows()) << E(id, 1) + 2 * VA.rows(), E(id, 0) + 2 * VA.rows(), E(id, 1) + 2 * VA.rows() + V.rows();
			FB.row(2 * id + 1 + 2 * FA.rows()) << E(id, 0) + 2 * VA.rows() + V.rows(), E(id, 1) + 2 * VA.rows() + V.rows(), E(id, 0) + 2 * VA.rows();
		}

		Eigen::MatrixXd VC;
		Eigen::MatrixXi FC, SVI, SVJ;
		igl::remove_duplicate_vertices(VB, FB, FLOAT_ERROR_LARGE, VC, SVI, SVJ, FC);

		Convert2PolyMesh(VC, FC, out);
	}
}

//**************************************************************************************//
//                      Private Functions
//**************************************************************************************//


bool MeshConverter::IsCoplanar(pPolygon polyA, pPolygon polyB)
{
    Vector3f cenDiffVec = polyB->center - polyA->center;
    cenDiffVec = cenDiffVec / len(cenDiffVec);

    float dotpA = polyA->normal DOT polyB->normal;
    float dotpB = polyA->normal DOT cenDiffVec;
    float dotpC = polyB->normal DOT cenDiffVec;


    //printf("dotpA: %.6f    dotpB: %.6f \n", dotpA, dotpB);

    if ( fabs(1-dotpA) < CO_PLANAR_THRES &&  // plane normals should be parallel
         fabs(dotpB)   < CO_PLANAR_THRES &&  // plane point should be co-planar
         fabs(dotpC)   < CO_PLANAR_THRES  )  // plane point should be co-planar
    {
        return true;
    }
    else
    {
        return false;
    }
}

void MeshConverter::UpdatePolyList( pPolygon polyA,
                                    int polyAID,
                                    pPolygon polyB,
                                    int polyBID,
                                    const vector<Vector3f> vertexList,
                                    vector<pPolygon> &polyList)
{
    pPolygon newPoly;
    MergePolygons(polyA, polyB, vertexList, newPoly);

    vector<pPolygon> newPolyList;
    for (int i = 0; i < polyList.size(); i++)
    {
        if (i == polyAID || i == polyBID)
        {
            continue;
        }
        newPolyList.push_back(polyList[i]);
    }

    //newPoly->Print();
    polyA.reset();
    polyB.reset();

    newPolyList.push_back(newPoly);

    polyList = newPolyList;
}

// Note: assume the merged polygon is convex as we use quick hull to compute the merged polygon
void MeshConverter::MergePolygons(pPolygon polyA, pPolygon polyB, const vector<Vector3f> vertexList, pPolygon &newPoly)
{
    ///////////////////////////////////////////////////////
    // 1. Unique vertices of ployA and polyB

    vector<_Vertex> polyVers;
    polyVers = polyA->vers;

    for (int i = 0; i < polyB->vers.size(); i++)
    {
        Vector3f ver = polyB->vers[i].pos;

        if (polyA->GetVertexIndexInList(ver) == ELEMENT_OUT_LIST)
        {
            polyVers.push_back(ver);
        }
    }


    ///////////////////////////////////////////////////////
    // 2. Compute transform matrix for the vertex projection

    double transMat[16];
    Vector3f planeVec = polyB->center - polyA->center;
    GetPlaneTransformMatrix(polyA->normal, planeVec, transMat);

    double inveTransMat[16];
    memcpy(inveTransMat, transMat, sizeof(double) * 16);
    if (invert4by4(inveTransMat) == 0)  printf("Inverse Matrix Error \n");


    ///////////////////////////////////////////////////////
    // 3. Transform the vertices such that their z-coordinates are the same

    vector<Vector3f> projPtList;
    for (int i = 0; i < polyVers.size(); i++)
    {
        Vector3f projPt;
        MultiplyPoint(polyVers[i].pos, transMat, projPt);

        projPtList.push_back(projPt);
    }


    ///////////////////////////////////////////////////////
    // 4. Compute 2D convex hull of the projected vertices

    vector<Vector3f> hullPtList = convex_hull(projPtList);


    ///////////////////////////////////////////////////////
    // 5. Project back the hull vertices (note: order is important)

    vector<Vector3f> newPolyVers;
    for (int i = 0; i < hullPtList.size(); i++)
    {
        Vector3f newVer;
        MultiplyPoint(hullPtList[i], inveTransMat, newVer);

        newPolyVers.push_back(newVer);
    }

    vector<int> newPolyVerIDs;
    for (int i = 0; i < newPolyVers.size(); i++)
    {
        Vector3f newVer = newPolyVers[i];

        int verID = GetPointIndexInList(newVer, vertexList);
        if (verID == ELEMENT_OUT_LIST)
        {
            printf("Warning: newVer should be inside verList. \n");
            break;
        }

        newPolyVerIDs.push_back(verID);
        newPolyVers[i] = vertexList[verID];  // Avoid modifying vertices during the projection
    }


    ///////////////////////////////////////////////////////
    // 6. Construct the merged polygon

    newPoly.reset();
    newPoly = make_shared<_Polygon>();

    for (int i = 0; i < newPolyVers.size(); i++)
    {
        newPoly->vers.push_back(_Vertex(newPolyVers[i]));
    }

    //newPoly->ComputeCenter();
    newPoly->ComputeNormal();
}
