///////////////////////////////////////////////////////////////
//
// Mesh.cpp
//
//   Polygonal Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 09/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef CATCH2_UNITTEST

#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"

#include "igl/boundary_loop.h"
#include "igl/readOBJ.h"
#include "igl/lscm.h"

#include "Polygon.h"
#include "PolyMesh.h"
#include "MeshConverter.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PolyMesh::~PolyMesh()
{
	ClearMesh();
}

PolyMesh::PolyMesh(const PolyMesh &_mesh): TopoObject(_mesh)
{
    ClearMesh();
    vertexList = _mesh.vertexList;
    texCoordList = _mesh.texCoordList;
    texturedModel = _mesh.texturedModel;

    bbox = _mesh.bbox;
    centroid = _mesh.centroid;
    volume = _mesh.volume;
    lowestPt = _mesh.lowestPt;

    for(int id = 0; id < _mesh.polyList.size(); id++)
    {
        shared_ptr<_Polygon> poly = make_shared<_Polygon>(*_mesh.polyList[id]);
        polyList.push_back(poly);
    }

    return;
}


void PolyMesh::ClearMesh()
{
	polyList.clear();
    vertexList.clear();
    texCoordList.clear();
}

void PolyMesh::Print()
{
	printf("vertex num: %lu \n", vertexList.size());
	for (int i = 0; i < vertexList.size(); i++)
	{
		Vector3f ver = vertexList[i];
		printf("%9.6f  %9.6f  %9.6f \n", ver.x, ver.y, ver.z);
	}
	printf("\n");

	printf("polygon num: %lu \n", polyList.size());
	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];
		for(int j=0; j<poly->verIDs.size(); j++)
		{
			printf("%d ", poly->verIDs[j]);
		}
		printf("\n");
	}
	printf("\n");
}

//**************************************************************************************//
//                                  Mesh Operations
//**************************************************************************************//

void PolyMesh::ComputeBBox()
{
	float minX, minY, minZ, maxX, maxY, maxZ;
	minX = minY = minZ = MAX_FLOAT;
	maxX = maxY = maxZ = MIN_FLOAT;

	for (int i = 0; i < vertexList.size(); i++)
	{
		Vector3f ver = vertexList[i];

		if (maxX < ver.x)  maxX = ver.x;
		if (minX > ver.x)  minX = ver.x;

		if (maxY < ver.y)  maxY = ver.y;
		if (minY > ver.y)  minY = ver.y;

		if (maxZ < ver.z)  maxZ = ver.z;
		if (minZ > ver.z)  minZ = ver.z;
	}

	bbox.minPt = Vector3f(minX, minY, minZ);
	bbox.maxPt = Vector3f(maxX, maxY, maxZ);
	bbox.cenPt = 0.5f*(bbox.minPt + bbox.maxPt);
	bbox.size = bbox.maxPt - bbox.minPt;
}

Box PolyMesh::ComputeTextureBBox()
{
	Box texBBox;

	float minU, minV, maxU, maxV;
	minU = minV = MAX_FLOAT;
	maxU = maxV = MIN_FLOAT;

	for (int i = 0; i < texCoordList.size(); i++)
	{
		Vector2f texCoord = texCoordList[i];

		if (maxU < texCoord.x)   maxU = texCoord.x;
		if (minU > texCoord.x)   minU = texCoord.x;

		if (maxV < texCoord.y)   maxV = texCoord.y;
		if (minV > texCoord.y)   minV = texCoord.y;
	}

	texBBox.minPt = Vector3f(minU, minV, 0);
	texBBox.maxPt = Vector3f(maxU, maxV, 0);
	return texBBox;
}

void PolyMesh::NormalizeMesh()
{
	ComputeBBox();

	float scale = 2.0f / _MAX(bbox.size.x, _MAX(bbox.size.y, bbox.size.z));
	Vector3f trans = -bbox.cenPt;

	for (int i = 0; i < vertexList.size(); ++i)
	{
		Vector3f origVer = vertexList[i];
		vertexList[i] = (origVer+trans) * scale;
	}

	for (int i = 0; i < polyList.size(); i++)
	{
		for (int j = 0; j < polyList[i]->vers.size(); ++j)
		{
			Vector3f origVer = polyList[i]->vers[j].pos;

			polyList[i]->vers[j].pos = (origVer + trans) * scale;
		}
	}
}

//**************************************************************************************//
//                                  Volume and Centroid
//**************************************************************************************//

float PolyMesh::ComputeVolume()
{
	// Convert into triangular mesh
	MeshConverter meshConverter(getVarList());
	vector<pTriangle> triList;
	meshConverter.Convert2TriMesh(this, triList);

	// Compute volume of the triangular mesh
	volume = ComputeVolume(triList);

	// Release the memory
	triList.clear();

	return volume;
}
/////////////////////////////////////////////////////////////////////
// Compute model volume following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf

float PolyMesh::ComputeVolume(vector<pTriangle> triList)
{
	float volume = 0;

	// Accumulate volume value for each triangle
	for (int i = 0; i < triList.size(); i++)
	{
		pTriangle tri = triList[i];

		Vector3f ver0 = tri->v[0];
		Vector3f crossVec = -1.0f * (tri->v[2] - tri->v[0]) CROSS (tri->v[1] - tri->v[0]);
		float dotP = ver0 DOT crossVec;

		volume += dotP;
	}

	volume = volume / 6.0f;

	return volume;
}

void PolyMesh::ComputeCentroid()
{
	// Convert into triangular mesh
	MeshConverter meshConverter;
	vector<pTriangle> triList;
	meshConverter.Convert2TriMesh(this, triList);

	// Compute centroid of the triangular mesh
	centroid = ComputeCentroid(triList);

	// Release the memory
	triList.clear();
}
/////////////////////////////////////////////////////////////////////
// Compute model centroid following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf

Vector3f PolyMesh::ComputeCentroid(vector<pTriangle> triList)
{
	Vector3f centroid = Vector3f(0, 0, 0);

	// Save the 3 major axes
	Vector3f axes[3];
	axes[0] = Vector3f(1, 0, 0);
	axes[1] = Vector3f(0, 1, 0);
	axes[2] = Vector3f(0, 0, 1);

	// Accumulate centroid value for each major axes
	for (int i = 0; i < 3; i++)
	{
		Vector3f axis = axes[i];

		for (int j = 0; j < triList.size(); j++)
		{
			pTriangle tri = triList[j];
			Vector3f crossVec = -1.0f * (tri->v[2] - tri->v[0]) CROSS(tri->v[1] - tri->v[0]);

			centroid[i] += (1 / 24.0f) * (crossVec DOT axis) *
				(pow((tri->v[0] + tri->v[1]) DOT axis, 2) +
				pow((tri->v[1] + tri->v[2]) DOT axis, 2) +
				pow((tri->v[2] + tri->v[0]) DOT axis, 2));
		}
	}

	// Compute volume and centroid
	float volume = ComputeVolume(triList);
	centroid = centroid / (2.0f*volume);

	return centroid;
}

void PolyMesh::ComputeLowestPt()
{
	Vector3f down_direc (0.0, -1.0, 0.0);
	lowestPt = ComputeExtremeVertex(down_direc);
}

Vector3f PolyMesh::ComputeExtremeVertex(Vector3f rayDir)
{
	Vector3f extremVer(0, 0, 0);

	// Push back face center points
	vector<Vector3f> pointList;
	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];
		pointList.push_back(poly->center);
	}

	//Push back edge middle points
	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];
		for (int j = 0; j < poly->vers.size(); j++)
		{
			Vector3f staPt = poly->vers[j].pos;
			Vector3f endPt = poly->vers[(j + 1) % poly->vers.size()].pos;

			Vector3f edgeMidPt = (staPt + endPt) / 2.0f;

			pointList.push_back(edgeMidPt);
		}
	}

	//Push back all vertices
	pointList.insert(pointList.end(), vertexList.begin(), vertexList.end());

	// Find the extreme point with the largest distance along ray direction and the shortest distance along perpendicular direction
	const float weight = 0.3f;
	float maxDist = -std::numeric_limits<float>::max();
	for (int i = 0; i < pointList.size(); i++)
	{
		Vector3f ver = pointList[i];

//		float verDist = len(ver);
//		float rayDist = ver DOT rayDir;
//		float perpDist = sqrt(verDist*verDist - rayDist*rayDist);
//
//
//		float dist = rayDist - weight*perpDist;
		float  dist = ver DOT rayDir /(len(rayDir));
		if (dist > maxDist + 1e-4)
		{
			maxDist = dist;
			extremVer = ver;
		}
	}
	return extremVer;
}




//**************************************************************************************//
//                                   Transform Mesh
//**************************************************************************************//

void PolyMesh::RotateMesh(Vector3f rotCenter, Vector3f rotAxis, float rotAngle)
{
	for (int i = 0; i < vertexList.size(); ++i)
	{
		Vector3f &ver = vertexList[i];

		ver = RotateVector(rotCenter, rotAxis, rotAngle, ver);
	}

	for (int i = 0; i < polyList.size(); ++i)
	{
		pPolygon poly = polyList[i];

		for (int j = 0; j < poly->vers.size(); ++j)
		{
			Vector3f &ver = poly->vers[j].pos;

			ver = RotateVector(rotCenter, rotAxis, rotAngle, ver);
		}
	}
}

void PolyMesh::TranslateMesh(Vector3f move)
{
    for (int i = 0; i < vertexList.size(); ++i) {
        Vector3f &ver = vertexList[i];
        ver+= move;
    }

    for (int i = 0; i < polyList.size(); ++i)
    {
        pPolygon poly = polyList[i];
        for (int j = 0; j < poly->vers.size(); ++j)
        {
            Vector3f &ver = poly->vers[j].pos;
            ver += move;
        }
    }
}


//**************************************************************************************//
//                                   Read Save OBJ File
//**************************************************************************************//

bool PolyMesh::ReadOBJModel(
        const char *fileName,
        bool &textureModel_,
        bool normalized)
{
	vector<vector<double>> V, TC, CN;
	vector<vector<int>> F, FTC, FN;

	if(igl::readOBJ(fileName, V, TC, CN, F, FTC, FN))
	{
	    ClearMesh();

        for(int id = 0; id < V.size(); id++)
        {
            vertexList.push_back(Vector3f(V[id][0], V[id][1], V[id][2]));
            if(!TC.empty()) texCoordList.push_back(Vector2f(TC[id][0], TC[id][1]));
        }
        for(int id = 0; id < F.size(); id++)
        {
			shared_ptr<_Polygon> poly = make_shared<_Polygon>();
            poly->verIDs.resize(F[id].size());
            poly->vers.resize(F[id].size());

            for(int jd = 0; jd < F[id].size(); jd++)
			{
				poly->verIDs[jd] = F[id][jd];
				poly->vers[jd].pos = vertexList[F[id][jd]];
			}

            if(!TC.empty()){
                poly->texIDs.resize(F[id].size());

				for(int jd = 0; jd < F[id].size(); jd++)
				{
					poly->texIDs[jd] = FTC[id][jd];
					poly->vers[jd].texCoord = texCoordList[FTC[id][jd]];
				}
            }
            polyList.push_back(poly);
        }

	    if(TC.empty()){
	        texturedModel = textureModel_ = false;
	    }
	    else{
            texturedModel = textureModel_ = true;
	    }
	    if(normalized) NormalizeMesh();
        UpdateVertices();
		return true;
	}
	else {
	    return false;
	}
}

shared_ptr<PolyMesh> PolyMesh::getTextureMesh()
{
    shared_ptr<PolyMesh> polymesh = make_shared<PolyMesh>(getVarList());
    for (int i = 0; i < polyList.size(); i++) {
        pPolygon poly = make_shared<_Polygon>(*polyList[i]);
        for(int j = 0; j < poly->vers.size(); j++){
            poly->vers[j].pos = Vector3f(poly->vers[j].texCoord[0], poly->vers[j].texCoord[1], 0);
        }
        polymesh->polyList.push_back(poly);
    }

    return polymesh;
}


void PolyMesh::WriteOBJModel(const char *objFileName, bool triangulate)
{
	FILE *fp;
	if ((fp = fopen(objFileName, "w+")) == NULL)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		///////////////////////////////////////////////////////////////////
		// 1. Compute vertices and vertexIDs in case they are missing

		UpdateVertices();

		///////////////////////////////////////////////////////////////////
		// 2. Write the vertex info of the mesh

		//fprintf(fp, "# Wavefront OBJ generated by Peng SONG \n\n");

		fprintf(fp, "# %d vertices \n", (int)vertexList.size());

		for (int i = 0; i < vertexList.size(); i++)
		{
			fprintf(fp, "v %f %f %f \n", vertexList[i].x , vertexList[i].y, vertexList[i].z );
		}
		fprintf(fp, "\n");


		for(int i = 0; i < texCoordList.size(); i++)
		{
			fprintf(fp, "vt %f %f \n", texCoordList[i].x , texCoordList[i].y);
		}
		fprintf(fp, "\n");

		///////////////////////////////////////////////////////////////////
		// 3. Write each polygon info of the mesh

		fprintf(fp, "# %d faces \n", (int)polyList.size());

		for (int i = 0; i < polyList.size(); i++)
		{
			pPolygon poly = polyList[i];

			//fprintf(fp, "f ");
			//for (int j = 0; j < poly->verIDs.size(); j++)
			//{
			//	//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
			//	fprintf(fp, " %d", poly->verIDs[j] + 1);
			//}
			//fprintf(fp, "\n");

            if(!triangulate){
                fprintf(fp, "f ");
                for (int j = 0; j < poly->vers.size(); j++)
                {
                    int verID = poly->verIDs[j];
                    //Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
                    if(texCoordList.empty())
                        fprintf(fp, " %d", verID + 1);
                    else
                    {
                        int texID = poly->texIDs[j];
                        fprintf(fp, " %d/%d", verID + 1, texID + 1);
                    }

                }
                fprintf(fp, "\n");
            }
            else{
                for(int j = 0; j < poly->vers.size() - 2; j++)
                {
                    fprintf(fp, "f ");
                    int indices[3] = {0, j + 1, j + 2};
                    for(int k = 0; k < 3; k++)
                    {
						int verID = poly->verIDs[indices[k]];
                        if(texCoordList.empty())
                            fprintf(fp, " %d", verID + 1);
                        else{
                            int texID = poly->texIDs[indices[k]];
                            fprintf(fp, " %d/%d", verID + 1, texID + 1);
                        }
                    }
                    fprintf(fp, "\n");
                }
            }
		}
		fprintf(fp, "\n");

		fclose(fp);
	}
}

void PolyMesh::UpdateVertices()
{
	vertexList.clear();
	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];

		for (int j = 0; j < poly->vers.size(); j++)
		{
			Vector3f ver = poly->vers[j].pos;
			if( GetPointIndexInList(ver, vertexList) == ELEMENT_OUT_LIST)
			{
				vertexList.push_back(ver);
			}
		}
	}

	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];

		poly->verIDs.clear();  // Note: verIDs mostly should be empty

		for (int j = 0; j < poly->vers.size(); j++)
		{
			Vector3f ver = poly->vers[j].pos;

			int index = GetPointIndexInList(ver, vertexList);

			if (index == ELEMENT_OUT_LIST)
			{
				//vertexList.push_back(ver);
				//poly->verIDs.push_back(vertexList.size()-1);
				printf("Warning: the vertex should be in the list. \n");
			}
			else
			{
				poly->verIDs.push_back(index);
			}
		}
	}
}



#else

#include <catch2/catch.hpp>
#include "PolyMesh.h"
#include <cmath>
TEST_CASE("Class Mesh")
{


    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    //CASE 1: Read Mesh
    PolyMesh poly(varList);
    bool textureModel;
    REQUIRE(poly.ReadOBJModel("../data/igloo.obj", textureModel, false) == true);

    //CASE 2: Copy and Construct
    PolyMesh polyB(poly);
    REQUIRE(poly.vertexList.size() == polyB.vertexList.size());
    REQUIRE(poly.getVarList() == polyB.getVarList());

    //CASE3: Compute Volume
    REQUIRE(poly.ReadOBJModel("../data/tetrahedron.obj", textureModel, false) == true);
    REQUIRE(std::abs(poly.ComputeVolume() - 1.0f/6) < 1e-5);
}

#endif