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

#include "Utility/HelpDefine.h"
#include "Utility/math3D.h"

#include "igl/boundary_loop.h"
#include "igl/readOBJ.h"
#include "igl/lscm.h"
#include "Polygon.h"
#include "PolyMesh.h"


#include <set>

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

template<typename Scalar>
PolyMesh<Scalar>::~PolyMesh()
{
	clear();
}

template<typename Scalar>
void PolyMesh<Scalar>::clear()
{
	polyList.clear();

	vertexList.clear();

    texturedModel = false;

    bbox = Box<Scalar>();

    centroid = Vector3(0, 0, 0);

    volume = 0;

    lowestPt = Vector3(0, 0, 0);
}

template<typename Scalar>
void PolyMesh<Scalar>::setPolyLists(vector<pPolygon> _polyList)
{
    //clear
    clear();

    //storage
    for(int id = 0; id < _polyList.size(); id++)
    {
        polyList.push_back(_polyList[id]);
    }

    //compute
    texturedModel = false;

    computeVertexList();

    computeCentroid();

    computeLowestPt();

    computeVolume();

    computeBBox();

    return;
}

template<typename Scalar>
PolyMesh<Scalar>::PolyMesh(const PolyMesh &_mesh): TopoObject(_mesh)
{
    //clear
    clear();

    //storage
    for(int id = 0; id < _mesh.polyList.size(); id++)
    {
        shared_ptr<_Polygon<Scalar>> poly = make_shared<_Polygon<Scalar>>(*_mesh.polyList[id]);
        polyList.push_back(poly);
    }

    //compute
    texturedModel = _mesh.texturedModel;

    computeVertexList();

    computeTexList();

    computeCentroid();

    computeLowestPt();

    computeVolume();

    computeBBox();
    return;
}

template<typename Scalar>
void PolyMesh<Scalar>::print() const
{
    printf("vertex num: %lu \n", vertexList.size());
    for (int i = 0; i < vertexList.size(); i++)
    {
        Vector3 ver = vertexList[i]->pos;
        printf("%9.6f  %9.6f  %9.6f \n", ver.x(), ver.y(), ver.z());
    }
    printf("\n");

    printf("polygon num: %lu \n", polyList.size());
    for (int i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        for(int j=0; j < poly->vers.size(); j++)
        {
            printf("%d ", poly->vers[j]->verID);
        }
        printf("\n");
    }
    printf("\n");
}

template<typename Scalar>
void PolyMesh<Scalar>::normalize(Vector3 &trans, Scalar &scale)
{
    computeBBox();

    scale = 2.0 / std::max(bbox.size.x(), std::max(bbox.size.y(), bbox.size.z()));
    trans = -bbox.cenPt;

    for (int i = 0; i < vertexList.size(); ++i)
    {
        Vector3 origVer = vertexList[i]->pos;
        vertexList[i]->pos = (origVer + trans) * scale;
    }

    for (pPolygon poly: polyList){
        poly->update();
    }
}

//**************************************************************************************//
//                                  Mesh Operations
//**************************************************************************************//

template<typename Scalar>
void PolyMesh<Scalar>::computeBBox()
{
	Scalar minX, minY, minZ, maxX, maxY, maxZ;
	minX = minY = minZ = MAX_FLOAT;
	maxX = maxY = maxZ = MIN_FLOAT;

	for (int i = 0; i < vertexList.size(); i++)
	{
		Vector3 ver = vertexList[i]->pos;

		if (maxX < ver.x())  maxX = ver.x();
		if (minX > ver.x())  minX = ver.x();

		if (maxY < ver.y())  maxY = ver.y();
		if (minY > ver.y())  minY = ver.y();

		if (maxZ < ver.z())  maxZ = ver.z();
		if (minZ > ver.z())  minZ = ver.z();
	}

	bbox.minPt = Vector3(minX, minY, minZ);
	bbox.maxPt = Vector3(maxX, maxY, maxZ);
	bbox.cenPt = (bbox.minPt + bbox.maxPt) / 2;
	bbox.size = bbox.maxPt - bbox.minPt;
}

template<typename Scalar>
void PolyMesh<Scalar>::computeTextureBBox()
{

    Scalar minU, minV, maxU, maxV;
    minU = minV = MAX_FLOAT;
    maxU = maxV = MIN_FLOAT;

    for (int i = 0; i < vertexList.size(); i++)
    {
        Vector2 texCoord = vertexList[i]->tex;

        if (maxU < texCoord.x)   maxU = texCoord.x();
        if (minU > texCoord.x)   minU = texCoord.x();

        if (maxV < texCoord.y)   maxV = texCoord.y();
        if (minV > texCoord.y)   minV = texCoord.y();
    }

    texBBox.minPt = Vector3(minU, minV, 0);
    texBBox.maxPt = Vector3(maxU, maxV, 0);

}

template<typename Scalar>
void PolyMesh<Scalar>::computeVolume()
{
    // Convert into triangular mesh
    vector<pTriangle> triList;
    convertToTriMesh(triList);

    // Compute volume of the triangular mesh
    volume = computeVolume(triList);

    // Release the memory
    triList.clear();
}

/////////////////////////////////////////////////////////////////////
// Compute model volume following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf
template<typename Scalar>
Scalar PolyMesh<Scalar>::computeVolume(vector<pTriangle> triList) const
{
    Scalar volume = 0;

    // Accumulate volume value for each triangle
    for (int i = 0; i < triList.size(); i++)
    {
        pTriangle tri = triList[i];

        Vector3 ver0 = tri->v[0];
        Vector3 crossVec = -1.0f * (tri->v[2] - tri->v[0]).cross(tri->v[1] - tri->v[0]);
        Scalar dotP = ver0.dot(crossVec);

        volume += dotP;
    }

    volume = volume / 6.0;

    return volume;
}

template<typename Scalar>
void PolyMesh<Scalar>::computeCentroid()
{
    // Convert into triangular mesh
    vector<pTriangle> triList;
    convertToTriMesh(triList);

    // Compute centroid of the triangular mesh
    centroid = computeCentroid(triList);

    // Release the memory
    triList.clear();
}

/////////////////////////////////////////////////////////////////////
// Compute model centroid following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf
template<typename Scalar>
Matrix<Scalar, 3, 1> PolyMesh<Scalar>::computeCentroid(vector<pTriangle> triList) const
{
    Vector3 centroid = Vector3(0, 0, 0);

    // Save the 3 major axes
    Vector3 axes[3];
    axes[0] = Vector3(1, 0, 0);
    axes[1] = Vector3(0, 1, 0);
    axes[2] = Vector3(0, 0, 1);

    // Accumulate centroid value for each major axes
    for (int i = 0; i < 3; i++)
    {
        Vector3 axis = axes[i];

        for (int j = 0; j < triList.size(); j++)
        {
            pTriangle tri = triList[j];
            Vector3 crossVec = -1.0f * (tri->v[2] - tri->v[0]).cross(tri->v[1] - tri->v[0]);

            centroid[i] += (1 / 24.0f) * (crossVec.dot(axis)) *
                           (pow((tri->v[0] + tri->v[1]).dot(axis), 2) +
                            pow((tri->v[1] + tri->v[2]).dot(axis), 2) +
                            pow((tri->v[2] + tri->v[0]).dot(axis), 2));
        }
    }

    // Compute volume and centroid
    Scalar volume = computeVolume(triList);

    if(volume > FLOAT_ERROR_SMALL){
        centroid = centroid / (2.0 * volume);
    }
    else{
        centroid = Vector3d(0, 0, 0);
    }

    return centroid;
}

template<typename Scalar>
void PolyMesh<Scalar>::computeLowestPt()
{
    Vector3 down_direc (0.0, -1.0, 0.0);
    lowestPt = computeExtremeVertex(down_direc);
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PolyMesh<Scalar>::computeExtremeVertex(Vector3 rayDir) const
{
    Vector3 extremVer(0, 0, 0);

    // Push back face center points
    vector<Vector3> pointList;
    for (int i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        poly->computeCenter();
        pointList.push_back(poly->center());
    }

    //Push back edge middle points
    for (int i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        for (int j = 0; j < poly->vers.size(); j++)
        {
            Vector3 staPt = poly->vers[j]->pos;
            Vector3 endPt = poly->vers[(j + 1) % poly->vers.size()]->pos;

            Vector3 edgeMidPt = (staPt + endPt) / 2.0;

            pointList.push_back(edgeMidPt);
        }
    }

    //Push back all vertices
    for(pVertex vertex: vertexList){
        pointList.push_back(vertex->pos);
    }

    // Find the extreme point with the largest distance along ray direction and the shortest distance along perpendicular direction
    const Scalar weight = 0.3;
    Scalar maxDist = -std::numeric_limits<Scalar>::max();
    for (int i = 0; i < pointList.size(); i++)
    {
        Vector3 ver = pointList[i];
        Scalar  dist = ver.dot(rayDir) /(rayDir.norm());

        if (dist > maxDist + 1e-4)
        {
            maxDist = dist;
            extremVer = ver;
        }
    }
    return extremVer;
}


template<typename Scalar>
void PolyMesh<Scalar>::computeVertexList()
{
    vertexList.clear();

    //1) check whether the vertices of all polygons already have vertex index
    bool hasVerID = true;
    int numVertex = 0;
    for(pPolygon poly: polyList)
    {
        for(pVertex vertex : poly->vers)
        {
            if(vertex->verID == -1){
                hasVerID = false;
                break;
            }
            else{
                numVertex = std::max(numVertex, vertex->verID + 1);
            }
        }
    }

    if(hasVerID){
        //2.1) if vertices have vertex index
        vertexList.resize(numVertex);
        for(pPolygon poly: polyList) {
            for (pVertex vertex : poly->vers) {
                vertexList[vertex->verID] = vertex;
            }
        }
    }
    else{
        //2.2) if vertices do not have vertex index
        for(pPolygon poly: polyList) {
            for (pVertex vertex : poly->vers)
            {
                vertex->verID = vertexList.size();
                vertexList.push_back(vertex);
            }
        }
    }
}

template<typename Scalar>
void PolyMesh<Scalar>::computeTexList()
{
    texList.clear();

    //1) check whether the vertices of all polygons already have vertex index
    bool hasTexID = true;
    int numTex = 0;
    for(pPolygon poly: polyList)
    {
        for(pVTex tex : poly->texs)
        {
            if(tex->texID == -1){
                hasTexID = false;
                break;
            }
            else{
                numTex = std::max(numTex, tex->texID + 1);
            }
        }
    }

    if(hasTexID){
        //2.1) if vertices have vertex index
        texList.resize(numTex);
        for(pPolygon poly: polyList) {
            for (pVTex tex : poly->texs) {
                texList[tex->texID] = tex;
            }
        }
    }
    else{
        //2.2) if vertices do not have vertex index
        for(pPolygon poly: polyList) {
            for (pVTex tex : poly->texs)
            {
                tex->texID = texList.size();
                texList.push_back(tex);
            }
        }
    }
}


//**************************************************************************************//
//                                   Transform Mesh
//**************************************************************************************//

template<typename Scalar>
void PolyMesh<Scalar>::rotateMesh(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle)
{
	for (int i = 0; i < vertexList.size(); ++i)
	{
		pVertex ver = vertexList[i];
		ver->pos = OrientPoint<Scalar>::rotateVecAroundAxis(rotCenter, rotAxis, rotAngle);
	}
}

template<typename Scalar>
void PolyMesh<Scalar>::translateMesh(Vector3 move)
{
    for (int i = 0; i < vertexList.size(); ++i) {
        pVertex ver = vertexList[i];
        ver->pos += move;
    }
}

template<typename Scalar>
void PolyMesh<Scalar>::scaleMesh(Vector3 scale)
{
    for (int i = 0; i < vertexList.size(); ++i) {
        pVertex ver = vertexList[i];
        ver->pos[0] *= scale[0];
        ver->pos[1] *= scale[1];
        ver->pos[2] *= scale[2];
    }
}


//**************************************************************************************//
//                                   Read Save OBJ File
//**************************************************************************************//

template<typename Scalar>
bool PolyMesh<Scalar>::readOBJModel(
        const char *fileName,
        bool &textureModel_,
        bool normalized)
{
	vector<vector<double>> V, TC, CN;
	vector<vector<int>> F, FTC, FN;

	clear();

	if(igl::readOBJ(fileName, V, TC, CN, F, FTC, FN))
	{
	    clear();

        for(int id = 0; id < V.size(); id++) {
            {
                shared_ptr<VPoint<Scalar>> ver = make_shared<VPoint<Scalar>>();
                ver->pos = Vector3(V[id][0], V[id][1], V[id][2]);
                ver->verID = id;
                vertexList.push_back(ver);
            }
        }

        for(int id = 0; id < TC.size(); id++)
        {
                shared_ptr<VTex<Scalar>> vtex = make_shared<VTex<Scalar>>();
                vtex->texCoord = Vector2(TC[id][0], TC[id][1]);
                vtex->texID = id;
                texList.push_back(vtex);
        }

        for(int id = 0; id < F.size(); id++)
        {
			shared_ptr<_Polygon<Scalar>> poly = make_shared<_Polygon<Scalar>>();
            for(int jd = 0; jd < F[id].size(); jd++)
			{
				poly->vers.push_back(vertexList[F[id][jd]]);
				poly->texs.push_back(texList[FTC[id][jd]]);
			}
            poly->update();
            polyList.push_back(poly);
        }

	    if(TC.empty()){
	        texturedModel = textureModel_ = false;
	    }
	    else{
            texturedModel = textureModel_ = true;
	    }

        Vector3 trans;
	    Scalar scale;
	    if(normalized) normalize(trans, scale);
        removeDuplicatedVertices();
		return true;
	}
	else {
	    return false;
	}
}

template<typename Scalar>
shared_ptr<PolyMesh<Scalar>> PolyMesh<Scalar>::getTextureMesh() const
{
    shared_ptr<PolyMesh> polymesh = make_shared<PolyMesh>(getVarList());
    for (int i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = make_shared<_Polygon<Scalar>>(*polyList[i]);
        for(int j = 0; j < poly->texs.size(); j++)
        {
            if(poly->texs[j] != nullptr){
                poly->vers[j]->pos = Vector3(poly->texs[j]->texCoord.x(), poly->texs[j]->texCoord.y(), 0);
            }
        }
        poly->texs.clear();
        polymesh->polyList.push_back(poly);
    }

    return polymesh;
}


template<typename Scalar>
void PolyMesh<Scalar>::writeOBJModel(const char *objFileName, bool triangulate) const
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
		// 1. Check whether vertexList and texList exists
		if(vertexList.empty() || texList.empty()) return;

		///////////////////////////////////////////////////////////////////
		// 2. Write the vertex info of the mesh

		fprintf(fp, "# Wavefront OBJ generated by Ziqi & Peng SONG \n\n");

		fprintf(fp, "# %d vertices \n", (int)vertexList.size());

		for (int i = 0; i < vertexList.size(); i++)
		{
			fprintf(fp, "v %f %f %f \n", vertexList[i]->pos.x() , vertexList[i]->pos.y(), vertexList[i]->pos.z());
		}
		fprintf(fp, "\n");

        fprintf(fp, "# %d tex points \n", (int)texList.size());
		for(int i = 0; i < texList.size(); i++)
		{
			fprintf(fp, "vt %f %f \n", texList[i]->texCoord.x(), texList[i]->texCoord.y());
		}
		fprintf(fp, "\n");

		///////////////////////////////////////////////////////////////////
		// 3. Write each polygon info of the mesh

		fprintf(fp, "# %d faces \n", (int)polyList.size());

		for (int i = 0; i < polyList.size(); i++)
		{
			pPolygon poly = polyList[i];

            if(!triangulate)
            {
                fprintf(fp, "f ");
                for (int j = 0; j < poly->vers.size(); j++)
                {
                    int verID = poly->vers[j]->verID;
                    //Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index

                    if(texList.empty())
                        fprintf(fp, " %d", verID + 1);
                    else
                    {
                        int texID = poly->texs[j]->texID;
                        fprintf(fp, " %d/%d", verID + 1, texID + 1);
                    }

                }
                fprintf(fp, "\n");
            }
            else{
                for(int j = 0; j < (int)(poly->vers.size()) - 2; j++)
                {
                    fprintf(fp, "f ");
                    int indices[3] = {0, j + 1, j + 2};
                    for(int k = 0; k < 3; k++)
                    {
						int verID = poly->vers[indices[k]]->verID;
                        if(texList.empty())
                            fprintf(fp, " %d", verID + 1);
                        else{
                            int texID = poly->texs[indices[k]]->texID;
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

template<typename Scalar>
void PolyMesh<Scalar>::removeDuplicatedVertices(double eps)
{
    std::set<sort_vertex, sort_vertex_compare> setVertices;

    vertexList.clear();

    int vID = 0;
	for (int i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];
		if(poly == nullptr) continue;
		for (int j = 0; j < poly->vers.size(); j++)
		{
            sort_vertex sver;
            {
                sver.pos = poly->vers[j]->pos;
                sver.vID = 0;
                sver.eps = eps;
                sver.ptr = poly->vers[j];
            }

			auto find_it = setVertices.find(sver);
			if(find_it != setVertices.end())
			{
			    poly->vers[j] = find_it->ptr;
			}
			else{
                vertexList.push_back(sver.ptr);
                sver.vID = vID ++;
                poly->vers[j]->verID = sver.vID;
			    setVertices.insert(sver);
			}
		}
	}
}

//**************************************************************************************//
//                                   Mesh Converter
//**************************************************************************************//

template<typename Scalar>
void PolyMesh<Scalar>::convertToTriMesh(vector<pTriangle> &triList) const
{
    for (int i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];

        vector<pTriangle> triangles;
        poly->convertToTriangles(triangles);

        for (int j = 0; j < triangles.size(); j++)
        {
            triList.push_back(triangles[j]);
        }
    }
}