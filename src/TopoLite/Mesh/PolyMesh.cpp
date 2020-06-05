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

#include "igl/boundary_loop.h"
#include "igl/readOBJ.h"
#include "Polygon.h"
#include "PolyMesh.h"
#include <unordered_map>
#include <queue>


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

	textureList.clear();

    texturedModel = false;
}

/*
 * Construct From a Eigen Mesh
 */

template<typename Scalar>
void PolyMesh<Scalar>::fromEigenMesh(const MatrixX &V, const MatrixX &T, const MatrixXi &F){
    clear();

    size_t psize = F.rows();

    //add vertex
    for(int id = 0; id < V.rows(); id++){
        pVertex vertex = make_shared<VPoint<Scalar>>(V.row(id));
        vertex->verID = id;
        vertexList.push_back(vertex);
    }

    //add tex
    for(int id = 0; id < T.rows(); id++){
        pVTex tex = make_shared<VTex<Scalar>>(T.row(id));
        tex->texID = id;
        textureList.push_back(tex);
    }

    //add polygons
    for(int id = 0; id < F.rows(); id++){
        pPolygon polygon = make_shared<_Polygon<Scalar>>();
        for(int jd = 0; jd < 3; jd++){
            polygon->vers.push_back(vertexList[F(id, jd)]);
            polygon->texs.push_back(textureList[F(id, jd)]);
        }
        polyList.push_back(polygon);
    }

    texturedModel = true;
    return;
}


/*
 * Defines vertices for a list of polygons
 */
template<typename Scalar>
void PolyMesh<Scalar>::setPolyLists(vector<pPolygon> _polyList)
{
    clear();

    // create polygon list

    size_t psize = _polyList.size();

    for(size_t id = 0; id < psize; id++)
        polyList.push_back(_polyList[id]);

    texturedModel = false;

    computeVertexList();

    computeTextureList();

}

template<typename Scalar>
PolyMesh<Scalar>::PolyMesh(const PolyMesh &_mesh): TopoObject(_mesh)
{
    clear();

    // create polygon list

    size_t msize = _mesh.polyList.size();

    for(size_t id = 0; id < msize; id++)
    {
        shared_ptr<_Polygon<Scalar>> poly = make_shared<_Polygon<Scalar>>(*_mesh.polyList[id]);
        polyList.push_back(poly);
    }

    texturedModel = _mesh.texturedModel;

    computeVertexList();

    computeTextureList();

}

template<typename Scalar>
void PolyMesh<Scalar>::print() const
{
    printf("vertex num: %lu \n", vertexList.size());
    for (size_t i = 0; i < vertexList.size(); i++)
    {
        Vector3 ver = vertexList[i]->pos;
        printf("%9.6f  %9.6f  %9.6f \n", ver.x(), ver.y(), ver.z());
    }
    printf("\n");

    printf("polygon num: %lu \n", polyList.size());
    for (size_t i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        for(size_t j=0; j < poly->vers.size(); j++)
        {
            printf("%d ", poly->vers[j]->verID);
        }
        printf("\n");
    }
    printf("\n");
}

template<typename Scalar>
std::pair<Matrix<Scalar, 3, 1>, Scalar> PolyMesh<Scalar>::normalize()
{
    Box<Scalar> bbox_ = bbox();

    Scalar scale = 2.0 / std::max(bbox_.size.x(), std::max(bbox_.size.y(), bbox_.size.z()));
    Vector3 trans = -bbox_.cenPt;

    for (size_t i = 0; i < vertexList.size(); ++i)
    {
        Vector3 origVer = vertexList[i]->pos;
        vertexList[i]->pos = (origVer + trans) * scale;
    }

    return std::pair<Matrix<Scalar, 3, 1>, Scalar>(trans, scale);
}

//**************************************************************************************//
//                                  Mesh Operations
//**************************************************************************************//

/*
 * Defines a box delimited by a given number of polygons
 * - minPt = point with lowest  individual coordinates
 * - maxPt = point with highest individual coordinates
 */
template<typename Scalar>
Box<Scalar> PolyMesh<Scalar>::bbox() const
{
	Scalar minX, minY, minZ, maxX, maxY, maxZ;
	minX = minY = minZ = MAX_FLOAT;
	maxX = maxY = maxZ = MIN_FLOAT;

	for (const auto &v: vertexList)
	{
		if (maxX < v->pos.x())  maxX = v->pos.x();
		if (minX > v->pos.x())  minX = v->pos.x();

		if (maxY < v->pos.y())  maxY = v->pos.y();
		if (minY > v->pos.y())  minY = v->pos.y();

		if (maxZ < v->pos.z())  maxZ = v->pos.z();
		if (minZ > v->pos.z())  minZ = v->pos.z();
	}

	Box<Scalar> bbox_;
	bbox_.minPt = Vector3(minX, minY, minZ);
	bbox_.maxPt = Vector3(maxX, maxY, maxZ);
	bbox_.cenPt = (bbox_.minPt + bbox_.maxPt) / 2;
	bbox_.size = bbox_.maxPt - bbox_.minPt;

	return bbox_;
}

template<typename Scalar>
Box<Scalar> PolyMesh<Scalar>::texBBox() const
{

    Scalar minU, minV, maxU, maxV;
    minU = minV = MAX_FLOAT;
    maxU = maxV = MIN_FLOAT;

    for (size_t i = 0; i < textureList.size(); i++)
    {
        Vector2 texCoord = textureList[i]->texCoord;

        if (maxU < texCoord.x())   maxU = texCoord.x();
        if (minU > texCoord.x())   minU = texCoord.x();

        if (maxV < texCoord.y())   maxV = texCoord.y();
        if (minV > texCoord.y())   minV = texCoord.y();
    }

    Box<Scalar> texBBox_;
    texBBox_.minPt = Vector3(minU, minV, 0);
    texBBox_.maxPt = Vector3(maxU, maxV, 0);
    texBBox_.cenPt = (texBBox_.minPt + texBBox_.maxPt) / 2;
    texBBox_.size = texBBox_.maxPt - texBBox_.minPt;

    return texBBox_;
}

template<typename Scalar>
Scalar PolyMesh<Scalar>::volume() const
{
    // Convert into triangular mesh
    vector<pTriangle> triList;
    convertToTriMesh(triList);

    // Compute volume of the triangular mesh
    return computeVolume(triList);
}

/*
 * Compute model volume following Dr Robert Nurnberg's method at:
 * http://wwwf.imperial.ac.uk/~rn/centroid.pdf
 */
template<typename Scalar>
Scalar PolyMesh<Scalar>::computeVolume(vector<pTriangle> triList) const
{
    Scalar volume = 0;

    // Accumulate volume value for each triangle
    for (size_t i = 0; i < triList.size(); i++)
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
Matrix<Scalar, 3, 1> PolyMesh<Scalar>::centroid() const
{
    // Convert into triangular mesh
    vector<pTriangle> triList;
    convertToTriMesh(triList);

    // Compute centroid of the triangular mesh
    return computeCentroid(triList);
}

/*
* Compute model centroid following Dr Robert Nurnberg's method at:
* http://wwwf.imperial.ac.uk/~rn/centroid.pdf
*/
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

        for (size_t j = 0; j < triList.size(); j++)
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
        centroid = Vector3(0, 0, 0);
    }

    return centroid;
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PolyMesh<Scalar>::lowestPt() const
{
    Vector3 down_direc (0.0, -1.0, 0.0);
    return computeExtremeVertex(down_direc);
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PolyMesh<Scalar>::computeExtremeVertex(Vector3 rayDir) const
{
    Vector3 extremVer(0, 0, 0);

    // Push back face center points
    vector<Vector3> pointList;
    for (size_t i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        pointList.push_back(poly->center());
    }

    // Push back edge middle points
    for (size_t i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];
        for (size_t j = 0; j < poly->vers.size(); j++)
        {
            Vector3 staPt = poly->vers[j]->pos;
            Vector3 endPt = poly->vers[(j + 1) % poly->vers.size()]->pos;

            Vector3 edgeMidPt = (staPt + endPt) / 2.0;

            pointList.push_back(edgeMidPt);
        }
    }

    // Push back all vertices
    for(pVertex vertex: vertexList){
        pointList.push_back(vertex->pos);
    }

    // Find the extreme point with the largest distance along ray direction and the shortest distance along perpendicular direction
    const Scalar weight = 0.3;
    Scalar maxDist = -std::numeric_limits<Scalar>::max();
    for (size_t i = 0; i < pointList.size(); i++)
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

    // 1) check whether the vertices of all polygons already have vertex index
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
        // 2.1) if vertices have vertex index
        vertexList.resize(numVertex);
        for(pPolygon poly: polyList) {
            for (pVertex vertex : poly->vers) {
                vertexList[vertex->verID] = vertex;
            }
        }
    }
    else{
        // 2.2) if vertices do not have vertex index
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
void PolyMesh<Scalar>::computeTextureList()
{
    textureList.clear();

    // 1) check whether the vertices of all polygons already have vertex index
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
        // 2.1) if vertices have vertex index
        textureList.resize(numTex);
        for(pPolygon poly: polyList) {
            for (pVTex tex : poly->texs) {
                textureList[tex->texID] = tex;
            }
        }
    }
    else{
        // 2.2) if vertices do not have vertex index
        for(pPolygon poly: polyList) {
            for (pVTex tex : poly->texs)
            {
                tex->texID = textureList.size();
                textureList.push_back(tex);
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
	for (size_t i = 0; i < vertexList.size(); ++i)
	{
		pVertex ver = vertexList[i];
		ver->pos = OrientPoint<Scalar>::rotateVecAroundAxis(rotCenter, rotAxis, rotAngle);
	}
}

template<typename Scalar>
void PolyMesh<Scalar>::translateMesh(Vector3 move)
{
    for (size_t i = 0; i < vertexList.size(); ++i) {
        pVertex ver = vertexList[i];
        ver->pos += move;
    }
}

template<typename Scalar>
void PolyMesh<Scalar>::scaleMesh(Vector3 scale)
{
    for (size_t i = 0; i < vertexList.size(); ++i) {
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
bool PolyMesh<Scalar>::readOBJModel(const vector<vector<double>> &V, const vector<vector<double>> &TC, const vector<vector<int>> &F, const vector<vector<int>> &FTC, bool normalized)
{
    clear();
    
    for(size_t id = 0; id < V.size(); id++) {
        {
            shared_ptr<VPoint<Scalar>> ver = make_shared<VPoint<Scalar>>();
            ver->pos = Vector3(V[id][0], V[id][1], V[id][2]);
            ver->verID = id;
            vertexList.push_back(ver);
        }
    }

    for(size_t id = 0; id < TC.size(); id++)
    {
            shared_ptr<VTex<Scalar>> vtex = make_shared<VTex<Scalar>>();
            vtex->texCoord = Vector2(TC[id][0], TC[id][1]);
            vtex->texID = id;
            textureList.push_back(vtex);
    }

    for(size_t id = 0; id < F.size(); id++)
    {
        shared_ptr<_Polygon<Scalar>> poly = make_shared<_Polygon<Scalar>>();
        for(size_t jd = 0; jd < F[id].size(); jd++)
        {
            poly->vers.push_back(vertexList[F[id][jd]]);
            if(FTC[id].size() > jd)
                poly->texs.push_back(textureList[FTC[id][jd]]); // can have a memory error without the if condition
        }
        polyList.push_back(poly);
    }

    if(TC.empty())
    {
        texturedModel = false;
    }
    else
    {
        texturedModel = true;
    }
    
    if(normalized)
    {
        normalize();
    }
    
    removeDuplicatedVertices();
    return true;
}


template<typename Scalar>
bool PolyMesh<Scalar>::readOBJModel(
        const char *fileName,
        bool normalized)
{
	vector<vector<double>> V, TC, N;
	vector<vector<int>> F, FTC, FTN;
    
	if(igl::readOBJ(fileName, V, TC, N,F, FTC, FTN))
	{
        return readOBJModel(V, TC, F, FTC, normalized);
    }
	else {
	    return false;
	}
}

template<typename Scalar>
void PolyMesh<Scalar>::writeOBJModel(const char *objFileName, bool triangulate) const
{
	FILE *fp;
	if ((fp = fopen(objFileName, "w+")) == nullptr)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		///////////////////////////////////////////////////////////////////
		// 1. Check whether vertexList and textureList exists
		if(vertexList.empty()) return;

		///////////////////////////////////////////////////////////////////
		// 2. Write the vertex info of the mesh

		fprintf(fp, "# Wavefront OBJ generated by Ziqi & Peng SONG \n\n");

		fprintf(fp, "# %d vertices \n", (int)vertexList.size());

		for (size_t i = 0; i < vertexList.size(); i++)
		{
			fprintf(fp, "v %f %f %f \n", vertexList[i]->pos.x() , vertexList[i]->pos.y(), vertexList[i]->pos.z());
		}
		fprintf(fp, "\n");

        fprintf(fp, "# %d tex points \n", (int)textureList.size());
		for(size_t i = 0; i < textureList.size(); i++)
		{
			fprintf(fp, "vt %f %f \n", textureList[i]->texCoord.x(), textureList[i]->texCoord.y());
		}
		fprintf(fp, "\n");

		///////////////////////////////////////////////////////////////////
		// 3. Write each polygon info of the mesh

		fprintf(fp, "# %d faces \n", (int)polyList.size());

		for (size_t i = 0; i < polyList.size(); i++)
		{
			pPolygon poly = polyList[i];

            if(!triangulate)
            {
                fprintf(fp, "f ");
                for (size_t j = 0; j < poly->vers.size(); j++)
                {
                    int verID = poly->vers[j]->verID;
                    //Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index

                    if(textureList.empty())
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
                        if(textureList.empty())
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


template <typename Scalar>
void PolyMesh<Scalar>::mergeFaces(double eps)
{

    this->removeDuplicatedVertices(eps);

    vector<vector<wpPolygon>> vertexFaces(vertexList.size());
    for(pPolygon poly: polyList)
    {
        for(pVertex vertex: poly->vers)
        {
            vertexFaces[vertex->verID].push_back(poly);
        }
    }

    auto find_vertex_localID = [=](pVertex vertex, pPolygon poly) -> int{
        for(int id = 0; id < poly->vers.size(); id++){
            if(poly->vers[id] == vertex){
                return id;
            }
        }
        return -1;
    };

    auto find_neighbor = [=](pPolygon face) -> vector<wpPolygon>{
        std::unordered_map<pPolygon, bool> visited;
        vector<wpPolygon> neighbors;
        visited[face] = true;
        for(int id = 0; id < face->vers.size(); id++)
        {
            pVertex vertex = face->vers[id];
            pVertex next_vertex = face->vers[(id + 1) % face->vers.size()];
            for(wpPolygon neighbor: vertexFaces[vertex->verID])
            {
                if(visited.find(neighbor.lock()) == visited.end()){
                    int neighbor_size = neighbor.lock()->vers.size();
                    int neighbor_vertex_localID = find_vertex_localID(vertex, neighbor.lock());
                    int neighbor_prev_vertex_localID = (neighbor_vertex_localID - 1 + neighbor_size) % neighbor_size;
                    pVertex neighbor_pre_vertex = neighbor.lock()->vers[neighbor_prev_vertex_localID];
                    if(next_vertex == neighbor_pre_vertex){
                        neighbors.push_back(neighbor);
                        visited[neighbor.lock()] = true;
                    }
                }
            }
        }
        return neighbors;
    };

    auto check_edge_at_boundary = [=](pVertex vertex, pPolygon poly, std::unordered_map<pPolygon, bool> face_in_merge) -> bool{

        std::unordered_map<pPolygon, bool> visited;
        for(wpPolygon face: vertexFaces[vertex->verID]){
            if(face_in_merge.find(face.lock()) != face_in_merge.end()){
                visited[face.lock()] = true;
            }
        }

        int vertex_localID = find_vertex_localID(vertex, poly);
        int next_localID = (vertex_localID + 1) % poly->size();
        pVertex next_vertex = poly->vers[next_localID];

        for(wpPolygon next_poly: vertexFaces[next_vertex->verID]){
            if(next_poly.lock() != poly && visited.find(next_poly.lock()) != visited.end()){
                return false;
            }
        }
        return true;
    };

    std::unordered_map<pPolygon, bool> visited;

    for(pPolygon face: polyList){
        if(visited.find(face) == visited.end()){

            vector<pPolygon> merged_faces;
            merged_faces.push_back(face);
            std::queue<pPolygon> queue_faces;

            queue_faces.push(face);
            visited[face] = true;

            std::unordered_map<pPolygon, bool> localvisited = visited;
            while(!queue_faces.empty()){
                pPolygon u = queue_faces.front(); queue_faces.pop();
                vector<wpPolygon> neighbors = find_neighbor(u);
                Vector3 u_normal = u->normal();
                for(wpPolygon neighbor: neighbors){
                    if(localvisited.find(neighbor.lock()) == localvisited.end()){
                        Vector3 neighbor_normal = neighbor.lock()->normal();
                        if(std::abs(u_normal.cross(neighbor_normal).norm()) < eps
                        && u_normal.dot(neighbor_normal) >= 0){
                            merged_faces.push_back(neighbor.lock());
                            queue_faces.push(neighbor.lock());
                        }
                        localvisited[neighbor.lock()] = true;
                    }
                }
            }

            std::unordered_map<pPolygon, bool> faces_in_merge;
            for(pPolygon face: merged_faces){
                visited[face] = true;
                faces_in_merge[face] = true;
                face->edge_at_boundary.clear();
            }

            for(pPolygon face: merged_faces)
            {
                for(pVertex vertex: face->vers)
                {
                    face->edge_at_boundary.push_back(check_edge_at_boundary(vertex, face, faces_in_merge));
                }
            }
        }
    }

}

template<typename Scalar>
void PolyMesh<Scalar>::removeDuplicatedVertices(double eps)
{
    std::set<sort_vertex, sort_vertex_compare> setVertices;

    vertexList.clear();

    int vID = 0;
	for (size_t i = 0; i < polyList.size(); i++)
	{
		pPolygon poly = polyList[i];
		if(poly == nullptr) continue;
		for (size_t j = 0; j < poly->vers.size(); j++)
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
    for (size_t i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = polyList[i];

        vector<pTriangle> triangles;
        poly->triangulateNaive(triangles);

        for (size_t j = 0; j < triangles.size(); j++)
        {
            triList.push_back(triangles[j]);
        }
    }
}

template<typename Scalar>
void PolyMesh<Scalar>::convertPosToEigenMesh(PolyMesh::MatrixX &V, PolyMesh::MatrixXi &F, Eigen::VectorXi &C)
{
    //V
    if(vertexList.empty()) return;
    V = MatrixX(vertexList.size(), 3);

    for(size_t id = 0; id < vertexList.size(); ++id){
        V.row(id) = vertexList[id]->pos;
    }

    //F
    if(polyList.empty()) return;
    int numFace = 0;
    //check all vertex has vertex ID
    for(pPolygon poly: polyList)
    {
        if(poly->vers.size() <= 2) return;
        for(pVertex vertex : poly->vers)
        {
            if(vertex->verID == -1){
                return;
            }
        }
        numFace += poly->vers.size() - 2;
    }

    F = MatrixXi(numFace, 3);
    C = Eigen::VectorXi(numFace);

    int fid  = 0;
    for(size_t pID = 0; pID < polyList.size(); pID ++)
    {
        pPolygon poly = polyList[pID];
        for(int id = 0; id < (int)poly->vers.size() - 2; id++)
        {
            int ori_vid = poly->vers[0]->verID;
            int curr_vid = poly->vers[id + 1]->verID;
            int next_vid = poly->vers[id + 2]->verID;
            F(fid, 0) = ori_vid;
            F(fid, 1) = curr_vid;
            F(fid, 2) = next_vid;
            C(fid) = pID;
            fid ++;
        }
    }

    return;
}

template<typename Scalar>
void PolyMesh<Scalar>::convertTexToEigenMesh(PolyMesh::MatrixX &V, PolyMesh::MatrixXi &F, Eigen::VectorXi &C)
{
    //V
    if(textureList.empty()) return;
    V = MatrixX(textureList.size(), 2);

    for(size_t id = 0; id < textureList.size(); ++id){
        V.row(id) = textureList[id]->texCoord;
    }

    //F
    if(polyList.empty()) return;
    int numFace = 0;
    //check all tex has tex ID
    for(pPolygon poly: polyList)
    {
        if(poly->texs.size() <= 2) return;
        for(pVTex tex : poly->texs)
        {
            if(tex->texID == -1){
                return;
            }
        }
        numFace += poly->texs.size() - 2;
    }

    F = MatrixXi(numFace, 3);
    C = Eigen::VectorXi(numFace);

    int fid  = 0;
    for(size_t pID = 0; pID < polyList.size(); pID ++)
    {
        pPolygon poly = polyList[pID];
        for(int id = 0; id < (int)poly->texs.size() - 2; id++)
        {
            int ori_tid = poly->texs[0]->texID;
            int curr_tid = poly->texs[id + 1]->texID;
            int next_tid = poly->texs[id + 2]->texID;
            F(fid, 0) = ori_tid;
            F(fid, 1) = curr_tid;
            F(fid, 2) = next_tid;
            C(fid) = pID;
            fid ++;
        }
    }

}

template<typename Scalar>
void PolyMesh<Scalar>::convertPosTexToEigenMesh(MatrixX &V, MatrixX &T, MatrixXi &F)
{
    Eigen::VectorXi C;
    convertPosToEigenMesh(V, F, C);
    T = MatrixX::Zero(V.rows(), 2);
    if(polyList.empty()) return;

    //update T
    for(pPolygon poly: polyList)
    {
        if(poly->vers.size() <= 2) return;
        for(int id = 0; id < poly->texs.size(); id++)
        {
            int verID = poly->vers[id]->verID;
            T.row(verID) = poly->texs[id]->texCoord;
        }
    }
}


template<typename Scalar>
shared_ptr<PolyMesh<Scalar>> PolyMesh<Scalar>::getTextureMesh() const
{
    shared_ptr<PolyMesh> polymesh = make_shared<PolyMesh>(getVarList());
    for (size_t i = 0; i < polyList.size(); i++)
    {
        pPolygon poly = make_shared<_Polygon<Scalar>>(*polyList[i]);
        for(size_t j = 0; j < poly->texs.size(); j++)
        {
            if(poly->texs[j] != nullptr){
                poly->vers[j]->pos = Vector3(poly->texs[j]->texCoord.x(), poly->texs[j]->texCoord.y(), 0);
            }
        }
        poly->texs.clear();
        polymesh->polyList.push_back(poly);
    }

    //rebuild the vertexList and textureList of the polymesh
    polymesh->update();
    return polymesh;
}

template<typename Scalar>
nlohmann::json PolyMesh<Scalar>::dump() const {
    nlohmann::json mesh_json;
    if (vertexList.empty()) return nlohmann::json();

    //1) write vertices
    {
        nlohmann::json vertices_json;
        vertices_json["n_vertices"] = (int) vertexList.size();
        for (size_t i = 0; i < vertexList.size(); i++) {
            std::string name = std::to_string(i);
            vertices_json[name] = {vertexList[i]->pos.x(),
                                   vertexList[i]->pos.y(),
                                   vertexList[i]->pos.z()};
        }

        mesh_json["vertices"] = vertices_json;
    }

    //2) write faces
    {
        nlohmann::json faces_json;
        faces_json["n_faces"] = (int) polyList.size();

        for (size_t i = 0; i < polyList.size(); i++) {
            shared_ptr<_Polygon<Scalar>> poly = polyList[i];
            vector<int> verIDs;
            vector<int> texIDs;
            for (size_t j = 0; j < poly->vers.size(); j++) {
                verIDs.push_back(poly->vers[j]->verID);
                if (!textureList.empty()) {
                    texIDs.push_back(poly->texs[j]->texID);
                }
            }

            std::string name = std::to_string(i);
            nlohmann::json face_json;
            face_json["verID"] = verIDs;
            face_json["texID"] = texIDs;
            faces_json[name] = face_json;
        }
        mesh_json["faces"] = faces_json;
    }
    return mesh_json;
}
template class PolyMesh<double>;
template class PolyMesh<float>;
