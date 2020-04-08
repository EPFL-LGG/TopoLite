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

	textureList.clear();

    texturedModel = false;
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
        centroid = Vector3d(0, 0, 0);
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
bool PolyMesh<Scalar>::readOBJModel(
        const char *fileName,
        bool &textureModel_,
        bool normalized)
{
	vector<vector<double>> V, TC;
	vector<vector<int>> F, FTC;

	clear();

	if(readOBJ(fileName, V, TC, F, FTC))
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

	    if(TC.empty()){
	        texturedModel = textureModel_ = false;
	    }
	    else{
            texturedModel = textureModel_ = true;
	    }

	    if(normalized)
	        normalize();

        removeDuplicatedVertices();
		return true;
	}
	else {
	    return false;
	}
}

template<typename Scalar>
bool PolyMesh<Scalar>::readOBJ(       const std::string obj_file_name,
                    vector<vector<double>> &V,
                    vector<vector<double>> &TC,
                    vector<vector<int>> &F,
                    vector<vector<int>> &FTC){
    FILE * obj_file = fopen(obj_file_name.c_str(),"r");
    if(NULL==obj_file)
    {
        fprintf(stderr,"IOError: %s could not be opened...\n",
                obj_file_name.c_str());
        return false;
    }

    // File open was successful so clear outputs
    V.clear();
    TC.clear();
    F.clear();
    FTC.clear();

    // variables and constants to assist parsing the .obj file
    // Constant strings to compare against
    std::string v("v");
    std::string vt("vt");
    std::string f("f");
    std::string tic_tac_toe("#");
#ifndef IGL_LINE_MAX
#  define IGL_LINE_MAX 2048
#endif

    char line[IGL_LINE_MAX];
    int line_no = 1;
    while (fgets(line, IGL_LINE_MAX, obj_file) != NULL)
    {
        char type[IGL_LINE_MAX];
        // Read first word containing type
        if(sscanf(line, "%s",type) == 1)
        {
            // Get pointer to rest of line right after type
            char * l = &line[strlen(type)];
            if(type == v)
            {
                std::istringstream ls(&line[1]);
                std::vector<Scalar > vertex{ std::istream_iterator<Scalar >(ls), std::istream_iterator<Scalar >() };

                if (vertex.size() < 3)
                {
                    fprintf(stderr,
                            "Error: readOBJ() vertex on line %d should have at least 3 coordinates",
                            line_no);
                    fclose(obj_file);
                    return false;
                }

                V.push_back(vertex);
            }else if(type == vt)
            {
                double x[3];
                int count =
                        sscanf(l,"%lf %lf %lf\n",&x[0],&x[1],&x[2]);
                if(count != 2 && count != 3)
                {
                    fprintf(stderr,
                            "Error: readOBJ() texture coords on line %d should have 2 "
                            "or 3 coordinates (%d)",
                            line_no,count);
                    fclose(obj_file);
                    return false;
                }
                std::vector<Scalar > tex(count);
                for(int i = 0;i<count;i++)
                {
                    tex[i] = x[i];
                }
                TC.push_back(tex);
            }else if(type == f)
            {
                const auto & shift = [&V](const int i)->int
                {
                    return i<0 ? i+V.size() : i-1;
                };
                const auto & shift_t = [&TC](const int i)->int
                {
                    return i<0 ? i+TC.size() : i-1;
                };
                std::vector<int > f;
                std::vector<int > ftc;
                std::vector<int > fn;
                // Read each "word" after type
                char word[IGL_LINE_MAX];
                int offset;
                while(sscanf(l,"%s%n",word,&offset) == 1)
                {
                    // adjust offset
                    l += offset;
                    // Process word
                    long int i,it,in;
                    if(sscanf(word,"%ld/%ld/%ld",&i,&it,&in) == 3)
                    {
                        f.push_back(shift(i));
                        ftc.push_back(shift_t(it));
                    }else if(sscanf(word,"%ld/%ld",&i,&it) == 2)
                    {
                        f.push_back(shift(i));
                        ftc.push_back(shift_t(it));
                    }else if(sscanf(word,"%ld//%ld",&i,&in) == 2)
                    {
                        f.push_back(shift(i));
                    }else if(sscanf(word,"%ld",&i) == 1)
                    {
                        f.push_back(shift(i));
                    }else
                    {
                        fprintf(stderr,
                                "Error: readOBJ() face on line %d has invalid element format\n",
                                line_no);
                        fclose(obj_file);
                        return false;
                    }
                }
                if(
                        (f.size()>0 && ftc.size() == 0) ||
                        (f.size()>0 && ftc.size() == 0) ||
                        (f.size()>0 && ftc.size() == f.size()) ||
                        (f.size()>0 && ftc.size() == f.size()))
                {
                    // No matter what add each type to lists so that lists are the
                    // correct lengths
                    F.push_back(f);
                    FTC.push_back(ftc);
                }else
                {
                    fprintf(stderr,
                            "Error: readOBJ() face on line %d has invalid format\n", line_no);
                    fclose(obj_file);
                    return false;
                }
            }else if(strlen(type) >= 1 && (type[0] == '#' ||
                                           type[0] == 'g'  ||
                                           type[0] == 's'  ||
                                           strcmp("usemtl",type)==0 ||
                                           strcmp("mtllib",type)==0))
            {
                //ignore comments or other shit
            }else
            {
                //ignore any other lines
                fprintf(stderr,
                        "Warning: readOBJ() ignored non-comment line %d:\n  %s",
                        line_no,
                        line);
            }
        }else
        {
            // ignore empty line
        }
        line_no++;
    }
    fclose(obj_file);
    assert(F.size() == FTC.size());

    return true;
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
    struct plane_contact
    {
        Vector3 nrm;
        double D;
        int groupID;
        wpPolygon polygon;
        double eps;
    };

    struct plane_contact_compare
    {
        bool operator()(const plane_contact& A, const plane_contact& B) const
        {
            double eps = A.eps / 2;

            for(int id = 0; id < 3; id++){
                if (A.nrm[id] - B.nrm[id] < -eps)
                    return true;
                if (A.nrm[id] - B.nrm[id] > eps)
                    return false;
            }

            if (A.D - B.D < -eps)
                return true;
            if (A.D - B.D > eps)
                return false;

            return false;
        }
    };

    vector<plane_contact> planes;
    std::set<plane_contact, plane_contact_compare> setPlanes;

    //0) scale the meshes into united box
    double maxD = 1;
    for (pPolygon face : this->polyList) {
        //1.1) construct plane
        plane_contact plane;
        Vector3 nrm = face->normal();
        Vector3 center = face->vers[0]->pos;
        plane.nrm = nrm;

        plane.D = nrm.dot(center);
        maxD = std::max(maxD, std::abs(plane.D));
    }

    int groupID = 0;
    for (pPolygon face : this->polyList)
    {
        //1.1) construct plane
        plane_contact plane;
        Vector3 nrm = face->normal();
        Vector3 center = face->center();
        plane.nrm = nrm;

        plane.D = (nrm.dot(center)) / maxD;
        plane.polygon = face;
        plane.eps = eps;

        //1.2) find groupID
        auto find_it = setPlanes.find(plane);

        if(find_it == setPlanes.end()){
            plane.groupID = groupID ++;
            setPlanes.insert(plane);
        }
        else{
            plane.groupID = find_it->groupID;
        }

        planes.push_back(plane);
    }

    std::sort(planes.begin(), planes.end(), [&](const plane_contact &A, const plane_contact &B){
        return A.groupID < B.groupID;
    });

    int sta = 0, end = 0;
    vector<pPolygon> polygons;
    while(sta < planes.size())
    {
        for(end = sta + 1; end < planes.size(); end++)
        {
            if(planes[sta].groupID != planes[end].groupID)
            {
                break;
            }
        }

        if (end - sta > 1)
        {
            vector<vector<Vector3>> allFaces;
            for (int kd = sta; kd < end; kd++)
            {
                if(planes[kd].polygon.lock())
                    allFaces.push_back(planes[kd].polygon.lock()->getVertices());
            }
            vector<vector<Vector3>> mergeFaces;
            PolyPolyBoolean<Scalar> polyBoolean(getVarList());
            polyBoolean.computePolygonsUnion(allFaces, mergeFaces);
            for(size_t kd = 0; kd < mergeFaces.size(); kd++){
                pPolygon poly = make_shared<_Polygon<Scalar>>();
                poly->setVertices(mergeFaces[kd]);
                polygons.push_back(poly);
            }
        }
        else{
            polygons.push_back(planes[sta].polygon.lock());
        }
        sta = end;
    }

    this->clear();
    this->setPolyLists(polygons);
    this->removeDuplicatedVertices();
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
        poly->convertToTriangles(triangles);

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
